#include <micro_ros_arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <esp_sleep.h>
#include <esp_wifi.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>          // rmw_uros_sync_session / epoch_nanos
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>

// ==================== WiFi / Agent ====================
// 真实凭据放在同目录的 secrets.h（已加 .gitignore）。
// 第一次编译前：cp secrets.h.template secrets.h，把占位符改成你自己的值。
#if __has_include("secrets.h")
  #include "secrets.h"
#else
  #error "secrets.h not found — copy secrets.h.template -> secrets.h and fill it in"
#endif

char WIFI_SSID[] = ARM_WIFI_SSID;
char WIFI_PASS[] = ARM_WIFI_PASS;
char AGENT_IP[]  = ARM_AGENT_IP;
#define AGENT_PORT ARM_AGENT_PORT

// ==================== LED ====================
#define LED_PIN  48
#define NUM_LEDS 1

// ==================== I2C / PCA9685 ====================
#define I2C_SDA         8
#define I2C_SCL         9
#define PCA9685_ADDRESS 0x40

#define SERVO_COUNT      6
#define SERVO_FREQ       50
#define SERVO_MIN_PULSE  150
#define SERVO_MAX_PULSE  600
#define SERVO_MIN_ANGLE  0.0f
#define SERVO_MAX_ANGLE  180.0f

// ==================== 控制参数(默认值) ====================
// 这些是 boot 默认值;运行时可通过订阅 /servo_tuning(Float32MultiArray
// [alpha, max_step_deg, deadband_deg])动态覆盖,无需重烧固件。
#define SERVO_DEADBAND_DEG_DEFAULT  1.5f
#define SERVO_SMOOTH_ALPHA_DEFAULT  0.18f
#define SERVO_MAX_STEP_DEG_DEFAULT  3.0f
#define CMD_FLASH_MS                80

// 可调运行时变量(/servo_tuning 写入)
volatile float servo_deadband_deg = SERVO_DEADBAND_DEG_DEFAULT;
volatile float servo_smooth_alpha = SERVO_SMOOTH_ALPHA_DEFAULT;
volatile float servo_max_step_deg = SERVO_MAX_STEP_DEG_DEFAULT;

// ==================== 软启动 / 软停 / 浅睡眠 ====================
// 上电后等待主电源/电容稳定的时间
#define SOFT_START_SETTLE_MS       500
// 通道错峰启用间隔（错开各舵机的瞬态启动电流）
#define SOFT_START_CHANNEL_GAP_MS  250
// 关机时通道错峰断 PWM 的间隔
#define SOFT_STOP_CHANNEL_GAP_MS   200
// 软停归位时每帧最大角度步长（度）
#define SOFT_RAMP_STEP_DEG         1.0f
// 软停每帧时长（20 ms ≈ 50 Hz）
#define SOFT_RAMP_FRAME_MS         20
// 关机后浅睡眠的单次时长（μs）；醒来后 spin executor 收开机指令
#define LIGHT_SLEEP_WINDOW_US      100000ULL

// ==================== micro-ROS 对象 ====================
rcl_subscription_t joint_sub;
rcl_subscription_t array_sub;
rcl_subscription_t power_sub;
rcl_subscription_t reboot_sub;
rcl_subscription_t tuning_sub;
rcl_publisher_t    joint_pub;
rcl_publisher_t    i2c_status_pub;

sensor_msgs__msg__JointState     joint_msg;        // 订阅 /joint_commands
sensor_msgs__msg__JointState     joint_state_msg;  // 发布 /joint_states
std_msgs__msg__Float32MultiArray array_msg;        // 订阅 /servo_angles
std_msgs__msg__Float32MultiArray tuning_msg;       // 订阅 /servo_tuning
std_msgs__msg__String            i2c_status_msg;   // 发布 /servo_i2c_status
std_msgs__msg__Bool              power_msg;        // 订阅 /arm_power_control
std_msgs__msg__Empty             reboot_msg;       // 订阅 /arm_reboot

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rcl_timer_t     timer;

// ==================== 消息静态内存池 ====================
// 每个消息独立一块静态缓冲，大小用 micro_ros_utilities_get_static_size 估算后留余量
// JointState: 6 joints * (name 16B + position 8B) + frame_id 32B ≈ 256B，给 512B 更安全
static uint8_t joint_msg_buf[512];
static uint8_t joint_state_msg_buf[512];
// Float32MultiArray: layout(dim 0 个元素) + 6 x float = 约 64B
static uint8_t array_msg_buf[128];
static uint8_t tuning_msg_buf[128];          // 同上,但 3 floats 就够
// String: 64B 消息内容
static uint8_t i2c_status_msg_buf[128];

// ==================== 硬件对象 ====================
Adafruit_NeoPixel      strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_PWMServoDriver pwm(PCA9685_ADDRESS);

// ==================== 运行时状态 ====================
float current_angles[SERVO_COUNT] = {90, 90, 90, 90, 90, 60};
float target_angles[SERVO_COUNT]  = {90, 90, 90, 90, 90, 60};
const float home_angles[SERVO_COUNT] = {90, 90, 90, 90, 90, 60};

bool entities_created = false;
bool power_disabled   = false;   // true = 关机状态，舵机停止，等待开机指令
bool time_synced      = false;   // set true after rmw_uros_sync_session
static uint32_t flash_until_ms = 0;

// ==================== 状态机 ====================
typedef enum {
  CONNECTING_WIFI,
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} AgentState;

AgentState state = CONNECTING_WIFI;

// ==================== LED ====================
void showColor(uint8_t r, uint8_t g, uint8_t b) {
  strip.setPixelColor(0, strip.Color(r, g, b));
  strip.show();
}

void showState() {
  switch (state) {
    case CONNECTING_WIFI:    showColor(128, 0, 128); break;
    case WAITING_AGENT:      showColor(255, 255, 0); break;
    case AGENT_AVAILABLE:    showColor(0, 0, 255); break;
    case AGENT_CONNECTED:    showColor(0, 255, 0); break;
    case AGENT_DISCONNECTED: showColor(255, 0, 0); break;
    default:                 showColor(0, 0, 0); break;
  }
}

void triggerCmdFlash() {
  flash_until_ms = millis() + CMD_FLASH_MS;
  showColor(0, 255, 255);
}

// ==================== 时间戳 ====================
// 在 createEntities 之后跑一次 rmw_uros_sync_session 拿到 Agent 的 wallclock。
// 同步成功后用 rmw_uros_epoch_nanos；失败就退化用 millis() —— 至少 stamp
// 单调递增，MoveIt 的"start state too old"检查不会立刻挂掉。
void fillStamp(builtin_interfaces__msg__Time * stamp) {
  int64_t ns;
  if (time_synced) {
    ns = rmw_uros_epoch_nanos();
  } else {
    ns = (int64_t)millis() * 1000000LL;
  }
  stamp->sec     = (int32_t)(ns / 1000000000LL);
  stamp->nanosec = (uint32_t)(ns % 1000000000LL);
}

// ==================== I2C 状态上报 ====================
void publishI2CStatus(const char* msg_str) {
  if (!entities_created) return;
  // 直接写入已分配好的字符串缓冲
  i2c_status_msg.data = micro_ros_string_utilities_set(i2c_status_msg.data, msg_str);
  rcl_ret_t ret = rcl_publish(&i2c_status_pub, &i2c_status_msg, NULL);
  if (ret != RCL_RET_OK) {
    Serial.println("警告：i2c_status 发布失败");
  }
}

// ==================== 舵机辅助 ====================
uint16_t angleToPulse(float angle) {
  angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  return (uint16_t)(SERVO_MIN_PULSE +
         (angle / SERVO_MAX_ANGLE) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));
}

bool setPWMWithCheck(uint8_t servo_num, uint16_t pulse) {
  uint8_t reg = 0x06 + 4 * servo_num;
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(reg);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(pulse & 0xFF);
  Wire.write((pulse >> 8) & 0x0F);
  return (Wire.endTransmission() == 0);
}

bool checkPCA9685() {
  Wire.beginTransmission(PCA9685_ADDRESS);
  return (Wire.endTransmission() == 0);
}

void setServoAngle(uint8_t servo_num, float angle) {
  if (servo_num >= SERVO_COUNT) return;
  target_angles[servo_num] = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
}

void smoothDriveServos() {
  static float smoothed_angles[SERVO_COUNT] = {90, 90, 90, 90, 90, 60};
  static float last_driven_angles[SERVO_COUNT] = {-999, -999, -999, -999, -999, -999};

  // 缓存到本地变量;volatile 读多了不便宜,而且这样保证一帧内不变化。
  const float alpha    = servo_smooth_alpha;
  const float max_step = servo_max_step_deg;
  const float deadband = servo_deadband_deg;

  for (int i = 0; i < SERVO_COUNT; i++) {
    // EMA 先算步长，再限幅 → max_step 才是真实单周期上限
    float step = (target_angles[i] - smoothed_angles[i]) * alpha;
    step = constrain(step, -max_step, max_step);
    smoothed_angles[i] += step;

    if (fabsf(smoothed_angles[i] - last_driven_angles[i]) < deadband) continue;

    uint16_t pulse = angleToPulse(smoothed_angles[i]);
    bool ok = setPWMWithCheck(i, pulse);

    if (ok) {
      current_angles[i] = smoothed_angles[i];
      last_driven_angles[i] = smoothed_angles[i];
      Serial.printf("舵机 %d -> %.1f deg (pulse %u) OK\n", i, smoothed_angles[i], pulse);
    } else {
      char errbuf[48];
      snprintf(errbuf, sizeof(errbuf), "ERR servo_%d i2c_write_failed", i + 1);
      publishI2CStatus(errbuf);
      Serial.printf("舵机 %d I2C 失败\n", i);
    }
  }
}

// 关闭所有通道的 PWM 输出（pulse=0 → PCA9685 LED_OFF=0 → 舵机不通电）
void disableAllPWM() {
  for (int i = 0; i < SERVO_COUNT; i++) {
    setPWMWithCheck(i, 0);
  }
}

// 上电软启动：先停 PWM、等电源稳定，再错峰逐通道写入 home 脉宽。
// 单舵机内部闭环依然是硬启动，但 6 路峰值电流被分散到 6 × CHANNEL_GAP_MS
// 的时间窗里，瞬态对底盘的反作用力显著减弱。
void softStartToHome() {
  Serial.println("软启动：错峰启用舵机通道...");
  for (int i = 0; i < SERVO_COUNT; i++) {
    current_angles[i] = home_angles[i];
    target_angles[i]  = home_angles[i];
    setPWMWithCheck(i, angleToPulse(home_angles[i]));
    Serial.printf("  通道 %d 上电 -> %.1f°\n", i, home_angles[i]);
    delay(SOFT_START_CHANNEL_GAP_MS);
  }
  Serial.println("软启动完成");
}

// 软停归位：以 SOFT_RAMP_STEP_DEG/帧 平滑插值到 home，绕过 smoothDriveServos
// 的 EMA，确保归位过程是匀速、缓慢的，不会瞬间提扭。
void softRampToHome() {
  Serial.println("软停止：缓慢归位...");
  bool done = false;
  while (!done) {
    done = true;
    for (int i = 0; i < SERVO_COUNT; i++) {
      float diff = home_angles[i] - current_angles[i];
      if (fabsf(diff) < 0.5f) continue;
      float step = constrain(diff, -SOFT_RAMP_STEP_DEG, SOFT_RAMP_STEP_DEG);
      current_angles[i] += step;
      setPWMWithCheck(i, angleToPulse(current_angles[i]));
      done = false;
    }
    delay(SOFT_RAMP_FRAME_MS);
  }
  // 同步 target_angles，避免 timer_callback 醒来后又把舵机推走
  for (int i = 0; i < SERVO_COUNT; i++) target_angles[i] = home_angles[i];
  Serial.println("软停止：已归位");
}

// 进入一次定时唤醒的浅睡眠。WiFi 在 WIFI_PS_MIN_MODEM 下保持关联，
// AP 在 DTIM 间隔会缓存包；醒来后立刻 spin executor 即可收到开机指令。
void enterLightSleepWindow() {
  esp_sleep_enable_timer_wakeup(LIGHT_SLEEP_WINDOW_US);
  esp_light_sleep_start();
}

// ==================== 消息内存初始化（关键）====================
bool init_message_buffers() {
  // ----- 发布：joint_state_msg -----
  // rules 指定：name 字段每个字符串 16 字节，position/velocity/effort 序列 6 个元素
  static micro_ros_utilities_memory_rule_t pub_rules[] = {
    {"name",     SERVO_COUNT},   // name[] 序列长度 = 6
    {"name.*",   16},            // 每个 joint name 字符串最多 16 字节
    {"position", SERVO_COUNT},
    {"velocity", 0},             // 不使用，分配 0
    {"effort",   0},
    {"header.frame_id", 32},
  };
  static micro_ros_utilities_memory_conf_t pub_conf = {0};
  pub_conf.rules   = pub_rules;
  pub_conf.n_rules = sizeof(pub_rules) / sizeof(pub_rules[0]);

  if (!micro_ros_utilities_create_static_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        &joint_state_msg, pub_conf,
        joint_state_msg_buf, sizeof(joint_state_msg_buf))) {
    Serial.println("错误：joint_state_msg 内存初始化失败");
    return false;
  }

  // 填充关节名
  for (int i = 0; i < SERVO_COUNT; i++) {
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "joint_%d", i + 1);
    joint_state_msg.name.data[i] =
        micro_ros_string_utilities_set(joint_state_msg.name.data[i], tmp);
    joint_state_msg.position.data[i] =
        (double)(current_angles[i] * M_PI / 180.0f);
  }
  joint_state_msg.name.size     = SERVO_COUNT;
  joint_state_msg.position.size = SERVO_COUNT;
  joint_state_msg.velocity.size = 0;
  joint_state_msg.effort.size   = 0;
  // frame_id 一次写好；后续 publish 只刷 stamp
  joint_state_msg.header.frame_id = micro_ros_string_utilities_set(
      joint_state_msg.header.frame_id, "base_link");

  // ----- 订阅：joint_msg -----
  static micro_ros_utilities_memory_rule_t sub_rules[] = {
    {"name",            SERVO_COUNT},
    {"name.*",          16},
    {"position",        SERVO_COUNT},
    {"velocity",        0},
    {"effort",          0},
    {"header.frame_id", 32},
  };
  static micro_ros_utilities_memory_conf_t sub_conf = {0};
  sub_conf.rules   = sub_rules;
  sub_conf.n_rules = sizeof(sub_rules) / sizeof(sub_rules[0]);

  if (!micro_ros_utilities_create_static_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        &joint_msg, sub_conf,
        joint_msg_buf, sizeof(joint_msg_buf))) {
    Serial.println("错误：joint_msg 内存初始化失败");
    return false;
  }
  joint_msg.position.capacity = SERVO_COUNT;
  joint_msg.position.size     = 0;
  joint_msg.name.capacity     = SERVO_COUNT;
  joint_msg.name.size         = 0;

  // ----- 订阅：array_msg -----
  // Float32MultiArray 里 layout.dim 不用，data 只需 6 个 float
  static micro_ros_utilities_memory_rule_t arr_rules[] = {
    {"data",        SERVO_COUNT},
    {"layout.dim",  0},           // 不使用 dim，分配 0
  };
  static micro_ros_utilities_memory_conf_t arr_conf = {0};
  arr_conf.rules   = arr_rules;
  arr_conf.n_rules = sizeof(arr_rules) / sizeof(arr_rules[0]);

  if (!micro_ros_utilities_create_static_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        &array_msg, arr_conf,
        array_msg_buf, sizeof(array_msg_buf))) {
    Serial.println("错误：array_msg 内存初始化失败");
    return false;
  }
  array_msg.data.size = 0;

  // ----- 订阅：tuning_msg(/servo_tuning) -----
  // 期望 data[0..2] = [alpha, max_step_deg, deadband_deg]
  static micro_ros_utilities_memory_rule_t tun_rules[] = {
    {"data",        3},
    {"layout.dim",  0},
  };
  static micro_ros_utilities_memory_conf_t tun_conf = {0};
  tun_conf.rules   = tun_rules;
  tun_conf.n_rules = sizeof(tun_rules) / sizeof(tun_rules[0]);
  if (!micro_ros_utilities_create_static_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        &tuning_msg, tun_conf,
        tuning_msg_buf, sizeof(tuning_msg_buf))) {
    Serial.println("错误：tuning_msg 内存初始化失败");
    return false;
  }
  tuning_msg.data.size = 0;

  // ----- 发布：i2c_status_msg -----
  static micro_ros_utilities_memory_rule_t str_rules[] = {
    {"data", 64},
  };
  static micro_ros_utilities_memory_conf_t str_conf = {0};
  str_conf.rules   = str_rules;
  str_conf.n_rules = sizeof(str_rules) / sizeof(str_rules[0]);

  if (!micro_ros_utilities_create_static_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        &i2c_status_msg, str_conf,
        i2c_status_msg_buf, sizeof(i2c_status_msg_buf))) {
    Serial.println("错误：i2c_status_msg 内存初始化失败");
    return false;
  }

  Serial.println("消息缓冲区初始化完成（micro_ros_utilities）");
  return true;
}

// ==================== 订阅回调 ====================
void joint_state_callback(const void * msgin) {
  const sensor_msgs__msg__JointState * msg =
      (const sensor_msgs__msg__JointState *)msgin;

  Serial.printf("[回调] JointState, position.size = %zu\n", msg->position.size);

  if (msg->position.data == NULL || msg->position.size < (size_t)SERVO_COUNT) {
    Serial.println("警告：position 数据不足，忽略本帧");
    publishI2CStatus("WARN joint_state data insufficient");
    return;
  }

  for (int i = 0; i < SERVO_COUNT; i++) {
    setServoAngle(i, (float)(msg->position.data[i] * 180.0 / M_PI));
  }
  triggerCmdFlash();
}

void array_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg =
      (const std_msgs__msg__Float32MultiArray *)msgin;

  Serial.printf("[回调] Float32MultiArray, data.size = %zu\n", msg->data.size);

  if (msg->data.data == NULL || msg->data.size < (size_t)SERVO_COUNT) {
    Serial.println("警告：data 数据不足，忽略本帧");
    publishI2CStatus("WARN array data insufficient");
    return;
  }

  for (int i = 0; i < SERVO_COUNT; i++) {
    setServoAngle(i, msg->data.data[i]);
  }
  triggerCmdFlash();
}

// /servo_tuning 订阅 —— 运行时改 EMA / 步长 / 死区,不用重烧固件。
// 期望: data[0]=alpha, data[1]=max_step_deg, data[2]=deadband_deg
// 任何元素 NaN / 越界都跳过,只更新合法值。
void tuning_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg =
      (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.data == NULL || msg->data.size < 3) {
    publishI2CStatus("WARN tuning data insufficient");
    return;
  }
  float a  = msg->data.data[0];
  float ms = msg->data.data[1];
  float db = msg->data.data[2];

  bool changed = false;
  if (a > 0.0f && a <= 1.0f && !isnan(a)) {
    servo_smooth_alpha = a; changed = true;
  }
  if (ms > 0.0f && ms <= 30.0f && !isnan(ms)) {   // 30°/cycle 已是激进上限
    servo_max_step_deg = ms; changed = true;
  }
  if (db >= 0.0f && db <= 10.0f && !isnan(db)) {
    servo_deadband_deg = db; changed = true;
  }

  if (changed) {
    char buf[80];
    snprintf(buf, sizeof(buf),
             "INFO tuning alpha=%.3f step=%.2f dead=%.2f",
             servo_smooth_alpha, servo_max_step_deg, servo_deadband_deg);
    publishI2CStatus(buf);
    Serial.println(buf);
  } else {
    publishI2CStatus("WARN tuning all values out of range");
  }
}

void power_control_callback(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  if (msg->data) {
    // true = 开机：唤醒 PCA9685 → 软启动到 home
    if (power_disabled) {
      power_disabled = false;
      pwm.begin();
      pwm.setPWMFreq(SERVO_FREQ);
      disableAllPWM();
      delay(SOFT_START_SETTLE_MS);
      softStartToHome();
      publishI2CStatus("INFO power_on");
      showState();
      Serial.println("[电源] 开机：已软启动到 home");
    }
    return;
  }
  // false = 关机：软停止归位 → 错峰断 PWM → PCA9685 进入 sleep → 主控转浅睡眠
  if (!power_disabled) {
    Serial.println("[电源] 关机：软停止中...");
    publishI2CStatus("WARN power_off_pending");
    showColor(255, 0, 0);   // 红色 = 关机状态
    softRampToHome();
    // 错峰断 PWM：每路间隔关闭，减小六路同时失扭对底盘的冲击
    for (int i = 0; i < SERVO_COUNT; i++) {
      setPWMWithCheck(i, 0);
      delay(SOFT_STOP_CHANNEL_GAP_MS);
    }
    pwm.sleep();
    power_disabled = true;
    publishI2CStatus("INFO power_off_lightsleep");
    Serial.println("[电源] 已关机，主控进入浅睡眠等待开机指令");
  }
}

void arm_reboot_callback(const void * msgin) {
  (void)msgin;
  // 收到重启指令：软停止归位后硬件重启
  Serial.println("[电源] 收到 REBOOT 指令，软停止后重启...");
  publishI2CStatus("WARN hard_reboot");
  showColor(255, 128, 0);   // 橙色 = 重启中
  softRampToHome();
  delay(300);
  Serial.println("[电源] 重启 ESP32");
  ESP.restart();
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) return;
  if (power_disabled) return;   // 关机状态：跳过舵机驱动和发布

  smoothDriveServos();

  for (int i = 0; i < SERVO_COUNT; i++) {
    joint_state_msg.position.data[i] =
        (double)(current_angles[i] * M_PI / 180.0f);
  }
  // 真实 wallclock 时间（同步成功）或本地 millis（同步失败）。MoveIt 的
  // "start state too old" 检查会因 stamp=0 拒绝规划，必须填。
  // NOTE: 没编码器，position 是"最后一次驱动 PCA9685 的命令角"，不是测量。
  fillStamp(&joint_state_msg.header.stamp);

  if (rcl_publish(&joint_pub, &joint_state_msg, NULL) != RCL_RET_OK) {
    Serial.println("警告：joint_states 发布失败");
  }
}

// ==================== WiFi ====================
bool connectWiFi() {
  Serial.printf("正在连接 WiFi: %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(200);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  for (int i = 0; i < 30; i++) {
    wl_status_t s = WiFi.status();
    Serial.printf("[%2d] status=%d\n", i, (int)s);
    if (s == WL_CONNECTED) {
      Serial.printf("WiFi 已连接, IP: %s\n", WiFi.localIP().toString().c_str());
      return true;
    }
    showColor(128, 0, 128); delay(150);
    showColor(0, 0, 0);     delay(150);
  }
  Serial.printf("WiFi 连接失败, 最终状态=%d\n", (int)WiFi.status());
  return false;
}

// ==================== 实体管理 ====================
bool createEntities() {
  if (entities_created) return true;

  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    Serial.println("错误：rclc_support_init 失败"); return false;
  }
  if (rclc_node_init_default(&node, "servo_controller_node", "", &support) != RCL_RET_OK) {
    Serial.println("错误：node_init 失败"); return false;
  }
  if (rclc_subscription_init_default(
        &joint_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_commands") != RCL_RET_OK) {
    Serial.println("错误：joint_sub 初始化失败"); return false;
  }
  if (rclc_subscription_init_default(
        &array_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "servo_angles") != RCL_RET_OK) {
    Serial.println("错误：array_sub 初始化失败"); return false;
  }
  if (rclc_subscription_init_default(
        &power_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "arm_power_control") != RCL_RET_OK) {
    Serial.println("错误：power_sub 初始化失败"); return false;
  }
  if (rclc_subscription_init_default(
        &reboot_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
        "arm_reboot") != RCL_RET_OK) {
    Serial.println("错误：reboot_sub 初始化失败"); return false;
  }
  if (rclc_subscription_init_default(
        &tuning_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "servo_tuning") != RCL_RET_OK) {
    Serial.println("错误：tuning_sub 初始化失败"); return false;
  }
  if (rclc_publisher_init_default(
        &joint_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states") != RCL_RET_OK) {
    Serial.println("错误：joint_pub 初始化失败"); return false;
  }
  if (rclc_publisher_init_default(
        &i2c_status_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "servo_i2c_status") != RCL_RET_OK) {
    Serial.println("错误：i2c_status_pub 初始化失败"); return false;
  }
  if (rclc_timer_init_default(
        &timer, &support,
        RCL_MS_TO_NS(100), timer_callback) != RCL_RET_OK) {
    Serial.println("错误：timer 初始化失败"); return false;
  }
  // executor handles: joint + array + power + reboot + tuning + timer = 6
  if (rclc_executor_init(&executor, &support.context, 6, &allocator) != RCL_RET_OK) {
    Serial.println("错误：executor 初始化失败"); return false;
  }
  if (rclc_executor_add_subscription(
        &executor, &joint_sub, &joint_msg,
        &joint_state_callback, ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("错误：添加 joint_sub 到 executor 失败"); return false;
  }
  if (rclc_executor_add_subscription(
        &executor, &array_sub, &array_msg,
        &array_callback, ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("错误：添加 array_sub 到 executor 失败"); return false;
  }
  if (rclc_executor_add_subscription(
        &executor, &power_sub, &power_msg,
        &power_control_callback, ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("错误：添加 power_sub 到 executor 失败"); return false;
  }
  if (rclc_executor_add_subscription(
        &executor, &reboot_sub, &reboot_msg,
        &arm_reboot_callback, ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("错误：添加 reboot_sub 到 executor 失败"); return false;
  }
  if (rclc_executor_add_subscription(
        &executor, &tuning_sub, &tuning_msg,
        &tuning_callback, ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("错误：添加 tuning_sub 到 executor 失败"); return false;
  }
  if (rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK) {
    Serial.println("错误：添加 timer 到 executor 失败"); return false;
  }

  entities_created = true;
  Serial.println("micro-ROS 实体创建成功");

  // 跟 Agent 对时 —— /joint_states 的 header.stamp 才有意义。
  // 失败也不致命，fillStamp() 会退化用本地 millis()。
  if (rmw_uros_sync_session(1000) == RMW_RET_OK) {
    time_synced = true;
    Serial.println("时钟与 Agent 同步成功");
  } else {
    time_synced = false;
    Serial.println("警告：时钟同步失败，使用本地 millis()");
  }

  char info_buf[64];
  snprintf(info_buf, sizeof(info_buf),
           "INFO micro_ros connected clk:%s", time_synced ? "OK" : "LOCAL");
  publishI2CStatus(info_buf);
  return true;
}

void destroyEntities() {
  if (!entities_created) return;

  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  (void)rcl_subscription_fini(&joint_sub,   &node);
  (void)rcl_subscription_fini(&array_sub,   &node);
  (void)rcl_subscription_fini(&power_sub,   &node);
  (void)rcl_subscription_fini(&reboot_sub,  &node);
  (void)rcl_subscription_fini(&tuning_sub,  &node);
  (void)rcl_publisher_fini(&joint_pub,      &node);
  (void)rcl_publisher_fini(&i2c_status_pub, &node);
  (void)rcl_timer_fini(&timer);
  (void)rclc_executor_fini(&executor);
  (void)rcl_node_fini(&node);
  (void)rclc_support_fini(&support);

  entities_created = false;
  time_synced = false;          // 重连时重新对时
  delay(500);
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
  Serial.println("micro-ROS 实体已销毁，传输层已重置");
}

// ==================== setup / loop ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n====== 6轴舵机控制器启动 ======");

  strip.begin();
  showColor(0, 0, 0);

  Wire.begin(I2C_SDA, I2C_SCL);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  // 先把所有通道置为不输出 PWM，避免 begin() 完成瞬间六轴同时被指令到 home
  disableAllPWM();
  delay(100);

  if (checkPCA9685()) {
    Serial.println("PCA9685 在线");
  } else {
    Serial.println("警告：PCA9685 未响应，请检查接线");
  }

  // 等待主电源/电容稳定，再错峰启用各舵机
  delay(SOFT_START_SETTLE_MS);
  softStartToHome();

  if (!init_message_buffers()) {
    Serial.println("致命错误：消息内存初始化失败，停机");
    while (true) { showColor(255, 0, 0); delay(500); showColor(0, 0, 0); delay(500); }
  }

  state = CONNECTING_WIFI;
  showState();

  if (connectWiFi()) {
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
    // 启用 WiFi modem sleep：DTIM 间隙关闭射频，配合关机后的浅睡眠节流
    WiFi.setSleep(WIFI_PS_MIN_MODEM);
    state = WAITING_AGENT;
    showState();
    Serial.printf("等待 micro-ROS Agent (%s:%d)...\n", AGENT_IP, AGENT_PORT);
  } else {
    Serial.println("WiFi 初始连接失败，将在 loop() 中重试");
  }
}

void loop() {
  if (flash_until_ms > 0 && millis() >= flash_until_ms) {
    flash_until_ms = 0;
    showState();
  }

  if (WiFi.status() != WL_CONNECTED) {
    if (entities_created) {
      Serial.println("WiFi 断开，销毁实体");
      destroyEntities();
    }
    state = CONNECTING_WIFI;
    showState();
    if (connectWiFi()) {
      set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
      state = WAITING_AGENT;
      showState();
      Serial.printf("等待 micro-ROS Agent (%s:%d)...\n", AGENT_IP, AGENT_PORT);
    }
    delay(100);
    return;
  }

  switch (state) {
    case CONNECTING_WIFI:
      state = WAITING_AGENT;
      showState();
      break;

    case WAITING_AGENT:
      if (RMW_RET_OK == rmw_uros_ping_agent(500, 2)) {
        Serial.println("Agent 可达，开始创建实体...");
        state = AGENT_AVAILABLE;
        showState();
      }
      delay(300);
      break;

    case AGENT_AVAILABLE:
      if (createEntities()) {
        state = AGENT_CONNECTED;
        showState();
        Serial.println("已连接，开始运行");
      } else {
        Serial.println("实体创建失败，回退等待 Agent");
        destroyEntities();
        state = WAITING_AGENT;
        showState();
        delay(2000);
      }
      break;

    case AGENT_CONNECTED:
      if (RMW_RET_OK == rmw_uros_ping_agent(500, 2)) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
        if (power_disabled) {
          // 关机状态：进入定时唤醒的浅睡眠，醒来后再 spin executor
          // 收开机指令。WiFi modem sleep 保持 AP 关联，UDP 包在 DTIM
          // 间隙由 AP 缓存。
          enterLightSleepWindow();
        }
      } else {
        Serial.println("Agent 失联，进入断线状态");
        state = AGENT_DISCONNECTED;
        showState();
      }
      break;

    case AGENT_DISCONNECTED:
      destroyEntities();
      state = WAITING_AGENT;
      showState();
      Serial.println("实体已清理，重新等待 Agent...");
      delay(500);
      break;

    default:
      state = WAITING_AGENT;
      showState();
      break;
  }

  delay(10);
}
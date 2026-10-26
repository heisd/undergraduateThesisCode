# ROS 2 × ESP32 机械臂轨迹桥接

本科毕业设计相关代码，将 ROS 2 的 `FollowJointTrajectory` 动作转换为舵机角度数组，经 micro-ROS 发送到 ESP32，再由 PCA9685 产生 PWM。仓库包含主机桥接节点、参数、启动文件和下位机固件，配套实验记录位于 [doc/](./doc)。

## 数据链路与单位

```text
轨迹客户端 / MoveIt
    │ FollowJointTrajectory：关节位置为弧度
    ▼
trajectory_bridge（插值、关节映射、固定频率发布）
    │ /servo_angles：Float32MultiArray，单位为度
    ▼
micro-ROS Agent ── Wi-Fi ── ESP32 ── I²C ── PCA9685 ── 舵机
```

弧度与角度转换由桥接节点完成。不要将 ROS 轨迹中的弧度值直接当作 `/servo_angles` 的度数发送。

## 目录与关键实现

| 路径 | 内容 |
| --- | --- |
| [src/trajectory_bridge/src/trajectory_bridge_node.cpp](./src/trajectory_bridge/src/trajectory_bridge_node.cpp) | Action 服务、轨迹处理、发布与诊断日志 |
| [src/trajectory_bridge/config/trajectory_bridge.yaml](./src/trajectory_bridge/config/trajectory_bridge.yaml) | 发布频率、关节顺序与 home 配置 |
| [src/trajectory_bridge/launch/trajectory_bridge.launch.py](./src/trajectory_bridge/launch/trajectory_bridge.launch.py) | 加载参数并启动节点 |
| [src/trajectory_bridge/package.xml](./src/trajectory_bridge/package.xml) | ROS 包依赖声明 |
| [firmware/esp32.ino](./firmware/esp32.ino) | ESP32 固件 |
| [doc/](./doc) | micro-ROS、Action Bridge 与舵机调试记录 |

主机实现使用 C++17、ament_cmake、rclcpp、rclcpp_action、std_msgs 和 control_msgs；源码还使用轨迹消息类型。不同 ROS 发行版应按实际依赖解析结果检查编译，仓库没有给出全部发行版的验证矩阵。

## 主机侧构建

在已安装 ROS 2、colcon，并加载相应 ROS 环境的终端中执行：

```bash
git clone --branch main https://github.com/heisd/undergraduateThesisCode.git
cd undergraduateThesisCode
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select trajectory_bridge
source install/setup.bash
ros2 launch trajectory_bridge trajectory_bridge.launch.py
```

这里假设 rosdep 已初始化。启动文件加载包内 YAML，并允许覆盖发布频率：

```bash
ros2 launch trajectory_bridge trajectory_bridge.launch.py publish_rate:=50.0
```

## 参数约定

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `publish_rate` | 50.0 Hz | 舵机角度发布频率 |
| `joint_names` | joint_1 … joint_6 | 输出数组的固定索引顺序 |
| `home_positions_rad` | 前五关节 π/2，第六关节 π/3 | 节点启动时的关节状态 |

home 对应前五关节 90°、夹爪 60°，与固件 `home_angles[]` 一致。两组数组长度应一致；代码遇到长度不一致会补零或截断并警告，因此应主动修正配置，避免依赖自动补齐。

关节顺序必须与 PCA9685 通道 0–5 对应。更换机械臂时，应同时核对关节名、角度方向、零点与 PWM 范围。

## ROS 接口

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/arm_controller/follow_joint_trajectory` | control_msgs/action/FollowJointTrajectory | 机械臂轨迹 |
| `/hand_controller/follow_joint_trajectory` | 同上 | 夹爪轨迹 |
| `/servo_angles` | std_msgs/msg/Float32MultiArray | 固定顺序的舵机角度数组 |

上表是无额外命名空间时的名称。机械臂与夹爪维护独立的执行标志；同一个 controller 在旧目标执行时会拒绝新的重叠目标。

节点根据轨迹点做插值，并输出时间间隔、位置跳变等诊断。Action 的 `actual.positions` 当前由指令位置填入，**不是来自真实关节传感器的闭环跟踪结果**；成功返回表示指令执行流程结束，不能据此确认实体机械臂到达目标。

## ESP32 固件准备

固件导入 micro_ros_arduino、Adafruit NeoPixel、Adafruit PWM Servo Driver 等库，并使用 Wi-Fi 连接 Agent。源码默认：

| 配置 | 值 |
| --- | --- |
| LED | GPIO48 |
| I²C SDA / SCL | GPIO8 / GPIO9 |
| PCA9685 地址 | 0x40 |
| 舵机数量 / PWM 频率 | 6 / 50 Hz |
| 名义角度范围 | 0–180° |
| 脉宽计数范围 | 150–600 |

这些是当前代码常量，实际舵机的机械限位与可用脉宽需单独标定。

源码通过 `secrets.h` 读取连接配置。当前仓库缺少注释所指的 `secrets.h.template`，需要在 `firmware/` 内自行准备：

```cpp
#pragma once
#define ARM_WIFI_SSID "YOUR_WIFI_SSID"
#define ARM_WIFI_PASS "YOUR_WIFI_PASSWORD"
#define ARM_AGENT_IP "YOUR_AGENT_IP"
#define ARM_AGENT_PORT 8888
```

以上内容是按源码所需宏整理的配置示例，不是仓库已有模板。Agent 地址、端口和安装的 micro-ROS 库版本需要互相匹配。

固件还包含 `/joint_commands`、`/joint_states`、`/servo_tuning`、`/arm_power_control`、`/arm_reboot` 和 `/servo_i2c_status` 等接口，可直接阅读固件定义。发布的关节状态不能自动视为独立位置传感器测量。

## 分层检查与排障

```bash
ros2 node list
ros2 action list -t
ros2 topic info /servo_angles -v
ros2 topic echo /servo_angles
ros2 topic hz /servo_angles
```

1. **主机无节点**：检查 ROS 环境、构建和 `source install/setup.bash`。
2. **有动作接口但无硬件响应**：先确认角度数组发布，再检查 Agent、Wi-Fi 和固件连接。
3. **消息正常但舵机不动**：检查 PCA9685、I²C、舵机电源及通道映射。
4. **收到重叠目标被拒绝**：等待当前 controller 完成，或检查上游是否重复发起目标。
5. **启动角度异常**：对齐 YAML 的 home、关节顺序与固件 home。
6. **轨迹抖动**：结合日志检查轨迹点时间与位置跳变，再核对固件的平滑、死区和步长参数。

应先验证消息链路与参数，再进行小范围实体动作测试。该仓库没有包含一套完整 MoveIt 机器人配置，桥接节点也不替代上层规划器。

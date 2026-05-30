#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <atomic>
#include <vector>
#include <cmath>
#include <string>
#include <map>
#include <mutex>
#include <thread>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <numeric>

// 默认关节顺序 + home 位置 —— 与 arm 固件 `home_angles[]` 一致(joints 1-5
// = 90°、joint_6 = 60°)。可以通过 `joint_names` 和 `home_positions_rad`
// 参数覆盖。
static const std::vector<std::string> DEFAULT_JOINT_ORDER = {
  "joint_1", "joint_2", "joint_3",
  "joint_4", "joint_5", "joint_6"
};
static const std::vector<double> DEFAULT_HOME_RAD = {
  M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2,   // 90°
  60.0 * M_PI / 180.0,                       // 60° 夹爪
};

class TrajectoryBridge : public rclcpp::Node
{
  using FJT           = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FJT>;

public:
  TrajectoryBridge() : Node("trajectory_bridge")
  {
    declare_parameter("publish_rate", 50.0);
    declare_parameter<std::vector<std::string>>("joint_names", DEFAULT_JOINT_ORDER);
    declare_parameter<std::vector<double>>("home_positions_rad", DEFAULT_HOME_RAD);

    publish_rate_ = get_parameter("publish_rate").as_double();
    joint_order_  = get_parameter("joint_names").as_string_array();
    auto home_rad = get_parameter("home_positions_rad").as_double_array();

    if (joint_order_.empty()) {
      RCLCPP_ERROR(get_logger(),
        "joint_names 参数为空 —— 退回到 DEFAULT_JOINT_ORDER");
      joint_order_ = DEFAULT_JOINT_ORDER;
    }
    if (home_rad.size() != joint_order_.size()) {
      RCLCPP_WARN(get_logger(),
        "home_positions_rad 长度(%zu) != joint_names 长度(%zu)。"
        "短了补 0,长了截断 —— 强烈建议在 yaml 里对齐。",
        home_rad.size(), joint_order_.size());
      home_rad.resize(joint_order_.size(), 0.0);
    }

    // 初始化关节角度为 home(与 arm 固件 home_angles[] 一致)。
    // 不要用 0 —— 第一个 hand-only goal 会让 arm 5 关节冲到 0°,详见
    // README 的 "home 初始化" 部分。
    for (size_t i = 0; i < joint_order_.size(); ++i) {
      joint_map_[joint_order_[i]] = home_rad[i];
    }

    // 使用 RELIABLE QoS 以匹配 micro-ROS (ESP32) 的默认订阅 QoS
    // rclc_subscription_init_default = RELIABLE（不是 BEST_EFFORT）
    // RELIABLE publisher + RELIABLE subscriber = 兼容 ✅
    pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/servo_angles", 10);

    // 两个 action server 共享一个 handle_goal / handle_accepted。controller_name_
    // 在 lambda 里捕获，让回调能区分谁在调（用来 reject 同名重入 goal）。
    arm_server_ = rclcpp_action::create_server<FJT>(
      this,
      "arm_controller/follow_joint_trajectory",
      [this](const rclcpp_action::GoalUUID & u, std::shared_ptr<const FJT::Goal> g) {
        return handle_goal("arm", u, g);
      },
      std::bind(&TrajectoryBridge::handle_cancel,   this, std::placeholders::_1),
      [this](std::shared_ptr<GoalHandleFJT> gh) { handle_accepted("arm", gh); });

    hand_server_ = rclcpp_action::create_server<FJT>(
      this,
      "hand_controller/follow_joint_trajectory",
      [this](const rclcpp_action::GoalUUID & u, std::shared_ptr<const FJT::Goal> g) {
        return handle_goal("hand", u, g);
      },
      std::bind(&TrajectoryBridge::handle_cancel,   this, std::placeholders::_1),
      [this](std::shared_ptr<GoalHandleFJT> gh) { handle_accepted("hand", gh); });

    {
      std::ostringstream os;
      for (size_t i = 0; i < joint_order_.size(); ++i) {
        if (i) os << ",";
        os << joint_order_[i];
      }
      RCLCPP_INFO(get_logger(),
        "TrajectoryBridge Action Server 已启动 — 关节(%zu): [%s], 发布频率: %.1f Hz",
        joint_order_.size(), os.str().c_str(), publish_rate_);
    }
  }

private:
  // ── 接受目标 —— 同一 controller 不允许新旧轨迹重叠 ────────────
  // 旧实现总是 ACCEPT_AND_EXECUTE 然后 detach 新线程，导致两个 execute() 同时
  // 写 joint_map_ 互相覆盖，机械臂在两条插值轨迹之间反复横跳。
  rclcpp_action::GoalResponse handle_goal(
    const std::string & controller,
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const FJT::Goal> goal)
  {
    auto & in_flight = (controller == "arm") ? arm_in_flight_ : hand_in_flight_;
    if (in_flight.load()) {
      RCLCPP_WARN(get_logger(),
        "%s_controller 已有轨迹在执行中（%zu 点），拒绝新 goal",
        controller.c_str(), goal->trajectory.points.size());
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(get_logger(), "[%s] 收到轨迹目标，共 %zu 个点",
      controller.c_str(), goal->trajectory.points.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // ── 接受取消请求 ────────────────────────────────────────────────
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFJT> /*goal_handle*/)
  {
    RCLCPP_INFO(get_logger(), "收到取消请求");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // ── 在新线程执行轨迹 ────────────────────────────────────────────
  void handle_accepted(const std::string & controller,
                       const std::shared_ptr<GoalHandleFJT> goal_handle)
  {
    auto & in_flight = (controller == "arm") ? arm_in_flight_ : hand_in_flight_;
    in_flight.store(true);
    std::thread([this, controller, goal_handle]() {
      execute(goal_handle);
      auto & flag = (controller == "arm") ? arm_in_flight_ : hand_in_flight_;
      flag.store(false);
    }).detach();
  }

  // ── 轨迹执行主循环 ──────────────────────────────────────────────
  void execute(const std::shared_ptr<GoalHandleFJT> goal_handle)
  {
    const auto   goal       = goal_handle->get_goal();
    auto         feedback   = std::make_shared<FJT::Feedback>();
    auto         result     = std::make_shared<FJT::Result>();
    const auto & traj       = goal->trajectory;
    const auto   start_time = this->now();

    const size_t N = traj.points.size();
    RCLCPP_INFO(get_logger(), "开始执行轨迹，共 %zu 个点，插值频率 %.0f Hz",
      N, publish_rate_);
    log_trajectory_quality(traj);

    if (N == 0) {
      result->error_code = FJT::Result::SUCCESSFUL;
      goal_handle->succeed(result);
      return;
    }

    // ── 工具 lambda ────────────────────────────────────────────────
    // 轨迹点时间戳 → 秒（double）
    auto pt_time = [](const trajectory_msgs::msg::JointTrajectoryPoint & pt) -> double {
      return pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;
    };
    // double 秒 → rclcpp::Time（相对 start_time）
    auto make_target = [&](double t) -> rclcpp::Time {
      int32_t  s  = static_cast<int32_t>(t);
      uint32_t ns = static_cast<uint32_t>((t - s) * 1e9);
      return start_time + rclcpp::Duration(s, ns);
    };
    // 忙等到目标时刻，每 2 ms 检查一次取消
    auto wait_until = [&](rclcpp::Time target) {
      while (this->now() < target) {
        if (goal_handle->is_canceling()) break;
        rclcpp::sleep_for(std::chrono::milliseconds(2));
      }
    };
    // 将插值位置写入 joint_map_ 并发布
    auto publish_interp = [&](
      const trajectory_msgs::msg::JointTrajectoryPoint & p0,
      const trajectory_msgs::msg::JointTrajectoryPoint & p1,
      double alpha)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (size_t j = 0; j < traj.joint_names.size(); ++j) {
        double pos0 = (j < p0.positions.size()) ? p0.positions[j] : 0.0;
        double pos1 = (j < p1.positions.size()) ? p1.positions[j] : 0.0;
        joint_map_[traj.joint_names[j]] = pos0 + alpha * (pos1 - pos0);
      }
    };

    size_t total_publishes = 0;

    // ── 段间线性插值：遍历相邻点对 [i, i+1] ───────────────────────
    for (size_t i = 0; i + 1 < N; ++i) {

      if (goal_handle->is_canceling()) {
        result->error_code = FJT::Result::SUCCESSFUL;
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "轨迹已取消");
        return;
      }

      const auto & p0 = traj.points[i];
      const auto & p1 = traj.points[i + 1];
      double t0 = pt_time(p0);
      double t1 = pt_time(p1);
      double dt = t1 - t0;

      // 本段细分步数（至少 1 步）
      int n_steps = std::max(1, static_cast<int>(std::round(dt * publish_rate_)));

      for (int k = 0; k < n_steps; ++k) {

        if (goal_handle->is_canceling()) {
          result->error_code = FJT::Result::SUCCESSFUL;
          goal_handle->canceled(result);
          RCLCPP_INFO(get_logger(), "轨迹已取消");
          return;
        }

        double alpha = static_cast<double>(k) / n_steps;
        double t_sub = t0 + alpha * dt;

        wait_until(make_target(t_sub));
        publish_interp(p0, p1, alpha);
        publish_servos();
        ++total_publishes;

        // 仅在段起始点（k==0）向 MoveIt 发送反馈
        // **开环说明**：PCA9685 + 普通舵机没有编码器，这里的 "actual" 复用
        // commanded 位置，error 直接留空（不要伪造 0 —— 那会让 MoveIt 以为
        // 轨迹跟踪完美，再也察觉不到撞物 / 卡死）。详见包 README "Open-loop"
        // 段落。
        if (k == 0) {
          feedback->joint_names       = traj.joint_names;
          feedback->desired.positions = p0.positions;
          feedback->actual.positions  = p0.positions;
          feedback->error.positions.clear();   // empty == "unknown"
          goal_handle->publish_feedback(feedback);
          RCLCPP_DEBUG(get_logger(), "段 %zu/%zu，插值 %d 步",
            i + 1, N - 1, n_steps);
        }
      }
    }

    // ── 发布最后一个规划点（精确到位）────────────────────────────
    {
      const auto & p_last = traj.points[N - 1];
      wait_until(make_target(pt_time(p_last)));
      {
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t j = 0; j < traj.joint_names.size(); ++j) {
          if (j < p_last.positions.size())
            joint_map_[traj.joint_names[j]] = p_last.positions[j];
        }
      }
      publish_servos();
      ++total_publishes;

      feedback->joint_names       = traj.joint_names;
      feedback->desired.positions = p_last.positions;
      feedback->actual.positions  = p_last.positions;   // 开环，见上方注释
      feedback->error.positions.clear();
      goal_handle->publish_feedback(feedback);
    }

    // 执行完毕 → 返回成功
    result->error_code = FJT::Result::SUCCESSFUL;
    goal_handle->succeed(result);
    // 防退化轨迹除零：所有点都在 t=0 时 pt_time(last) = 0。
    const double total_t = pt_time(traj.points[N - 1]);
    if (total_t > 1e-6) {
      RCLCPP_INFO(get_logger(),
        "轨迹执行完成（规划点 %zu 个，插值发布 %zu 次，实际频率 ≈ %.0f Hz）",
        N, total_publishes, total_publishes / total_t);
    } else {
      RCLCPP_INFO(get_logger(),
        "轨迹执行完成（规划点 %zu 个，插值发布 %zu 次，时长 ≈ 0s）",
        N, total_publishes);
    }
  }

  // ── 轨迹质量打印：position 序列 + 时间间隔统计 ─────────────────
  void log_trajectory_quality(const trajectory_msgs::msg::JointTrajectory & traj)
  {
    const auto & pts = traj.points;
    const size_t N   = pts.size();
    const size_t J   = traj.joint_names.size();

    if (N == 0) {
      RCLCPP_WARN(get_logger(), "[轨迹质量] 轨迹为空！");
      return;
    }

    // ── 表头 ───────────────────────────────────────────────────────
    {
      std::ostringstream hdr;
      hdr << std::left << std::setw(5) << "idx"
          << std::setw(10) << "time_s";
      for (const auto & name : traj.joint_names)
        hdr << std::setw(11) << name;
      hdr << "  dt_ms    Δmax_deg";
      RCLCPP_INFO(get_logger(), "━━━━ 轨迹质量检查 (%zu pt, %zu joints) ━━━━", N, J);
      RCLCPP_INFO(get_logger(), "%s", hdr.str().c_str());
    }

    // ── 逐点打印 ───────────────────────────────────────────────────
    std::vector<double> dt_list;
    dt_list.reserve(N > 0 ? N - 1 : 0);

    double prev_t = 0.0;
    std::vector<double> prev_deg(J, 0.0);

    for (size_t i = 0; i < N; ++i) {
      const auto & pt = pts[i];
      double t = pt.time_from_start.sec
               + pt.time_from_start.nanosec * 1e-9;

      std::ostringstream row;
      row << std::fixed;
      row << std::left  << std::setw(5)  << i;
      row << std::right << std::setw(8)  << std::setprecision(3) << t << "s  ";

      double max_dpos = 0.0;
      for (size_t j = 0; j < J; ++j) {
        double deg = (j < pt.positions.size())
                   ? pt.positions[j] * 180.0 / M_PI
                   : 0.0;
        row << std::setw(10) << std::setprecision(3) << deg << "° ";
        if (i > 0)
          max_dpos = std::max(max_dpos, std::abs(deg - prev_deg[j]));
        prev_deg[j] = deg;
      }

      if (i > 0) {
        double dt_ms = (t - prev_t) * 1000.0;
        dt_list.push_back(t - prev_t);
        row << "  " << std::setw(7)  << std::setprecision(1) << dt_ms << "ms";
        row << "  " << std::setw(7)  << std::setprecision(2) << max_dpos << "°";
      } else {
        row << "  (起始点)";
      }

      RCLCPP_INFO(get_logger(), "%s", row.str().c_str());
      prev_t = t;
    }

    // ── 时间间隔统计 ───────────────────────────────────────────────
    if (!dt_list.empty()) {
      double sum  = std::accumulate(dt_list.begin(), dt_list.end(), 0.0);
      double mean = sum / dt_list.size();
      double mn   = *std::min_element(dt_list.begin(), dt_list.end());
      double mx   = *std::max_element(dt_list.begin(), dt_list.end());
      double var  = 0.0;
      for (double d : dt_list) var += (d - mean) * (d - mean);
      double std_ms = std::sqrt(var / dt_list.size()) * 1000.0;
      bool uniform  = std_ms < 5.0;   // 抖动 < 5ms 视为均匀

      RCLCPP_INFO(get_logger(),
        "━━ dt统计: mean=%.1fms  min=%.1fms  max=%.1fms  std=%.2fms  %s",
        mean * 1000.0, mn * 1000.0, mx * 1000.0, std_ms,
        uniform ? "✓ 时间均匀" : "✗ 时间不均匀（建议检查规划器插值设置）");

      // 检测位置跳变：相邻点最大角度差 > 10° 发出警告
      bool smooth = true;
      std::vector<double> max_step(J, 0.0);
      for (size_t i = 1; i < N; ++i) {
        for (size_t j = 0; j < J; ++j) {
          if (j < pts[i].positions.size() && j < pts[i-1].positions.size()) {
            double d = std::abs(pts[i].positions[j] - pts[i-1].positions[j])
                       * 180.0 / M_PI;
            max_step[j] = std::max(max_step[j], d);
            if (d > 10.0) smooth = false;
          }
        }
      }
      if (smooth) {
        RCLCPP_INFO(get_logger(), "━━ 位置平滑度: ✓ 无大跳变（各关节相邻点差 < 10°）");
      } else {
        std::ostringstream ws;
        ws << std::fixed << std::setprecision(2);
        ws << "━━ 位置平滑度: ✗ 检测到跳变！各关节最大步长: ";
        for (size_t j = 0; j < J; ++j)
          ws << traj.joint_names[j] << "=" << max_step[j] << "°  ";
        RCLCPP_WARN(get_logger(), "%s", ws.str().c_str());
      }
    }
    RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  }

  // ── 将 joint_map_ 按固定顺序发布为度数 ─────────────────────────
  void publish_servos()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std_msgs::msg::Float32MultiArray out;
    out.data.reserve(joint_order_.size());

    for (const auto & name : joint_order_) {
      auto it = joint_map_.find(name);
      float deg = (it != joint_map_.end())
        ? static_cast<float>(it->second * 180.0 / M_PI)
        : 0.0f;
      out.data.push_back(deg);
    }

    pub_->publish(out);
  }

  // ── 成员变量 ────────────────────────────────────────────────────
  rclcpp_action::Server<FJT>::SharedPtr arm_server_;
  rclcpp_action::Server<FJT>::SharedPtr hand_server_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;

  std::map<std::string, double> joint_map_;
  std::vector<std::string> joint_order_;   // 实际驱动的 publish 顺序(参数化)
  std::mutex mutex_;
  // In-flight 标志：同一 controller 的新 goal 在旧 goal 未完时被拒绝。
  // arm 和 hand 各自独立，互不阻塞。
  std::atomic<bool> arm_in_flight_  {false};
  std::atomic<bool> hand_in_flight_ {false};
  double publish_rate_{50.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryBridge>());
  rclcpp::shutdown();
  return 0;
}
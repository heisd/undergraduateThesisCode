# undergraduateThesisCode

本科毕业设计代码：ROS 2 轨迹桥接节点与 ESP32 micro-ROS 舵机固件。桥接节点接收机械臂和夹爪的 FollowJointTrajectory 动作，发布 /servo_angles 给下位机。

## 内容导航

- [src/trajectory_bridge](./src/trajectory_bridge)：ROS 2 C++ 轨迹桥接包
- [src/trajectory_bridge/config/trajectory_bridge.yaml](./src/trajectory_bridge/config/trajectory_bridge.yaml)：关节顺序、初始位置与发布频率
- [src/trajectory_bridge/launch/trajectory_bridge.launch.py](./src/trajectory_bridge/launch/trajectory_bridge.launch.py)：节点启动入口
- [firmware/esp32.ino](./firmware/esp32.ino)：ESP32 舵机固件
- [doc](./doc)：配套资料

## 参数约定

默认发布频率为 50 Hz，包含 `joint_1` 至 `joint_6`。初始位置由 `home_positions_rad` 指定，前五个关节为 90°、第六个为 60°，配置单位为弧度。关节数组顺序必须与固件 PCA9685 通道对应。

## 构建与准备

在已配置 ROS 2、colcon 及包依赖的终端中，从仓库根目录执行：

```bash
colcon build --packages-select trajectory_bridge
source install/setup.bash
ros2 launch trajectory_bridge trajectory_bridge.launch.py
```

固件使用 micro_ros_arduino、Adafruit NeoPixel 与 Adafruit PWM Servo Driver，需另行配置 micro-ROS Agent 与 Wi-Fi。`esp32.ino` 要求同目录提供 `secrets.h`；当前仓库未包含代码注释所指的 `secrets.h.template`，需自行提供 `ARM_WIFI_SSID`、`ARM_WIFI_PASS`、`ARM_AGENT_IP` 和 `ARM_AGENT_PORT` 定义。

本项目面向当前六通道舵机配置，适配其他 PWM 舵机时应重新核对脉宽、角度限位、供电与关节映射。

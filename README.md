# NXROBO Spark (Noetic)

## 特性 / Features
- 包含 Spark 的核心源码、驱动和若干示例功能包
- 演示 SLAM、导航、目标跟随与机械臂抓取等功能

## 目录 / Table of Contents
- 功能包说明 (packages-overview)
- 系统要求与安装 (requirements & install)

## 系统要求 / Requirements
- 操作系统：Ubuntu 20.04+
- ROS 版本：ROS Noetic

## 快速安装 / Quick Install
```bash
# 克隆仓库
git clone https://github.com/Ljunyan/RAICOM-Robotics-Competition.git
cd RAICOM-Robotics-Competition

# 安装系统依赖（在 Ubuntu 上）
sudo apt update
# sudo apt install <必要依赖包>    # 根据需要安装额外依赖

# 使用 rosdep 安装 ROS 依赖（如有 package.xml 指定）
# sudo apt install python3-rosdep
# sudo rosdep init
# rosdep update
# rosdep install --from-paths src --ignore-src -r -y

# 编译
catkin_make

# 编译完成后在新终端或当前 shell 中 source 工作空间
source devel/setup.bash
```

## 快速运行 / One-key run
编译成功后可以直接运行仓库内提供的便捷脚本：
```bash
./onekey.sh
```

# 工作空间树

```
spark_noetic/
├── src/                          # ROS 源码包目录
│   ├── spark_common/             # 通用工具、消息、参数模板
│   ├── spark_driver/             # 底盘驱动与硬件接口
│   ├── spark_navigation/         # 导航相关（map、amcl、move_base 配置）
│   ├── spark_slam/               # SLAM 演示（gmapping/rtabmap 等）
│   ├── spark_follower/           # 跟随功能包（视觉/激光跟随）
│   ├── spark_carry_object/       # 机械臂抓取与视觉识别示例
│   ├── auto/                     # 全自动（详见下文）
│   ├── semi/                     # 半自动（详见下文）
│   └── tools/                    # 辅助脚本、仿真与数据处理工具
├── launch/                       # 全局 launch 脚本集合
├── config/                       # 全局参数与地图、yaml 配置
├── scripts/                      # 一键运行脚本（onekey.sh 等）
├── README.md                     # 简洁说明
├── README_Detailed.md            # 详细文档
└── .gitignore
```

## 各工作空间简短介绍
- spark_common：公共消息、工具函数、参数模板，其他包依赖的基础库。
- spark_driver：底盘与传感器驱动（串口/Can/USB 接口、里程计发布）。
- spark_navigation：导航栈配置（move_base、costmap、路径规划器、AMCL）。
- spark_slam：SLAM 演示与配置（gmapping、rtabmap 等 launch 示例）。
- spark_follower：目标跟随（基于摄像头/激光或融合跟随逻辑）。
- spark_carry_object：机械臂 + 视觉抓取流程和示例节点。
- tools：数据记录、回放、地图处理、仿真 launch 脚本等。
- auto：全自动整车流程（见下）。
- semi：半自动（见下）。

## 重点：auto（全自动运行）
目的：实现尽可能自主的巡航与任务执行（路径规划 → 行为决策 → 控制）。

内容概览：
- nodes/
  - perception/：障碍检测、目标识别、车道/地标检测（camera/lidar fusion）
  - localization/：里程计融合、滤波器（EKF/robot_localization）、AMCL 集成
  - planning/：行为规划与全局/局部路径生成（全局路径 + 局部避障）
  - control/：速度与转向控制器（PID / MPC）
  - monitor/：安全监控、急停逻辑、状态机
- launch/
  - auto_bringup.launch：整车上电启动（传感器+定位+规划+控制）
  - auto_sim.launch：仿真模式（带 bag 或 Gazebo）
- config/
  - params.yaml：控制/规划/传感器标定参数
  - safety.yaml：安全阈值与急停配置

关键 topic / 参数（常见）：
- /odom, /tf, /scan, /camera/image_raw
- /cmd_vel（控制输出）
- 参数：max_vel, max_accel, safety_distance, planner_type

## 重点：semi（半自动）
目的：提供手动控制接口。

内容概览：
- nodes/
  - teleop/：遥控接口（joystick / web teleop）
  - assist/：路径辅助、速度限制、自动避障建议（保留人工决策）
  - split_control/：切换逻辑（自动/手动优先级）
  - ui/：可视化与远程监控（ROS web tools / RViz launch）
- launch/
  - semi_bringup.launch：半自动模式启动（teleop + assist）
  - semi_ui.launch：远程监控与控制界面
- config/
  - semi_params.yaml：手动/自动切换阈值、最大手动速度限制

快速运行示例：
```bash
source devel/setup.bash
roslaunch semi semi_bringup.launch
# 启动 teleop：
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```


# rm_vision_ws

睿曼(RealMan)机器人视觉抓取系统 - ROS2工作空间

## 项目简介
车臂协同项目，实现多功能车臂任务。
底盘车型号：轮趣科技 S300
机械臂型号：瑞尔曼 ECO63
夹爪型号：时因科技 EG2-4C2
任务目标：自动寻路避障，语音控制前往目标；抓取物品；准确识别并抓取卡片
项目代码：此为机械臂部分代码，用于验证机械臂功能完整。包含机械臂功能包和视觉识别功能包
基于ROS2的机器人视觉抓取系统，专为睿曼RM Eco63系列机械臂设计。该系统集成了视觉识别、运动规划、夹爪控制等核心功能，实现了智能化的物体抓取任务。
## 相关网站
机械臂ROS2官方文档：https://develop.realman-robotics.com/robot4th/ros2/getStarted/
夹爪官网：https://www.inspire-robots.com/dexterous%20hands/eg2-4c-series/
底盘车官网：https://www.wheeltec.net/

## 系统架构

### 核心功能包

| 功能包 | 说明 |
|--------|------|
| `rm_bringup` | 该功能包为机械臂的节点启动功能包，其作用为快速启动多节点复合的机械臂功能。 |
| `rm_control` | 轨迹控制器，采用三次样条插值实现平滑运动控制 |
| `rm_description` | 机器人URDF模型和几何描述文件 |
| `rm_driver` | 机器人通信驱动，实现与机械臂的实时数据交互 |
| `rm_moveit2_config` | MoveIt2运动规划配置，支持虚拟和真实机械臂控制 |
| `rm_ros_interfaces` | 自定义ROS2消息类型定义 |
| `vi_grab` | 视觉抓取模块，包含目标检测和抓取规划 |

### 视觉模块

- **目标检测**: 基于YOLOv8的实时物体检测
- **深度感知**: Intel RealSense深度相机支持
- **抓取规划**: 智能抓取位姿计算和路径规划

## 环境要求

- **操作系统**: Ubuntu 22.04 (推荐)
- **ROS版本**: ROS2 Humble Hawksbill
- **MoveIt2**: 最新稳定版本
- **Python**: 3.10+

## 快速开始

### 1. 安装依赖

```bash
# 安装ROS2
./src/rm_robot/rm_install/scripts/ros2_install.sh

# 安装MoveIt2
./src/rm_robot/rm_install/scripts/moveit2_install.sh

# 安装Python依赖
pip3 install -r ReadMe/requirements.txt
```
### 2. 配置功能包环境
```bash
cd ~/ros2_rm_robot/rm_driver/scripts/
sudo bash lib_install.sh
```

### 3. 编译工作空间

```bash
colcon build --symlink-install
source install/setup.bash
```

### 4. 启动系统


#### 启动真实机械臂

```bash
# 1. 启动机器人驱动
ros2 launch rm_bringup rm_bringup_lite.launch.py

ros2 launch rm_bringup rm_bringup.launch.py
```

#### 启动视觉抓取系统

```bash
# 一键启动全部功能
./start_card.sh

# 或分别启动
ros2 launch rm_bringup rm_bringup.launch.py
ros2 run vi_grab yolo_rs_publisher
python3 src/vi_grab/vi_grab/camera_to_base.py
ros2 run vi_grab grasp_card_executor  #抓取水瓶
```

## 功能说明

### 夹爪控制

系统支持多种夹爪控制模式：

```python
# 打开夹爪
open_gripper(position=1000)

# 力控夹取
force_grip(speed=500, force=400)

# 设置夹爪位置
set_gripper(position)
```

### 运动控制

支持多种运动模式：

- **关节运动** (`/arm_movej`): 关节角度控制
- **直线运动** (`/arm_movel`): 末端直线移动
- **圆弧运动** (`/arm_movec`): 圆弧轨迹规划
- **力位混合控制**: 力控与位置控制的结合


## 目录结构

```
rm_vision_ws/
├── src/
│   ├── rm_robot/          # 机器人核心功能包
│   │   ├── rm_bringup/    # 系统启动
│   │   ├── rm_control/    # 运动控制
│   │   ├── rm_description/# 机器人模型
│   │   ├── rm_driver/     # 硬件驱动
│   │   ├── rm_moveit2_config/ # MoveIt2配置
│   │   └── rm_ros_interfaces/ # 消息定义
│   └── vi_grab/           # 视觉抓取
│         ├── camera_to_base.py   # 坐标转换
│         ├── grasp_card_executor.py   # 抓取卡片执行器
│         ├── grasp_bottle_executor.py   # 抓取水瓶执行器
│         └── yolo_rs_publisher.py # YOLO检测发布器
│         └── config 
│               └──grasp_config.yaml #卡片夹取配置文件
├── models/                # YOLO模型
│   └── yolov8n.pt
├── ReadMe/                # 文档和依赖
└── start.sh / stop.sh     # 启动/停止脚本
```

## API参考

### 话题订阅

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/object_info` | ObjectInfo | 目标物体信息 |
| `/arm_pose` | Pose | 机械臂当前位姿 |
| `/gripper/result` | Bool | 夹爪执行结果 |

### 话题发布

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/arm_movej` | Movej | 关节运动指令 |
| `/gripper/control` | GripperControl | 夹爪控制指令 |

## 注意事项

⚠️ **安全警告**

1. 首次使用真实机械臂前，务必在仿真环境中充分测试
2. 运行时保持急停开关在可及范围内
3. 确保工作空间无障碍物，防止碰撞
4. 定期检查机械臂关节限位设置

## 许可证

本项目遵循各功能包对应的开源许可证协议。

## 技术支持

如有问题或建议，请通过项目页面提交Issue。

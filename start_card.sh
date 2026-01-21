#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 设置环境
source "$SCRIPT_DIR/install/setup.bash"

# 启动机械臂驱动
echo "启动机械臂驱动..."
ros2 launch rm_bringup rm_bringup_lite.launch.py &
DRIVER_PID=$!
sleep 3

# 启动夹取卡片程序
echo "启动夹取卡片程序..."
ros2 run vi_grab grasp_card_executor &
GRASP_PID=$!

# 保存进程ID到文件
echo "$DRIVER_PID" > "$SCRIPT_DIR/.card_driver.pid"
echo "$GRASP_PID" > "$SCRIPT_DIR/.card_grasp.pid"

echo "✓ 已启动"
echo "  机械臂驱动 PID: $DRIVER_PID"
echo "  卡片抓取程序 PID: $GRASP_PID"
echo ""
echo "使用 stop_card.sh 来停止程序"

# 保持脚本运行
wait

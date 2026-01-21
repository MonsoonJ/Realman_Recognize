#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 设置环境
source "$SCRIPT_DIR/install/setup.bash"

# 清理函数
cleanup() {
    echo ""
    echo "程序已结束，5秒后强制杀死..."
    sleep 5
    
    echo "强制杀死进程..."
    if [ -f "$SCRIPT_DIR/.card_grasp.pid" ]; then
        GRASP_PID=$(cat "$SCRIPT_DIR/.card_grasp.pid")
        kill -9 "$GRASP_PID" 2>/dev/null
        rm -f "$SCRIPT_DIR/.card_grasp.pid"
    fi
    
    if [ -f "$SCRIPT_DIR/.card_driver.pid" ]; then
        DRIVER_PID=$(cat "$SCRIPT_DIR/.card_driver.pid")
        kill -9 "$DRIVER_PID" 2>/dev/null
        rm -f "$SCRIPT_DIR/.card_driver.pid"
    fi
    
    # 清理ROS进程
    killall -9 ros2 2>/dev/null
    killall -9 python3 2>/dev/null
    
    echo "✓ 已清理所有进程"
    exit 0
}

# 设置陷阱
trap cleanup EXIT

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

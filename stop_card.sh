#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 停止夹取卡片程序
if [ -f "$SCRIPT_DIR/.card_grasp.pid" ]; then
    GRASP_PID=$(cat "$SCRIPT_DIR/.card_grasp.pid")
    if kill -0 "$GRASP_PID" 2>/dev/null; then
        echo "停止卡片抓取程序 (PID: $GRASP_PID)..."
        kill "$GRASP_PID"
        sleep 1
        # 如果还没停止，强制杀死
        kill -9 "$GRASP_PID" 2>/dev/null
        echo "✓ 卡片抓取程序已停止"
    fi
    rm -f "$SCRIPT_DIR/.card_grasp.pid"
fi

# 停止机械臂驱动
if [ -f "$SCRIPT_DIR/.card_driver.pid" ]; then
    DRIVER_PID=$(cat "$SCRIPT_DIR/.card_driver.pid")
    if kill -0 "$DRIVER_PID" 2>/dev/null; then
        echo "停止机械臂驱动 (PID: $DRIVER_PID)..."
        kill "$DRIVER_PID"
        sleep 1
        # 如果还没停止，强制杀死
        kill -9 "$DRIVER_PID" 2>/dev/null
        echo "✓ 机械臂驱动已停止"
    fi
    rm -f "$SCRIPT_DIR/.card_driver.pid"
fi

# 清理ROS进程
echo "清理ROS进程..."
killall -9 ros2 2>/dev/null
killall -9 python3 2>/dev/null

echo "✓ 已停止所有程序"

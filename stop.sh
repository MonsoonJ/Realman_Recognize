#!/bin/bash
# 瑞尔曼ECO63机械臂抓取系统 - 停止脚本

# 颜色定义
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}=========================================${NC}"
echo -e "${BLUE}机械臂抓取系统停止${NC}"
echo -e "${BLUE}=========================================${NC}"
echo

# 停止函数
stop_process() {
    local name=$1
    local pid_file=$2
    
    if [[ -f "$pid_file" ]]; then
        local pid=$(cat "$pid_file")
        if kill -0 "$pid" 2>/dev/null; then
            echo -e "${YELLOW}停止 $name (PID: $pid)...${NC}"
            kill -TERM "$pid" 2>/dev/null || true
            
            # 等待进程结束
            local count=0
            while kill -0 "$pid" 2>/dev/null && [[ $count -lt 5 ]]; do
                sleep 0.5
                ((count++))
            done
            
            # 强制杀死未结束的进程
            if kill -0 "$pid" 2>/dev/null; then
                kill -KILL "$pid" 2>/dev/null || true
                echo -e "${RED}   强制停止 $name${NC}"
            else
                echo -e "${GREEN}   ✓ $name 已停止${NC}"
            fi
        else
            echo -e "${YELLOW}   $name 进程不存在${NC}"
        fi
        rm -f "$pid_file"
    else
        echo -e "${YELLOW}   未找到 $name 的PID文件${NC}"
    fi
}

# 停止所有组件
echo -e "${BLUE}正在停止系统组件...${NC}"
echo

# 停止抓取执行器
stop_process "抓取执行器" ".grasp.pid"
sleep 0.5

# 停止坐标变换
stop_process "坐标变换" ".transform.pid"
sleep 0.5

# 停止YOLO视觉识别
stop_process "视觉识别" ".vision.pid"
sleep 0.5

# 停止机械臂驱动
stop_process "机械臂驱动" ".driver.pid"
sleep 1

# 清理ROS2进程
echo -e "${BLUE}清理ROS2进程...${NC}"
pkill -f "rm_bringup_lite" 2>/dev/null || true
pkill -f "yolo_rs_publisher" 2>/dev/null || true
pkill -f "camera_to_base" 2>/dev/null || true
pkill -f "grasp_executor3" 2>/dev/null || true

# 清理gazebo、rviz等（如果运行中）
pkill -f "gazebo" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true

echo
echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}系统已完全停止${NC}"
echo -e "${GREEN}=========================================${NC}"
echo
echo -e "${BLUE} 停止状态:${NC}"
echo -e "${GREEN}   机械臂驱动 - 已停止${NC}"
echo -e "${GREEN}   YOLO视觉识别 - 已停止${NC}"
echo -e "${GREEN}   坐标变换 - 已停止${NC}"
echo -e "${GREEN}   抓取执行器 - 已停止${NC}"
echo -e "${GREEN}   ROS2进程 - 已清理${NC}"
echo
echo -e "${BLUE}系统已安全关闭！${NC}"
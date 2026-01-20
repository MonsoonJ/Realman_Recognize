#!/bin/bash

# 颜色定义
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}=========================================${NC}"
echo -e "${BLUE} 机械臂抓取系统启动${NC}"
echo -e "${BLUE}=========================================${NC}"
echo

# # 检查并激活conda环境
# CURRENT_CONDA_ENV=$(conda info --envs | grep -E '^\*|^[[:space:]]*recogn' | grep -v '^*' | head -1 | awk '{print $1}')
# if [[ "$CURRENT_CONDA_ENV" != "recogn" ]]; then
#     echo -e "${BLUE} 切换到recogn conda环境...${NC}"
#     conda activate recogn
#     if [[ $? -ne 0 ]]; then
#         echo -e "${YELLOW} 无法激活recogn环境，请确保已安装conda且recogn环境存在${NC}"
#         exit 1
#     fi
#     echo -e "${GREEN} 已切换到recogn环境${NC}"
# else
#     echo -e "${GREEN} 已在recogn环境中${NC}"
# fi

# echo -e "${GREEN} 正在启动系统组件...${NC}"
# echo

# 设置环境
source install/setup.bash

# 1. 启动瑞尔曼机械臂驱动
echo -e "${BLUE}1. 启动机械臂驱动...${NC}"
cd src/rm_robot/rm_bringup/launch
ros2 launch rm_bringup_lite.launch.py &
DRIVER_PID=$!
echo -e "${GREEN}   ✓ 驱动已启动 (PID: $DRIVER_PID)${NC}"

# 等待驱动启动
sleep 5

# 2. 启动YOLO视觉识别
echo -e "${BLUE}2. 启动YOLO视觉识别...${NC}"
cd ../..
cd ../..
ros2 run vi_grab yolo_rs_publisher &
VISION_PID=$!
echo -e "${GREEN}   ✓ 视觉识别已启动 (PID: $VISION_PID)${NC}"

# 等待视觉节点启动
sleep 3

# 3. 启动坐标变换
echo -e "${BLUE}3. 启动坐标变换...${NC}"
cd src/vi_grab/vi_grab
python3 camera_to_base.py &
TRANSFORM_PID=$!
echo -e "${GREEN}   ✓ 坐标变换已启动 (PID: $TRANSFORM_PID)${NC}"

# 等待坐标变换节点启动
sleep 3

# 4. 启动抓取执行器
echo -e "${BLUE}4. 启动抓取执行器...${NC}"
python3 grasp_executor3.py &
GRASP_PID=$!
echo -e "${GREEN}   抓取执行器已启动 (PID: $GRASP_PID)${NC}"

echo
echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN} 系统启动完成 ${NC}"
echo -e "${GREEN}=========================================${NC}"
echo
echo -e "${BLUE} 启动状态:${NC}"
echo -e "${GREEN}   机械臂驱动 - 运行中${NC}"
echo -e "${GREEN}   YOLO视觉识别 - 运行中${NC}"
echo -e "${GREEN}   坐标变换 - 运行中${NC}"
echo -e "${GREEN}   抓取执行器 - 运行中${NC}"
echo
echo -e "${BLUE}🔍 系统状态: 等待目标物体识别...${NC}"
echo
echo -e "${YELLOW} 使用说明:${NC}"
echo -e "   • 将目标物体放在相机视野中"
echo -e "   • 系统将自动识别并抓取目标"
echo -e "   • 按 Ctrl+C 停止系统"
echo

# 保存PID到文件
# echo "$DRIVER_PID" > .driver.pid
# echo "$VISION_PID" > .vision.pid
# echo "$TRANSFORM_PID" > .transform.pid
# echo "$GRASP_PID" > .grasp.pid

# 等待用户中断
trap 'echo; echo -e "${YELLOW}正在停止系统...${NC}"; bash stop.sh; exit 0' INT
wait
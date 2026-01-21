
本项目已优化为使用YAML配置文件管理所有主要参数，配置文件位于: `vi_grab/config/grasp_config.yaml`

# 1. 相机与手眼标定 (`camera`) 修改这些参数需要重新进行手眼标定。

camera:
  width: 640              # 图像宽度（像素）
  height: 480             # 图像高度（像素）
  fps: 30                 # 帧率（帧/秒）
  format: "bgr8"          # 图像格式
  
  rotation_cam_to_ee:     # 相机到末端执行器的旋转矩阵（3x3）
  translation_cam_to_ee:  # 相机到末端执行器的平移向量（3x1）

# 2. 模型配置 (`model`)

model:
  yolo_model_path: "models/card.pt"  # YOLO模型文件路径
  target_class: "card"               # 目标检测类别
  detection_confidence: 0.5          # 检测置信度阈值


# 3. 路径配置 (`paths`)

paths:
  photo_save_dir: "img_card"  # 保存检测图片的目录

# 4. 机械臂初始位置 (`home_position`)

home_position:
  x: -0.4   # X坐标（米）
  y: 0.0    # Y坐标（米）
  z: 0.4    # Z坐标（米）

# 5. 放置点位置 (`place_position`)

place_position:
  x: 0.01   # X坐标（米）
  y: -0.2   # Y坐标（米）
  z: 0.75   # Z坐标（米）

# 6. 步骤执行参数 (`steps`) 每个步骤都有独立配置，包含超时时间、速度、算法参数等。

## Step 2: 视觉对齐并下降

step2:
  timeout: 4.0                        # 步骤超时时间（秒）
  speed: 40                           # 移动速度
  hsv_lower: [0, 90, 40]             # HSV下界（用于矩形检测）
  hsv_upper: [30, 255, 255]          # HSV上界
  morphology_kernel_size: 5          # 形态学处理的核大小
  descend_distance: 0.12             # 下降距离（米）
  camera_y_offset: 0.04              # 相机Y轴偏移（米）

修改 `hsv_lower` 和 `hsv_upper` 来调整颜色检测范围
调整 `camera_y_offset` 校准对齐精度
增加 `morphology_kernel_size` 可以过滤更多噪声

## Step 4-9其他步骤

每个步骤都有 `timeout`（超时）、`speed`（速度）等参数。
若步骤执行不完整，增加 `timeout`
若移动过快导致不稳定，降低 `speed`

## 7. 夹爪控制 (`gripper`)

gripper:
  open_position: 850      # 张开位置（电气单位）
  pick_speed: 500         # 闭合速度
  pick_force: 100         # 闭合力度

## 8. 深度检测参数 (`depth`)

depth:
  search_radius: 2        # 搜索半径（像素）
  min_depth: 0.0          # 最小深度（米）
  max_depth: 2.0          # 最大深度（米）

增加 `search_radius` 提高深度鲁棒性但降低精度
根据实际场景调整深度范围

## 常见调整场景


1. 调整Step 2的HSV参数:
   ```yaml
   step2:
     hsv_lower: [0, 80, 30]      # 降低要求
     hsv_upper: [30, 255, 255]
   ```

2. 调整YOLO置信度:
   ```yaml
   model:
     detection_confidence: 0.3   # 降低阈值
   ```


3. 调整相机偏移:
   ```yaml
   step2:
     camera_y_offset: 0.05       # 增加偏移
   ```

4. 调整搜索半径:
   ```yaml
   depth:
     search_radius: 3            # 增加搜索范围
   ```


# 保存当前配置为备份
cp vi_grab/config/grasp_config.yaml grasp_config_backup_$(date +%Y%m%d_%H%M%S).yaml

# 恢复备份
cp grasp_config_backup_20260121_143000.yaml vi_grab/config/grasp_config.yaml


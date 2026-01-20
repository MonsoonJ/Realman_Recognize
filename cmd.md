
colcon build --packages-select rm_ros_interfaces
#机械臂/分别启动控制
ros2 launch rm_driver rm_eco63_driver.launch.py
ros2 launch rm_description rm_eco63_display.launch.py
ros2 launch rm_control rm_eco63_control.launch.py
ros2 launch rm_eco63_config real_moveit_demo.launch.py
#夹爪控制
python3 gripper_arm_sleep.py  #取水固定动作演示
ros2 topic pub --once /rm_driver/set_gripper_position_cmd rm_ros_interfaces/msg/Gripperset "{position: 1, block: true, timeout: 3}"  #单独控制夹爪
ros2 topic pub --once /rm_driver/set_gripper_position_cmd rm_ros_interfaces/msg/Gripperset "{position: 999, block: true, timeout: 3}"


#流程命令
conda activate recogn && source install/setup.bash	#环境
colcon build  && source install/setup.bash


ros2 launch rm_bringup rm_bringup_lite.launch.py	#机械臂驱动启动
ros2 run vi_grab grasp_card_executor 
ros2 run vi_grab grasp_bottle_executor



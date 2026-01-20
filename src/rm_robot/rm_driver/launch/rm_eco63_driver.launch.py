import launch
import os
import yaml
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    arm_config = os.path.join(get_package_share_directory('rm_driver'),'config','rm_eco63_config.yaml')
    with open(arm_config,'r') as f:
        params = yaml.safe_load(f)["rm_driver"]["ros__parameters"]
    rm_eco63_driver = Node(
        package= "rm_driver",
        executable= "rm_driver",
        parameters= [arm_config ],
        output= 'screen'
    )
    # 新增：设置工具电压节点
    set_tool_voltage = Node(
        package='rm_driver',  # 你的包名
        executable='set_tool_voltage.py',  # 文件名（不带.py，如果用entry_point则用'set_tool_voltage'）
        name='set_tool_voltage',
        output='screen'
    )
    ld.add_action(rm_eco63_driver)
    ld.add_action(set_tool_voltage)  # 在driver后执行
    return ld
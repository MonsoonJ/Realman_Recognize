from setuptools import setup
import os
from glob import glob

package_name = 'vi_grab'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('vi_grab', 'config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishros',
    maintainer_email='fishros@todo.todo',
    description='YOLOv8 + RealSense publisher',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_bottle_executor = vi_grab.grasp_bottle_executor:main',
            'grasp_card_executor = vi_grab.grasp_card_executor:main',
            'gripper_arm_demo = vi_grab.gripper_arm_demo:main',
        ],
    },
)


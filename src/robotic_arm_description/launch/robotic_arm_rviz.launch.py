from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
        
    return LaunchDescription([
        # Launch robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open('src/robotic_arm_description/urdf/robotic_arm.urdf').read()
            }]
        ),
        # Launch joint state publisher GUI to control joints
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        # Launch RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', 'src/robotic_arm_description/config/robotic_arm.rviz']
        )
    ])
    



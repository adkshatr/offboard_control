from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os
def generate_launch_description():

    offboard_node = Node(
                                package='offboard_control',
                                executable='offboard_control',
                                name='offboard_controlS',
                                parameters=['/home/root/ros_ws/src/mocap_pose/param/parameter.yaml'],
                                output='screen'
                            )

    
    

    return LaunchDescription([
        offboard_node
    ])
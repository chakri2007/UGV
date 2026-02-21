from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pid_node = Node(
        package='ugv_controller',
        executable='pid',
        output='screen'
    )

    cmd_pwm_node = Node(
        package='ugv_controller',
        executable='cmd_pwm',
        output='screen'
    )

    gps_node = Node(
        package='ugv_controller',
        executable='gps',
        output='screen'
    )

    odom_node = Node(
        package='ugv_controller',
        executable='odom',
        output='screen'
    )

    return LaunchDescription([
        pid_node,
        cmd_pwm_node,
        gps_node,
        odom_node
    ])

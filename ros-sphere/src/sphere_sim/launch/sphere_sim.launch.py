import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('sphere_sim')
    world_path = os.path.join(pkg_share, 'worlds', 'oscillating_sphere.sdf')

    # 1. Launch Gazebo Harmonic with world
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    # 2. ROS-Gazebo Bridge for Twist commands
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/oscillating_sphere/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )

    # 3. Sphere Controller Node
    controller = Node(
        package='sphere_sim',
        executable='sphere_controller',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge,
        controller
    ])
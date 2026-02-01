from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_path = get_package_share_directory('cartpole_sim')
    urdf_path = os.path.join(pkg_path, 'urdf', 'cartpole.urdf')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_path).read()
        }]
    )

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'cartpole',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    delayed_spawn = TimerAction(
        period=2.0,
        actions=[spawn]
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        delayed_spawn
    ])

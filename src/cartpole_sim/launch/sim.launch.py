from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_path = get_package_share_directory('cartpole_sim')
    urdf_path = os.path.join(pkg_path, 'urdf', 'cartpole.urdf')

    return LaunchDescription([

        # Start Gazebo Harmonic
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),

        # Spawn cartpole
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'cartpole',
                '-file', urdf_path
            ],
            output='screen'
        ),
    ])

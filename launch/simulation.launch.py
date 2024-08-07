import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sdf_dir = os.path.join(get_package_share_directory('cudatech_description'), 'models')
    sdf_file = os.path.join(sdf_dir, 'cudatech_car.sdf')

    gz = launch.actions.ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'cudatech_car',
            '-x', '0',
            '-y', '0',
            '-z', '1',
            '-file', sdf_file,
        ],
        output='screen'
    )

    return LaunchDescription([
        gz,
        spawn_entity,
    ])

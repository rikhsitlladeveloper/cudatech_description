import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'name',
            default_value='default_folder',
            description='Subfolder under ~/.cudatech/saves for saving files',
        ),
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context):
    home_dir = os.path.expanduser('~')
    subfolder = LaunchConfiguration('name').perform(context)
    save_dir = os.path.join(home_dir, '.cudatech', 'saves', subfolder)
    os.makedirs(save_dir, exist_ok=True)
    path = os.path.join(save_dir, 'snapshot.bag.pbstream')

    if not os.path.exists(path):
        return []

    pkg_share = FindPackageShare(package='cudatech_description').find('cudatech_description')
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(pkg_share, 'config') )
    configuration_basename = LaunchConfiguration('configuration_basename', default='cartographer_2d.lua')

    return [
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            arguments=[
                '-load_state_filename', path,
                '-configuration_directory', configuration_directory,
                '-configuration_basename', configuration_basename,
            ],
        )
    ]

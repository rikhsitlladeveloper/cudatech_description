from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'name',
            default_value='default_folder',
            description='Subfolder under ~/.cudatech/saves/ for saving files'
        ),
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context):
    home_dir = os.path.expanduser('~')
    subfolder = LaunchConfiguration('name').perform(context)
    save_dir = os.path.join(home_dir, '.cudatech', 'saves', subfolder)

    os.makedirs(save_dir, exist_ok=True)
    pbstream_filename = os.path.join(save_dir, 'snapshot.bag.pbstream')

    return [
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/finish_trajectory', 'cartographer_ros_msgs/srv/FinishTrajectory', 
                 '{"trajectory_id": 0}'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/write_state', 'cartographer_ros_msgs/srv/WriteState', 
                 '{"filename": "' + pbstream_filename + '", "include_unfinished_submaps": true}'],
            output='screen'
        ),
    ]

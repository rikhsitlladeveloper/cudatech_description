import os
from os.path import join
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def find_tty_usb(idVendor, idProduct):
    """find_tty_usb('067b', '2302') -> '/dev/ttyUSB0'"""
    # Note: if searching for a lot of pairs, it would be much faster to search
    # for the enitre lot at once instead of going over all the usb devices
    # each time.
    for dnbase in os.listdir('/sys/bus/usb/devices'):
        dn = join('/sys/bus/usb/devices', dnbase)
        if not os.path.exists(join(dn, 'idVendor')):
            continue
        idv = open(join(dn, 'idVendor')).read().strip()
        if idv != idVendor:
            continue
        idp = open(join(dn, 'idProduct')).read().strip()
        if idp != idProduct:
            continue
        for subdir in os.listdir(dn):
            if subdir.startswith(dnbase+':'):
                for subsubdir in os.listdir(join(dn, subdir)):
                    if subsubdir.startswith('ttyUSB'):
                        return join('/dev', subsubdir)


def generate_launch_description():
    pkg_share = FindPackageShare(package='cudatech_description').find('cudatech_description')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(pkg_share, 'config') )
    configuration_basename = LaunchConfiguration('configuration_basename', default='cartographer_2d.lua')

    urdf = os.path.join(
        get_package_share_directory('cudatech_description'),
        'urdf/cudatech.urdf.xacro')



    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    param_dir = get_package_share_directory('cudatech_description')
    nav2_params_path = os.path.join(param_dir, 'config', 'nav2_params.yaml')
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params_path,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'yaml_filename': '/home/cudatech/map.yaml'},
            convert_types=True),
        allow_substs=True)
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]


    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[urdf]),
        Node(
            package='cudatech_description',
            executable='dynamic_frame_tf2_broadcaster',
            name='dynamic_broadcaster',
        ),
        Node(
            package='cudatech_description',
            executable='state_publisher',
            name='state_publisher',
            output='screen',
        ),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':'serial',
                         'serial_port': find_tty_usb('10c4', 'ea60'), 
                         'serial_baudrate': 115200, 
                         'frame_id': 'rplidar_link',
                         'inverted': False, 
                         'angle_compensate': False, 
                         'scan_mode': 'Sensitivity'}],
            output='screen'),




        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_file': '/home/cudatech/map.yaml',
                #'params_file': nav2_params_path,
            }]
        ),

        # Node(
        #     name='nav2_container',
        #     package='rclcpp_components',
        #     executable='component_container_isolated',
        #     parameters=[configured_params, {'autostart': True}],
        #     remappings=remappings,
        #     output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                       'bringup_launch.py')),
            launch_arguments={
                              'map': '/home/cudatech/map.yaml',
                              'use_sim_time': use_sim_time,
                              'autostart': 'true'}.items()),

        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(os.path.join(launch_dir,
        #                                               'localization_launch.py')),
        #    launch_arguments={
        #                      'map': '/home/cudatech/map.yaml',
        #                      'use_sim_time': use_sim_time,
        #                      'autostart': 'True',
        #                      'params_file': nav2_params_path,
        #                      'use_composition': 'True',
        #                      'use_respawn': 'False',
        #                      'container_name': 'nav2_container'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={
                              'use_sim_time': use_sim_time,
                              'autostart': 'true'}.items()),



        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', configuration_directory,
                    '-configuration_basename', configuration_basename]
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-resolution', '0.001',
                '-map-file', '/home/cudatech/map.yaml'
            ]
        ),

        Node(
            package='cudatech_description',
            executable='serialport_controller',
            name='serialport_controller',
            output='screen',
            arguments=[
                'serial_port','/dev/vtmx',
            ]
        ),

        # Node(
        #     package='teleop_twist_keyboard',
        #     executable='teleop_twist_keyboard',
        #     name='teleop_twist_keyboard',
        #     output='screen',
        #     prefix='xterm -e'
        # ),
        #ros2 run cartographer_ros cartographer_occupancy_grid_node resolution:=0.05
    ])
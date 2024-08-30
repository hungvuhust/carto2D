import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    #Run Cartographer
    useSimTime = LaunchConfiguration('use_sim_time', default='false')
    rtcCartographerPrefix = get_package_share_directory('cartographer_ros')
    cartographerConfigDir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  rtcCartographerPrefix, 'configuration_files'))
    locconfigurationBasename = LaunchConfiguration('loc_configuration_basename',
                                                default='localization.lua')
    mappingconfigurationBasename = LaunchConfiguration('mapping_configuration_basename',
                                                 default='cartographer.lua')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publishPeriodSec = LaunchConfiguration('publish_period_sec', default='1.0')

    #Load State
    loadStateFilename = LaunchConfiguration('file_name', default='')
    
    return LaunchDescription([
        
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographerConfigDir,
            description='Full path to config file to load'),

        DeclareLaunchArgument(
            'loc_configuration_basename',
            default_value=locconfigurationBasename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'mapping_configuration_basename',
            default_value=mappingconfigurationBasename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'file_name',
            default_value=loadStateFilename,
            description='Name of lua file for cartographer'),
 
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': useSimTime}],
            arguments=['-configuration_directory', cartographerConfigDir,
                       '-configuration_basename', mappingconfigurationBasename,
                       '-load_state_filename',  loadStateFilename
                       ],
            remappings=[('scan', 'scan')]
        ),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publishPeriodSec,
            description='OccupancyGrid publishing period'),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancygrid.launch.py']),
        #     launch_arguments={'use_sim_time': useSimTime, 'resolution': resolution,
        #                       'publish_period_sec': publishPeriodSec}.items(),
        # ),

        
        
        
        
    ])

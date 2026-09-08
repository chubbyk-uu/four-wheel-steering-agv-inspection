"""Optional simulation measurement sources plus production adapters and dual EKFs."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share=Path(get_package_share_directory('agv_localization'))
    common={'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower()=='true',
            'profile':LaunchConfiguration('profile').perform(context),
            'config':LaunchConfiguration('config').perform(context) or str(share/'config/measurements.yaml'),
            'platform':LaunchConfiguration('platform').perform(context),
            'camera_config':LaunchConfiguration('camera_config').perform(context),
            'output_dir':LaunchConfiguration('output_dir').perform(context)}
    actions=[Node(package='agv_localization',executable='measurement_adapter',parameters=[common],output='screen')]
    if LaunchConfiguration('simulate_gnss').perform(context).lower()=='true':
        actions.append(Node(package='agv_localization',executable='gnss_simulator',parameters=[common],output='screen'))
    for kind in ('local','global'):
        actions.append(Node(package='robot_localization',executable='ekf_node',name='ekf_'+kind,
            parameters=[str(share/f'config/ekf_{kind}.yaml'),{'use_sim_time':common['use_sim_time']}],
            remappings=[('odometry/filtered','/odometry/'+kind)],output='screen'))
    # Known ENU scene/map registration, independent of vehicle state.
    actions.append(Node(package='tf2_ros',executable='static_transform_publisher',name='world_map_registration',
        arguments=['--frame-id','world','--child-frame-id','map'],parameters=[{'use_sim_time':common['use_sim_time']}],output='screen'))
    return actions


def generate_launch_description():
    desc=Path(get_package_share_directory('agv_description'))/'config'
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        DeclareLaunchArgument('profile',default_value='normal',choices=['normal','zero']),
        DeclareLaunchArgument('config',default_value=''),
        DeclareLaunchArgument('platform',default_value=str(desc/'platform.yaml')),
        DeclareLaunchArgument('camera_config',default_value=str(desc/'linescan.yaml')),
        DeclareLaunchArgument('output_dir',default_value=''),
        DeclareLaunchArgument('simulate_gnss',default_value='true'),
        OpaqueFunction(function=setup)])

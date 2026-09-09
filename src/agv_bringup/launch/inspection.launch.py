"""An idle operator session with coherent RViz, OptiX capture and archived navigation."""
from pathlib import Path
import time
import uuid
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share=Path(get_package_share_directory('agv_bringup'))
    root=Path(LaunchConfiguration('session_dir').perform(context)).resolve();root.mkdir(parents=True,exist_ok=False)
    scene=Path(LaunchConfiguration('scene_manifest').perform(context)).resolve()
    if not scene.is_file():raise ValueError('Restore road assets and set scene_manifest to the baked road manifest')
    config=yaml.safe_load((Path(get_package_share_directory('agv_description'))/'config/linescan.yaml').read_text())
    config.update(projected_encoder=True,max_yaw_rate_rad_s=.08,max_scan_lateral_m_s=.10,max_scan_residual_m_s=.03)
    camera=root/'camera.yaml';camera.write_text(yaml.safe_dump(config))
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/sim.launch.py')),launch_arguments={
        'localization':'true','rviz':'true','linescan':'true','linescan_backend':'optix',
        'scene_manifest':str(scene),'camera_config':str(camera),'scan_speed_limit':'.8','spawn_x':'3',
        'capture_dir':str(root/'raw'),'localization_output_dir':str(root/'navigation'),
        'gpu_backend':LaunchConfiguration('gpu_backend').perform(context),
        'headless':LaunchConfiguration('headless').perform(context)}.items()),
        Node(package='agv_mission',executable='mission_operator',parameters=[{'use_sim_time':True,'output_root':str(root/'tasks'),'navigation_dir':str(root/'navigation')}],output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('session_dir',default_value='local_data/inspection_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]),
        DeclareLaunchArgument('scene_manifest',default_value='assets/road/baked_fullwidth_20m_v1/manifest.json'),
        DeclareLaunchArgument('gpu_backend',default_value='d3d12',choices=['d3d12','native']),
        DeclareLaunchArgument('headless',default_value='false'),OpaqueFunction(function=setup)])

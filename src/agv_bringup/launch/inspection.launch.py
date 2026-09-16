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
from agv_mission.camera_limits import inspection_camera_config
from agv_mission.road_display import build_road_display
import json


def setup(context):
    share=Path(get_package_share_directory('agv_bringup'))
    root=Path(LaunchConfiguration('session_dir').perform(context)).resolve();root.mkdir(parents=True,exist_ok=False)
    scene=Path(LaunchConfiguration('scene_manifest').perform(context)).resolve()
    if not scene.is_file():raise ValueError('Default road asset missing: follow docs/ROAD_ASSETS.md (2 mm / 20 cm / 1025 heightmap), or explicitly set scene_manifest')
    config=yaml.safe_load((Path(get_package_share_directory('agv_description'))/'config/linescan.yaml').read_text())
    platform=yaml.safe_load((Path(get_package_share_directory('agv_description'))/'config/platform.yaml').read_text())
    config=inspection_camera_config(config,platform)
    camera=root/'camera.yaml';camera.write_text(yaml.safe_dump(config))
    road=build_road_display(scene,root/'rviz_road');road_config=root/'rviz_road.json';road_config.write_text(json.dumps(road,indent=2))
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/sim.launch.py')),launch_arguments={
        'localization':'true','rviz':'true','linescan':'true','linescan_backend':'optix',
        'scene_manifest':str(scene),'camera_config':str(camera),'scan_speed_limit':str(config['max_scan_speed_m_s']),'spawn_x':LaunchConfiguration('spawn_x').perform(context),
        'capture_dir':str(root/'raw'),'localization_output_dir':str(root/'navigation'),
        'gpu_backend':LaunchConfiguration('gpu_backend').perform(context),
        'localization_config':LaunchConfiguration('localization_config').perform(context),
        'headless':LaunchConfiguration('headless').perform(context)}.items()),
        Node(package='agv_mission',executable='mission_operator',parameters=[{'use_sim_time':True,'output_root':str(root/'tasks'),'navigation_dir':str(root/'navigation'),'road_display':str(road_config)}],output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('session_dir',default_value='local_data/inspection_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]),
        DeclareLaunchArgument('scene_manifest',default_value='assets/road/runtime_fullwidth_100m_rough_marked_2mm_20cm_heightmap_v1/manifest.json'),
        DeclareLaunchArgument('gpu_backend',default_value='d3d12',choices=['d3d12','native']),
        DeclareLaunchArgument('spawn_x',default_value='3'),
        # A repeat run has to name its noise seed without editing the shipped config.
        DeclareLaunchArgument('localization_config',default_value=''),
        DeclareLaunchArgument('headless',default_value='false'),OpaqueFunction(function=setup)])

"""AGV simulation; ground truth is reserved for sensor simulation and evaluation."""
from pathlib import Path
import tempfile
import os
import xml.etree.ElementTree as ET
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    desc = Path(get_package_share_directory('agv_description'))
    bringup = Path(get_package_share_directory('agv_bringup'))
    platform = LaunchConfiguration('platform').perform(context)
    config = yaml.safe_load(Path(platform).read_text())
    if not 400 <= config['mass'] <= 700:
        raise ValueError('Large AGV mass must be within validated design range 400 to 700 kg')
    behavior = str(bringup / 'config/motion.yaml')
    camera_config = LaunchConfiguration('camera_config').perform(context)
    robot = xacro.process_file(str(desc / 'urdf/agv.urdf.xacro'), mappings={
        'platform': platform, 'camera_config': camera_config, 'controllers': str(bringup / 'config/controllers.yaml')}).toxml()
    from agv_linescan.robot_scene import split_visual_links
    robot = split_visual_links(robot)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    linescan = LaunchConfiguration('linescan').perform(context).lower() == 'true'
    backend = LaunchConfiguration('linescan_backend').perform(context)
    world_path = str(bringup / 'worlds/flat.sdf')
    shared_manifest = LaunchConfiguration('scene_manifest').perform(context)
    if shared_manifest:
        from agv_linescan.shared_scene import validate, validate_spawn_position
        manifest_path = Path(shared_manifest).resolve()
        shared = validate(manifest_path)
        world_path = str(manifest_path.parent / shared['world'])
        spawn_x = float(LaunchConfiguration('spawn_x').perform(context))
        spawn_y = float(LaunchConfiguration('spawn_y').perform(context))
        validate_spawn_position(shared, spawn_x, spawn_y)
        if linescan and backend not in ('render', 'optix'):
            raise ValueError('shared 3D scene supports render or optix only; planar samplers cannot represent this geometry')
    if linescan and backend == 'optix' and not shared_manifest:
        raise ValueError('optix requires a validated scene_manifest')
    if linescan and backend in ('cuda_grid', 'cuda_tiles') and LaunchConfiguration('scan_probe').perform(context).lower() == 'true':
        raise ValueError('cuda_grid only supports an unobstructed plane; use render for the occlusion probe')
    if linescan:
        from agv_linescan.grid_world import write_grid_world
        source_world = world_path
        with tempfile.NamedTemporaryFile(prefix='agv_grid_', suffix='.sdf', delete=False) as file:
            world_path = file.name
        if shared_manifest:
            ET.parse(source_world).write(world_path, encoding='unicode')
        elif backend == 'cuda_tiles':
            from agv_linescan.grid_world import write_tiled_world
            write_tiled_world(bringup / 'worlds/flat.sdf', world_path,
                              LaunchConfiguration('terrain_manifest').perform(context))
        else:
            write_grid_world(bringup / 'worlds/flat.sdf', world_path,
                             yaml.safe_load(Path(camera_config).read_text()),
                             probe=LaunchConfiguration('scan_probe').perform(context).lower() == 'true')
        if backend in ('render', 'cuda_grid', 'cuda_tiles', 'optix'):
            tree = ET.parse(world_path)
            plugin = ET.SubElement(tree.getroot().find('world'), 'plugin',
                filename=str(Path(get_package_prefix('agv_linescan'))/'lib/libagv_gz_linescan.so'),
                name='agv_linescan::GzLineScan')
            for key, value in {'config': camera_config, 'platform': platform, 'backend': backend,
                               'output_dir': LaunchConfiguration('capture_dir').perform(context),
                               'max_scan_speed': LaunchConfiguration('scan_speed_limit').perform(context)}.items():
                ET.SubElement(plugin, key).text = value
            if backend == 'optix':
                from agv_linescan.robot_scene import export
                robot_manifest = export(robot, tempfile.mkdtemp(prefix='agv_robot_scene_'))
                ET.SubElement(plugin, 'scene_manifest').text = str(manifest_path)
                ET.SubElement(plugin, 'robot_manifest').text = str(robot_manifest)
                ET.SubElement(plugin, 'optix_ptx').text = str(Path(get_package_share_directory('agv_linescan'))/'agv_optix_scan.ptx')
            if backend == 'cuda_tiles':
                ET.SubElement(plugin, 'terrain_manifest').text = LaunchConfiguration('terrain_manifest').perform(context)
            block_rows = LaunchConfiguration('scan_block_rows').perform(context)
            if block_rows:
                ET.SubElement(plugin, 'block_rows').text = block_rows
            tree.write(world_path, encoding='unicode')
    sensor_tree = ET.parse(world_path)
    sensor_world = sensor_tree.getroot().find('world')
    if not any(p.get('name') == 'gz::sim::systems::Sensors' for p in sensor_world.findall('plugin')):
        sensors = ET.SubElement(sensor_world, 'plugin', filename='gz-sim-sensors-system', name='gz::sim::systems::Sensors')
        ET.SubElement(sensors, 'render_engine').text = 'ogre2'
    if not any(p.get('name') == 'gz::sim::systems::Imu' for p in sensor_world.findall('plugin')):
        ET.SubElement(sensor_world, 'plugin', filename='gz-sim-imu-system', name='gz::sim::systems::Imu')
    with tempfile.NamedTemporaryFile(prefix='agv_sensors_', suffix='.sdf', delete=False) as f:
        world_path = f.name
    sensor_tree.write(world_path, encoding='unicode')
    gz = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + ('-s ' if headless else '--gui-config ' + LaunchConfiguration('gui_config').perform(context) + ' ') + world_path,
                          'on_exit_shutdown': 'true'}.items())
    if linescan and not any(os.environ.get(k) for k in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE')):
        gz = GroupAction(actions=[SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',
                                  str(bringup/'config/fastdds_linescan.xml')), gz])
    controller = Node(package='agv_control', executable='swerve_controller',
                      parameters=[config, behavior, {'use_sim_time': True}], output='screen')
    spawner = Node(package='controller_manager', executable='spawner', arguments=[
        'joint_state_broadcaster', 'steering_controller', 'drive_controller',
        '--controller-manager-timeout', '180', '--switch-timeout', '120',
        '--service-call-timeout', '150'])
    def check_spawn(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Controller activation failed'))]
        return []
    actions = []
    if LaunchConfiguration('localization').perform(context).lower() == 'true':
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('agv_localization'))/'launch/localization.launch.py')),
            launch_arguments={'profile': LaunchConfiguration('localization_profile'),
                              'config': LaunchConfiguration('localization_config'),
                              'output_dir': LaunchConfiguration('localization_output_dir'),
                              'platform': platform, 'camera_config': camera_config,
                              'use_sim_time': 'true'}.items()))
    correction_profile=LaunchConfiguration('correction_profile').perform(context)
    if correction_profile:
        if not linescan:raise ValueError('correction_profile requires linescan:=true')
        actions.append(Node(package='agv_linescan',executable='linescan_correction',
            parameters=[{'profile':correction_profile,'capture_config':camera_config,
                         'output_dir':str(Path(LaunchConfiguration('capture_dir').perform(context))/'corrected')}],
            additional_env={} if any(os.environ.get(k) for k in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE')) else
                {'FASTRTPS_DEFAULT_PROFILES_FILE':str(bringup/'config/fastdds_linescan.xml')},output='screen'))
    if linescan and backend == 'analytic':
        actions.append(Node(package='agv_linescan', executable='linescan_node',
                            parameters=[{'use_sim_time': True, 'config': camera_config,
                                         'platform': platform,
                                         'output_dir': LaunchConfiguration('capture_dir')}], output='screen'))
    if LaunchConfiguration('gpu_backend').perform(context) == 'd3d12':
        actions.extend([
            SetEnvironmentVariable('GALLIUM_DRIVER', 'd3d12'),
            SetEnvironmentVariable('MESA_D3D12_DEFAULT_ADAPTER_NAME',
                                   LaunchConfiguration('gpu_adapter'))])
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(package='agv_bringup', executable='visualization_tf.py',
                            parameters=[{'use_sim_time': True}], output='screen'))
        rviz_node=Node(package='rviz2', executable='rviz2', name='agv_rviz',
                            arguments=['-d', str(bringup/'config/inspection.rviz')],
                            parameters=[{'use_sim_time': True}], remappings=[('/tf','/visualization/tf')], output='screen')
        if not headless and LaunchConfiguration('gpu_backend').perform(context)=='d3d12':
            # Stagger the two WSLg graphics clients; do not race their startup
            # context creation. Controller failure still shuts down the launch.
            actions.append(RegisterEventHandler(OnProcessExit(target_action=spawner,
                on_exit=lambda event,context:[rviz_node] if event.returncode==0 else [])))
        else:actions.append(rviz_node)
    if not headless and LaunchConfiguration('follow_camera').perform(context).lower() == 'true':
        actions.append(Node(package='agv_bringup', executable='follow_camera.py', output='screen'))
    actions.append(SetEnvironmentVariable('GZ_GUI_PLUGIN_PATH', str(Path(get_package_prefix('agv_bringup'))/'lib') + os.pathsep + os.environ.get('GZ_GUI_PLUGIN_PATH','')))
    return actions + [gz,
        RegisterEventHandler(OnProcessExit(target_action=controller,
            on_exit=[EmitEvent(event=Shutdown(reason='Motion controller exited'))])),
        RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=check_spawn)),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot, 'use_sim_time': True, 'publish_frequency': 100.0}]),
        Node(package='ros_gz_sim', executable='create', arguments=[
            '-name', 'agv', '-topic', 'robot_description', '-z', str(config['base_height']+.005), '-x', LaunchConfiguration('spawn_x'), '-y', LaunchConfiguration('spawn_y'), '-Y', LaunchConfiguration('spawn_yaw')]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/sensors/imu/raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/lidar/left/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/lidar/right/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked']),
        spawner, controller]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('follow_camera', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('gui_config', default_value=str(Path(get_package_share_directory('agv_bringup'))/'config/gui.config')),
        DeclareLaunchArgument('linescan', default_value='false'),
        DeclareLaunchArgument('correction_profile', default_value='', description='Optional online diagnostic only; normal workflow corrects archived raw images offline'),
        DeclareLaunchArgument('camera_config', default_value=str(Path(get_package_share_directory('agv_description'))/'config/linescan.yaml')),
        DeclareLaunchArgument('linescan_backend', default_value='render', choices=['render', 'analytic', 'cuda_grid', 'cuda_tiles', 'optix']),
        DeclareLaunchArgument('terrain_manifest', default_value=''),
        DeclareLaunchArgument('scene_manifest', default_value=''),
        DeclareLaunchArgument('spawn_x', default_value='0'),
        DeclareLaunchArgument('spawn_y', default_value='0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0'),
        DeclareLaunchArgument('scan_speed_limit', default_value='0.25'),
        DeclareLaunchArgument('scan_probe', default_value='false'),
        DeclareLaunchArgument('scan_block_rows', default_value='', description='C++ sensor lines per image (1..16384); empty uses camera YAML block_rows, default 4096'),
        DeclareLaunchArgument('capture_dir', default_value='/tmp/agv_linescan'),
        DeclareLaunchArgument('gpu_backend', default_value='d3d12', choices=['d3d12', 'native']),
        DeclareLaunchArgument('gpu_adapter', default_value='NVIDIA'),
        DeclareLaunchArgument('localization', default_value='false'),
        DeclareLaunchArgument('localization_profile', default_value='normal', choices=['normal','zero']),
        DeclareLaunchArgument('localization_config', default_value=''),
        DeclareLaunchArgument('localization_output_dir', default_value=''),
        DeclareLaunchArgument('platform', default_value=str(Path(
            get_package_share_directory('agv_description')) / 'config/platform.yaml')),
        OpaqueFunction(function=setup)])

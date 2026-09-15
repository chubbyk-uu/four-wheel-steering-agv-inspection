"""AGV simulation; ground truth is reserved for sensor simulation and evaluation."""
from pathlib import Path
import tempfile
import os
import shutil
import xml.etree.ElementTree as ET
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent, SetEnvironmentVariable, TimerAction, ExecuteProcess, LogInfo
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
    actual_diameter = float(LaunchConfiguration('actual_wheel_diameter').perform(context))
    if not .38 <= actual_diameter <= .42:
        raise ValueError('actual_wheel_diameter must be within the small tyre experiment range [0.38, 0.42] m')
    from agv_linescan.encoder import line_spacing
    camera_values = yaml.safe_load(Path(camera_config).read_text())
    flex = camera_values.get('mount_flex', {})
    if flex.get('enabled', False):
        import math
        for key, lo, hi in [('stiffness_nm_rad', 40, 2000), ('damping_nms_rad', .1, 30)]:
            value = flex.get(key, float('nan'))
            if not isinstance(value, (int, float)) or not math.isfinite(value) or not lo <= value <= hi:
                raise ValueError('mount_flex '+key+' outside experimental range')
    effective_spacing = line_spacing(camera_values, config['wheel_radius'])
    if camera_values['line_spacing_m'] != effective_spacing:
        camera_values['line_spacing_m'] = effective_spacing
        with tempfile.NamedTemporaryFile(mode='w', prefix='agv_encoder_', suffix='.yaml', delete=False) as f:
            yaml.safe_dump(camera_values, f)
            camera_config = f.name
    robot = xacro.process_file(str(desc / 'urdf/agv.urdf.xacro'), mappings={
        'platform': platform, 'actual_wheel_diameter': str(actual_diameter), 'camera_config': camera_config, 'controllers': str(bringup / 'config/controllers.yaml')}).toxml()
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
                               'actual_wheel_diameter': str(actual_diameter),
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
    def gz_sim(args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': args, 'on_exit_shutdown': 'true'}.items())
    # Server and GUI are separate processes even with a GUI, so that the GUI does not
    # create its graphics context while the sensor server creates its own. Both still
    # stop the run when they exit: a mission observed headless is not the joint GUI
    # acceptance it would be mistaken for. The GUI is built further down rather than
    # here, because its exit code has to be inspected before that rule is applied.
    gz = gz_sim('-s -r ' + world_path)
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
        if yaml.safe_load(Path(camera_config).read_text()).get('wheel_encoder'):
            raise ValueError('wheel_encoder requires a C++ sensor backend; analytic is a legacy ideal-distance simulator')
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
                            parameters=[{'use_sim_time': True, 'camera_flex_enabled': bool(flex.get('enabled', False))}], output='screen'))
        rviz_node=Node(package='rviz2', executable='rviz2', name='agv_rviz',
                            arguments=['-d', str(bringup/'config/inspection.rviz')],
                            parameters=[{'use_sim_time': True}], remappings=[('/tf','/visualization/tf')], output='screen')
        if not headless and LaunchConfiguration('gpu_backend').perform(context)=='d3d12':
            # Stagger the two WSLg graphics clients; do not race their startup
            # context creation. Controller failure still shuts down the launch.
            actions.append(RegisterEventHandler(OnProcessExit(target_action=spawner,
                on_exit=lambda event,context:[rviz_node] if event.returncode==0 else [])))
        else:actions.append(rviz_node)
    # follow_camera.py gives up 90 s after it starts, so it waits with the GUI rather
    # than from the beginning of the launch: the stagger and any GUI retry would
    # otherwise come out of its budget and leave the camera unlocked on a run that
    # recovered.
    follow_actions = ([Node(package='agv_bringup', executable='follow_camera.py', output='screen')]
                      if not headless and LaunchConfiguration('follow_camera').perform(context).lower() == 'true'
                      else [])
    actions.append(SetEnvironmentVariable('GZ_GUI_PLUGIN_PATH', str(Path(get_package_prefix('agv_bringup'))/'lib') + os.pathsep + os.environ.get('GZ_GUI_PLUGIN_PATH','')))
    if not headless:
        # The GUI runs as its own ExecuteProcess rather than a third gz_sim.launch.py
        # include, so that an abort while it creates its graphics context can be
        # retried instead of ending the run. That include contributes nothing else
        # here: its computed model and plugin paths are both empty in this workspace,
        # so the only behaviour given up is its unconditional Shutdown, reinstated
        # below for every exit that is not a startup abort.
        gz_executable = shutil.which('gz')
        if gz_executable is None:
            raise ValueError('gz is not on PATH; the Gazebo GUI client cannot be started')
        gui_config_path = LaunchConfiguration('gui_config').perform(context)
        gui_delay = float(LaunchConfiguration('gui_start_delay').perform(context))
        gui_retry_delay = float(LaunchConfiguration('gui_retry_delay').perform(context))
        gui_retries = int(LaunchConfiguration('gui_abort_retries').perform(context))

        def gui_with_retry(remaining):
            process = ExecuteProcess(cmd=['ruby', gz_executable, 'sim', '-g',
                                          '--gui-config', gui_config_path, '--force-version', '8'],
                                     name='gazebo_gui', output='screen')

            def exited(event, context):
                # 134 is SIGABRT, which is the only way the Qt context failure has ever
                # ended: the D3D12 screen is lost and this Mesa has no software driver
                # to fall back to. A clean close, a signal, or any other code is a real
                # exit and still stops the run, because a mission observed headless is
                # not the joint GUI acceptance it would be mistaken for.
                if event.returncode == 134 and remaining > 0:
                    return [LogInfo(msg='Gazebo GUI aborted creating its graphics context; '
                                        'retrying, %d attempt(s) left' % remaining),
                            TimerAction(period=gui_retry_delay, actions=gui_with_retry(remaining - 1))]
                return [EmitEvent(event=Shutdown(reason='Gazebo GUI client exited'))]

            return [RegisterEventHandler(OnProcessExit(target_action=process, on_exit=exited)), process]

        # Hold the GUI until the controllers are up, then wait again so that it is not
        # creating its graphics context while RViz creates its own. Both used to start
        # on this one event and came up with consecutive pids; all three recorded
        # startup failures have that shape, with the GUI the one that loses its device.
        # The delay is the separation the comment above RViz always claimed but never had.
        # A failed spawn already shuts the run down through check_spawn.
        actions.append(RegisterEventHandler(OnProcessExit(target_action=spawner,
            on_exit=lambda event,context:[TimerAction(period=gui_delay,
                actions=gui_with_retry(gui_retries) + follow_actions)] if event.returncode==0 else [])))
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
        DeclareLaunchArgument('gui_start_delay', default_value='6.0', description='Seconds between the RViz and Gazebo GUI graphics clients; 0 restores the simultaneous start that every recorded context failure shows'),
        DeclareLaunchArgument('gui_abort_retries', default_value='2', description='Retries when the Gazebo GUI aborts creating its graphics context; 0 restores the previous behaviour of ending the run on the first abort'),
        DeclareLaunchArgument('gui_retry_delay', default_value='10.0', description='Seconds to let the graphics stack settle before retrying an aborted GUI'),
        DeclareLaunchArgument('follow_camera', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('gui_config', default_value=str(Path(get_package_share_directory('agv_bringup'))/'config/gui.config')),
        DeclareLaunchArgument('linescan', default_value='false'),
        DeclareLaunchArgument('correction_profile', default_value='', description='Optional online diagnostic only; normal workflow corrects archived raw images offline'),
        DeclareLaunchArgument('actual_wheel_diameter', default_value='0.40', description='Physical diameter of all four tyres in metres; calibrated wheel_radius stays unchanged'),
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

import importlib.util
import json
from pathlib import Path


PACKAGE = Path(__file__).parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


sim = load('agv_sim_launch', PACKAGE / 'launch' / 'sim.launch.py')
follow = load('agv_follow_camera', PACKAGE / 'scripts' / 'follow_camera.py')


def test_gui_retry_is_limited_to_d3d12_startup_abort():
    assert sim.should_retry_gui(134, ready=False, remaining=1, d3d12=True)
    assert sim.should_retry_gui(-6, ready=False, remaining=1, d3d12=True)
    assert not sim.should_retry_gui(134, ready=True, remaining=1, d3d12=True)
    assert not sim.should_retry_gui(-6, ready=True, remaining=1, d3d12=True)
    assert not sim.should_retry_gui(134, ready=False, remaining=0, d3d12=True)
    assert not sim.should_retry_gui(134, ready=False, remaining=1, d3d12=False)
    assert not sim.should_retry_gui(1, ready=False, remaining=1, d3d12=True)


def test_gui_options_are_bounded():
    sim.validate_gui_options(6., 10., 2)
    for values in ((float('nan'), 10., 2), (-1., 10., 2),
                   (6., 121., 2), (6., 10., 6), (6., 120., 3)):
        try:
            sim.validate_gui_options(*values)
        except ValueError:
            pass
        else:
            raise AssertionError(f'accepted invalid GUI options {values}')


def test_readiness_timeout_covers_the_same_retry_budget():
    assert sim.gui_readiness_timeout(10., 2) == 90.
    assert sim.gui_readiness_timeout(120., 2) == 300.
    assert sim.gui_readiness_timeout(30., 5) == 255.
    # gui_start_delay is absent intentionally: probe and first GUI start together
    # after that delay, so it consumes neither process's runtime budget.
    sim.validate_gui_options(120., 120., 2)


def test_follow_status_acceptance_modes():
    tracked = {'trackMode': 'FOLLOW_LOOK_AT',
               'followTarget': {'name': 'agv'},
               'trackTarget': {'name': 'agv'}}
    assert follow.acceptable(tracked)
    assert not follow.acceptable({})
    assert not follow.acceptable([])
    assert follow.acceptable({}, ready_only=True)
    assert not follow.acceptable([], ready_only=True)


def test_launch_ros_arguments_do_not_reach_helper_parser():
    assert follow.own_arguments(['--ready-only', '--ros-args', '-r', 'a:=b']) == [
        '--ready-only']
    assert follow.own_arguments(['--timeout', '10']) == ['--timeout', '10']


def test_readiness_evidence_is_atomic_and_explicit(tmp_path):
    ready = tmp_path / 'nested' / 'ready.json'
    follow.write_ready(ready, ready_only=True)
    evidence = json.loads(ready.read_text())
    assert evidence['ready'] is True
    assert evidence['mode'] == 'gui_only'
    assert isinstance(evidence['pid'], int)
    assert evidence['wall_monotonic_s'] > 0
    assert list(ready.parent.glob('*.tmp.*')) == []


def scene(display):
    assets = [{'name': 'terrain_0', 'mesh': 'terrain_0.obj'}]
    if display:
        assets[0]['display_mesh'] = {'mesh': 'display_terrain_0.obj'}
    return {'assets': assets}


def test_render_backend_is_refused_on_a_scene_with_a_lighter_display_mesh():
    """One Ogre2 scene per server, so this cannot be separated by assets."""
    import pytest
    with pytest.raises(ValueError, match='display_mesh'):
        sim.check_display_mesh_backend(scene(True), 'render')


def test_other_backends_and_plain_scenes_are_unaffected():
    for backend in ('optix', 'cuda_grid', 'cuda_tiles', 'analytic'):
        sim.check_display_mesh_backend(scene(True), backend)
    for backend in ('render', 'optix'):
        sim.check_display_mesh_backend(scene(False), backend)


def test_inspection_forwards_physical_diameter_without_recalibrating(tmp_path, monkeypatch):
    import pytest
    import yaml
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    inspection = load('inspection_diameter', PACKAGE / 'launch/inspection.launch.py')
    declarations = {a.name: a for a in inspection.generate_launch_description().entities
                    if isinstance(a, DeclareLaunchArgument)}
    context = LaunchContext()
    for declaration in declarations.values():
        declaration.execute(context)
    assert context.launch_configurations['actual_wheel_diameter'] == '0.40'
    scene = tmp_path / 'scene.json'
    scene.write_text('{}')
    monkeypatch.setattr(inspection, 'build_road_display', lambda *args: {})
    descriptions = PACKAGE.parent / 'agv_description'
    monkeypatch.setattr(inspection, 'get_package_share_directory',
                        lambda name: str(descriptions if name == 'agv_description' else PACKAGE))
    spacings = []
    for diameter in ('0.39', '0.41'):
        session = tmp_path / diameter
        context.launch_configurations.update(session_dir=str(session), scene_manifest=str(scene),
                                             actual_wheel_diameter=diameter)
        include = inspection.setup(context)[0]
        assert dict(include.launch_arguments)['actual_wheel_diameter'] == diameter
        spacings.append(yaml.safe_load((session / 'camera.yaml').read_text())['line_spacing_m'])
    assert spacings == pytest.approx([0.0003657010198328743] * 2)


def test_operator_diameter_reaches_launch_and_is_recorded(tmp_path, monkeypatch):
    import pytest
    import sys
    tools = PACKAGE.parents[1] / 'tools'
    monkeypatch.syspath_prepend(str(tools))
    operator = load('operator_diameter', tools / 'validate_operator_session.py')
    class LaunchIntercepted(Exception):
        pass
    commands = []
    def intercept(command, **kwargs):
        commands.append(command)
        kwargs['stdout'].close()
        raise LaunchIntercepted()
    monkeypatch.setattr(operator.subprocess, 'Popen', intercept)
    for diameter in ('0.39', '0.41'):
        output = tmp_path / diameter
        monkeypatch.setattr(sys, 'argv', ['probe', '--output', str(output),
                                         '--actual-wheel-diameter', diameter])
        with pytest.raises(LaunchIntercepted):
            operator.main()
        assert 'actual_wheel_diameter:=' + diameter in commands[-1]
        assert json.loads((output / 'experiment_parameters.json').read_text()) == {
            'actual_wheel_diameter_m': float(diameter)}
    for diameter in ('nan', '0.37', '0.43'):
        output = tmp_path / diameter
        monkeypatch.setattr(sys, 'argv', ['probe', '--output', str(output),
                                         '--actual-wheel-diameter', diameter])
        with pytest.raises(SystemExit) as error:
            operator.main()
        assert error.value.code == 2
        assert not output.exists()

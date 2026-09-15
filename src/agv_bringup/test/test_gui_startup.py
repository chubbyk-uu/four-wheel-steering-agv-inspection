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

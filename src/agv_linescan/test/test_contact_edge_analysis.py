import importlib.util
import math
from pathlib import Path
import pytest


path = Path(__file__).resolve().parents[3] / 'tools' / 'analyze_contact_edge_fault.py'
spec = importlib.util.spec_from_file_location('contact_edge_analysis', path)
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


def point(x, y, normal=(0., 0., 1.)):
    return {'position_world_m': [x, y, 0.], 'normal': list(normal),
            'depth_m': 0., 'force_magnitude_n': 0.}


def test_grid_boundary_and_internal_diagonal_are_distinct():
    on_diagonal = analysis.phase_distance(point(.025, .025), .1, (0., 0.))
    assert on_diagonal['internal_diagonal_m'] == 0
    assert on_diagonal['nearest_grid_boundary_m'] == pytest.approx(.025)
    near_boundary = analysis.phase_distance(point(.001, .04), .1, (0., 0.))
    assert near_boundary['nearest_grid_boundary_m'] == pytest.approx(.001)
    assert near_boundary['internal_diagonal_m'] == pytest.approx(abs(.001-.04)/math.sqrt(2))


def test_normal_change_is_reported_without_inventing_one_for_no_contact():
    assert analysis.angle([0., 0., 0.], [0., 0., 1.]) is None
    assert analysis.angle([0., 0., 1.], [0., 1., 0.]) == pytest.approx(math.pi/2)


def test_individual_edge_normal_is_not_hidden_by_mean():
    wheel = {
        'available': True, 'pair_count': 1, 'point_count': 3,
        'stored_point_count': 3, 'truncated': False, 'max_depth_m': 0.,
        'max_force_magnitude_n': 1.,
        'points': [point(.01, .01), point(.02, .02), point(.03, .03, (1., 0., 0.))],
    }
    summary = analysis.wheel_summary(wheel, .1, (0., 0.))
    assert summary['max_normal_tilt_from_world_up_rad'] == pytest.approx(math.pi/2)
    assert summary['contact_locations'][-1]['normal'] == [1., 0., 0.]

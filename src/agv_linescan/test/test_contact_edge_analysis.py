import importlib.util
import math
from pathlib import Path
import pytest


path = Path(__file__).resolve().parents[3] / 'tools' / 'analyze_contact_edge_fault.py'
spec = importlib.util.spec_from_file_location('contact_edge_analysis', path)
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


def point(x, y, normal=(0., 0., 1.)):
    return {'position_world_m': [x, y, 0.], 'normal': list(normal)}


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

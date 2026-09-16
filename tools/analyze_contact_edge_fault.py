#!/usr/bin/env python3
"""Relate a v2 flight dump to the road grid without claiming causation.

The production rough-road proxy uses a regular square grid split along each
cell's low-low to high-high diagonal.  This report asks whether the rejected
step coincides with a contact-manifold discontinuity at either a cell boundary
or that internal diagonal.  Proximity alone is not proof: a tyre patch spans
several triangles during normal running, so the report also compares point
count, mean normal, penetration and force with the preceding physics step.
"""
import argparse
import json
import math
from pathlib import Path

WHEELS = ('fl', 'fr', 'rl', 'rr')


def mean_normal(points):
    value = [sum(p['normal'][axis] for p in points) for axis in range(3)]
    length = math.sqrt(sum(v*v for v in value))
    return [v/length for v in value] if length else [0., 0., 0.]


def angle(a, b):
    if not any(a) or not any(b):
        return None
    return math.acos(max(-1., min(1., sum(x*y for x, y in zip(a, b)))))


def phase_distance(point, step, origin):
    x, y = point['position_world_m'][:2]
    u = ((x-origin[0])/step) % 1.
    v = ((y-origin[1])/step) % 1.
    return {
        'position_world_m': point['position_world_m'],
        'cell_ij': [math.floor((x-origin[0])/step), math.floor((y-origin[1])/step)],
        'cell_uv': [u, v],
        'nearest_grid_boundary_m': step*min(u, 1-u, v, 1-v),
        'internal_diagonal_m': step*abs(u-v)/math.sqrt(2),
    }


def wheel_summary(wheel, step, origin):
    locations = [phase_distance(p, step, origin) for p in wheel['points']]
    return {
        'available': wheel['available'],
        'pair_count': wheel['pair_count'],
        'point_count': wheel['point_count'],
        'stored_point_count': wheel['stored_point_count'],
        'truncated': wheel['truncated'],
        'max_depth_m': wheel['max_depth_m'],
        'max_force_magnitude_n': wheel['max_force_magnitude_n'],
        'mean_normal': mean_normal(wheel['points']),
        'nearest_grid_boundary_m': min((p['nearest_grid_boundary_m'] for p in locations), default=None),
        'nearest_internal_diagonal_m': min((p['internal_diagonal_m'] for p in locations), default=None),
        'contact_locations': locations,
    }


def delta(previous, fault):
    return {
        'point_count': fault['point_count']-previous['point_count'],
        'max_depth_m': fault['max_depth_m']-previous['max_depth_m'],
        'max_force_magnitude_n': fault['max_force_magnitude_n']-previous['max_force_magnitude_n'],
        'mean_normal_angle_rad': angle(previous['mean_normal'], fault['mean_normal']),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('flight', type=Path)
    parser.add_argument('--grid-step-m', type=float, default=.1)
    parser.add_argument('--origin-x-m', type=float, default=0.)
    parser.add_argument('--origin-y-m', type=float, default=0.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.grid_step_m) or args.grid_step_m <= 0:
        raise ValueError('grid step must be positive')
    document = json.loads(args.flight.read_text())
    if document.get('schema') != 'agv.linescan.flight_recorder.v2':
        raise ValueError('contact evidence requires agv.linescan.flight_recorder.v2')
    records = document.get('records', [])
    if len(records) < 2:
        raise ValueError('flight dump needs at least two records')
    previous_record, fault_record = records[-2:]
    origin = (args.origin_x_m, args.origin_y_m)
    previous = [wheel_summary(w, args.grid_step_m, origin) for w in previous_record['wheel_contact']]
    fault = [wheel_summary(w, args.grid_step_m, origin) for w in fault_record['wheel_contact']]
    if len(previous) != 4 or len(fault) != 4:
        raise ValueError('flight dump does not contain four wheel contacts')
    report = {
        'schema': 'agv.contact_edge_fault_analysis.v1',
        'source': str(args.flight),
        'fault_reason': document.get('reason'),
        'fault_simulation_time_s': document.get('fault_simulation_time_s'),
        'grid': {
            'step_m': args.grid_step_m,
            'origin_xy_m': list(origin),
            'triangulation': 'low-low to high-high diagonal (v <= u / v > u)',
        },
        'interpretation_limit': (
            'Contact proximity to a grid edge is not causal proof. Confirm the hypothesis only when '
            'a residual spike, a contact count/normal/depth/force discontinuity and edge proximity '
            'coincide on the same wheel and physics step.'),
        'previous_time_s': previous_record['t'],
        'fault_time_s': fault_record['t'],
        'wheels': [],
    }
    for index, name in enumerate(WHEELS):
        report['wheels'].append({
            'name': name,
            'fault_wheel_residual_m_s': fault_record['wheel_residual_m_s'][index],
            'previous': previous[index],
            'fault': fault[index],
            'fault_minus_previous': delta(previous[index], fault[index]),
        })
    text = json.dumps(report, indent=2)+'\n'
    if args.output:
        args.output.write_text(text)
    else:
        print(text, end='')


if __name__ == '__main__':
    main()

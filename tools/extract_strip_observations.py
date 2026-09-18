#!/usr/bin/env python3
"""Write the strip optimiser's input table for one archived session."""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT/'src/agv_mission'))
sys.path.append(str(ROOT/'src/agv_linescan'))
from agv_mission.strip_observations import extract, load   # noqa: E402


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session', type=Path, required=True, help='a session directory holding raw/ and navigation/')
    p.add_argument('--plan', type=Path, help='task plan; required when the session holds several')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    table = extract(a.session, a.plan)
    a.output.write_text(json.dumps(table, indent=2)+'\n')
    load(a.output)                      # read it back the way a consumer must
    o = table['observations']
    print(json.dumps(dict(output=str(a.output), observations=len(o),
                          segments=len({v['segment_id'] for v in o}),
                          rows=[o[0]['global_row'], o[-1]['global_row']],
                          sigma_x_m=[min(v['sigma_x_m'] for v in o), max(v['sigma_x_m'] for v in o)],
                          sigma_y_m=[min(v['sigma_y_m'] for v in o), max(v['sigma_y_m'] for v in o)],
                          sigma_heading_rad=[min(v['sigma_heading_rad'] for v in o),
                                             max(v['sigma_heading_rad'] for v in o)]), indent=2))


if __name__ == '__main__':
    main()

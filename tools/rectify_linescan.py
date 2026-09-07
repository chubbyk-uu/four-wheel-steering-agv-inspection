#!/usr/bin/env python3
"""Nominal optics or measured dark/flat + horizontal correction, preserving rows."""
import argparse
from pathlib import Path
import sys
import json
import numpy as np
import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/agv_linescan'))
from agv_linescan.core import Camera
from agv_linescan.calibration import Correction, SCHEMA


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--calibration', required=True)
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--capture-config', help='Required camera YAML for measured calibration compatibility')
    args = parser.parse_args()
    text = Path(args.calibration).read_text()
    try:
        profile = json.loads(text)
    except json.JSONDecodeError:
        profile = yaml.safe_load(text)
    measured = profile.get('schema') == SCHEMA
    camera = Correction(profile) if measured else Camera(profile)
    if measured:
        if not args.capture_config:
            parser.error('--capture-config is required for measured calibration')
        camera.check_capture(yaml.safe_load(Path(args.capture_config).read_text()))
    with Image.open(args.input) as raw:
        if raw.mode != 'L':
            raise ValueError('this prototype requires Mono8')
        if measured:
            result, quality = camera.apply(np.asarray(raw))
            valid = camera.valid
        else:
            result, valid = camera.rectify(np.asarray(raw))
    destination = Path(args.output)
    if destination.exists() or destination.with_suffix('.json').exists():
        raise FileExistsError('refusing to overwrite output')
    destination.parent.mkdir(parents=True, exist_ok=True)
    metadata = {}
    if measured:
        source_meta = Path(args.input).with_suffix('.json')
        metadata = camera.metadata(json.loads(source_meta.read_text()))
        metadata.update(quality)
    Image.fromarray(result).save(destination)
    metadata.update(dict(
        calibration_id=profile['calibration_id'],
        valid_column_ranges=[int(np.flatnonzero(valid)[0]), int(np.flatnonzero(valid)[-1])] if valid.any() else [],
        operation='dark_flat_horizontal_rectification' if measured else 'horizontal_optical_rectification_only',
        rows_preserved=True, invalid_columns_filled_with_zero=True))
    destination.with_suffix('.json').write_text(json.dumps(metadata, indent=2)+'\n')


if __name__ == '__main__':
    main()

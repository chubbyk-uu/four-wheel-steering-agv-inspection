#!/usr/bin/env python3
"""Diagnostics for the fixed short Concrete047A arrow/box experiment, not a stitcher."""
import argparse
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation


def patch_shifts(a, b):
    window = cv2.createHanningWindow((512, 512), cv2.CV_32F)
    records = []
    for row in range(512, min(len(a), len(b))-512, 256):
        for col in (768, 3328):
            # OpenCV may multiply its inputs by the window in place: copy the
            # ROI so overlapping later measurements cannot mutate one another.
            dxdy, response = cv2.phaseCorrelate(
                a[row-256:row+256, col-256:col+256].astype(np.float32, copy=True),
                b[row-256:row+256, col-256:col+256].astype(np.float32, copy=True), window)
            records.append([row, col, *dxdy, response])
    return np.asarray(records)


def shaft_edge(image):
    rows = np.arange(5700, 8650, 32)
    edges = []
    for row in rows:
        profile = np.median(image[row-12:row+12, 1580:1650], axis=0)
        gradient = np.diff(gaussian_filter1d(profile.astype(float), 1))
        k = np.argmax(gradient)
        if not 0 < k < len(gradient)-1:
            raise ValueError('arrow edge outside the experiment ROI')
        delta = .5*(gradient[k-1]-gradient[k+1])/(gradient[k-1]-2*gradient[k]+gradient[k+1])
        edges.append(1580+k+.5+delta)
    return rows, np.asarray(edges)


def sparse_pose_diagnostics(source):
    tags = []
    for path in sorted(source.glob('block_*.json')):
        tags.extend(json.loads(path.read_text())['pose_tags'])
    # Optical frame basis from the fixed nominal mount, no flexible residual.
    camera = np.asarray([tag['camera_rotation_world'] for tag in tags])
    base = camera @ np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    rpy = Rotation.from_matrix(base).as_euler('xyz', degrees=True)
    return dict(sample_count=len(tags), rpy_std_deg=rpy.std(axis=0).tolist(),
        rpy_peak_to_peak_deg=np.ptp(rpy, axis=0).tolist(),
        camera_z_std_mm=float(np.std([t['camera_position_world_m'][2] for t in tags])*1000),
        scope='Sparse truth tags for diagnostics only; not whole-run extrema or a vibration spectrum.')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--captures', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--figure', type=Path, required=True)
    args = parser.parse_args()
    images = [np.array(Image.open(args.captures/'comparison'/(n+'_strip.png'))) for n in ('flat', 'rough')]
    shifts = patch_shifts(*images)
    accepted = shifts[:, 4] >= .6
    good = shifts[accepted]
    if len(good) < .8*len(shifts):
        raise ValueError('too few reliable textured windows for this experiment')
    median = np.median(good[:, 2:4], axis=0)
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    for column, label in ((768, 'Left texture'), (3328, 'Right texture')):
        selected = good[good[:, 1] == column]
        for k in range(2):
            axes[k].plot(selected[:, 0], selected[:, k+2]-median[k], '.-', label=label)
    axes[0].set_ylabel('Across displacement minus common median (px)')
    axes[1].set_ylabel('Along displacement minus common median (px)')
    edges = {}
    for name, image in zip(('flat', 'rough'), images):
        rows, x = shaft_edge(image)
        axes[2].plot(rows, x-x.mean(), label=name.capitalize())
        edges[name] = dict(edge_peak_to_peak_px=float(np.ptp(x)),
            residual_std_after_linear_fit_px=float(np.std(x-np.polyval(np.polyfit(rows, x, 1), rows))))
    axes[2].set_ylabel('Arrow shaft edge minus its mean (px)')
    for ax in axes:
        ax.set_xlabel('Row in flat / own strip')
        ax.grid(alpha=.3)
        ax.legend()
    fig.suptitle('Measured image differences: diagnostic offsets only, no image warping\n'
                 'Constant acquisition-origin offset removed from plots; subpixel estimates include matching error')
    fig.tight_layout(rect=[0, 0, 1, .88])
    args.figure.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.figure, dpi=130)
    plt.close(fig)
    report = dict(schema='agv.rough_textured_image_diagnostics.v1',
        response_threshold=.6, accepted_windows=len(good), total_windows=len(shifts),
        patch_size_px=512, patch_row_step=256, common_median_displacement_px=median.tolist(),
        displacement_peak_to_peak_px=np.ptp(good[:, 2:4], axis=0).tolist(),
        patch_records_columns=['row', 'column', 'dx', 'dy', 'response'], patch_records=shifts.tolist(),
        arrow_shaft_roi=dict(rows=[5700, 8650], columns=[1580, 1650]), arrow_shaft=edges,
        sparse_pose={n:sparse_pose_diagnostics(args.captures/n) for n in ('flat', 'rough')},
        scope='Measured corrected-image differences, not true geometric error or correction accuracy. '
              'Origins differ; the near-constant longitudinal offset is not stretching. '
              'Images were never resampled by these measurements.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='patch_records'}))


if __name__ == '__main__':
    main()

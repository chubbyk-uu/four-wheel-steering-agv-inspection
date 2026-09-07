#!/usr/bin/env python3
"""Capture the same trajectory through the CUDA sensor with controlled lighting."""
import argparse
import copy
import json
from pathlib import Path
import subprocess
import numpy as np
from PIL import Image
import yaml


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--terrain', required=True)
    p.add_argument('--output', required=True)
    args = p.parse_args()
    root = Path(__file__).resolve().parents[1]
    out = Path(args.output); out.mkdir(parents=True, exist_ok=False)
    config = yaml.safe_load((root/'src/agv_description/config/linescan.yaml').read_text())
    cases = {
        'sun_only': {'led_peak_relative': 0},
        'led_only': {'ambient_relative': 0},
        'sun_led': {},
        'shadow_led': {'shadow_start_x_m': -100, 'shadow_end_x_m': 200},
        'exposure_10us': {'exposure_s': 10e-6},
        'exposure_5us': {'exposure_s': 5e-6},
        'noisy': {'noise': True},
        'saturated': {'led_peak_relative': 4000},
    }
    arrays = {}
    for name, overrides in cases.items():
        c = copy.deepcopy(config); c['radiometry']['noise'] = False
        for key, value in overrides.items():
            if key == 'exposure_s': c[key] = value
            else: c['radiometry'][key] = value
        cfg = out/(name+'.yaml'); cfg.write_text(yaml.safe_dump(c))
        with (out/(name+'.log')).open('w') as log:
            subprocess.run(['ros2', 'run', 'agv_linescan', 'benchmark_cuda_grid', str(cfg),
                str(out/name), '1', '19000', '0', '0', str(Path(args.terrain).resolve())],
                stdout=log, stderr=subprocess.STDOUT, check=True, timeout=90)
        report = json.loads((out/name/'summary.json').read_text())
        assert report['invalid_pixels'] == 0 and report['tiles']['radiometry_enabled']
        with Image.open(out/name/'block_0.pgm') as im:
            arrays[name] = np.asarray(im).astype(np.float32)
    black = config['radiometry']['black_level_dn']
    both, shade = arrays['sun_led'], arrays['shadow_led']
    # Bright grid cells only; black stripes have large relative quantization error.
    mask = both > 80
    relative = (both[mask]-shade[mask])/(both[mask]-black)
    # Compare mean column signal to avoid dividing individual small quantized sun signals.
    led_signal = (arrays['led_only']-black).mean(axis=0)
    sun_signal = (arrays['sun_only']-black).mean(axis=0)
    usable = sun_signal >= 2  # Exclude dark stripes dominated by Mono8 quantization.
    ratio = led_signal[usable]/sun_signal[usable]
    assert usable.sum() > 3000
    exposure_errors = {}
    for name, scale in [('exposure_10us', .5), ('exposure_5us', .25)]:
        # Exposure also changes sample locations: exclude grid edges for brightness scaling.
        stable = mask.copy()
        for shift in [-2, -1, 1, 2]:
            stable &= abs(np.roll(both, shift, axis=0)-both) < 3
        error = abs((arrays[name]-black)[stable]-scale*(both-black)[stable])
        exposure_errors[name] = float(np.percentile(error, 99.9))
        assert exposure_errors[name] <= 1.1, exposure_errors
    results = {
        'model': 'relative_projected_strip_v1', 'source': str(out),
        'minimum_column_led_to_sun_signal_ratio': float(ratio.min()),
        'signal_ratio_columns_above_quantization_floor': int(usable.sum()),
        'shadow_relative_change_mean': float(relative.mean()),
        'shadow_relative_change_p99': float(np.percentile(relative, 99)),
        'shadow_difference_dn_max': float((both-shade).max()),
        'exposure_scaling_error_dn_p999': exposure_errors,
        'default_saturated_fraction': float((arrays['noisy'] == 255).mean()),
        'overexposed_saturated_fraction': float((arrays['saturated'] == 255).mean()),
        'noise_difference_std_dn': float((arrays['noisy']-both).std()),
        'full_scene_acceptance_passed': False,
    }
    assert results['minimum_column_led_to_sun_signal_ratio'] >= 20
    assert results['shadow_relative_change_p99'] <= .04
    assert results['default_saturated_fraction'] == 0
    assert results['overexposed_saturated_fraction'] > .99
    assert results['noise_difference_std_dn'] > .5
    Image.fromarray(arrays['noisy'].astype('uint8')).save(out/'capture_4k.png')
    # Scientific comparison with identical display limits; never auto-normalize exposure cases.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 3, figsize=(12, 8))
    titles = [('sun_only', 'Sun only, 20 us'), ('sun_led', 'LED + sun, 20 us'),
              ('shadow_led', 'LED + 90% sunlight shadow'), ('exposure_10us', 'LED + sun, 10 us'),
              ('exposure_5us', 'LED + sun, 5 us'), ('noisy', 'LED + sun + sensor noise')]
    for ax, (name, title) in zip(axes.flat, titles):
        ax.imshow(arrays[name][::8, ::8], cmap='gray', vmin=0, vmax=255)
        ax.set_title(title); ax.axis('off')
    fig.tight_layout(); fig.savefig(out/'comparison.png', dpi=140); plt.close(fig)
    (out/'validation.json').write_text(json.dumps(results, indent=2)+'\n')
    print(json.dumps(results))


if __name__ == '__main__':
    main()

"""Image-estimated, fixed-plane line-scan calibration; no simulator optics input."""
import hashlib
import json
import numpy as np
from numpy.polynomial.polynomial import polyval, polyfit

SCHEMA = 'agv.linescan.measured_calibration.v1'


def capture_signature(config):
    # Exclude runtime gates/block size, retain all optical and radiometric settings.
    keys = ('width', 'pixel_pitch_m', 'focal_length_m', 'nominal_width_m',
            'exposure_s', 'ray_polynomial', 'camera_x_m', 'base_nominal_height_m',
            'led_forward_offset_m', 'led_height_m', 'radiometry')
    settings = {key: config[key] for key in keys}
    return hashlib.sha256(json.dumps(settings, sort_keys=True).encode()).hexdigest()


def mono(image, width=None):
    image = np.asarray(image)
    if image.dtype != np.uint8 or image.ndim != 2 or not image.size:
        raise ValueError('expected nonempty Mono8 image')
    if width is not None and image.shape[1] != width:
        raise ValueError('image width differs from calibration')
    return image


def flat_field(dark, bright):
    dark, bright = mono(dark), mono(bright)
    mono(bright, dark.shape[1])
    if min(len(dark), len(bright)) < 256:
        raise ValueError('at least 256 dark and bright lines required')
    offset = dark.mean(axis=0)
    signal = bright.mean(axis=0)-offset
    valid = (signal >= 16) & (np.mean(bright == 255, axis=0) < .001)
    if valid.mean() < .95:
        raise ValueError('flat field has too many weak or saturated columns')
    target = float(np.median(signal[valid]))
    gain = np.divide(target, signal, out=np.zeros_like(signal), where=valid)
    return dict(offset=offset.tolist(), gain=gain.tolist(), valid=valid.tolist(),
                target_signal_dn=target, dark_rows=len(dark), bright_rows=len(bright))


def stripe_centers(image):
    """Dark longitudinal stripes on a bright, uniform calibration board.

    Median rejects transverse crossing lines; thresholded connected groups are
    weighted by darkness. Target ordering/metric positions must be supplied.
    """
    a = np.asarray(image, dtype=float)
    if a.ndim != 2 or not np.isfinite(a).all():
        raise ValueError('invalid calibration image')
    profile = np.median(a, axis=0)
    lo, hi = np.percentile(profile, [1, 75])
    if hi-lo < 16:
        raise ValueError('insufficient calibration contrast')
    indices = np.flatnonzero(profile < lo+.4*(hi-lo))
    groups = np.split(indices, np.flatnonzero(np.diff(indices) > 1)+1)
    centers = []
    for g in groups:
        if len(g) >= 2 and g[0] > 0 and g[-1] < len(profile)-1:
            weights = hi-profile[g]
            centers.append(float(np.average(g, weights=weights)))
    return np.asarray(centers)


def geometry(raw_columns, target_across_m, width, output_width_m, degree=3):
    """Fit raw normalized column -> metric board coordinate at one fixed height.

    Does not separate focal length, mounting height and principal-point offset.
    No extrapolation past measured features is permitted in corrected output.
    """
    x, y = np.asarray(raw_columns, float), np.asarray(target_across_m, float)
    if (width < 2 or not np.isfinite(output_width_m) or output_width_m <= 0 or
            degree not in (1, 2, 3) or x.ndim != 1 or x.shape != y.shape or
            len(x) < degree+3 or not np.isfinite(x).all() or not np.isfinite(y).all() or
            np.any(np.diff(x) <= 0) or np.any(np.diff(y) <= 0) or x[0] < 0 or x[-1] >= width):
        raise ValueError('invalid or insufficient ordered metric correspondences')
    q = (x-(width-1)/2)/(width/2)
    coefficients = polyfit(q, y, degree)
    sensor_q = (np.arange(width)-(width-1)/2)/(width/2)
    mapping = polyval(sensor_q, coefficients)
    if np.any(np.diff(mapping) <= 0):
        raise ValueError('estimated mapping is not monotonic')
    output_y = sensor_q*output_width_m/2
    lookup = np.interp(output_y, mapping, np.arange(width))
    valid = (output_y >= y[0]) & (output_y <= y[-1]) & (lookup >= x[0]) & (lookup <= x[-1])
    residual = (polyval(q, coefficients)-y)/(output_width_m/width)
    if np.max(abs(residual)) > 1:
        raise ValueError('calibration fit residual exceeds one output pixel')
    return dict(metric_polynomial=coefficients.tolist(), lookup=lookup.tolist(),
                valid=valid.tolist(), output_width_m=output_width_m,
                measured_raw_columns=x.tolist(), target_across_m=y.tolist(),
                fit_error_px_max=float(np.max(abs(residual))),
                model='fixed_height_planar_raw_column_to_across_coordinate')


def make_profile(flat, optical, conditions, sources):
    width = len(flat['offset'])
    if len(optical['lookup']) != width:
        raise ValueError('calibration width mismatch')
    p = dict(schema=SCHEMA, width=width, flat=flat, geometry=optical,
             conditions=conditions, sources=sources)
    p['calibration_id'] = 'measured-'+hashlib.sha256(json.dumps(p, sort_keys=True).encode()).hexdigest()[:16]
    return p


class Correction:
    def __init__(self, profile):
        if profile['schema'] != SCHEMA:
            raise ValueError('unknown measured calibration schema')
        self.profile = profile
        self.width = int(profile['width'])
        f, g = profile['flat'], profile['geometry']
        self.offset, self.gain = np.asarray(f['offset'], np.float32), np.asarray(f['gain'], np.float32)
        source = np.asarray(g['lookup'], np.float64)
        arrays = [self.offset, self.gain, source, np.asarray(f['valid']), np.asarray(g['valid'])]
        if any(a.shape != (self.width,) or not np.isfinite(a).all() for a in arrays):
            raise ValueError('invalid calibration array')
        if np.any(self.gain < 0) or np.any(source < 0) or np.any(source > self.width-1) or np.any(np.diff(source) < 0):
            raise ValueError('invalid gain or lookup')
        self.left = np.floor(source).astype(np.intp)
        self.right = np.minimum(self.left+1, self.width-1)
        w = (source-self.left).astype(np.float32)
        fv = np.asarray(f['valid'], bool)
        self.valid = np.asarray(g['valid'], bool) & fv[self.left] & fv[self.right]
        # Fuse affine flat correction and interpolation; no intermediate clipping.
        self.a = (1-w)*self.gain[self.left]
        self.b = w*self.gain[self.right]
        self.c = -self.a*self.offset[self.left]-self.b*self.offset[self.right]

    def check_capture(self, config):
        if capture_signature(config) != self.profile['conditions']['capture_signature']:
            raise ValueError('optical / LED / exposure / sensor settings changed; recalibrate')

    def apply(self, image):
        image = mono(image, self.width)
        out = np.empty_like(image)
        saturated = 0
        for start in range(0, len(image), 128):
            raw = image[start:start+128]
            values = raw[:, self.left]*self.a+raw[:, self.right]*self.b+self.c
            invalid = (raw[:, self.left] == 255) | (raw[:, self.right] == 255) | ~self.valid
            saturated += int(np.count_nonzero((values >= 255) & ~invalid))
            values[invalid] = 0
            out[start:start+128] = np.rint(np.clip(values, 0, 255)).astype(np.uint8)
        return out, dict(valid_columns=self.valid.tolist(),
                        raw_saturated_pixels=int(np.count_nonzero(image == 255)),
                        corrected_clipped_pixels=saturated)

    def metadata(self, metadata):
        if metadata.get('width') != self.width or metadata.get('reference') != 'last_line_exposure_midpoint':
            raise ValueError('incompatible source metadata')
        conditions = self.profile['conditions']
        if metadata.get('exposure_s') != conditions['exposure_s'] or metadata.get('calibration_id') != conditions['source_calibration_id']:
            raise ValueError('capture conditions differ from calibration')
        if 'scene_backend' in conditions and metadata.get('scene_backend') != conditions['scene_backend']:
            raise ValueError('capture backend differs from calibration')
        if 'robot_source_sha256' in conditions and metadata.get('robot_contract',{}).get('source_sha256') != conditions['robot_source_sha256']:
            raise ValueError('robot mounting / geometry differs from calibration')
        return dict(metadata, source_calibration_id=metadata['calibration_id'],
                    calibration_id=self.profile['calibration_id'],
                    correction='column_dark_flat_then_horizontal_remap',
                    radiometric_zero='dark_subtracted', rows_preserved=True)

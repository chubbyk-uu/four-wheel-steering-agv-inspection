"""Calibrated metric interpretation of fixed wheel-shaft AB/rescaler phase."""
import math


def line_spacing(config, calibrated_radius):
    encoder = config.get('wheel_encoder')
    if encoder is None:
        return config['line_spacing_m']  # archived ideal-distance configuration
    for key in ('ppr', 'decode', 'multiplier', 'divider'):
        if type(encoder.get(key)) is not int:
            raise ValueError('encoder parameters must be integers')
    ppr, decode, multiplier, divider = (encoder[k] for k in ('ppr', 'decode', 'multiplier', 'divider'))
    if (encoder.get('wheel') not in ('fl', 'fr', 'rl', 'rr') or
            not 0 < ppr <= 1000000 or decode != 4 or
            not 0 < multiplier <= 128 or multiplier & (multiplier-1) or
            not 0 < divider <= 255 or not math.isfinite(calibrated_radius) or calibrated_radius <= 0):
        raise ValueError('invalid wheel encoder rescaler')
    return 2*math.pi*calibrated_radius/(ppr*decode*multiplier/divider)

"""Physical apron and optical tile bounds, independent of rendering/resources."""
import math

CORE = 2048
GUTTER = 2
TEXEL = .00025
TILE_METRES = CORE * TEXEL


def layout(length, end_buffer=0., side_buffer=0.):
    if not all(math.isfinite(v) and v >= 0 for v in (length, end_buffer, side_buffer)) or length <= 0:
        raise ValueError('invalid road dimensions')
    # Preserve the existing full-width 20m tile layout when buffers are zero.
    ox = -math.ceil((end_buffer + 1.024) / TILE_METRES) * TILE_METRES
    half_y = math.ceil((5 + side_buffer + 1.024) / TILE_METRES) * TILE_METRES
    nx = math.ceil((length + end_buffer + 1.024 - ox) / TILE_METRES)
    ny = round(2 * half_y / TILE_METRES)
    return dict(inspection_bounds_xy_m=[0., float(length), -5., 5.],
                drivable_bounds_xy_m=[-end_buffer, length + end_buffer, -5-side_buffer, 5+side_buffer],
                optical_valid_bounds_xy_m=[ox, ox+nx*TILE_METRES, -half_y, half_y],
                tiles_x=nx, tiles_y=ny, core=CORE, gutter=GUTTER, texel_m=TEXEL,
                texture_bytes=nx*ny*(CORE+2*GUTTER)**2*3)


def display_regions(length, bounds):
    x0,x1,y0,y1=bounds
    # Keep the detailed cracks wholly inside their original 5m slab partitions.
    # Split apron too, so large buffers never create oversized display textures.
    cuts=[x0]+[float(x) for x in range(math.ceil(x0/5)*5,math.ceil(x1/5)*5,5) if x0<x<x1]+[x1]
    return [(lo,hi,by,ey) for lo,hi in zip(cuts[:-1],cuts[1:]) for by,ey in ((y0,0.),(0.,y1))]

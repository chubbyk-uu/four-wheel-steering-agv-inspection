"""Flat-ground projection of continuous scan lines, without truth pose inputs."""
import numpy as np


def line_times(metadata):
    anchors={t['global_line']:t['time_s'] for t in metadata['pose_tags']}
    for key in ('first','last'):
        tag=metadata[key];anchors[tag['global_line']]=tag['time_s']
    lines=np.array(sorted(anchors));times=np.array([anchors[n] for n in lines])
    first=metadata['first']['global_line'];last=metadata['last']['global_line']
    if (last-first+1!=metadata['rows'] or lines[0]!=first or lines[-1]!=last or
            not np.isfinite(times).all() or np.any(np.diff(times)<=0)):
        raise ValueError('invalid row/time anchors')
    return np.interp(np.arange(first,last+1),lines,times)


def project_flat(positions,body_rotations,offset,mount,rays,ground_z=0.):
    """Each row has one pose; rays are calibrated camera-frame directions."""
    positions=np.asarray(positions,float);rays=np.asarray(rays,float)
    if positions.ndim!=2 or positions.shape[1]!=3 or rays.ndim!=2 or rays.shape[1]!=3:
        raise ValueError('invalid position/ray shape')
    origins=positions+body_rotations.apply(offset)
    rotations=(body_rotations*mount).as_matrix()
    directions=np.einsum('rij,uj->rui',rotations,rays)
    dz=directions[:,:,2]
    if (not np.isfinite(origins).all() or not np.isfinite(directions).all() or
            not np.isfinite(ground_z) or np.any(dz>=-1e-8) or np.any(origins[:,2]<=ground_z)):
        raise ValueError('rays do not intersect ground below camera')
    distance=(ground_z-origins[:,2,None])/dz
    return origins[:,None,:2]+distance[:,:,None]*directions[:,:,:2]


def navigation_at(navigation,times):
    """Interpolate archived fused poses; reject missing navigation brackets."""
    times=np.asarray(times);indices=np.searchsorted(navigation.times,times)
    if np.any(indices==0) or np.any(indices==len(navigation.times)):
        raise ValueError('navigation extrapolation')
    if np.any(navigation.times[indices]-navigation.times[indices-1]>.06):
        raise ValueError('navigation gap')
    positions=np.column_stack([np.interp(times,navigation.times,navigation.positions[:,i]) for i in range(3)])
    return positions,navigation.rotations(times)

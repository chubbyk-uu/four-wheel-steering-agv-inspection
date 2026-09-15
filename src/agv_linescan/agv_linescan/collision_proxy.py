"""Explicit, bounded physical proxy; optical meshes remain authoritative for imaging."""
from pathlib import Path
import hashlib
import numpy as np


def mesh_arrays(path):
    from .obj_arrays import read_obj
    v,f,_=read_obj(path)
    return v,f


def shallow_rectangle(v,f,z,limit):
    if not np.isfinite([z,limit]).all() or not 0<limit<=.003:
        raise ValueError('collision proxy depth bound must be at most 3 mm')
    if np.any(v[:,2]>z+1e-8) or np.any(v[:,2]<z-limit-1e-8):
        raise ValueError('surface outside collision proxy depth bound')
    lo=v[:,:2].min(axis=0);hi=v[:,:2].max(axis=0)
    d=v[f[:,1:],:2]-v[f[:,:1],:2]
    area=(d[:,0,0]*d[:,1,1]-d[:,0,1]*d[:,1,0])/2
    if not np.all(area>0) or not np.isclose(area.sum(),np.prod(hi-lo),rtol=1e-8,atol=1e-10):
        raise ValueError('collision proxy needs upward rectangular height surface')
    # Limit this first policy to sparse shallow defects, not flattened sloping/undulating land.
    changed=(v[f,2].min(axis=1)<z-1e-8)
    fraction=float(area[changed].sum()/area.sum())
    if fraction>.01:
        raise ValueError('collision proxy would flatten more than 1% of surface')
    return lo,hi,fraction


def validate_proxy(root,asset,arrays=None):
    proxy=asset['collision_proxy']
    if proxy.get('method')=='layered_heightfield_shallow_v1':
        return validate_layered_proxy(root,asset,arrays)
    if proxy.get('method')!='shallow_horizontal_rectangle_v1':
        raise ValueError('unknown collision proxy policy')
    path=(root/proxy['mesh']).resolve()
    if path.parent!=root or hashlib.sha256(path.read_bytes()).hexdigest()!=proxy['sha256']:
        raise ValueError('collision proxy checksum mismatch')
    v,f=mesh_arrays(root/asset['mesh']) if arrays is None else arrays
    z=proxy['plane_z_m'];limit=proxy['max_surface_deviation_m']
    lo,hi,_=shallow_rectangle(v,f,z,limit)
    pv,pf=mesh_arrays(path)
    expected=np.array([[lo[0],lo[1],z],[hi[0],lo[1],z],[hi[0],hi[1],z],[lo[0],hi[1],z]])
    if pv.shape!=(4,3) or not np.allclose(pv,expected,rtol=0,atol=1e-9) or not np.array_equal(pf,[[0,1,2],[0,2,3]]):
        raise ValueError('collision proxy footprint/plane mismatch')
    return path


def validate_layered_proxy(root,asset,arrays=None):
    """Keep source shallow defects while adding a checked common heightfield.

    Exact at optical vertices. Face-interior approximation is separately sampled;
    it is not claimed that two different triangulations coincide everywhere.
    """
    from .heightfield import Heightfield
    proxy=asset['collision_proxy']
    def checked(entry):
        path=(root/entry['mesh']).resolve()
        if path.parent!=root or hashlib.sha256(path.read_bytes()).hexdigest()!=entry['sha256']:raise ValueError('layered proxy source checksum mismatch')
        return path
    source=checked(proxy['reference_surface']);field=Heightfield(checked(proxy['heightfield']))
    reference,faces=mesh_arrays(source)
    lo,hi,_=shallow_rectangle(reference,faces,proxy['reference_plane_z_m'],proxy['max_surface_deviation_m'])
    v,f=mesh_arrays(root/asset['mesh']) if arrays is None else arrays
    expected=reference.copy();expected[:,2]+=field.sample(expected[:,:2])
    if v.shape!=expected.shape or not np.array_equal(f,faces) or not np.allclose(v,expected,atol=1.1e-9,rtol=0):raise ValueError('optical surface does not preserve source plus heightfield')
    path=checked(proxy);pv,pf=mesh_arrays(path);ev,ef=field.mesh(lo,hi)
    ev[:,2]+=proxy['reference_plane_z_m']
    if pv.shape!=ev.shape or not np.allclose(pv,ev,atol=1.1e-9,rtol=0) or not np.array_equal(pf,ef):raise ValueError('collision mesh differs from checked heightfield')
    # Check the base displacement interpolation (defect depth is preserved separately).
    displacement=v[:,2]-reference[:,2];max_error=0.
    for weights in ((.5,.5,0),(.5,0,.5),(0,.5,.5),(1/3,1/3,1/3)):
        xy=np.einsum('nki,k->ni',reference[f,:2],weights)
        predicted=displacement[f]@np.asarray(weights)
        max_error=max(max_error,float(np.max(abs(predicted-field.sample(xy)))))
    if max_error>.00015:raise ValueError('optical heightfield interpolation discrepancy exceeds 0.15 mm sample budget')
    return path

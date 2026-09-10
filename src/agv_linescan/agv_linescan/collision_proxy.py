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

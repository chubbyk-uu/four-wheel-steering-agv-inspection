"""Explicit, bounded physical proxy; optical meshes remain authoritative for imaging."""
from pathlib import Path
import hashlib
import numpy as np


def mesh_arrays(path):
    vertices=[];faces=[]
    for line in Path(path).read_text().splitlines():
        p=line.split()
        if not p: continue
        if p[0]=='v': vertices.append([float(x) for x in p[1:4]])
        if p[0]=='f':
            if len(p)!=4: raise ValueError('collision proxy requires triangular surface')
            faces.append([int(x.split('/')[0])-1 for x in p[1:]])
    v=np.asarray(vertices,dtype=float);f=np.asarray(faces,dtype=int)
    if v.ndim!=2 or v.shape[1]!=3 or not np.isfinite(v).all() or f.ndim!=2 or f.shape[1]!=3 or f.min()<0 or f.max()>=len(v):
        raise ValueError('invalid collision proxy mesh')
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


def validate_proxy(root,asset):
    proxy=asset['collision_proxy']
    if proxy.get('method')!='shallow_horizontal_rectangle_v1':
        raise ValueError('unknown collision proxy policy')
    path=(root/proxy['mesh']).resolve()
    if path.parent!=root or hashlib.sha256(path.read_bytes()).hexdigest()!=proxy['sha256']:
        raise ValueError('collision proxy checksum mismatch')
    v,f=mesh_arrays(root/asset['mesh'])
    z=proxy['plane_z_m'];limit=proxy['max_surface_deviation_m']
    lo,hi,_=shallow_rectangle(v,f,z,limit)
    pv,pf=mesh_arrays(path)
    expected=np.array([[lo[0],lo[1],z],[hi[0],lo[1],z],[hi[0],hi[1],z],[lo[0],hi[1],z]])
    if pv.shape!=(4,3) or not np.allclose(pv,expected,rtol=0,atol=1e-9) or not np.array_equal(pf,[[0,1,2],[0,2,3]]):
        raise ValueError('collision proxy footprint/plane mismatch')
    return path

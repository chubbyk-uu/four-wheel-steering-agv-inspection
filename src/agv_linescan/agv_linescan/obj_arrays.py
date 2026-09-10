"""Vectorized reading of the project's triangular OBJ assets; no validation cache."""
from pathlib import Path
import re
import numpy as np


try:
    from ._obj_arrays import read_obj as _native_read
except ImportError:
    _native_read=None


def read_obj(path,with_uv=False):
    if _native_read is not None:return _native_read(str(path),with_uv)
    return read_obj_python(path,with_uv)


def read_obj_python(path,with_uv=False):
    data=Path(path).read_bytes()
    def coordinates(tag,columns):
        lines=re.findall(rb'^'+tag+rb'[ \t]+([^\r\n]+)',data,re.M)
        # Optional trailing OBJ components are irrelevant to XYZ / UV.
        fields=[b' '.join(line.split()[:columns]) for line in lines]
        values=np.fromstring(b'\n'.join(fields).decode('ascii'),sep=' ')
        if values.size!=len(lines)*columns:raise ValueError('invalid OBJ coordinate array')
        return values.reshape(-1,columns)
    vertices=coordinates(b'v',3)
    lines=re.findall(rb'^f[ \t]+([^\r\n]+)',data,re.M)
    if any(len(line.split())!=3 for line in lines):raise ValueError('collision proxy requires triangular surface')
    faces_data=b'\n'.join(lines)
    indices=np.fromstring(re.sub(rb'/[^ \t\r\n]+',b'',faces_data).decode('ascii'),dtype=np.int64,sep=' ')
    if not len(vertices) or indices.size!=3*len(lines) or not len(indices) or not np.isfinite(vertices).all() or indices.min()<1 or indices.max()>len(vertices):
        raise ValueError('invalid collision proxy mesh')
    faces=indices.reshape(-1,3)-1
    uv=None
    if with_uv:
        # Require every corner to have an explicit vertex/UV pair. Empty UVs,
        # malformed tokens and mismatched indices cannot pass as plain faces.
        corners=re.findall(rb'(?<!\S)([1-9]\d*)/([1-9]\d*)(?:/-?\d+)?(?!\S)',faces_data)
        if len(corners)!=indices.size or any(v!=t for v,t in corners):raise ValueError('display mesh face UV indices mismatch')
        uv=coordinates(b'vt',2)
    return vertices,faces,uv

#!/usr/bin/env python3
"""Closed convex shell: front/rear roof slopes and inward-sloping side shoulders."""
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull

root = Path(__file__).resolve().parents[1]
vertices = np.array(
    [(x,y,z) for z in (.08,.23) for x in (-.95,.95) for y in (-.55,.55)] +
    [(x,y,z) for x,z,w in [(-.95,.29,.47),(-.48,.425,.38),(.48,.425,.38),(.95,.29,.47)] for y in (-w,w)])
hull = ConvexHull(vertices)
lines = ['mtllib large_body.mtl', 'usemtl body_orange']
lines += ['v '+' '.join(f'{x:.7f}' for x in v) for v in vertices]
faces=[]
for ids, plane in zip(hull.simplices, hull.equations):
    a,b,c=map(int,ids)
    normal=np.cross(vertices[b]-vertices[a],vertices[c]-vertices[a])
    if np.dot(normal,plane[:3])<0: b,c=c,b
    faces.append((a,b,c))
    lines.append('vn '+' '.join(f'{x:.9f}' for x in plane[:3]))
lines += [f'f {a+1}//{i} {b+1}//{i} {c+1}//{i}' for i,(a,b,c) in enumerate(faces,1)]
(root/'src/agv_description/meshes/large_body.obj').write_text('\n'.join(lines)+'\n')
print(f'Closed shell: {len(vertices)} vertices, {len(faces)} triangles')

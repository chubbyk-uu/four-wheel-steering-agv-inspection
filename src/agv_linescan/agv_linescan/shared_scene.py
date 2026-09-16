"""Versioned shared static geometry for GZ visual/collision and OptiX readers."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
from .collision_proxy import validate_proxy
from .obj_arrays import read_obj


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_spawn_position(scene, x, y, radius=1.5):
    """Keep the audited large-AGV envelope on the declared drivable surface."""
    bounds=scene.get('drivable_bounds_xy_m',scene.get('inspection_bounds_xy_m',
        [0.,scene['length_m'],-scene['width_m']/2,scene['width_m']/2]))
    if len(bounds)!=4 or not np.isfinite([x,y,radius,*bounds]).all() or radius<=0:
        raise ValueError('invalid shared-scene spawn bounds')
    x0,x1,y0,y1=bounds
    if not (x0+radius<=x<=x1-radius and y0+radius<=y<=y1-radius):
        raise ValueError('spawn vehicle envelope leaves the declared drivable terrain')


def calibration_scene(directory, base_world, phase=None, width=2.):
    """Coplanar segmented diffuse board, same mesh/material values for GZ and RT.

    Stripes are geometry partitions, not raised bars or shader-only targets.
    No optical model is used to construct the known 50 mm target coordinates.
    """
    if not np.isfinite(width) or width<2:raise ValueError('calibration board width must be at least 2 m')
    directory=Path(directory).resolve();directory.mkdir(parents=True,exist_ok=False)
    half=width/2;cuts=[-half,half]
    if phase is not None:
        count=int(np.ceil(half/.05))+1
        for center in np.arange(-count,count+1)*.05+phase:
            cuts.extend(x for x in (center-.001,center+.001) if -half<x<half)
    cuts=sorted(set(cuts));faces={'white':[], 'black':[]}
    for lo,hi in zip(cuts,cuts[1:]):
        dark=phase is not None and abs(((lo+hi)/2-phase+.025)%.05-.025)<.001000001
        faces['black' if dark else 'white'].append((lo,hi))
    tree=ET.parse(base_world);world=tree.getroot().find('world')
    for m in list(world.findall('model')):world.remove(m)
    assets=[]
    for name,intervals in faces.items():
        if not intervals:continue
        path=directory/(name+'.obj');lines=['vn 0 0 1']
        for i,(lo,hi) in enumerate(intervals):
            lines.extend([f'v 0 {lo:.12f} 0',f'v 10 {lo:.12f} 0',f'v 10 {hi:.12f} 0',f'v 0 {hi:.12f} 0',
                          f'f {4*i+1}//1 {4*i+2}//1 {4*i+3}//1',f'f {4*i+1}//1 {4*i+3}//1 {4*i+4}//1'])
        path.write_text('\n'.join(lines)+'\n');value=(25 if name=='black' else 200)/255
        assets.append(dict(name=name,mesh=path.name,sha256=digest(path),triangles=2*len(intervals),linear_reflectance=value))
        model=ET.SubElement(world,'model',name=name);ET.SubElement(model,'static').text='true';link=ET.SubElement(model,'link',name='link')
        for kind in ('visual','collision'):
            item=ET.SubElement(link,kind,name=kind);mesh=ET.SubElement(ET.SubElement(item,'geometry'),'mesh')
            ET.SubElement(mesh,'uri').text=str(path);ET.SubElement(mesh,'scale').text='1 1 1'
            if kind=='visual':
                mat=ET.SubElement(item,'material')
                for tag in ('ambient','diffuse'):ET.SubElement(mat,tag).text=f'{value} {value} {value} 1'
    tree.write(directory/'world.sdf',encoding='unicode')
    m=dict(schema='agv.shared.static_scene.v1',frame='world',units='m',transform='identity_world_baked',
           profile='calibration_board',length_m=10,width_m=width,assets=assets,world='world.sdf',
           world_sha256=digest(directory/'world.sdf'),display_materials={},stripe_phase_m=phase)
    (directory/'manifest.json').write_text(json.dumps(m,indent=2)+'\n')
    validate(directory/'manifest.json')
    return directory/'manifest.json'


def generate(directory, base_world, length=100., width=10., profile="flat", margin=2.):
    if profile not in ("flat", "optical_stress"):
        raise ValueError("unknown scene profile")
    if not np.isfinite(margin) or not 0 <= margin <= 10:
        raise ValueError('invalid ground margin')
    directory = Path(directory).resolve()
    if not (10 <= length <= 200 and 2 <= width <= 30):
        raise ValueError('scene extent out of prototype bounds')
    directory.mkdir(parents=True, exist_ok=False)
    # Display-only regular grid; OptiX retains its procedural linear-reflectance grid.
    # Material equality is NOT implied by the shared geometry schema.
    grid=np.full((256,256),210,np.uint8);grid[:3,:]=25;grid[-3:,:]=25;grid[:,:3]=25;grid[:,-3:]=25
    Image.fromarray(grid).save(directory/'grid.png')
    (directory/'grid.mtl').write_text('newmtl grid\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nmap_Kd grid.png\n')
    assets=[]
    def mesh(name, vertices, faces):
        vertices=np.asarray(vertices,dtype=np.float32);faces=np.asarray(faces,dtype=np.int64)
        normals=np.cross(vertices[faces[:,1]]-vertices[faces[:,0]],vertices[faces[:,2]]-vertices[faces[:,0]])
        normals/=np.linalg.norm(normals,axis=1)[:,None]
        path=directory/(name+'.obj')
        with path.open('w') as f:
            f.write('mtllib grid.mtl\nusemtl grid\n')
            for v in vertices: f.write('v '+' '.join(format(float(x),'.9g') for x in v)+'\n')
            for v in vertices: f.write(f'vt {float(v[0])*10:.9g} {float(v[1])*10:.9g}\n')
            for n in normals: f.write('vn '+' '.join(format(float(x),'.9g') for x in n)+'\n')
            for j,face in enumerate(faces): f.write('f '+' '.join(f'{i+1}/{i+1}/{j+1}' for i in face)+'\n')
        assets.append(dict(name=name,mesh=path.name,sha256=digest(path),triangles=len(faces)))
    nx,ny=round(length/.1),round(width/.1)
    x,y=np.meshgrid(np.linspace(0,length,nx+1),np.linspace(-width/2,width/2,ny+1),indexing='ij')
    z=np.zeros_like(x)
    if profile == "optical_stress":
        # Explicit optical regression fixture, not the driving acceptance surface.
        z=.025*np.sin(x*.7)*np.cos(y*1.7)-.10*np.exp(-((x-5)**2+y*y)/.08)
        z+=np.clip(x-7,0,2)*.03
    a=(np.arange(nx)[:,None]*(ny+1)+np.arange(ny)[None,:]).ravel()
    faces=np.stack([np.stack([a,a+ny+1,a+ny+2],1),np.stack([a,a+ny+2,a+1],1)],1).reshape(-1,3)
    vertices=np.stack([x,y,z],-1).reshape(-1,3)
    if margin and profile == "flat":
        inner=np.array([[0,-width/2,0],[length,-width/2,0],[length,width/2,0],[0,width/2,0]])
        outer=inner+np.array([[-margin,-margin,0],[margin,-margin,0],[margin,margin,0],[-margin,margin,0]])
        start=len(vertices)
        ring=[]
        for i in range(4):
            j=(i+1)%4
            ring.extend([[start+i,start+4+i,start+4+j],[start+i,start+4+j,start+j]])
        vertices=np.concatenate([vertices,inner,outer])
        faces=np.concatenate([faces,np.array(ring)])
    mesh('terrain',vertices,faces)
    def box(name,lo,hi):
        x0,y0,z0=lo;x1,y1,z1=hi
        v=[(x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0),
           (x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)]
        f=[(0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),
           (1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7)]
        mesh(name,v,f)
    for i,sx in enumerate(np.arange(5.12,length-1,9) if profile == "optical_stress" else []):
        box(f'screen_{i}',(sx-.01,-.25,-.15),(sx+.01,.25,.25))
        box(f'canopy_{i}',(sx+.4,-.3,.15),(sx+.65,.3,.17))
    tree=ET.parse(base_world);world=tree.getroot().find('world')
    for m in list(world.findall('model')):world.remove(m)
    for a in assets:
        m=ET.SubElement(world,'model',name=a['name']);ET.SubElement(m,'static').text='true'
        link=ET.SubElement(m,'link',name='link')
        for kind in ('visual','collision'):
            item=ET.SubElement(link,kind,name=kind)
            geo=ET.SubElement(ET.SubElement(item,'geometry'),'mesh')
            ET.SubElement(geo,'uri').text=str(directory/a['mesh'])
            ET.SubElement(geo,'scale').text='1 1 1'
    tree.write(directory/'world.sdf',encoding='unicode')
    manifest=dict(schema='agv.shared.static_scene.v1',frame='world',units='m',
        transform='identity_world_baked',profile=profile,length_m=length,width_m=width,assets=assets,
        world='world.sdf',world_sha256=digest(directory/'world.sdf'),
        display_materials={n:digest(directory/n) for n in ('grid.png','grid.mtl')},
        material_scope='geometry shared exactly; GZ display texture and OptiX procedural reflectance are not photometrically matched')
    if profile == "flat":
        manifest['inspection_bounds_xy_m']=[0.,length,-width/2,width/2]
        manifest['ground_bounds_xy_m']=[-margin,length+margin,-width/2-margin,width/2+margin]
        manifest['ground_margin_m']=margin
    (directory/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    validate(directory/'manifest.json')
    return manifest


def validate_display_uv(asset,manifest,vertices,uv):
    projection=asset.get('display_uv_projection',manifest.get('display_uv_projection'))
    origin=np.asarray(projection['origin_xy_m']);span=np.asarray(projection['span_xy_m'])
    if origin.shape!=(2,) or span.shape!=(2,) or not np.isfinite(origin).all() or not np.isfinite(span).all() or not np.all(span>0):
        raise ValueError('invalid display UV projection')
    expected_uv=(vertices[:,:2]-origin)/span
    direction=projection.get('v_direction','increasing_world_y')
    if direction=='decreasing_world_y':expected_uv[:,1]=1-expected_uv[:,1]
    elif direction!='increasing_world_y':raise ValueError('unknown display V direction')
    if len(uv)!=len(vertices) or not np.allclose(uv,expected_uv,rtol=0,atol=1e-8):
        raise ValueError('display mesh UV/world-coordinate mismatch')

def validate_material_height_bounds(material,vertices):
    """Match the OptiX streaming height guard before launching the simulator."""
    if material['schema'] not in ('agv.ground_material.tiles.v1','agv.ground_material.recipe.v1'):
        return
    bounds=np.asarray(material['height_bounds_m'],float)
    if (bounds.shape!=(2,) or not np.isfinite(bounds).all() or bounds[0]>bounds[1]
            or np.any(vertices[:,2]<bounds[0]-1e-7) or np.any(vertices[:,2]>bounds[1]+1e-7)):
        raise ValueError('textured mesh violates prefetch height bounds')


def validate(manifest):
    path=Path(manifest).resolve();m=json.loads(path.read_text());root=path.parent
    if m['schema']!='agv.shared.static_scene.v1' or m['frame']!='world' or m['units']!='m' or m['transform']!='identity_world_baked':
        raise ValueError('unsupported scene contract')
    assets={a['name']:a for a in m['assets']}
    if len(assets)!=len(m['assets']):raise ValueError('duplicate asset')
    for a in assets.values():
        p=root/a['mesh']
        if p.parent.resolve()!=root or digest(p)!=a['sha256']:raise ValueError('mesh checksum mismatch')
        if 'linear_reflectance' in a and not 0<=a['linear_reflectance']<=1:raise ValueError('invalid diffuse reflectance')
    if digest(root/m['world'])!=m['world_sha256']:raise ValueError('world checksum mismatch')
    for name,h in m['display_materials'].items():
        if digest(root/name)!=h:raise ValueError('display material checksum mismatch')
    world=ET.parse(root/m['world']).getroot().find('world');models=world.findall('model')
    heightmap=m.get('physics_heightmap')
    heightmap_name=heightmap.get('model_name') if heightmap else None
    expected_models=set(assets)|({heightmap_name} if heightmap_name else set())
    if {v.get('name') for v in models}!=expected_models or len(models)!=len(expected_models):raise ValueError('scene model mismatch')
    if heightmap:
        image=(root/heightmap['image']).resolve();source=(root/heightmap['source_heightfield']).resolve()
        if image.parent!=root or source.parent!=root or digest(image)!=heightmap['sha256'] or digest(source)!=heightmap['source_sha256']:
            raise ValueError('physics heightmap checksum mismatch')
        if heightmap.get('encoding')!='png_uint16_min_to_max_v1':raise ValueError('unsupported physics heightmap encoding')
        if len(heightmap.get('size_m',[]))!=3 or len(heightmap.get('position_m',[]))!=3:raise ValueError('invalid physics heightmap geometry')
        if world.findtext('physics/dart/collision_detector')!=heightmap.get('collision_detector'):
            raise ValueError('physics heightmap collision detector mismatch')
    for model in models:
        if model.findtext('static')!='true' or len(model.findall('link'))!=1:raise ValueError('static scene required')
        for pose in model.iter('pose'):
            if any(float(x)!=0 for x in pose.text.split()):raise ValueError('unexpected scene transform')
        if model.get('name')==heightmap_name:
            link=model.find('link')
            if link.findall('visual') or len(link.findall('collision'))!=1:raise ValueError('heightmap must be collision-only')
            shape=link.find('collision/geometry/heightmap')
            if shape is None or Path(shape.findtext('uri')).resolve()!=image:raise ValueError('physics heightmap URI mismatch')
            if not np.allclose([float(x) for x in shape.findtext('size').split()],heightmap['size_m'],rtol=0,atol=1e-11) or not np.allclose([float(x) for x in shape.findtext('pos').split()],heightmap['position_m'],rtol=0,atol=1e-11):
                raise ValueError('physics heightmap transform mismatch')
            continue
        link=model.find('link');asset=assets[model.get('name')]
        if 'collision_proxy' in asset or ('ground_material' in m and asset.get('material')=='ground'):
            mesh_data=read_obj(root/asset['mesh'],with_uv='ground_material' in m and asset.get('material')=='ground')
            if 'ground_material' in m and asset.get('material')=='ground':
                validate_display_uv(asset,m,mesh_data[0],mesh_data[2])
                validate_material_height_bounds(m['ground_material'],mesh_data[0])
        collision_path=validate_proxy(root,asset,mesh_data[:2]) if 'collision_proxy' in asset else (root/asset['mesh']).resolve()
        for kind in ('visual','collision'):
            items=link.findall(kind)
            expected_count=0 if kind=='collision' and heightmap else 1
            if len(items)!=expected_count:raise ValueError('geometry count mismatch')
            if expected_count==0:continue
            geo=items[0].find('geometry/mesh')
            expected=collision_path if kind=='collision' else (root/asset['mesh']).resolve()
            if geo is None or Path(geo.findtext('uri')).resolve()!=expected or geo.findtext('scale')!='1 1 1':
                raise ValueError('visual/collision geometry mismatch')
            if kind=='visual' and 'linear_reflectance' in asset:
                expected=[asset['linear_reflectance']]*3+[1.]
                if [float(x) for x in items[0].findtext('material/diffuse','').split()]!=expected:
                    raise ValueError('diffuse material mismatch')
    if 'ground_material' in m:
        material=m['ground_material']
        if material['schema']=='agv.ground_material.xy.v1':
            w,h=material['width'],material['height']
            if not (0<w<=32768 and 0<h<=32768):raise ValueError('invalid material resolution')
            for field,channels in (('color',1),('normal',2),('roughness_map',1)):
                if field not in material:continue
                entry=material[field];file=root/entry['file']
                if file.parent.resolve()!=root or file.stat().st_size!=w*h*channels or digest(file)!=entry['sha256']:
                    raise ValueError('ground material integrity mismatch')
        elif material['schema']=='agv.ground_material.tiles.v1':
            nx,ny=material['tiles_x'],material['tiles_y'];stride=material['core_pixels']+2*material['gutter_pixels'];seen=set()
            for tile in material['tiles']:
                key=(tile['ix'],tile['iy'])
                if key in seen or not (0<=key[0]<nx and 0<=key[1]<ny):raise ValueError('invalid/duplicate tile key')
                seen.add(key)
                for field,channels in (('color',1),('normal',2)):
                    entry=tile[field];file=root/entry['file']
                    if file.parent.resolve()!=root or file.stat().st_size!=stride*stride*channels or len(entry['sha256'])!=64:
                        raise ValueError('tiled material file contract mismatch')
            if len(seen)!=nx*ny:raise ValueError('missing material tile entry')
            # Content hashes are checked in the loader before each tile becomes GPU-ready.
        elif material['schema']=='agv.ground_material.recipe.v1':
            entry=material['recipe'];file=root/entry['file']
            if file.parent.resolve()!=root or digest(file)!=entry['sha256']:raise ValueError('runtime recipe integrity mismatch')
            recipe=json.loads(file.read_text())
            if recipe['schema']!='agv.material.recipe.probe.v1':raise ValueError('unsupported runtime recipe')
            for a,b in [('tiles_x','nx'),('tiles_y','ny'),('core_pixels','core'),('gutter_pixels','gutter'),('texel_m','texel')]:
                if material[a]!=recipe[b]:raise ValueError('runtime recipe grid mismatch')
            if material['origin_xy_m']!=[recipe['ox'],recipe['oy']]:raise ValueError('runtime recipe origin mismatch')
            for name,h in recipe['payload_sha256'].items():
                payload=root/name
                if payload.parent.resolve()!=root or digest(payload)!=h:raise ValueError('runtime recipe payload integrity mismatch')
        else:raise ValueError('unsupported ground material')
        for model in models:
            if model.get('name')==heightmap_name:continue
            asset=assets[model.get('name')]
            if asset.get('material')!='ground':continue
            pbr=model.find('link/visual/material/pbr/metal')
            if pbr is None or float(pbr.findtext('roughness','-1'))!=material.get('roughness',.60):
                raise ValueError('GZ/OptiX roughness mismatch')
            for tag in ('albedo_map','normal_map'):
                file=Path(pbr.findtext(tag,''))
                if file.parent.resolve()!=root or file.name not in m['display_materials']:
                    raise ValueError('GZ material must reference checked shared display assets')
    return m

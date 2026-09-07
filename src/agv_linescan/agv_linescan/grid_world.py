"""Generate the GZ visual counterpart of the analytic sensor's grid plane."""
import xml.etree.ElementTree as ET


def write_grid_world(source, destination, config, probe=False):
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    model = ET.SubElement(world, 'model', name='calibration_grid')
    ET.SubElement(model, 'static').text = 'true'
    link = ET.SubElement(model, 'link', name='grid')
    extent, step = config['grid_extent_m'], config['grid_spacing_m']
    half = int(extent/(2*step))
    # Combine painted grid strips into a single mesh / draw object. Hundreds
    # of individual box visuals dominate narrow-line rendering on D3D12.
    from pathlib import Path
    mesh_path = Path(destination).with_suffix('.grid.obj')
    material_path = mesh_path.with_suffix('.mtl')
    material_path.write_text('newmtl grid_paint\nKa 0.05 0.05 0.05\nKd 0.05 0.05 0.05\nKs 0 0 0\nNs 1\nd 1\nillum 2\n')
    vertices, faces = [], []
    for axis in (0, 1):
        for i in range(-half, half+1):
            bounds = [[-extent/2, extent/2], [-extent/2, extent/2]]
            bounds[axis] = [i*step-config['grid_line_width_m']/2,
                            i*step+config['grid_line_width_m']/2]
            (x0, x1), (y0, y1) = bounds
            offset = len(vertices)+1
            vertices.extend([(x0,y0,.0003), (x1,y0,.0003),
                             (x1,y1,.0003), (x0,y1,.0003)])
            faces.extend([(offset,offset+1,offset+2),(offset,offset+2,offset+3)])
    with mesh_path.open('w') as mesh:
        mesh.write('mtllib '+material_path.name+'\nusemtl grid_paint\nvn 0 0 1\n')
        for vertex in vertices:
            mesh.write('v '+' '.join(map(str, vertex))+'\n')
        for face in faces:
            mesh.write('f '+' '.join(f'{index}//1' for index in face)+'\n')
    visual = ET.SubElement(link, 'visual', name='painted_grid')
    ET.SubElement(visual, 'cast_shadows').text = 'false'
    geometry = ET.SubElement(visual, 'geometry')
    ET.SubElement(ET.SubElement(geometry, 'mesh'), 'uri').text = str(mesh_path.resolve())
    material = ET.SubElement(visual, 'material')
    for tag in ('ambient', 'diffuse'):
        ET.SubElement(material, tag).text = '.05 .05 .05 1'
    if probe:
        marker = ET.SubElement(world, 'model', name='render_occlusion_probe')
        ET.SubElement(marker, 'static').text = 'true'
        ET.SubElement(marker, 'pose').text = '1.2 0.5 0.03 0 0 0'
        visual = ET.SubElement(ET.SubElement(marker, 'link', name='link'), 'visual', name='red_plate')
        ET.SubElement(ET.SubElement(ET.SubElement(visual, 'geometry'), 'box'), 'size').text = '.2 .2 .06'
        material = ET.SubElement(visual, 'material')
        for tag in ('ambient', 'diffuse'):
            ET.SubElement(material, tag).text = '.8 .02 .02 1'
    tree.write(destination, encoding='unicode')


def write_tiled_world(source, destination, manifest):
    """Coarse GUI overview only; CUDA samples the full-resolution disk tiles."""
    import json
    from pathlib import Path
    path = Path(manifest).resolve()
    c = json.loads(path.read_text())
    if c['schema'] != 'agv.terrain.tiles.v1' or not (path.parent/'overview.png').is_file():
        raise ValueError('terrain manifest or GUI overview missing')
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    model = ET.SubElement(world, 'model', name='runway_overview')
    ET.SubElement(model, 'static').text = 'true'
    # Explicit texture coordinates: image top = +world Y.
    ox, oy, length, width = (c[k] for k in ('origin_x_m', 'origin_y_m', 'length_m', 'width_m'))
    mesh_path = Path(destination).with_suffix('.runway.obj')
    mtl_path = mesh_path.with_suffix('.mtl')
    mtl_path.write_text('newmtl runway\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nillum 1\nmap_Kd '+str(path.parent/'overview.png')+'\n')
    mesh_path.write_text(f'mtllib {mtl_path.name}\nusemtl runway\n'
        f'v {ox} {oy} 0.0003\nv {ox+length} {oy} 0.0003\n'
        f'v {ox+length} {oy+width} 0.0003\nv {ox} {oy+width} 0.0003\n'
        'vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nvn 0 0 1\nf 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n')
    visual = ET.SubElement(ET.SubElement(model, 'link', name='link'), 'visual', name='overview')
    ET.SubElement(visual, 'cast_shadows').text = 'false'
    ET.SubElement(ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh'), 'uri').text = str(mesh_path)
    tree.write(destination, encoding='unicode')

"""Small flat-road display proxy using the shared scene's own color maps."""
import hashlib,json
from pathlib import Path
from PIL import Image
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker,MarkerArray
from .scene_bounds import scene_bounds


def build_road_display(manifest,output,max_pixels=1024):
    manifest=Path(manifest);scene=json.loads(manifest.read_text());output=Path(output)
    if scene['transform']!='identity_world_baked' or scene['frame']!='world':raise ValueError('RViz road proxy requires the declared world-baked flat scene')
    bounds=scene_bounds(scene)
    output.mkdir(parents=True,exist_ok=False);xmin,xmax,ymin,ymax=bounds['bounds']
    obj=['mtllib road.mtl'];mtl=[];count=0;texture_pixels=0
    for asset in scene['assets']:
        projection=asset.get('display_uv_projection')
        if not projection:continue
        if projection['v_direction']!='decreasing_world_y':raise ValueError('unsupported road display UV convention')
        ox,oy=projection['origin_xy_m'];sx,sy=projection['span_xy_m']
        x0,x1=max(xmin,ox),min(xmax,ox+sx);y0,y1=max(ymin,oy),min(ymax,oy+sy)
        if x1<=x0 or y1<=y0:continue
        suffix=asset['name'].rsplit('_',1)[1];name='display_color_'+suffix+'.png';source=manifest.parent/name
        if hashlib.sha256(source.read_bytes()).hexdigest()!=scene['display_materials'][name]:raise ValueError('road display texture hash mismatch')
        with Image.open(source) as im:
            im=im.convert('RGB');im.thumbnail((max_pixels,max_pixels));im.save(output/f'color_{count}.png');texture_pixels+=im.width*im.height
        material='tile_'+str(count);mtl.extend([f'newmtl {material}','Ka 1 1 1','Kd 1 1 1','Ks 0 0 0',f'map_Kd color_{count}.png'])
        obj.append('usemtl '+material)
        for x,y in ((x0,y0),(x1,y0),(x1,y1),(x0,y1)):obj.append(f'v {x} {y} -0.025')
        for x,y in ((x0,y0),(x1,y0),(x1,y1),(x0,y1)):obj.append(f'vt {(x-ox)/sx} {1-(y-oy)/sy}')
        a=4*count+1;obj.extend([f'f {a}/{a} {a+1}/{a+1} {a+2}/{a+2}',f'f {a}/{a} {a+2}/{a+2} {a+3}/{a+3}']);count+=1
    if not count:raise ValueError('scene has no road display tiles')
    (output/'road.obj').write_text('\n'.join(obj)+'\n');(output/'road.mtl').write_text('\n'.join(mtl)+'\n')
    return dict(mesh=(output/'road.obj').resolve().as_uri(),frame=scene['frame'],**bounds,triangles=2*count,texture_pixels=texture_pixels,rgba_mipmap_budget_bytes=texture_pixels*4*4//3)


def road_messages(road):
    markers=MarkerArray();mesh=Marker();mesh.header.frame_id=road['frame'];mesh.ns='road';mesh.id=0;mesh.type=Marker.MESH_RESOURCE
    mesh.pose.orientation.w=1.;mesh.scale.x=mesh.scale.y=mesh.scale.z=1.;mesh.color.r=mesh.color.g=mesh.color.b=mesh.color.a=1.
    mesh.mesh_resource=road['mesh'];mesh.mesh_use_embedded_materials=True;mesh.frame_locked=True;markers.markers.append(mesh)
    x0,x1,y0,y1=road['bounds'];boundary=Marker();boundary.header=mesh.header;boundary.ns='drivable_boundary';boundary.id=1;boundary.type=Marker.LINE_STRIP
    boundary.pose.orientation.w=1.;boundary.frame_locked=True;boundary.scale.x=.05;boundary.color.g=.85;boundary.color.b=1.;boundary.color.a=1.
    boundary.points=[Point(x=float(x),y=float(y),z=.025) for x,y in ((x0,y0),(x1,y0),(x1,y1),(x0,y1),(x0,y0))];markers.markers.append(boundary)
    label=Marker();label.header=mesh.header;label.ns='road_label';label.id=2;label.type=Marker.TEXT_VIEW_FACING;label.pose.orientation.w=1.;label.frame_locked=True
    label.pose.position.x=float((x0+x1)/2);label.pose.position.y=float(y1+.35);label.pose.position.z=.12;label.scale.z=.3
    label.color=boundary.color;label.text=f'Drivable: {x1-x0:g} x {y1-y0:g} m';markers.markers.append(label)
    if road.get('inspection_bounds',road['bounds'])!=road['bounds']:
        x0,x1,y0,y1=road['inspection_bounds'];roi=Marker();roi.header=mesh.header;roi.ns='inspection_boundary';roi.id=3;roi.type=Marker.LINE_STRIP
        roi.pose.orientation.w=1.;roi.frame_locked=True;roi.scale.x=.04;roi.color.r=1.;roi.color.g=.65;roi.color.a=1.
        roi.points=[Point(x=float(x),y=float(y),z=.03) for x,y in ((x0,y0),(x1,y0),(x1,y1),(x0,y1),(x0,y0))];markers.markers.append(roi)
        label.text+=f' / ROI: {x1-x0:g} x {y1-y0:g} m'
    return markers

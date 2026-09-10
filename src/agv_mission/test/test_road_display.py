import hashlib,json
from pathlib import Path
from PIL import Image
import pytest
from agv_mission.road_display import build_road_display,road_messages


def test_display_proxy_uses_scene_bounds_uv_and_bounded_textures(tmp_path):
    image=tmp_path/'display_color_0.png';Image.new('RGB',(2048,1024),'gray').save(image)
    scene=dict(transform='identity_world_baked',frame='world',inspection_bounds_xy_m=[0,10,-5,5],optical_valid_bounds_xy_m=[-1,11,-6,6],
        assets=[dict(name='terrain_0',display_uv_projection=dict(origin_xy_m=[-1,-6],span_xy_m=[12,12],v_direction='decreasing_world_y'))],
        display_materials={image.name:hashlib.sha256(image.read_bytes()).hexdigest()})
    source=tmp_path/'manifest.json';source.write_text(json.dumps(scene))
    road=build_road_display(source,tmp_path/'proxy')
    assert road['triangles']==2 and road['texture_pixels']==1024*512
    mesh=(tmp_path/'proxy/road.obj').read_text();assert 'v 0 -5 -0.025' in mesh and 'v 10 5 -0.025' in mesh
    markers=road_messages(road).markers
    assert markers[0].mesh_use_embedded_materials and markers[0].frame_locked
    assert markers[1].points[0].x==0 and markers[1].points[2].y==5
    assert 'Drivable: 10 x 10 m'==markers[2].text
    image.write_bytes(b'changed')
    with pytest.raises(ValueError,match='hash mismatch'):build_road_display(source,tmp_path/'bad')


def test_apron_is_visible_and_roi_has_separate_boundary(tmp_path):
    image=tmp_path/'display_color_0.png';Image.new('RGB',(128,128),'gray').save(image)
    scene=dict(transform='identity_world_baked',frame='world',inspection_bounds_xy_m=[0,100,-5,5],
        drivable_bounds_xy_m=[-8,108,-6.5,6.5],optical_valid_bounds_xy_m=[-9,109,-8,8],
        assets=[dict(name='terrain_0',display_uv_projection=dict(origin_xy_m=[-9,-8],span_xy_m=[118,16],v_direction='decreasing_world_y'))],
        display_materials={image.name:hashlib.sha256(image.read_bytes()).hexdigest()})
    source=tmp_path/'manifest.json';source.write_text(json.dumps(scene));road=build_road_display(source,tmp_path/'proxy')
    mesh=(tmp_path/'proxy/road.obj').read_text();assert 'v -8 -6.5 -0.025' in mesh
    markers=road_messages(road).markers
    assert markers[1].points[0].x==-8 and markers[3].points[0].x==0
    assert markers[3].points[2].y==5 and 'ROI: 100 x 10 m' in markers[2].text

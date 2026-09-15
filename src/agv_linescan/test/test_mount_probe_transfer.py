"""An experimental fixed-calibration transfer must reject optical/geometry changes."""
import hashlib,json,sys
from pathlib import Path
import pytest
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools'))
from transfer_mount_probe_calibration import transfer
from agv_linescan.calibration import capture_signature


@pytest.mark.parametrize('change',['none','geometry','color','optics','source_hash'])
def test_transfer_is_bounded_to_same_nominal_optical_scene(tmp_path,change):
    xml='''<robot><link name="base"/><link name="camera_optical_frame">
    <visual><geometry><box size=".1 .2 .3"/></geometry><material><color rgba=".5 .5 .5 1"/></material></visual></link>
    <link name="led_link"/><joint type="fixed"><parent link="base"/><child link="camera_optical_frame"/><origin xyz="1 0 1"/></joint>
    <joint type="fixed"><parent link="base"/><child link="led_link"/><origin xyz="1 0 .3"/></joint></robot>'''
    cfg=yaml.safe_load((Path(__file__).resolve().parents[3]/'src/agv_description/config/linescan.yaml').read_text())
    src=tmp_path/'src';dst=tmp_path/'dst';src.mkdir();dst.mkdir()
    sha=lambda text:hashlib.sha256(text.encode()).hexdigest()
    source_hash=sha(xml)
    (src/'robot.urdf').write_text(xml);(src/'calibration.yaml').write_text(yaml.safe_dump(cfg))
    target=xml
    if change=='geometry':target=target.replace('.1 .2 .3','.1 .2 .4')
    if change=='color':target=target.replace('.5 .5 .5 1','.4 .5 .5 1')
    if change=='optics':cfg['focal_length_m']*=1.1
    (dst/'robot.urdf').write_text(target);(dst/'calibration.yaml').write_text(yaml.safe_dump(cfg))
    (dst/'block_000000.json').write_text(json.dumps({'robot_contract':{'source_sha256':sha(target)}}))
    profile={'conditions':{'robot_source_sha256':source_hash,'capture_signature':capture_signature(yaml.safe_load((src/'calibration.yaml').read_text()))},'flat':{'offset':[4]},'geometry':{'lookup':[0]},'sources':{},'calibration_id':'fixed-test'}
    if change=='source_hash':profile['conditions']['robot_source_sha256']='wrong'
    if change!='none':
        with pytest.raises(AssertionError):transfer(profile,src,dst)
    else:
        result=transfer(profile,src,dst)
        assert result['flat']==profile['flat'] and result['geometry']==profile['geometry']
        assert result['conditions']['robot_source_sha256']==sha(target)
        assert 'EXPERIMENT ONLY' in result['conditions']['note']

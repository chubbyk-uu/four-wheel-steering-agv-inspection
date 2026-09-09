from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'tools'))
from validate_vehicle_envelope import envelope, current_urdf


def test_current_model_including_articulated_wheels_fits_envelope():
    bounds = envelope(current_urdf())
    assert 1.2 < bounds['led_link'] < 1.5
    assert max(bounds.values()) < 1.5


def test_rotating_long_attachment_cannot_hide_in_zero_pose():
    urdf = '''<robot><link name="base_link"/><link name="arm">
    <visual><origin xyz="0 0 1"/><geometry><box size=".1 .1 2"/></geometry></visual></link>
    <joint type="continuous"><parent link="base_link"/><child link="arm"/><axis xyz="0 1 0"/></joint></robot>'''
    assert envelope(urdf)['arm'] > 2


def test_fixed_attachment_extension_is_detected():
    urdf=current_urdf().replace('size="0.035 1.2 0.035"','size="0.035 3.0 0.035"')
    assert envelope(urdf)['led_link'] > 1.5

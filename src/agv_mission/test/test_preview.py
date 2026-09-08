from builtin_interfaces.msg import Time
from test_planner import request_data, vehicle
from agv_mission.planner import plan
from agv_mission.preview import messages


def test_preview_contract(request_data, vehicle):
    result = plan(request_data, vehicle)
    path, markers = messages(result, Time())
    assert path.header.frame_id == 'world'
    assert len(path.poses) == sum(len(s['points']) for s in result['segments'])
    assert all(p.header == path.header for p in path.poses)
    assert markers.markers[0].action == 3  # Clear previous plan, including surplus tracks.
    assert len({m.id for m in markers.markers}) == len(markers.markers)
    assert sum(m.ns == 'rotation_envelope' for m in markers.markers) == 3
    assert sum(m.ns == 'scan_footprint' for m in markers.markers) == 4
    # Exercise generated ROS serialization, not only Python attribute assignment.
    from rclpy.serialization import serialize_message, deserialize_message
    assert deserialize_message(serialize_message(path), type(path)) == path
    decoded = deserialize_message(serialize_message(markers), type(markers))
    assert len(decoded.markers) == len(markers.markers)
    import pytest
    for original, received in zip(markers.markers, decoded.markers):
        assert original.header == received.header
        assert original.points == received.points
        assert original.ns == received.ns and original.id == received.id
        assert received.color.a == pytest.approx(original.color.a)  # ColorRGBA is float32.

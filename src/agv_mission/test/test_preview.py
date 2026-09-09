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


def test_coverage_reverse_track_uses_road_coordinates():
    from agv_mission.preview import coverage_messages
    from builtin_interfaces.msg import Time
    report={'request':{'road':{'frame_id':'map','origin_xyz_m':[10.,20.,0.],'yaw_rad':0.},'region':{'start_xy_m':[6.,-1.],'length_m':3.}},'tracks':[{'direction':-1,'assigned_road_y_m':[0.,1.],'estimated_covered_along_m':[[0.,1.]],'unverified_along_m':[[1.,3.]]}]}
    markers=coverage_messages(report,Time()).markers
    assert markers[0].header.frame_id=='map'
    assert markers[1].frame_locked
    assert markers[1].pose.position.x==18.5 and markers[1].pose.position.y==20.5
    assert markers[2].scale.x==2.

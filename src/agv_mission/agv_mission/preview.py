"""ROS messages for a static plan; independent of Gazebo and vehicle control."""
import math
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


def messages(plan, stamp):
    markers = MarkerArray()
    delete = Marker(action=Marker.DELETEALL)
    markers.markers.append(delete)
    path = Path()
    path.header.frame_id, path.header.stamp = plan['frame_id'], stamp
    delete.header = path.header

    def line(namespace, xyz, color, width=.025, close=False):
        marker = Marker()
        marker.header = path.header
        marker.ns, marker.id = namespace, len(markers.markers)
        marker.type, marker.action = Marker.LINE_STRIP, Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.frame_locked = True
        marker.scale.x = width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        vertices = xyz + xyz[:1] if close else xyz
        marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])+.03) for p in vertices]
        markers.markers.append(marker)

    line('region', plan['region_xyz_m'], (1., .85, .1, 1.), .06, True)
    for track in plan['tracks']:
        line('scan_center', [track['scan_start_xyz_m'], track['scan_end_xyz_m']], (0., 1., .7, 1.), .045)
        line('scan_footprint', track['nominal_footprint_xyz_m'], (0., 1., .7, .45), close=True)
    for seg in plan['segments']:
        xyz = []
        for pt in seg['points']:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pt['pose']['position']
            q = pt['pose']['orientation_xyzw']
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
            path.poses.append(pose)
            xyz.append([pose.pose.position.x, pose.pose.position.y, plan['region_xyz_m'][0][2]])
        line('base_scan' if seg['capture'] else 'transfer', xyz,
             (.3, .65, 1., 1.) if seg['capture'] else (1., .3, .65, .8))
        if seg['kind'] == 'ROTATE_180':
            px, py, pz = xyz[0]
            r = plan['sweep_radius_m']
            line('rotation_envelope', [[px+r*math.cos(t*math.tau/64), py+r*math.sin(t*math.tau/64), pz]
                                      for t in range(64)], (1., .5, .1, .55), close=True)
    return path, markers


def publish(plan):
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    rclpy.init()
    node = rclpy.create_node('rectangle_plan_preview')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    path_pub = node.create_publisher(Path, '/mission/preview/base_path', qos)
    marker_pub = node.create_publisher(MarkerArray, '/mission/preview/markers', qos)

    def send():
        # Latest transform lookup keeps static world geometry compatible with display TF.
        from builtin_interfaces.msg import Time
        path, markers = messages(plan, Time())
        path_pub.publish(path)
        marker_pub.publish(markers)

    send()
    timer = node.create_timer(1.0, send)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_timer(timer)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def coverage_messages(report,stamp):
    """Road-frame coverage rectangles, separate from motion and raw-image truth."""
    from scipy.spatial.transform import Rotation
    result=MarkerArray();road=report['request']['road'];region=report['request']['region']
    rotation=Rotation.from_euler('z',road['yaw_rad']);quat=rotation.as_quat()
    clear=Marker(action=Marker.DELETEALL);clear.header.frame_id=road['frame_id'];clear.header.stamp=stamp;result.markers.append(clear)
    for track in report['tracks']:
        lo,hi=track['assigned_road_y_m']
        for key,color in (('estimated_covered_along_m',(0.,.8,.35,.20)),('unverified_along_m',(1.,.45,.05,.30))):
            for a,b in track[key]:
                along=(a+b)/2 if track['direction']==1 else region['length_m']-(a+b)/2
                position=rotation.apply([region['start_xy_m'][0]+along,(lo+hi)/2,.04])+road['origin_xyz_m']
                m=Marker();m.header=clear.header;m.ns=key;m.id=len(result.markers);m.type=Marker.CUBE;m.action=Marker.ADD;m.frame_locked=True
                m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,position)
                m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=map(float,quat)
                m.scale.x=b-a;m.scale.y=hi-lo;m.scale.z=.015
                m.color.r,m.color.g,m.color.b,m.color.a=color;result.markers.append(m)
    return result

#!/usr/bin/env python3
import os, yaml, rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA

if __name__ == '__main__':
    rospy.init_node('waypoint_viz')
    pub = rospy.Publisher('waypoints_markers', MarkerArray, queue_size=1, latch=True)

    # Carga los gates
    pkg = os.path.dirname(os.path.abspath(__file__))
    yaml_file = os.path.join(pkg, '../config/waypoints.yaml')
    data = yaml.safe_load(open(yaml_file))['waypoints']

    ma = MarkerArray()
    # Marker para segmentos (LINE_LIST)
    line_marker = Marker()
    line_marker.header.frame_id = 'odom'
    line_marker.header.stamp = rospy.Time.now()
    line_marker.ns = 'gates'
    line_marker.id = 0
    line_marker.type = Marker.LINE_LIST
    line_marker.action = Marker.ADD
    # Ancho de las líneas
    line_marker.scale = Vector3(0.05, 0.0, 0.0)
    # Color uniforme verde semitransparente
    line_marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)
    for w in data:
        p1 = Point(x=w['x1'], y=w['y1'], z=0.0)
        p2 = Point(x=w['x2'], y=w['y2'], z=0.0)
        line_marker.points.extend([p1, p2])
    ma.markers.append(line_marker)

    rospy.sleep(0.5)
    pub.publish(ma)
    rospy.loginfo(f"Publicado {len(ma.markers)} markers de gates en /waypoints_markers")
    rospy.spin()

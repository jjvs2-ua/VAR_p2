#!/usr/bin/env python3
import os, yaml, rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg      import Point
from std_msgs.msg           import ColorRGBA

if __name__ == '__main__':
    rospy.init_node('waypoint_viz')
    pub = rospy.Publisher('waypoints_markers', MarkerArray, queue_size=1, latch=True)

    # Carga los waypoints
    pkg = os.path.dirname(os.path.abspath(__file__))
    yaml_file = os.path.join(pkg, '../config/waypoints.yaml')
    data = yaml.safe_load(open(yaml_file))['waypoints']

    ma = MarkerArray()
    for i, w in enumerate(data):
        m = Marker()
        m.header.frame_id = 'odom'       # ajusta si tu TF base es distinto
        m.header.stamp    = rospy.Time.now()
        m.ns              = 'wps'
        m.id              = i
        m.type            = Marker.SPHERE
        m.action          = Marker.ADD
        m.pose.position.x = w['x']
        m.pose.position.y = w['y']
        m.pose.position.z = 0.05        # un poco por encima del suelo
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.1
        # color: de azul a rojo según índice
        hue = float(i) / max(1, len(data)-1)
        m.color = ColorRGBA(hue, 0.8, 1-hue, 0.8)
        ma.markers.append(m)

    rospy.sleep(0.5)  # que se conecten los subscribers
    pub.publish(ma)
    rospy.loginfo(f"Publicado {len(ma.markers)} waypoints en /waypoints_markers")
    rospy.spin()

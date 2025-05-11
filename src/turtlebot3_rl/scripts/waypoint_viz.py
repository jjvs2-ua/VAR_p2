#!/usr/bin/env python3
import os, yaml, rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg      import Point
from std_msgs.msg           import ColorRGBA

# --- cuántos marcadores quieres mostrar ---
STEP = 10            #  publica 1 de cada 10 way-points  →  ~36 en 359
# STEP = max(1, len(data)//36)    # alternativa automática

if __name__ == '__main__':
    rospy.init_node('waypoint_viz')
    pub = rospy.Publisher('waypoints_markers', MarkerArray,
                          queue_size=1, latch=True)

    # Cargar way-points
    pkg = os.path.dirname(os.path.abspath(__file__))
    yaml_file = os.path.join(pkg, '../config/waypoints.yaml')
    data = yaml.safe_load(open(yaml_file))['waypoints']

    # Filtrar: 1 de cada STEP
    data_sub = [w for i, w in enumerate(data) if i % STEP == 0]

    ma = MarkerArray()
    total = len(data_sub)
    for i, w in enumerate(data_sub):
        m = Marker()
        m.header.frame_id = 'odom'          # usa 'world' si lo prefieres
        m.header.stamp    = rospy.Time.now()
        m.ns   = 'wps'
        m.id   = i
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = w['x']
        m.pose.position.y = w['y']
        m.pose.position.z = 0.05
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.12          # un pelín mayor
        # degradado azul-rojo según orden
        hue = float(i) / max(1, total-1)
        m.color = ColorRGBA(hue, 0.8, 1.0-hue, 0.85)
        ma.markers.append(m)

    rospy.sleep(0.5)            # deja que RViz se suscriba
    pub.publish(ma)
    rospy.loginfo(f"Publicado {total} / {len(data)} way-points "
                  f"en /waypoints_markers (STEP={STEP})")
    rospy.spin()

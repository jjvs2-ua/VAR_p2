#!/usr/bin/env python3
"""
Visualizador de “gates” (líneas perpendiculares) en RViz.

Lee ../config/waypoints.yaml, donde cada gate viene dado por:
    x, y  – punto central
    nx,ny – vector normal (longitud 1) que define la orientación del gate
Para cada entrada se publica un Marker.LINE_STRIP de longitud GATE_LEN.
"""

import os, yaml, rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg      import Point
from std_msgs.msg           import ColorRGBA
import numpy as np

GATE_LEN = 3.0      # [m] longitud visualizada de cada línea
LINE_W   = 0.05     # [m] grosor de la línea en RViz

if __name__ == '__main__':
    rospy.init_node('gate_viz')
    pub = rospy.Publisher('gates_markers', MarkerArray, queue_size=1, latch=True)

    # --- Cargar gates ---
    pkg = os.path.dirname(os.path.abspath(__file__))
    yaml_file = os.path.join(pkg, '../config/waypoints.yaml')
    data = yaml.safe_load(open(yaml_file))['waypoints']

    ma = MarkerArray()

    for i, g in enumerate(data):
        center = np.array([g['x'],  g['y']],  dtype=float)
        normal = np.array([g['nx'], g['ny']], dtype=float)
        normal /= np.linalg.norm(normal) + 1e-9  # seguridad

        # End-points de la línea
        p1 = center + (GATE_LEN / 2.0) * normal
        p2 = center - (GATE_LEN / 2.0) * normal

        m = Marker()
        m.header.frame_id = 'odom'      # ajusta si tu TF base es distinto
        m.header.stamp    = rospy.Time.now()
        m.ns              = 'gates'
        m.id              = i
        m.type            = Marker.LINE_STRIP
        m.action          = Marker.ADD
        m.scale.x         = LINE_W       # grosor
        m.pose.orientation.w = 1.0

        # Añadir los dos puntos a la línea
        m.points = [Point(p1[0], p1[1], 0.05),
                    Point(p2[0], p2[1], 0.05)]

        # De azul (primer gate) a rojo (último gate)
        hue = float(i) / max(1, len(data) - 1)
        m.color = ColorRGBA(hue, 0.8, 1.0 - hue, 0.8)  # a=0.8

        ma.markers.append(m)

    rospy.sleep(0.5)  # esperar a que RViz se suscriba
    pub.publish(ma)
    rospy.loginfo(f"Publicado {len(ma.markers)} gates en /gates_markers")
    rospy.spin()

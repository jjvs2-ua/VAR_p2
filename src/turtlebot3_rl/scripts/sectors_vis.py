#!/usr/bin/env python3
import rospy, math
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class SectorMarker:
    def __init__(self):
        rospy.init_node('sector_marker', anonymous=True)
        self.pub = rospy.Publisher('scan_sectors', Marker, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.cb, queue_size=1)
        rospy.spin()

    def cb(self, msg: LaserScan):
        N    = len(msg.ranges)
        sec  = N // 24               # 15° por bloque
        ai   = msg.angle_increment
        amin = msg.angle_min

        # índices de los tres sectores reducidos
        idx_front = list(range(N-sec, N)) + list(range(0, sec))   # ±15°
        idx_left  = list(range(4*sec,  5*sec))                    # +60°…+75°
        idx_right = list(range(N-5*sec, N-4*sec))                 # −75°…−60°

        def make_marker(idxs, color, marker_id):
            m = Marker()
            m.header             = msg.header
            m.ns                 = 'scan_sectors'
            m.id                 = marker_id
            m.type               = Marker.LINE_STRIP
            m.action             = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x            = 0.02
            m.color.r, m.color.g, m.color.b, m.color.a = (*color, 0.6)
            for i in idxs:
                d_raw = msg.ranges[i]
                if not math.isfinite(d_raw):
                    continue                # descartamos inf/nan
                d = min(d_raw, msg.range_max) 
                ang = amin + ai * i
                m.points.append(Point(
                    d * math.cos(ang),
                    d * math.sin(ang),
                    0.0
                ))
            return m

        # publicamos los tres sectores sin puntos inválidos
        self.pub.publish(make_marker(idx_left,  (0.0, 1.0, 0.0), 0))  # verde: izquierda
        self.pub.publish(make_marker(idx_front, (0.0, 0.0, 1.0), 1))  # azul:  frontal
        self.pub.publish(make_marker(idx_right, (1.0, 0.0, 0.0), 2))  # rojo:   derecha

if __name__ == '__main__':
    SectorMarker()

#!/usr/bin/env python3
import rospy, time
from sensor_msgs.msg import Image

PUB_HZ = 2.0            # imágenes/seg deseadas
MIN_DT = 1.0 / PUB_HZ   # intervalo mínimo en segundos
last = 0.0              # última publicación (time.time)
start = 0.0             # marca de arranque

def cb(msg: Image):
    global last
    now = time.time()
    pub.publish(msg)
    last = now

def stop_timer(event):
    rospy.loginfo("Ha transcurrido 1 minuto: deteniendo subscriber.")
    sub.unregister()  # deja de recibir callbacks

if __name__ == "__main__":
    rospy.init_node("throttle_camera", anonymous=True)
    start = time.time()
    pub = rospy.Publisher("/camera/rgb/image_raw_throttled", Image, queue_size=1)
    sub = rospy.Subscriber("/camera/rgb/image_raw", Image, cb, queue_size=1)

    # Timer a 60 s para desactivar el subscriber
    rospy.Timer(rospy.Duration(100), stop_timer, oneshot=True)

    rospy.loginfo("throttle_camera activo a %.1f Hz (parará tras 60 s)", PUB_HZ)
    rospy.spin()

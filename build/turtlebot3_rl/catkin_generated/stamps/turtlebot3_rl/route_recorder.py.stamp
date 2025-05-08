
#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import csv
import os
import math
from message_filters import Subscriber, ApproximateTimeSynchronizer

# Número de puntos que queremos conservar del LIDAR (filtrado)
NUM_LASER_POINTS = 24

# Parámetro para reemplazar inf/nan y clippear
MAX_LIDAR_RANGE = 10.0

# Factor de peso para odometría (x, y, yaw)
ODOM_WEIGHT = 8.0

csv_file = None
csv_writer = None

def clean_range(val):
    """
    Reemplaza inf/nan por MAX_LIDAR_RANGE y clippea al rango [0, MAX_LIDAR_RANGE].
    """
    if math.isinf(val) or math.isnan(val):
        return MAX_LIDAR_RANGE
    return min(max(val, 0.0), MAX_LIDAR_RANGE)

def filter_laser(ranges, num_points=NUM_LASER_POINTS):
    """
    Filtra la lista de valores del LIDAR a num_points muestras equidistantes,
    limpiando inf/nan.
    """
    n = len(ranges)
    if n < num_points:
        # Si hay menos datos, limpiamos todos
        return [clean_range(r) for r in ranges]
    step = float(n) / num_points
    filtered = []
    for i in range(num_points):
        idx = int(i * step)
        filtered.append(clean_range(ranges[idx]))
    return filtered

def callback(odom, scan):
    global csv_writer, csv_file

    # Timestamp
    t = odom.header.stamp.to_sec()

    # Posición y orientación (yaw)
    pos = odom.pose.pose.position
    orientation_q = odom.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w
    ])

    # Filtrar LIDAR
    filtered_laser = filter_laser(scan.ranges, NUM_LASER_POINTS)

    # Aplicar peso a la odometría
    x_w = pos.x * ODOM_WEIGHT
    y_w = pos.y * ODOM_WEIGHT
    yaw_w = yaw * ODOM_WEIGHT

    # Escribir fila: time, x_w, y_w, yaw_w, lidar...
    row = [t, x_w, y_w, yaw_w] + filtered_laser
    csv_writer.writerow(row)
    csv_file.flush()

def main():
    global csv_writer, csv_file

    rospy.init_node("route_recorder", anonymous=True)

    # Directorio y archivo CSV
    directory = "./"
    os.makedirs(directory, exist_ok=True)
    csv_path = os.path.join(directory, "route_data.csv")

    # Abrir CSV y escribir header
    csv_file = open(csv_path, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    header = ["time", "pos_x_w", "pos_y_w", "yaw_w"] + [f"laser_{i}" for i in range(NUM_LASER_POINTS)]
    csv_writer.writerow(header)

    # Suscriptores sincronizados
    odom_sub = Subscriber("/odom", Odometry)
    scan_sub = Subscriber("/scan", LaserScan)
    ats = ApproximateTimeSynchronizer([odom_sub, scan_sub], queue_size=10, slop=0.1)
    ats.registerCallback(callback)

    rospy.loginfo(f"route_recorder iniciado, guardando en: {csv_path}")
    rospy.spin()
    csv_file.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

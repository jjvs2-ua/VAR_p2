#!/usr/bin/env python3
"""Spawnea y elimina obstáculos aleatorios en Gazebo cada RESET_INTERVAL,
colocándolos cerca de distintos waypoints."""
import os
import random
import math
import rospy
import rospkg
import yaml
from gazebo_msgs.srv import SpawnModel, DeleteModel, SpawnModelRequest
from std_srvs.srv import Empty

# ── rutas de paquete ────────────────────────────────────────────────────────
rospack      = rospkg.RosPack()
TB3_GZ_PATH  = rospack.get_path('turtlebot3_gazebo')
RL_PATH      = rospack.get_path('turtlebot3_rl')

# ── calcular puntos medios de cada gate ────────────────────────────────────
ways_cfg      = yaml.safe_load(open(os.path.join(RL_PATH, 'config', 'waypoints.yaml')))['waypoints']
GATE_MIDPOINTS = [
    ((w['x1'] + w['x2']) / 2.0, (w['y1'] + w['y2']) / 2.0)
    for w in ways_cfg
]
OBSTACLE_RADIUS = 0.5   # [m] radio alrededor del waypoint

def get_sdf(folder):
    """Devuelve el texto del model.sdf en turtlebot3_gazebo/models/<folder>/model.sdf."""
    path = os.path.join(TB3_GZ_PATH, 'models', folder, 'model.sdf')
    return open(path, 'r').read()

# ── configuración del spawner ──────────────────────────────────────────────
RESET_INTERVAL  = 25.0   # [s] cada cuánto respawnear
MAX_OBS         = 4      # para delete_all
OBSTACLE_MODELS = [
    ("cone",   "construction_cone"),
    ("waffle", "turtlebot3_waffle"),
]

def spawn_random():
    """Crea entre 1–2 de cada tipo, cada uno cerca de un waypoint distinto."""
    # preparar lista: de 1 a 2 por tipo
    obs_list = []
    for _, folder in OBSTACLE_MODELS:
        count = random.randint(1, 2)
        obs_list += [folder] * count

    total = len(obs_list)
    if total == 0:
        return

    # elegir gates únicos o con reemplazo
    if total <= len(GATE_MIDPOINTS):
        chosen = random.sample(GATE_MIDPOINTS, total)
    else:
        chosen = [random.choice(GATE_MIDPOINTS) for _ in range(total)]

    # para cada obstáculo, spawn alrededor del midpoint
    for idx, (folder, midpoint) in enumerate(zip(obs_list, chosen)):
        x0, y0 = midpoint
        theta  = random.uniform(0, 2*math.pi)
        r      = random.uniform(0, OBSTACLE_RADIUS)
        x = x0 + r * math.cos(theta)
        y = y0 + r * math.sin(theta)

        req = SpawnModelRequest()
        name = f"auto_obs_{folder}_{idx}"
        req.model_name       = name
        req.robot_namespace  = name
        req.model_xml        = get_sdf(folder)
        req.reference_frame  = 'world'
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.orientation.w = 1.0

        try:
            spawn(req)
        except rospy.ServiceException as e:
            rospy.logwarn(f"No se pudo spawn {name}: {e}")

def delete_all():
    """Borra todos los posibles obstáculos generados."""
    for _, folder in OBSTACLE_MODELS:
        for i in range(MAX_OBS):
            name = f"auto_obs_{folder}_{i}"
            try:
                delete(name)
            except rospy.ServiceException:
                pass

if __name__ == '__main__':
    rospy.init_node('obstacle_spawner')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn  = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete = rospy.ServiceProxy('/gazebo/delete_model',   DeleteModel)

    last_t = rospy.Time.now()
    rate   = rospy.Rate(2)  # 2 Hz

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        if (now - last_t).to_sec() > RESET_INTERVAL:
            delete_all()
            spawn_random()
            last_t = now
        try:
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            # reiniciar el timer tras reset_simulation
            last_t = rospy.Time.now()

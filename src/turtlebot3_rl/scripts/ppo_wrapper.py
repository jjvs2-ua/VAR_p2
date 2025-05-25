#!/usr/bin/env python3
"""
PathTrackingWrapper “simplificado”

· Waypoints = puntos (los centros de los antiguos gates)
· El robot sólo tiene que acercarse a cada punto y consideramos el
  “gate” cruzado cuando la distancia cae por debajo de GATE_RADIUS.
· Recompensa-shaping = acercarse al punto:
        r_prog = DIST_BETA · (dist_anterior − dist_actual)
"""

import os, math, yaml, random, rospy, rospkg
import numpy as np
import gym
from gym import spaces
from gym.utils import seeding
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnModel, DeleteModel, SpawnModelRequest
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler

# ───────── parámetros globales ─────────
V_MAX,  OMEGA_MAX = 0.22, 2.84
LASER_MAX         = 3.5
PROX_TH, COLL_TH  = 1.0, 0.15
TIMEOUT_T         = 500.0
STAGN_T, MOVE_EPS = 5.0, 0.05

DIST_BETA         = 15.0      # peso de la recompensa por acercarse
GATE_BONUS        = 3.0     # bonus al alcanzar el waypoint
GATE_RADIUS       = 0.2     # distancia para considerar “cruzado”

TURN_LAMBDA       = 0.3
STEP_PENAL        = 0.01
CENTERING_FACTOR  = 0.2

OBSTACLE_FOLDERS  = ['construction_cone','first_2015_trash_can','WoodenChair']
MAX_OBSTACLES_PER_TYPE = 3
OBSTACLE_DIST_FROM_MID = 1
MIN_DIST_START    = 2.0

rospack     = rospkg.RosPack()
TB3_GZ_PATH = rospack.get_path('turtlebot3_gazebo')

def get_sdf(folder:str)->str:
    path = os.path.join(TB3_GZ_PATH,'models',folder,'model.sdf')
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    return open(path).read()

# ───────────────── wrapper ─────────────────
class PathTrackingWrapper(gym.Env):
    metadata={'render_modes':[],'render_fps':10}

    # ─── INIT ───
    def __init__(self):
        super().__init__()
        if not rospy.core.is_initialized():
            rospy.init_node('ppo_env_wrapper',anonymous=True,disable_signals=True)

        # pubs/subs
        self.cmd_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        rospy.Subscriber('/scan',LaserScan,self._scan_cb,queue_size=1)
        rospy.Subscriber('/odom',Odometry,self._odom_cb,queue_size=1)

        # servicios gazebo
        self.spawn_srv  = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_srv = rospy.ServiceProxy('/gazebo/delete_model',    DeleteModel)
        self.reset_world= rospy.ServiceProxy('/gazebo/reset_world',     Empty)
        self.unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
        rospy.wait_for_service('/gazebo/spawn_sdf_model',30)
        try: self.unpause_physics()
        except rospy.ServiceException: pass

        # cargar waypoints (centros)
        cfg_path = os.path.join(os.path.dirname(__file__),'../config/waypoints.yaml')
        with open(cfg_path) as f:
            way_raw = yaml.safe_load(f)['waypoints']
        self.waypoints = [ np.array([(e['x1']+e['x2'])/2, (e['y1']+e['y2'])/2],np.float32)
                           for e in way_raw ]

        # espacios
        low  = np.array([0,0,0, -np.inf,-np.inf, -1,-1],np.float32)
        high = np.array([1,1,1,  np.inf, np.inf,  1, 1],np.float32)
        self.observation_space = spaces.Box(low,high,dtype=np.float32)
        self.action_space      = spaces.Box(np.array([0.0,-1.0]),np.array([1.0,1.0]),dtype=np.float32)

        # buffers
        self.current_scan_raw = np.full(360,LASER_MAX,np.float32)
        self.current_scan_processed = np.ones(3,np.float32)
        self.current_pos  = np.zeros(2,np.float32)
        self.current_yaw  = 0.0
        self.scan_ready=self.odom_ready=False
        self.ros_rate = rospy.Rate(10)
        self.obstacle_names_in_sim=[]
        self.seed()

        self.wait_for_sensors()
        self._init_episode()

    # ─── utilidades ───
    def seed(self,seed=None):
        self.np_random,s=seeding.np_random(seed); random.seed(s); return[s]

    def wait_for_sensors(self):
        self.scan_ready=self.odom_ready=False
        t_end=rospy.Time.now()+rospy.Duration(5)
        r=rospy.Rate(50)
        while not (self.scan_ready and self.odom_ready) and rospy.Time.now()<t_end: r.sleep()

    # ─── callbacks ───
    def _scan_cb(self, msg: LaserScan):
        """ Divide el LiDAR en 3 sectores:
            · Front  ±15°  ( sec muestras a cada lado )
            · Left   +60°…+75°
            · Right −75°…−60°
        """
        rng = np.array(msg.ranges, dtype=np.float32)
        rng[np.isinf(rng) | np.isnan(rng)] = LASER_MAX
        self.current_scan_raw = rng

        N   = len(rng)
        sec = N // 24            # 15°  →  360 / 24

        #  ±15 °
        front_idxs = np.r_[np.arange(N-sec, N), np.arange(0, sec)]
        # +60° … +75°
        left_idxs  = np.arange(4*sec, 5*sec)
        # −75° … −60°
        right_idxs = np.arange(N-5*sec, N-4*sec)

        L = rng[left_idxs ].mean()
        F = rng[front_idxs].mean()
        R = rng[right_idxs].mean()

        self.current_scan_processed = np.array([L, F, R], dtype=np.float32) / LASER_MAX
        self.scan_ready = True


    def _odom_cb(self,msg:Odometry):
        p=msg.pose.pose.position; self.current_pos=np.array([p.x,p.y],np.float32)
        o=msg.pose.pose.orientation
        self.current_yaw=math.atan2(2*(o.w*o.z+o.x*o.y),1-2*(o.y**2+o.z**2))
        self.odom_ready=True

    # ─── observación ───
    def _obs(self):
        wp=self.waypoints[self.curr_wp_idx]
        dx,dy = wp-self.current_pos
        return np.array([*self.current_scan_processed, dx,dy,
                         math.sin(self.current_yaw), math.cos(self.current_yaw)],np.float32)

    # ─── distancia ───
    def _dist_to_wp(self):
        return float(np.linalg.norm(self.waypoints[self.curr_wp_idx]-self.current_pos))

    def _spawn_obstacles(self):
        """Genera obstáculos aleatorios asegurando:
            · No más de MAX_OBSTACLES_PER_TYPE por tipo
            · No más de   (len(OBSTACLE_FOLDERS)*MAX_OBSTACLES_PER_TYPE)   en total
            · No solaparse entre sí ni con el robot.
        """
        self.obstacle_names_in_sim.clear()
        taken_pts = []                                     # puntos ya usados

        # número total que realmente vamos a colocar
        max_global = len(OBSTACLE_FOLDERS)*MAX_OBSTACLES_PER_TYPE

        for folder in OBSTACLE_FOLDERS:
            n_this = self.np_random.integers(0, MAX_OBSTACLES_PER_TYPE + 1)
            for _ in range(n_this):
                if len(self.obstacle_names_in_sim) >= max_global:
                    break  # tope global alcanzado

                # intenta como mucho 10 posiciones válidas
                for _attempt in range(10):
                    wp = self.waypoints[self.np_random.choice(len(self.waypoints))]
                    ang  = self.np_random.uniform(0, 2*math.pi)
                    dist = OBSTACLE_DIST_FROM_MID + self.np_random.uniform(-0.1, 0.1)
                    x, y = wp + dist*np.array([math.cos(ang), math.sin(ang)])

                    if math.hypot(x, y) < MIN_DIST_START:
                        continue
                    if any(math.hypot(x-ox, y-oy) < 0.6 for ox, oy in taken_pts):
                        continue

                    # petición de spawn
                    req               = SpawnModelRequest()
                    req.model_name    = f"{folder}_{self.np_random.integers(1e4,1e5)}"
                    req.model_xml     = get_sdf(folder)
                    req.robot_namespace = ""
                    req.reference_frame = "world"
                    pose = Pose()
                    pose.position.x, pose.position.y, pose.position.z = x, y, 0.01
                    pose.orientation.w = 1.0
                    req.initial_pose = pose

                    try:
                        if self.spawn_srv(req).success:
                            self.obstacle_names_in_sim.append(req.model_name)
                            taken_pts.append((x, y))
                            break
                    except rospy.ServiceException:
                        pass

    def _delete_obstacles(self):
        """Elimina todos los modelos que creamos, incluso si por algún
           motivo la lista local se ha perdido, usando GetWorldProperties.
        """
        try:
            from gazebo_msgs.srv import GetWorldProperties
            world_srv = rospy.ServiceProxy('/gazebo/get_world_properties',
                                           GetWorldProperties)
            world_models = world_srv().model_names
        except rospy.ServiceException:
            world_models = []

        # prefijos que hemos podido crear
        prefixes = tuple(f"{f}_" for f in OBSTACLE_FOLDERS)

        # borra los que estén vivos
        for name in world_models:
            if name.startswith(prefixes):
                try:
                    self.delete_srv(name)
                except rospy.ServiceException:
                    pass

        self.obstacle_names_in_sim.clear()


    # ─── episodio ───
    def _init_episode(self):
        self.curr_wp_idx=0
        self.prev_dist = self._dist_to_wp()
        self.last_move_time = rospy.Time.now()
        self.prev_pos = self.current_pos.copy()
        self.episode_start = rospy.Time.now()
        self.last_log = rospy.Time.now()

    # ─── reset ───
    def reset(self,*,seed=None,options=None):
        if seed is not None: self.seed(seed)
        self.cmd_pub.publish(Twist()); rospy.sleep(0.05)
        self._delete_obstacles(); self.reset_world(); self.wait_for_sensors()
        self._spawn_obstacles(); self._init_episode()
        return self._obs(), {}

    def step(self, action):
        # ───────── Control ─────────
        v = float(action[0]) * V_MAX
        w = float(action[1]) * OMEGA_MAX
        self.cmd_pub.publish(Twist(
            linear  = Twist().linear.__class__(x=v),
            angular = Twist().angular.__class__(z=w)
        ))
        self.ros_rate.sleep()

        # ───────── Recompensas ─────────
        dist     = self._dist_to_wp()
        r_prog   = DIST_BETA * (self.prev_dist - dist)
        self.prev_dist = dist

        L, F, R  = self.current_scan_processed
        r_center = CENTERING_FACTOR * (1 - abs(L - R)) * ((L + R) / 2)

        dmin     = float(self.current_scan_raw.min())
        r_prox   = 0.0
        if dmin < PROX_TH:
            r_prox -= ((PROX_TH - dmin) / PROX_TH) ** 2
        if dmin < COLL_TH:                     # colisión real
            r_prox -= 2.0                      # castigo fijo extra

        r_turn   = -TURN_LAMBDA * abs(w) / OMEGA_MAX
        r_step   = -STEP_PENAL
        reward   = r_prog + r_center + r_prox + r_turn + r_step

        # ───────── Waypoint alcanzado ─────────
        done_success = False
        if dist <= GATE_RADIUS:
            reward += GATE_BONUS
            rospy.loginfo(f"🏁 Waypoint {self.curr_wp_idx} alcanzado  (+{GATE_BONUS})")
            self.curr_wp_idx += 1
            if self.curr_wp_idx >= len(self.waypoints):
                done_success = True
            else:
                self.prev_dist = self._dist_to_wp()

        # ───────── Estancamiento / timeout ─────────
        move = np.linalg.norm(self.current_pos - self.prev_pos)
        now  = rospy.Time.now()
        if move > MOVE_EPS:
            self.last_move_time = now
            self.prev_pos       = self.current_pos.copy()

        stagnant = (now - self.last_move_time).to_sec() > STAGN_T
        timeout  = (now - self.episode_start).to_sec() > TIMEOUT_T
        if stagnant:
            reward -= 20.0   # penalización por estancamiento   
        done     = done_success or stagnant or timeout

        # ───────── Logs cada segundo ─────────
        front_m = F * LASER_MAX   # distancia frontal en metros

        if done:
            reason = "ÉXITO" if done_success else "ESTANCAMIENTO" if stagnant else "TIMEOUT"
            rospy.logwarn(
                f"DONE: {reason} | idx={self.curr_wp_idx} dist={dist:.2f} "
                f"tot={reward:+.2f} (prog={r_prog:+.2f}, center={r_center:+.2f}, "
                f"prox={r_prox:+.2f}, turn={r_turn:+.2f})  F={front_m:.2f} m"
            )
        elif (now - self.last_log).to_sec() > 1.0:
            rospy.loginfo(
                f"[DEBUG] idx={self.curr_wp_idx} dist={dist:.2f} "
                f"tot={reward:+.2f} (p={r_prog:+.2f},c={r_center:+.2f},x={r_prox:+.2f},"
                f"t={r_turn:+.2f})  F={front_m:.2f} m  L={L:.3f} R={R:.3f}"
            )
            self.last_log = now

        return self._obs(), reward, done, False, {}


    # ─── close ───
    def close(self):
        self._delete_obstacles()
        self.cmd_pub.publish(Twist())

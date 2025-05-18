#!/usr/bin/env python3
"""
Gym/SB3 wrapper para TurtleBot3 con “gates” perpendiculares:

– obs: 24 rayos LiDAR + dist. ortogonal y heading al gate
– act:  v≥0, ω  (Box[0,1] × [-1,1])
– reward: progreso + bonus Gate + rapidez + centrado – proximidad
– truncate: 5 min o 20 s sin avanzar.
"""

import os, math, yaml, rospy, numpy as np
from gym import Env, spaces
from geometry_msgs.msg import Twist
from sensor_msgs.msg   import LaserScan
from nav_msgs.msg      import Odometry

# ---------- Constantes ----------
V_MAX, OMEGA_MAX   = 0.22, 2.84
RAYS_USED          = 24
LASER_MAX          = 3.5
PROX_TH, COLL_TH   = 0.30, 0.12
STAGN_T, TIMEOUT_T = 20.0, 840.0     # s sin mover / s máx. episodio
MOVE_EPS           = 0.02            # m considerados avance
PRINT_DT           = 1.0             # s entre mensajes de debug

class PathTrackingWrapper(Env):
    def __init__(self):
        if not rospy.core.is_initialized():
            rospy.init_node('ppo_wrapper', anonymous=True, disable_signals=True)

        # Pubs / subs
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.scan_cb,  queue_size=1)
        rospy.Subscriber('/odom', Odometry,  self.odom_cb, queue_size=1)

        # Leer gates del YAML
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(pkg_dir, '../config/waypoints.yaml')) as f:
            data = yaml.safe_load(f)['waypoints']
            self.gates = [(
                np.array([w['x'],  w['y']], dtype=np.float32),
                np.array([w['nx'], w['ny']], dtype=np.float32)
            ) for w in data]

        # Espacios Gym
        self.observation_space = spaces.Box(
            low  = np.concatenate([np.zeros(RAYS_USED),  [0.0, -math.pi]]).astype(np.float32),
            high = np.concatenate([np.ones (RAYS_USED), [LASER_MAX, math.pi]]).astype(np.float32)
        )
        self.action_space = spaces.Box(np.array([0.0, -1.], dtype=np.float32),
                                       np.array([1.0,  1.], dtype=np.float32))

        self.rate = rospy.Rate(10)   # 10 Hz
        self.scan_ready = self.odom_ready = False

        # Esperar a que lleguen LiDAR y odometría
        self.wait_sensors()

        # Inicializar puerta actual
        self.reset_gates()

        # ---- depuración ----
        self._last_print_t = rospy.Time.now()

    # ---------- Callbacks ----------
    def scan_cb(self, msg):
        r = np.array(msg.ranges, dtype=np.float32)
        r[np.isinf(r)] = LASER_MAX
        self.scan = r[::len(r)//RAYS_USED][:RAYS_USED]
        self.scan_ready = True

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.pos = np.array([p.x, p.y], dtype=np.float32)
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                              1 - 2*(q.y*q.y + q.z*q.z))
        self.odom_ready = True

    # ---------- Utilidades ----------
    def wait_sensors(self):
        while not (self.scan_ready and self.odom_ready) and not rospy.is_shutdown():
            rospy.sleep(0.02)

    def reset_gates(self):
        self.gate_idx  = 0
        self.gate_p, self.gate_n = self.gates[0]
        self.prev_signed_dist    = np.dot(self.pos - self.gate_p, self.gate_n)
        self.steps_since_gate    = 0

    def _get_obs(self):
        scan_n = self.scan / LASER_MAX
        dist   = abs(np.dot(self.pos - self.gate_p, self.gate_n))          # ortogonal
        vec    = self.gate_p - self.pos
        head   = (math.atan2(vec[1], vec[0]) - self.yaw + math.pi)%(2*math.pi)-math.pi
        return np.concatenate([scan_n, [dist, head]]).astype(np.float32)

    # ---------- Gym API ----------
    def reset(self, *, seed=None, options=None):
        os.system('rosservice call /gazebo/reset_simulation "{}"')
        self.wait_sensors()
        self.reset_gates()

        self.prev_pos       = self.pos.copy()
        self.last_move_time = rospy.Time.now()
        self.start_time     = rospy.Time.now()
        self.prev_dist      = abs(np.dot(self.pos - self.gate_p, self.gate_n))
        self._last_print_t  = rospy.Time.now()
        self.steps          = 0
        return self._get_obs(), {}

    def step(self, action):
        # ---- actuar ----
        v = float(action[0]) * V_MAX
        w = float(action[1]) * OMEGA_MAX
        msg = Twist()
        msg.linear.x  = v
        msg.angular.z = w
        self.cmd_pub.publish(msg)
        self.rate.sleep()

        self.steps += 1
        self.steps_since_gate += 1

        obs   = self._get_obs()
        dist  = obs[-2]
        min_r = float(self.scan.min())

        # ---- progreso ortogonal ----
        progress    = self.prev_dist - dist
        r_progress  = 1.0 * progress
        self.prev_dist = dist

        # ---- gate cruzado ----
        signed_dist = np.dot(self.pos - self.gate_p, self.gate_n)
        r_gate = 0.0
        if self.prev_signed_dist > 0.0 and signed_dist <= 0.0:
            r_gate  = 5.0                                   # bonus base
            r_gate += 1.0 / (1.0 + 0.1 * self.steps_since_gate)  # rapidez
            if self.gate_idx < len(self.gates) - 1:
                self.gate_idx += 1
                self.gate_p, self.gate_n = self.gates[self.gate_idx]
            self.steps_since_gate = 0
            self.prev_dist        = abs(np.dot(self.pos - self.gate_p, self.gate_n))
        self.prev_signed_dist = signed_dist

        # ---- centrado LiDAR ----
        l, r = self.scan[:RAYS_USED//2].mean(), self.scan[RAYS_USED//2:].mean()
        r_center = 0.2 * (1 - abs(l - r) / LASER_MAX)

        # ---- proximidad a obstáculos ----
        r_prox = 0.0
        if min_r < PROX_TH:
            r_prox = -2.0 * (PROX_TH - min_r) / PROX_TH
        if min_r < COLL_TH:
            r_prox += -5.0

        reward = r_progress + r_gate + r_center + r_prox

        # ---- estancamiento / timeout ----
        moved = np.linalg.norm(self.pos - self.prev_pos) > MOVE_EPS
        if moved:
            self.last_move_time = rospy.Time.now()
        self.prev_pos = self.pos.copy()

        stagnant = (rospy.Time.now() - self.last_move_time).to_sec() > STAGN_T
        timeout  = (rospy.Time.now() - self.start_time)   .to_sec() > TIMEOUT_T
        done = stagnant or timeout
        info = {'gate_index': self.gate_idx, 'dist_gate': dist,
                'stagnant': stagnant, 'timeout': timeout}

        # ---- DEBUG cada segundo ----
        if (rospy.Time.now() - self._last_print_t).to_sec() >= PRINT_DT:
            pos_str  = f"Robot ({self.pos[0]:+5.2f},{self.pos[1]:+5.2f})"
            gate_str = f"GATE#{self.gate_idx} ({self.gate_p[0]:+5.2f},{self.gate_p[1]:+5.2f})"
            print(f"[DEBUG] {pos_str}  →  {gate_str}  dist={dist:4.2f} m")
            self._last_print_t = rospy.Time.now()

        return obs, reward, done, False, info

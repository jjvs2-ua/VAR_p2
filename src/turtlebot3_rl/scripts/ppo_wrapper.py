#!/usr/bin/env python3
"""
Gym/SB3 wrapper para TurtleBot3 con “gates” perpendiculares.

— Observación (26 dim.): 24 LiDAR + distancia ortogonal y heading al gate
— Acción:  v≥0, ω   (Box[0,1] × [-1,1])
— Recompensa: progreso + bonus-gate + rapidez-gate + bonus-circuito + centrado – proximidad
— El episodio finaliza si:
      · Se completa el circuito (gate final)  ó
      · Pasan 20 s sin avanzar  ó
      · Se supera TIMEOUT_T (8 min)
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
STAGN_T, TIMEOUT_T = 20.0, 480.0     # 8 min máx. episodio
MOVE_EPS           = 0.02
PRINT_DT           = 1.0

# Bonos temporales
GATE_TIME_SCALE      = 3.0     # ↘ recompensa extra =  GATE_TIME_SCALE / t_gate
CIRCUIT_BONUS_BASE   = 50.0    # premio fijo por completar vuelta
CIRCUIT_TIME_SCALE   = 300.0   # ↘ extra =  CIRCUIT_TIME_SCALE / t_total

class PathTrackingWrapper(Env):
    def __init__(self):
        if not rospy.core.is_initialized():
            rospy.init_node('ppo_wrapper', anonymous=True, disable_signals=True)

        # Pubs / subs
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.scan_cb,  queue_size=1)
        rospy.Subscriber('/odom', Odometry,  self.odom_cb, queue_size=1)

        # Leer gates
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
        self.wait_sensors()
        self.reset_gates()

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
        self.gate_idx         = 0
        self.gate_p, self.gate_n = self.gates[0]
        self.prev_signed_dist = np.dot(self.pos - self.gate_p, self.gate_n)
        self.steps_since_gate = 0
        self.gate_start_time  = rospy.Time.now()
        self.circuit_start    = rospy.Time.now()

    def _get_obs(self):
        scan_n = self.scan / LASER_MAX
        dist   = abs(np.dot(self.pos - self.gate_p, self.gate_n))
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
        self.prev_dist      = abs(np.dot(self.pos - self.gate_p, self.gate_n))
        self.steps          = 0
        self._last_print_t  = rospy.Time.now()
        return self._get_obs(), {}

    def step(self, action):
        # ---- actuar ----
        v = float(action[0]) * V_MAX
        w = float(action[1]) * OMEGA_MAX
        twist = Twist(); twist.linear.x = v; twist.angular.z = w
        self.cmd_pub.publish(twist)
        self.rate.sleep()

        self.steps += 1
        self.steps_since_gate += 1

        obs   = self._get_obs()
        dist  = obs[-2]
        min_r = float(self.scan.min())

        # ---- progreso ortogonal ----
        progress    = self.prev_dist - dist
        r_progress  = progress
        self.prev_dist = dist

        # ---- cruce de gate ----
        signed_dist = np.dot(self.pos - self.gate_p, self.gate_n)
        r_gate = 0.0
        circuit_done = False
        if self.prev_signed_dist > 0.0 and signed_dist <= 0.0:
            # tiempo empleado en este tramo
            t_gate = (rospy.Time.now() - self.gate_start_time).to_sec()
            r_gate  = 5.0                          # bonus fijo por cruzar
            r_gate += GATE_TIME_SCALE / max(t_gate, 1e-2)   # rapidez

            # ¿último gate?
            if self.gate_idx == len(self.gates) - 1:
                t_total = (rospy.Time.now() - self.circuit_start).to_sec()
                r_circuit  = CIRCUIT_BONUS_BASE
                r_circuit += CIRCUIT_TIME_SCALE / max(t_total, 1e-2)
                r_gate += r_circuit
                circuit_done = True
            else:
                # avanzar al siguiente gate
                self.gate_idx += 1
                self.gate_p, self.gate_n = self.gates[self.gate_idx]
                self.prev_dist = abs(np.dot(self.pos - self.gate_p, self.gate_n))
                self.gate_start_time = rospy.Time.now()
                self.steps_since_gate = 0

        self.prev_signed_dist = signed_dist

        # ---- centrado LiDAR ----
        l, r = self.scan[:RAYS_USED//2].mean(), self.scan[RAYS_USED//2:].mean()
        r_center = 0.2 * (1 - abs(l - r) / LASER_MAX)

        # ---- proximidad ----
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
        timeout  = (rospy.Time.now() - self.circuit_start ).to_sec() > TIMEOUT_T
        done = circuit_done or stagnant or timeout

        info = {'gate_index': self.gate_idx,
                'dist_gate': dist,
                'circuit_done': circuit_done,
                'stagnant': stagnant,
                'timeout': timeout}

        # ---- DEBUG ----
        if (rospy.Time.now() - self._last_print_t).to_sec() >= PRINT_DT:
            pos_str  = f"Robot ({self.pos[0]:+5.2f},{self.pos[1]:+5.2f})"
            gate_str = f"GATE#{self.gate_idx}"
            print(f"[DEBUG] {pos_str}  →  {gate_str}  dist={dist:4.2f}  r_tot={reward:5.2f}")
            self._last_print_t = rospy.Time.now()

        return obs, reward, done, False, info

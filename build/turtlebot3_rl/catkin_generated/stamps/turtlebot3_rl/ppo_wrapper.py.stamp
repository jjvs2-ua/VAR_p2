#!/usr/bin/env python3
"""
Gym/SB3 wrapper para TurtleBot3:
– obs: 24 rayos LiDAR + dist. y heading al waypoint
– act:  v≥0, ω  (Box[0,1]×[-1,1])
– reward: progreso + bonus WP + centrado – proximidad
– truncate: 5 min o 20 s sin avanzar.  NO se reinicia por lidar.
— Imprime cada segundo posición actual y waypoint destino.
"""

import os, math, yaml, rospy, numpy as np
from gym import Env, spaces
from geometry_msgs.msg import Twist
from sensor_msgs.msg   import LaserScan
from nav_msgs.msg      import Odometry

# ---------- Constantes ----------
V_MAX, OMEGA_MAX = 0.22, 2.84
RAYS_USED, LASER_MAX = 24, 3.5
PROX_TH, COLL_TH = 0.30, 0.12
STAGN_T, TIMEOUT_T = 20.0, 300.0       # s sin mover / s máx. episodio
MOVE_EPS = 0.02                        # m considerados avance
PRINT_DT = 1.0                         # s entre mensajes de debug

class PathTrackingWrapper(Env):
    def __init__(self):
        if not rospy.core.is_initialized():
            rospy.init_node('ppo_wrapper', anonymous=True, disable_signals=True)

        # Pubs / subs
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.scan_cb,  queue_size=1)
        rospy.Subscriber('/odom', Odometry,  self.odom_cb, queue_size=1)

        # Way-points
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(pkg_dir, '../config/waypoints.yaml')) as f:
            self.wps = [(w['x'], w['y']) for w in yaml.safe_load(f)['waypoints']]
        self.reset_waypoints()

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

    def reset_waypoints(self):
        self.wp_idx  = 0
        self.next_wp = np.array(self.wps[0], dtype=np.float32)

    def _get_obs(self):
        scan_n = self.scan / LASER_MAX
        vec    = self.next_wp - self.pos
        dist   = np.linalg.norm(vec)
        head   = (math.atan2(vec[1], vec[0]) - self.yaw + math.pi)%(2*math.pi)-math.pi
        return np.concatenate([scan_n, [dist, head]]).astype(np.float32)

    # ---------- Gym API ----------
    def reset(self, *, seed=None, options=None):
        os.system('rosservice call /gazebo/reset_simulation "{}"')
        self.wait_sensors()
        self.reset_waypoints()

        self.prev_pos       = self.pos.copy()
        self.last_move_time = rospy.Time.now()
        self.start_time     = rospy.Time.now()
        self.prev_dist      = np.linalg.norm(self.next_wp - self.pos)
        self._last_print_t  = rospy.Time.now()
        self.steps          = 0
        return self._get_obs(), {}

    def step(self, action):
        # ---- actuar ----
        v = float(action[0])*V_MAX
        w = float(action[1])*OMEGA_MAX
        msg = Twist(); msg.linear.x=v; msg.angular.z=w; self.cmd_pub.publish(msg)
        self.rate.sleep()

        self.steps += 1
        obs   = self._get_obs()
        dist  = obs[-2]
        min_r = float(self.scan.min())

        # ---- progreso ----
        progress    = self.prev_dist - dist
        r_progress  = 1.0*progress
        self.prev_dist = dist

        # waypoint alcanzado
        r_wp = 0.0
        if dist < 0.15 and self.wp_idx < len(self.wps)-1:
            self.wp_idx += 1
            self.next_wp = np.array(self.wps[self.wp_idx], dtype=np.float32)
            r_wp = +5.0

        # centrado
        l, r = self.scan[:RAYS_USED//2].mean(), self.scan[RAYS_USED//2:].mean()
        r_center = 0.2*(1-abs(l-r)/LASER_MAX)

        # proximidad
        r_prox = 0.0
        if min_r < PROX_TH:
            r_prox = -2.0*(PROX_TH-min_r)/PROX_TH
        if min_r < COLL_TH:
            r_prox += -5.0

        reward = r_progress + r_wp + r_center + r_prox

        # ---- estancamiento ----
        moved = np.linalg.norm(self.pos - self.prev_pos) > MOVE_EPS
        if moved:
            self.last_move_time = rospy.Time.now()
        self.prev_pos = self.pos.copy()

        stagnant = (rospy.Time.now()-self.last_move_time).to_sec() > STAGN_T
        timeout  = (rospy.Time.now()-self.start_time)   .to_sec() > TIMEOUT_T
        done = stagnant or timeout
        info = {'wp_index': self.wp_idx, 'dist_wp': dist,
                'stagnant': stagnant, 'timeout': timeout}

        # ---- DEBUG print cada segundo ----
        if (rospy.Time.now()-self._last_print_t).to_sec() >= PRINT_DT:
            pos_str = f"Robot ({self.pos[0]:+5.2f},{self.pos[1]:+5.2f})"
            wp_str  = f"WP#{self.wp_idx} ({self.next_wp[0]:+5.2f},{self.next_wp[1]:+5.2f})"
            print(f"[DEBUG] {pos_str}  →  {wp_str}  dist={dist:4.2f} m")
            self._last_print_t = rospy.Time.now()

        return obs, reward, done, False, info

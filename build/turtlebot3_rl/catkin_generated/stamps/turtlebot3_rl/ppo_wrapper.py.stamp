#!/usr/bin/env python3
import os, math, yaml, numpy as np, rospy
from gym import Env, spaces
from geometry_msgs.msg import Twist
from sensor_msgs.msg   import LaserScan
from nav_msgs.msg      import Odometry
from gazebo_msgs.msg   import ContactsState   # capa A

# ---------- Constantes ----------
V_MAX, OMEGA_MAX = 0.22, 2.84
RAYS_USED, LASER_MAX = 24, 3.5
PROX_TH, COLL_TH = 0.35, 0.15
MAX_STEPS = 1000
COLL_PEN  = -10.0        # castigo fuerte

class PathTrackingWrapper(Env):
    def __init__(self):
        if not rospy.core.is_initialized():
            rospy.init_node('ppo_wrapper', anonymous=True, disable_signals=True)

        # — ROS pubs / subs —
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/scan',  LaserScan,  self.scan_cb,  queue_size=1)
        rospy.Subscriber('/odom',  Odometry,   self.odom_cb,  queue_size=1)
        # capa A: contacto físico (si plugin activo)
        rospy.Subscriber('/collision_bumper', ContactsState,
                         self.contact_cb, queue_size=1)

        # Way-points
        pkg = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(pkg, '../config/waypoints.yaml')) as f:
            self.wps = [(w['x'], w['y']) for w in yaml.safe_load(f)['waypoints']]
        self.reset_waypoints()

        # Espacios Gym
        n_scan = np.zeros(RAYS_USED, np.float32)
        p_scan = np.ones (RAYS_USED, np.float32)
        self.observation_space = spaces.Box(
            low=np.concatenate([n_scan, [0.0, -math.pi]]),
            high=np.concatenate([p_scan, [LASER_MAX, math.pi]]),
            dtype=np.float32)
        self.action_space = spaces.Box(
            low=np.array([0.0, -1.0], np.float32),
            high=np.array([1.0,  1.0], np.float32),
            dtype=np.float32)

        # Estado interno
        self.rate = rospy.Rate(10)
        self.scan_ready = self.odom_ready = False
        self.contact  = False

    # ---------- Callbacks ----------
    def scan_cb(self, msg):
        arr = np.array(msg.ranges, np.float32)
        arr[np.isinf(arr)] = LASER_MAX
        self.scan = arr[::len(arr)//RAYS_USED][:RAYS_USED]
        self.scan_ready = True

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.pos = np.array([p.x, p.y], np.float32)
        q = msg.pose.pose.orientation
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1-2*(q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)
        self.odom_ready = True

    def contact_cb(self, msg: ContactsState):
        if msg.states:                    # alguna colisión registrada
            self.contact = True

    # ---------- Utilidades ----------
    def wait_sensors(self):
        while not (self.scan_ready and self.odom_ready) and not rospy.is_shutdown():
            rospy.sleep(0.01)

    def reset_waypoints(self):
        self.wp_idx  = 0
        self.next_wp = np.array(self.wps[0], np.float32)

    def _get_obs(self):
        scan_n = self.scan / LASER_MAX
        to_wp  = self.next_wp - self.pos
        d_wp   = np.linalg.norm(to_wp)
        head   = (math.atan2(to_wp[1], to_wp[0]) - self.yaw + math.pi)%(2*math.pi)-math.pi
        return np.concatenate([scan_n, [d_wp, head]]).astype(np.float32)

    # ========== Gym API ==========
    def reset(self, *, seed=None, options=None):
        os.system('rosservice call /gazebo/reset_simulation "{}"')
        self.reset_waypoints()
        self.steps = 0
        self.contact = False
        rospy.sleep(0.2)
        self.wait_sensors()
        obs = self._get_obs()
        self.prev_dist = obs[-2]
        print("⏺ RESET")
        return obs, {}

    def step(self, action):
        # Ejecuta acción
        cmd = Twist()
        cmd.linear.x  = float(action[0]) * V_MAX
        cmd.angular.z = float(action[1]) * OMEGA_MAX
        self.cmd_pub.publish(cmd)
        self.rate.sleep()
        self.steps += 1

        obs     = self._get_obs()
        scan    = self.scan
        dist_wp = obs[-2]
        min_r   = scan.min()

        # Way-point alcanzado
        reached = dist_wp < 0.15
        if reached and self.wp_idx < len(self.wps)-1:
            self.wp_idx += 1
            self.next_wp = np.array(self.wps[self.wp_idx], np.float32)
            print(f"▶️ WP {self.wp_idx} reached")

        # ======= Recompensa =======
        reward_dist    = 2.0 * (self.prev_dist - dist_wp)      # prioridad avanzar
        if reached:
            reward_dist += 1.0

        left, right    = scan[:RAYS_USED//2].mean(), scan[RAYS_USED//2:].mean()
        reward_center  = 0.05 * (1.0 - abs(left - right)/LASER_MAX)  # peso pequeño

        prox_pen       = -2.0 * max(0.0, (PROX_TH - min_r)/PROX_TH)

        reward = reward_dist + reward_center + prox_pen
        self.prev_dist = dist_wp

        # Debug recompensa
        sign = "✅" if reward > 0 else "❌"
        print(f"{sign} Δdist={reward_dist:+.3f}  cent={reward_center:+.3f} "
              f"prox={prox_pen:+.3f} → total={reward:+.3f}")

        # ======= Terminación =======
        collided = self.contact or (min_r < COLL_TH)
        if collided:
            print(f"💥 COLLISION  min_r={min_r:.3f}")
            reward += COLL_PEN
        terminated = collided
        truncated  = self.steps >= MAX_STEPS
        if truncated:
            print("⌛ TRUNCATE")

        info = {'wp_index': self.wp_idx, 'dist_wp': dist_wp}
        self.contact = False
        return obs, reward, terminated, truncated, info

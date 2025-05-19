#!/usr/bin/env python3
"""
Gym/SB3 wrapper para TurtleBot3 con gates y obstáculos dinámicos.
Reset GLOBAL de la simulación. BORRADO MANUAL DE OBSTÁCULOS MEJORADO.
DONE SOLO POR ESTANCAMIENTO, TIMEOUT O ÉXITO. RECOMPENSA POR IR CENTRADO AÑADIDA.
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

rospack = rospkg.RosPack()
TB3_GZ_PATH = rospack.get_path('turtlebot3_gazebo')

def get_sdf(folder: str) -> str:
    model_path = os.path.join(TB3_GZ_PATH, 'models', folder, 'model.sdf')
    if not os.path.exists(model_path):
        rospy.logerr(f"SDF file not found: {model_path}")
        raise FileNotFoundError(f"SDF file not found for model: {folder} at {model_path}")
    with open(model_path, 'r') as f:
        return f.read()

# Constantes
V_MAX, OMEGA_MAX = 0.22, 2.84
LASER_MAX = 3.5
PROX_TH, COLL_TH = 0.30, 0.12 
TIMEOUT_T = 300.0
STAGN_T = 5.0 
MOVE_EPS = 0.1 # Umbral de movimiento para estancamiento (10cm)
PRINT_DT = 1.0

# Recompensas
PROG_BETA = 4.0 # Ajustado como en tu último código
GATE_BONUS = 10.0
FINISH_BONUS = 100.0
TURN_LAMBDA = 0.2 # Ajustado como en tu último código
PROX_PENALTY_FACTOR = 2.0
COLL_PENALTY_VALUE = 10.0 # Penalización por colisión (no termina episodio)
STEP_PENAL = 0.01
CENTERING_REWARD_FACTOR = 0.2 # NUEVO: Factor para recompensa por ir centrado

# Obstáculos
OBSTACLE_FOLDERS = ['construction_cone', 'turtlebot3_burger']
MAX_OBSTACLES_PER_TYPE = 1
OBSTACLE_DIST_FROM_MIDPOINT = 1.0
MIN_OBSTACLE_DIST_FROM_PATH = 0.4
MIN_DIST_FROM_ROBOT_START = 0.8 

class PathTrackingWrapper(gym.Env):
    metadata = {'render_modes': [], 'render_fps': 10}

    def __init__(self):
        super().__init__()
        if not rospy.core.is_initialized():
            rospy.init_node('ppo_env_wrapper', anonymous=True, disable_signals=True)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self._scan_cb, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self._odom_cb, queue_size=1)

        rospy.loginfo("Esperando servicios de Gazebo...")
        rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=30)
        rospy.wait_for_service('/gazebo/delete_model', timeout=30)
        rospy.wait_for_service('/gazebo/reset_simulation', timeout=30)
        self.spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.reset_simulation_srv = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        rospy.loginfo("Servicios de Gazebo conectados.")

        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        cfg_path = os.path.join(pkg_dir, '../config/waypoints.yaml')
        if not os.path.exists(cfg_path):
            rospy.logfatal(f"Waypoints file not found: {cfg_path}")
            raise FileNotFoundError(f"Waypoints file not found: {cfg_path}")
        with open(cfg_path, 'r') as f:
            cfg_data = yaml.safe_load(f)
        if 'waypoints' not in cfg_data or not cfg_data['waypoints']:
            rospy.logfatal(f"No 'waypoints' found or list is empty in {cfg_path}")
            raise ValueError(f"Invalid waypoints configuration in {cfg_path}")
        self.gates_config = self._process_gates_config(cfg_data['waypoints'])
        self.gate_midpoints = [
            np.array([(g['p1'][0] + g['p2'][0]) / 2, (g['p1'][1] + g['p2'][1]) / 2])
            for g in self.gates_config
        ]

        low = np.array([0,0,0, -np.inf,-np.inf,-math.pi, 0,0,0], dtype=np.float32)
        high = np.array([1,1,1, np.inf, np.inf, math.pi, 1,1,1], dtype=np.float32)
        self.observation_space = spaces.Box(low, high, dtype=np.float32)
        self.action_space = spaces.Box(
            low=np.array([0.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32
        )

        self.scan_ready = self.odom_ready = False
        self.current_scan_raw = np.array([LASER_MAX] * 360, dtype=np.float32)
        self.current_scan_processed = np.array([1.0, 1.0, 1.0], dtype=np.float32) # obs[0], obs[1], obs[2]
        self.current_pos = np.array([0.0, 0.0], dtype=np.float32)
        self.current_yaw = 0.0
        
        self.np_random = None
        self.seed()

        self.wait_for_sensors()
        self.ros_rate = rospy.Rate(10)
        self.last_debug_print_time = rospy.Time.now()
        self.obstacle_names_in_sim = []

        self._initialize_episode_state()
        rospy.loginfo("PathTrackingWrapper (CON OBSTACULOS, CENTRADO, SIN DONE_COL) initialized.")

    def seed(self, seed=None):
        self.np_random, seed_val = seeding.np_random(seed)
        random.seed(seed_val)
        return [seed_val]

    def _scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges[np.isinf(ranges) | np.isnan(ranges)] = LASER_MAX
        self.current_scan_raw = ranges # Guardar el scan completo para min_dist_scan y r_center
        N = len(ranges)
        if N == 0:
            self.current_scan_processed = np.array([1.0, 1.0, 1.0], dtype=np.float32)
            self.scan_ready = True
            return
        
        # Los 3 valores para la observación (puedes ajustar estos sectores)
        front_cone_half_rays = N // 12 
        front_rays_obs = np.concatenate((ranges[-front_cone_half_rays:], ranges[:front_cone_half_rays]))
        left_rays_obs = ranges[N//12 : N//4] # Sector izquierdo para observación
        right_rays_obs = ranges[N - N//4 : N - N//12] # Sector derecho para observación
        
        self.current_scan_processed = np.array([
            np.mean(left_rays_obs) if len(left_rays_obs) > 0 else LASER_MAX,
            np.mean(front_rays_obs) if len(front_rays_obs) > 0 else LASER_MAX,
            np.mean(right_rays_obs) if len(right_rays_obs) > 0 else LASER_MAX
        ], dtype=np.float32) / LASER_MAX
        self.scan_ready = True

    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.current_pos = np.array([pos.x, pos.y], dtype=np.float32)
        orient = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2*(orient.w*orient.z + orient.x*orient.y), 
                                     1 - 2*(orient.y*orient.y + orient.z*orient.z))
        self.odom_ready = True

    def wait_for_sensors(self):
        self.scan_ready = self.odom_ready = False
        rate = rospy.Rate(50)
        rospy.sleep(0.2) 
        timeout_limit = rospy.Time.now() + rospy.Duration(5.0)
        while not (self.scan_ready and self.odom_ready) and not rospy.is_shutdown():
            if rospy.Time.now() > timeout_limit:
                rospy.logwarn("wait_for_sensors: TIMEOUT esperando sensores.")
                break
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                rospy.logwarn("wait_for_sensors: Interrupción.")
                break

    def _process_gates_config(self, ws_cfg):
        processed_gates = []
        for i, entry in enumerate(ws_cfg):
            p1 = np.array([entry['x1'], entry['y1']], dtype=np.float32)
            p2 = np.array([entry['x2'], entry['y2']], dtype=np.float32)
            n_cfg = np.array([entry['nx'], entry['ny']], dtype=np.float32)
            norm_n = np.linalg.norm(n_cfg)
            if norm_n < 1e-5: n_cfg = np.array([1.,0.]) if p1[1]==p2[1] else np.array([0.,1.])
            else: n_cfg /= norm_n
            tangent = p2 - p1
            norm_t = np.linalg.norm(tangent)
            if norm_t < 1e-5: tangent = np.array([-n_cfg[1], n_cfg[0]])
            else: tangent /= norm_t
            processed_gates.append({'id': i, 'p1': p1, 'p2': p2, 'normal': n_cfg, 'tangent': tangent, 'length': norm_t})
        if not processed_gates: raise ValueError("No gates loaded from config.")
        return processed_gates

    def _get_projection_on_current_gate_tangent(self):
        if not hasattr(self, 'current_gate_details') or self.current_gate_details is None:
            return 0.0
        vec_to_pos = self.current_pos - self.current_gate_details['p1']
        return float(np.dot(vec_to_pos, self.current_gate_details['tangent']))

    def _spawn_obstacles(self):
        spawned_locations_this_reset = []
        for model_folder in OBSTACLE_FOLDERS:
            num_to_spawn = self.np_random.integers(0, MAX_OBSTACLES_PER_TYPE + 1) 
            for _ in range(num_to_spawn):
                for _attempt in range(10): 
                    if not self.gate_midpoints: break
                    mid_idx = self.np_random.choice(len(self.gate_midpoints))
                    mid_x, mid_y = self.gate_midpoints[mid_idx]
                    angle = self.np_random.uniform(0, 2 * math.pi)
                    dist_offset = self.np_random.uniform(-0.3, 0.3)
                    dist = OBSTACLE_DIST_FROM_MIDPOINT + dist_offset
                    x = mid_x + dist * math.cos(angle)
                    y = mid_y + dist * math.sin(angle)

                    if math.hypot(x, y) < MIN_DIST_FROM_ROBOT_START:
                        continue 
                    if any(math.hypot(x - ox, y - oy) < 0.6 for ox, oy in spawned_locations_this_reset):
                        continue
                    
                    req = SpawnModelRequest()
                    obs_name = f"{model_folder.replace('_', '')}_{self.np_random.integers(10000, 99999)}"
                    req.model_name = obs_name
                    try: 
                        req.model_xml = get_sdf(model_folder)
                    except FileNotFoundError: 
                        rospy.logwarn(f"SDF para {model_folder} no encontrado, no se generará.")
                        break 
                    req.robot_namespace = ""
                    req.reference_frame = "world"
                    pose = Pose()
                    pose.position.x, pose.position.y, pose.position.z = x, y, 0.01
                    q_orient = quaternion_from_euler(0,0,self.np_random.uniform(0, 2*math.pi))
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q_orient
                    req.initial_pose = pose
                    try:
                        spawn_response = self.spawn_srv.call(req)
                        if spawn_response.success:
                            self.obstacle_names_in_sim.append(obs_name)
                            spawned_locations_this_reset.append((x,y))
                        break 
                    except rospy.ServiceException as e_spawn: 
                        rospy.logwarn(f"Excepción de servicio al generar {obs_name}: {e_spawn}")
                        pass 
                else: 
                    pass

    def _delete_obstacles(self):
        if not self.obstacle_names_in_sim:
            return
        for name in self.obstacle_names_in_sim:
            try: 
                self.delete_srv.call(name)
            except rospy.ServiceException: 
                pass 
        self.obstacle_names_in_sim.clear()
        
    def _initialize_episode_state(self):
        self.current_gate_index = 0
        if not self.gates_config:
             raise EnvironmentError("Configuración de gates no cargada para inicializar estado.")
        self.current_gate_details = self.gates_config[self.current_gate_index]
        self.previous_projection = self._get_projection_on_current_gate_tangent()
        self.previous_sign_to_normal = None
        self.episode_start_time = rospy.Time.now()
        self.last_gate_crossed_time = self.episode_start_time
        self.last_significant_move_time = self.episode_start_time
        self.previous_pos_for_stagnation = np.copy(self.current_pos)

    def _get_current_observation(self):
        time_now = rospy.Time.now()
        elapsed_sec = max(0.0, (time_now - self.episode_start_time).to_sec())
        since_gate_sec = max(0.0, (time_now - self.last_gate_crossed_time).to_sec())
        norm_elapsed = min(1.0, elapsed_sec / TIMEOUT_T)
        norm_since_gate = min(1.0, since_gate_sec / TIMEOUT_T)
        completed_flag = 1.0 if self.current_gate_index >= len(self.gates_config) else 0.0
        return np.array([
            *self.current_scan_processed,
            self.current_pos[0], self.current_pos[1], self.current_yaw,
            norm_elapsed, norm_since_gate, completed_flag
        ], dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.seed(seed)

        self.cmd_pub.publish(Twist())
        rospy.sleep(0.1) 

        self._delete_obstacles() 

        if os.system('rosservice call --wait /gazebo/reset_simulation "{}"') != 0:
            rospy.logfatal("Fallo crítico: rosservice call /gazebo/reset_simulation falló.")
            raise EnvironmentError("Fallo al resetear la simulación de Gazebo vía os.system.")

        self.wait_for_sensors()
        
        # self.obstacle_names_in_sim ya está .clear()'d por _delete_obstacles()
        self.current_gate_index = 0 
        self._spawn_obstacles()

        self._initialize_episode_state()
        
        self.last_debug_print_time = rospy.Time.now()
        obs = self._get_current_observation()
        return obs, {} 

    def step(self, action):
        v = float(action[0]) * V_MAX
        w = float(action[1]) * OMEGA_MAX
        cmd = Twist(); cmd.linear.x = v; cmd.angular.z = w
        self.cmd_pub.publish(cmd)
        self.ros_rate.sleep()

        current_ros_time = rospy.Time.now()
        reward = 0.0
        
        _reason_success = False
        _reason_stagnation = False
        _reason_timeout = False

        # 1. Progreso hacia Gate
        current_projection = self._get_projection_on_current_gate_tangent()
        progress_metric = current_projection - self.previous_projection
        reward += PROG_BETA * progress_metric
        self.previous_projection = current_projection

        # 2. Cruce de Gate
        vec_to_pos_from_p1 = self.current_pos - self.current_gate_details['p1']
        current_sign_to_normal = np.sign(np.dot(vec_to_pos_from_p1, self.current_gate_details['normal']))
        
        gate_crossed_validly_this_step = False
        if self.previous_sign_to_normal is not None and \
           current_sign_to_normal != self.previous_sign_to_normal and \
           current_sign_to_normal != 0:
            if 0 <= current_projection <= self.current_gate_details['length']:
                gate_crossed_validly_this_step = True
        
        if current_sign_to_normal != 0: # Actualizar el signo para el próximo paso
            self.previous_sign_to_normal = current_sign_to_normal

        if gate_crossed_validly_this_step:
            reward += GATE_BONUS
            # rospy.logwarn(f"!!!!!!!! GATE {self.current_gate_index} CRUZADO !!!!!!!! Recompensa gate: +{GATE_BONUS}") # DEBUG
            self.current_gate_index += 1
            self.last_gate_crossed_time = current_ros_time
            if self.current_gate_index >= len(self.gates_config):
                reward += FINISH_BONUS
                _reason_success = True 
            else:
                self.current_gate_details = self.gates_config[self.current_gate_index]
                self.previous_projection = self._get_projection_on_current_gate_tangent()
                self.previous_sign_to_normal = None

        # 3. Penalización por Proximidad (Colisión NO termina el episodio)
        min_dist_scan = np.min(self.current_scan_raw) if self.current_scan_raw.size > 0 else LASER_MAX
        if min_dist_scan < PROX_TH:
            reward -= PROX_PENALTY_FACTOR * ((PROX_TH - min_dist_scan) / PROX_TH)**2
        if min_dist_scan < COLL_TH:
            reward -= COLL_PENALTY_VALUE # Solo penalización, no 'terminated = True'

        # 4. Penalización por Giro
        reward -= TURN_LAMBDA * (abs(w) / OMEGA_MAX)
        
        # 5. Penalización por Paso de Tiempo
        reward -= STEP_PENAL

        # 6. NUEVO: Recompensa por Ir Centrado
        # Usar las lecturas de LiDAR de los sectores laterales para la observación
        # self.current_scan_processed[0] es izquierda_norm, self.current_scan_processed[2] es derecha_norm
        # Queremos que sean similares (diferencia pequeña) y ambos altos (lejos de paredes)
        # Las lecturas ya están normalizadas [0,1] donde 1 es LASER_MAX (lejos)
        # y 0 es muy cerca.
        scan_left_norm = self.current_scan_processed[0]
        scan_right_norm = self.current_scan_processed[2]
        
        # Penalizar diferencia entre lados: diff va de 0 (iguales) a 1 (uno es 0, otro es 1)
        diff_sides = abs(scan_left_norm - scan_right_norm)
        # Recompensa si están balanceados (1 - diff_sides es alto si son similares)
        # y si ambos lados están despejados ( (s_l+s_r)/2 es alto )
        # Esta es una forma simple, se puede refinar.
        # Queremos que diff_sides sea cercano a 0, y que scan_left_norm y scan_right_norm sean cercanos a 1.
        # Una forma: (1 - diff_sides) * (scan_left_norm + scan_right_norm) / 2.0
        # Esto da max recompensa si diff=0 y ambos lados=1.
        centering_metric = (1.0 - diff_sides) * ((scan_left_norm + scan_right_norm) / 2.0)
        reward += CENTERING_REWARD_FACTOR * centering_metric

        # --- Lógica de Fin de Episodio ---
        current_movement = np.linalg.norm(self.current_pos - self.previous_pos_for_stagnation)
        if current_movement > MOVE_EPS:
            self.last_significant_move_time = current_ros_time
            self.previous_pos_for_stagnation = np.copy(self.current_pos)
        
        time_since_last_move = (current_ros_time - self.last_significant_move_time).to_sec()
        if time_since_last_move > STAGN_T:
            if not _reason_success: _reason_stagnation = True 

        episode_duration = (current_ros_time - self.episode_start_time).to_sec()
        if episode_duration > TIMEOUT_T:
            if not _reason_success and not _reason_stagnation : _reason_timeout = True 

        terminated = _reason_success or _reason_stagnation
        truncated = _reason_timeout

        obs = self._get_current_observation()
        info = {
            'current_gate_idx': self.current_gate_index,
            'is_success': _reason_success,
            'is_stagnant': _reason_stagnation,
            'is_timeout': _reason_timeout,
            'is_collision_event': min_dist_scan < COLL_TH,
            'min_laser_reading': min_dist_scan,
            'reward': reward,
            'progress_metric': progress_metric,
            'centering_metric': centering_metric, # Para debug
            'current_movement': current_movement,
            'time_since_last_move': time_since_last_move
        }
        
        done_flag_for_sb3 = terminated or truncated

        if done_flag_for_sb3:
            reason_str = "RAZÓN DEL DONE: "
            if _reason_success: reason_str += "ÉXITO (todos los gates)"
            elif _reason_stagnation: reason_str += f"ESTANCAMIENTO (>{STAGN_T}s sin moverse, mov: {current_movement:.4f}m)"
            elif _reason_timeout: reason_str += f"TIMEOUT (>{TIMEOUT_T}s episodio)"
            else: reason_str += "DESCONOCIDA (BUG LÓGICO)"
            rospy.logwarn(reason_str + f" | Pos: ({self.current_pos[0]:.2f},{self.current_pos[1]:.2f}) Gate: {self.current_gate_index}")
        elif (current_ros_time - self.last_debug_print_time).to_sec() >= PRINT_DT:
            gate_target_mid_x = (self.current_gate_details['p1'][0] + self.current_gate_details['p2'][0]) / 2
            gate_target_mid_y = (self.current_gate_details['p1'][1] + self.current_gate_details['p2'][1]) / 2
            pos_str = f"Robot ({self.current_pos[0]:+5.2f},{self.current_pos[1]:+5.2f})"
            gate_str  = f"Gate#{self.current_gate_index} ({gate_target_mid_x:+5.2f},{gate_target_mid_y:+5.2f})"
            dist_approx = np.linalg.norm(self.current_pos - np.array([gate_target_mid_x, gate_target_mid_y]))
            rospy.loginfo(f"[DEBUG] {pos_str}  →  {gate_str}  dist_aprox={dist_approx:4.2f} m, Rew={reward:.2f}")
            self.last_debug_print_time = current_ros_time
        
        return obs, reward, done_flag_for_sb3, False, info

    def close(self):
        rospy.loginfo("Cerrando entorno PathTrackingWrapper (con obstáculos).")
        self._delete_obstacles() 
        self.cmd_pub.publish(Twist())
        rospy.loginfo("Entorno cerrado y robot detenido.")

    def render(self, mode='human'):
        pass
#!/usr/bin/env python3
"""train_ppo.py — Entrenamiento (y reanudación) con SubprocVecEnv."""
import os
import rospy
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from ppo_wrapper import PathTrackingWrapper

# Parámetros de entrenamiento
TOTAL_STEPS            = 400_000    # timesteps
SAVE_EVERY             = 25_000     # cada cuántos pasos guardar checkpoint
DEVICE                 = 'cpu'      # fuerza CPU
INITIAL_GAZEBO_WAIT    = 5.0        # Segundos de espera para que Gazebo se inicialice
PRETRAINED_MODEL_PATH  = './models/ppo_gates_simple_checkpoint_600000_steps'  # ruta sin .zip

def make_env():
    """Factory simple para DummyVecEnv."""
    return PathTrackingWrapper()

def main():
    # Inicializa ROS si no está
    if not rospy.core.is_initialized():
        rospy.init_node('train_ppo_script', anonymous=True, disable_signals=True)

    rospy.loginfo("--- Script de entrenamiento principal iniciado ---")
    rospy.loginfo(f"Esperando {INITIAL_GAZEBO_WAIT} segundos para que Gazebo arranque...")
    rospy.sleep(INITIAL_GAZEBO_WAIT)
    rospy.loginfo("Creando entornos...")

    # 1) Vector de entornos secuencial
    env      = DummyVecEnv([make_env])
    eval_env = DummyVecEnv([make_env])

    # 2) Callbacks
    chkpt_cb = CheckpointCallback(
        save_freq=SAVE_EVERY,
        save_path='./models/',
        name_prefix='ppo_gates_simple_checkpoint'
    )
    eval_cb = EvalCallback(
        eval_env,
        best_model_save_path='./best_gates_simple/',
        log_path='./logs_gates_simple/',
        eval_freq=SAVE_EVERY,
        deterministic=True,
        render=False
    )

    # 3) Crear o reanudar modelo
    if os.path.exists(PRETRAINED_MODEL_PATH + '.zip'):
        rospy.loginfo(f"Cargando modelo preentrenado desde {PRETRAINED_MODEL_PATH}.zip")
        print(f"Cargando modelo preentrenado desde {PRETRAINED_MODEL_PATH}.zip")
        model = PPO.load(
            PRETRAINED_MODEL_PATH,
            env=env,
            device=DEVICE,
            tensorboard_log='./tb_gates_simple/'
        )
    else:
        rospy.loginfo("Entrenando modelo desde cero (7-dim obs, obstáculos estáticos)")
        print("Entrenando modelo desde cero (7-dim obs, obstáculos estáticos)")
        model = PPO(
            policy='MlpPolicy',
            env=env,
            verbose=1,
            tensorboard_log='./tb_gates_simple/',
            device=DEVICE
        )

    # 4) Aprender
    rospy.loginfo("Comenzando aprendizaje del modelo...")
    model.learn(
        total_timesteps=TOTAL_STEPS,
        reset_num_timesteps=False,
        callback=[chkpt_cb, eval_cb]
    )
    rospy.loginfo("Aprendizaje completado.")

    # 5) Guardar y cerrar
    final_model_name = 'ppo_gates_simple_final'
    model.save(final_model_name)
    rospy.loginfo(f"Modelo guardado como {final_model_name}.zip")
    print(f"Modelo guardado como {final_model_name}.zip")

    env.close()
    if eval_env is not env:
        eval_env.close()
    rospy.loginfo("Entornos cerrados.")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.logwarn("⏹ Entrenamiento interrumpido por usuario")
        print("⏹ Entrenamiento interrumpido por usuario")
    except Exception as e:
        rospy.logfatal(f"EXCEPCIÓN NO CAPTURADA: {e}")
        print(f"EXCEPCIÓN NO CAPTURADA: {e}")
        import traceback; traceback.print_exc()
    finally:
        rospy.loginfo("--- Script finalizado (finally) ---")
        print("--- Script finalizado (finally) ---")

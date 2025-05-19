#!/usr/bin/env python3
"""train_ppo.py — Entrenamiento **desde cero** sin paralelizar con SubprocVecEnv."""
import os
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from ppo_wrapper import PathTrackingWrapper # Se usará el ppo_wrapper.py que te acabo de dar

# Es MUY RECOMENDABLE añadir rospy aquí si vas a usar rospy.sleep
# y para asegurar que el nodo se inicializa antes de cualquier cosa si es necesario,
# aunque el wrapper también lo inicializa. Por seguridad y para la pausa:
import rospy

# Parámetros de entrenamiento
TOTAL_STEPS = 400_000    # timesteps
SAVE_EVERY  = 25_000     # cada cuántos pasos guardar checkpoint
DEVICE      = 'cpu'      # fuerza CPU (útil sin GPU)
INITIAL_GAZEBO_WAIT = 5.0 # Segundos de espera para que Gazebo se inicialice

def make_env():
    """Factory simple para DummyVecEnv."""
    return PathTrackingWrapper()

def main():
    # Pausa inicial para Gazebo (MUY IMPORTANTE)
    # Aunque el wrapper inicializa su propio nodo, es bueno que el script principal
    # también lo haga si va a usar funciones de ROS como rospy.sleep o rospy.loginfo
    # antes de que el wrapper sea instanciado.
    if not rospy.core.is_initialized():
        rospy.init_node('train_ppo_script', anonymous=True, disable_signals=True)
    
    rospy.loginfo(f"--- Script de entrenamiento principal iniciado ---")
    rospy.loginfo(f"Esperando {INITIAL_GAZEBO_WAIT} segundos para que Gazebo se inicialice completamente...")
    rospy.sleep(INITIAL_GAZEBO_WAIT)
    rospy.loginfo("Pausa para Gazebo completada. Creando entornos...")


    # 1) Creamos un vector de 1 entorno (secuencial, en el mismo proceso)
    env      = DummyVecEnv([make_env])    # entrenamiento
    eval_env = DummyVecEnv([make_env])    # evaluación

    # 2) Callbacks
    chkpt_cb = CheckpointCallback(
        save_freq=SAVE_EVERY,
        save_path='./models/',
        name_prefix='ppo_gates_simple' # Nombre de modelo un poco más descriptivo
    )
    eval_cb = EvalCallback(
        eval_env,
        best_model_save_path='./best_gates_simple/', # Directorio para el mejor modelo
        log_path='./logs_gates_simple/',             # Directorio para logs de evaluación
        eval_freq=SAVE_EVERY,
        deterministic=True,
        render=False
    )

    # 3) Crear el modelo desde cero
    rospy.loginfo("🚀 Entrenando modelo desde cero (9-dim obs, reset global, sin obstáculos dinámicos)")
    print("🚀 Entrenando modelo desde cero (9-dim obs, reset global, sin obstáculos dinámicos)") # También en consola
    model = PPO(
        policy='MlpPolicy',
        env=env,
        verbose=1,
        tensorboard_log='./tb_gates_simple/', # Directorio para TensorBoard
        device=DEVICE
        # Usando los hiperparámetros por defecto de PPO en SB3 para esta configuración simple
    )

    # 4) Aprender
    rospy.loginfo("Comenzando el aprendizaje del modelo...")
    model.learn(
        total_timesteps=TOTAL_STEPS,
        reset_num_timesteps=False, # Continuar timesteps si se reanuda
        callback=[chkpt_cb, eval_cb]
    )
    rospy.loginfo("Aprendizaje completado.")

    # 5) Guardar y cerrar
    final_model_name = 'ppo_gates_simple_final'
    model.save(final_model_name)
    rospy.loginfo(f"Modelo final guardado como {final_model_name}.zip")
    print(f"Modelo final guardado como {final_model_name}.zip")
    
    env.close()
    if eval_env is not env: # Buena práctica cerrar eval_env también
        eval_env.close()
    rospy.loginfo("Entornos cerrados.")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.logwarn("⏹ Entrenamiento interrumpido por el usuario (KeyboardInterrupt)")
        print("⏹ Entrenamiento interrumpido por el usuario (KeyboardInterrupt)")
    except Exception as e:
        rospy.logfatal(f"EXCEPCIÓN NO CAPTURADA EN MAIN SCRIPT: {e}")
        print(f"EXCEPCIÓN NO CAPTURADA EN MAIN SCRIPT: {e}")
        import traceback
        traceback.print_exc() # Imprimir el traceback completo de la excepción
    finally:
        rospy.loginfo("--- Script de entrenamiento principal finalizado (bloque finally) ---")
        print("--- Script de entrenamiento principal finalizado (bloque finally) ---")
        # Asegurarse de que los nodos ROS se cierren si es necesario, aunque
        # disable_signals=True y el cierre del script deberían manejarlo.
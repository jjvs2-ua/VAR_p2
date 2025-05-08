#!/usr/bin/env python3
import os
import sys
import rospy
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from ppo_wrapper import PathTrackingWrapper

def main():
    # Inicializa nodo ROS (sin señales para capturar ROSInterruptException)
    rospy.init_node('train_ppo', disable_signals=True)

    # Crea el entorno
    env = PathTrackingWrapper()

    # Callbacks de guardado periódico y evaluación
    chkpt_cb = CheckpointCallback(
        save_freq=10_000,          # ahora cada 10k pasos
        save_path='./models/',
        name_prefix='ppo_cb',
        verbose=1
    )
    eval_cb = EvalCallback(
        env,
        best_model_save_path='./best/',
        log_path='./logs/',
        eval_freq=5_000,           # evalúa cada 5k pasos
        deterministic=True,
        verbose=1
    )

    # Asegura que existe la carpeta de modelos
    os.makedirs('models', exist_ok=True)
    os.makedirs('best', exist_ok=True)

    # Crea o carga modelo
    model = PPO(
        'MlpPolicy',
        env,
        verbose=1,
        tensorboard_log='./tensorboard/',
        learning_rate=3e-4,
        batch_size=64,
        n_steps=2048,
        clip_range=0.2
    )

    # Entrena y captura interrupciones
    try:
        model.learn(
            total_timesteps=1_000_000,
            callback=[chkpt_cb, eval_cb]
        )
        # Guarda modelo final
        model.save('ppo_turtlebot3_final')
        print("✅ Entrenamiento completado y modelo guardado como ppo_turtlebot3_final.zip")

    except (KeyboardInterrupt, rospy.exceptions.ROSInterruptException):
        # Guarda al interrumpir
        save_path = 'models/ppo_interrupted'
        model.save(save_path)
        print(f"⏹ Entrenamiento interrumpido; modelo guardado en {save_path}.zip")
        sys.exit(0)

if __name__ == '__main__':
    main()


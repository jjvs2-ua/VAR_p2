#!/usr/bin/env python3
import os
import rospy
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from ppo_wrapper import PathTrackingWrapper

def main():
    rospy.init_node('train_ppo', anonymous=False, disable_signals=True)
    env = PathTrackingWrapper()

    # ⟹ guardar modelo cada 200 k steps y evaluar cada 50 k
    chkpt_cb = CheckpointCallback(
        save_freq=200_000,
        save_path='./models/',
        name_prefix='ppo'
    )
    eval_cb = EvalCallback(
        env,
        best_model_save_path='./best/',
        log_path='./logs/',
        eval_freq=50_000,
        deterministic=True,
        render=False
    )

    # Ruta al modelo preentrenado
    pkg_dir   = os.path.dirname(os.path.abspath(__file__))
    pretrained_path = os.path.join(pkg_dir, 'models',
                                   'ppo_400000_steps_primerEntrenamiento.zip')

    if os.path.isfile(pretrained_path):
        rospy.loginfo(f"Cargando modelo preentrenado desde {pretrained_path}")
        model = PPO.load(
            pretrained_path,
            env=env,
            device='auto'
        )
        # Ajustamos la ruta de tensorboard si es necesario
        model.tensorboard_log = './tensorboard/'
    else:
        rospy.logwarn(f"No encontrado modelo en {pretrained_path}, entrenando desde cero")
        model = PPO(
            'MlpPolicy',
            env,
            verbose=1,
            tensorboard_log='./tensorboard/',
            batch_size=256,
            learning_rate=3e-4,
            n_steps=2048,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            device='auto'
        )

    # Continúa el entrenamiento hasta 1_000_000 timesteps adicionales,
    # sin reiniciar el contador de timesteps si cargamos un modelo.
    model.learn(
        total_timesteps=1_000_000,
        callback=[chkpt_cb, eval_cb],
        reset_num_timesteps=False
    )

    # Guardamos el modelo final y cerramos el entorno
    model.save('ppo_path_tracker_latest')
    env.close()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("⏹  Entrenamiento interrumpido por el usuario")

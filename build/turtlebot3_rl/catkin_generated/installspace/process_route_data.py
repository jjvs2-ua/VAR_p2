#!/usr/bin/env python3
import sys
import numpy as np
import pandas as pd
from scipy.interpolate import interp1d
import math

# Parámetros
ODOM_WEIGHT   = 8.0
STEP_DIST     = 0.1   # cada 0.1 m
MU, G         = 0.3, 9.81  # para el perfil de velocidad

def radius(p0,p1,p2):
    """Radio de circunferencia que pasa por tres puntos."""
    (x1,y1),(x2,y2),(x3,y3) = p0,p1,p2
    a = math.hypot(x2-x1, y2-y1)
    b = math.hypot(x3-x2, y3-y2)
    c = math.hypot(x1-x3, y1-y3)
    s = (a+b+c)/2
    A = math.sqrt(max(s*(s-a)*(s-b)*(s-c), 1e-6))
    return (a*b*c)/(4*A)

def main(csv_path):
    # 1) Carga CSV
    df = pd.read_csv(csv_path)

    # 2) Recupera x,y,yaw originales
    df['x']   = df['pos_x_w'] / ODOM_WEIGHT
    df['y']   = df['pos_y_w'] / ODOM_WEIGHT
    coords    = df[['x','y']].to_numpy()

    # 3) Calcula distancia acumulada
    d = np.sqrt(((coords[1:] - coords[:-1])**2).sum(axis=1))
    cd = np.concatenate([[0], np.cumsum(d)])

    # 4) Densifica cada STEP_DIST
    L = cd[-1]
    u = np.arange(0, L, STEP_DIST)
    fx = interp1d(cd, coords[:,0], kind='linear')
    fy = interp1d(cd, coords[:,1], kind='linear')
    waypoints = np.vstack((fx(u), fy(u))).T

    # 5) Genera perfil de velocidad
    N = waypoints.shape[0]
    v_profile = np.zeros(N)
    for i in range(1, N-1):
        r = radius(waypoints[i-1], waypoints[i], waypoints[i+1])
        v_max = math.sqrt(MU * G * r)
        v_profile[i] = np.clip(v_max, 0.1, 0.5)
    v_profile[0], v_profile[-1] = v_profile[1], v_profile[-2]

    # 6) Guarda ficheros
    np.save('waypoints.npy', waypoints)
    np.save('v_profile.npy', v_profile)
    print(f"Guardado waypoints.npy ({N} puntos) y v_profile.npy")

if __name__ == '__main__':
    if len(sys.argv)!=2:
        print("Uso: process_route_data.py <ruta/route_data.csv>")
        sys.exit(1)
    main(sys.argv[1])

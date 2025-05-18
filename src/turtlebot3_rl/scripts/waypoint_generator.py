#!/usr/bin/env python3
"""
Genera config/waypoints.yaml a partir de route_data.csv
En lugar de puntos aislados crea “gates”: líneas perpendiculares al eje
del circuito que el robot puede cruzar en cualquier punto de su anchura.

Cada gate se define por:
    (x, y)  → punto central de la línea
    (nx, ny)→ vector normal (longitud 1) que señala el lado “aún no cruzado”

Un nuevo gate se coloca cada MIN_DIST metros a lo largo de la trayectoria
registrada en route_data.csv.
"""
import pandas as pd
import numpy as np
import yaml
import pathlib

CSV         = 'route_data.csv'
OUT_YAML    = '../config/waypoints.yaml'          # dentro del paquete ROS
MIN_DIST    = 15.0                                # [m] separación mínima

def main():
    df  = pd.read_csv(CSV, usecols=['pos_x_w', 'pos_y_w'])
    pts = df[['pos_x_w', 'pos_y_w']].values        # (N, 2)

    gates = []
    last  = pts[0]

    for i in range(1, len(pts)):
        p = pts[i]
        d = np.linalg.norm(p - last)
        if d >= MIN_DIST:
            #   Tangente aproximada = dirección de avance
            t = p - last
            t = t / np.linalg.norm(t)
            #   Normal (giro 90°) → línea perpendicular
            n = np.array([-t[1], t[0]])
            gates.append({
                'x':  float(p[0]) / 8.0,
                'y':  float(p[1]) / 8.0,
                'nx': float(n[0]),
                'ny': float(n[1])
            })
            last = p

    pathlib.Path('../config').mkdir(parents=True, exist_ok=True)
    with open(OUT_YAML, 'w') as f:
        yaml.safe_dump({'waypoints': gates}, f)

    print(f'Generados {len(gates)} gates → {OUT_YAML}')

if __name__ == '__main__':
    main()

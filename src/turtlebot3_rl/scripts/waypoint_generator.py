#!/usr/bin/env python3
"""
Genera config/waypoints.yaml a partir de route_data.csv
Guarda un waypoint cada 0.30 m (ajusta MIN_DIST si lo deseas).
"""
import pandas as pd
import numpy as np
import yaml
import pathlib

CSV         = 'route_data.csv'
OUT_YAML    = '../config/waypoints.yaml'   # dentro del paquete ROS
MIN_DIST    = 2                      # [m] separación mínima entre way-points

def main():
    df  = pd.read_csv(CSV, usecols=['pos_x_w', 'pos_y_w'])
    pts = df[['pos_x_w', 'pos_y_w']].values                     # (N,2)
    waypoints = [tuple(pts[0])]
    last = pts[0]
    for p in pts[1:]:
        if np.linalg.norm(p - last) >= MIN_DIST:
            waypoints.append(tuple(p))
            last = p

    data = {'waypoints': [{'x': float(x)/8, 'y': float(y)/8} for x, y in waypoints]}
    pathlib.Path('../config').mkdir(parents=True, exist_ok=True)
    with open(OUT_YAML, 'w') as f: yaml.safe_dump(data, f)
    print(f'Generados {len(waypoints)} way-points → {OUT_YAML}')

if __name__ == '__main__':
    main()

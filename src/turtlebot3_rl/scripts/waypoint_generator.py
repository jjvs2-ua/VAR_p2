#!/usr/bin/env python3
import pandas as pd
import numpy as np
import yaml
import pathlib

CSV         = 'route_data.csv'
OUT_YAML    = '../config/waypoints.yaml'   # dentro del paquete ROS
MIN_DIST    = 15.0                         # [m] separación mínima entre gates
LONG_TOTAL  = 0.5                          # [m] longitud total de cada gate
SCALE       = 8.0                          # factor de escala inversa (raw→sim) si aplica


def main():
    df  = pd.read_csv(CSV, usecols=['pos_x_w', 'pos_y_w'])
    pts = df[['pos_x_w', 'pos_y_w']].values        # (N, 2) en unidades raw

    gates = []
    last  = pts[0]
    half_raw = (LONG_TOTAL / 2.0) * SCALE          # medio segmento en unidades raw

    for p in pts[1:]:
        d = np.linalg.norm(p - last)
        if d >= MIN_DIST:
            # dirección tangente unitaria (avance)
            t = (p - last) / d
            # dirección normal unitaria (90°)
            n = np.array([-t[1], t[0]], dtype=np.float64)
            # extremos del segmento en raw
            p1 = p + n * half_raw
            p2 = p - n * half_raw
            # convertir a sim (metros reales) y guardar
            gates.append({
                'x1': float(p1[0]) / SCALE,
                'y1': float(p1[1]) / SCALE,
                'x2': float(p2[0]) / SCALE,
                'y2': float(p2[1]) / SCALE,
                'nx': float(n[0]),
                'ny': float(n[1])
            })
            last = p

    pathlib.Path(pathlib.Path(OUT_YAML).parent).mkdir(parents=True, exist_ok=True)
    with open(OUT_YAML, 'w') as f:
        yaml.safe_dump({'waypoints': gates}, f, default_flow_style=False)

    print(f'Generados {len(gates)} gates de {LONG_TOTAL} m → {OUT_YAML}')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Estima el retardo mando -> efecto a partir del registro del controlador.

Lee el CSV de `log_csv` (rpp_node) y busca el desfase que mejor alinea:

- el mando de direccion publicado (`steer_cmd`) con la realimentacion
  `steer_fb` del puente: retardo puro del camino nodo -> puente ->
  simulador -> puente -> nodo (`command_delay`);
- la guinada que pediria ese mando por bicicleta cinematica,
  v * tan(steer_cmd * 0.524) / 0.33, con la guinada medida por el IMU
  (`yaw_rate`): retardo total incluido el servo. La diferencia con el
  anterior aproxima la constante del servo (`steering_lag`).

Ambas senales se remuestrean a 50 Hz y se correlacionan para desfases de
0 a 2 s; se imprime el de maxima correlacion y la correlacion conseguida
(cerca de 1 = medida fiable). Sirve cualquier pasada con curvas, incluso
una que acabe en choque.

  python3 tools/estimate_delay.py /tmp/rpp.csv
"""
import argparse
import math

import numpy as np


def best_lag(t, a, b, max_lag=2.0, fs=50.0):
    """Desfase (s) que hay que aplicar a `a` para que se parezca mas a `b`."""
    grid = np.arange(t[0], t[-1], 1.0 / fs)
    a = np.interp(grid, t, a)
    b = np.interp(grid, t, b)
    a = a - a.mean()
    b = b - b.mean()
    if a.std() < 1e-9 or b.std() < 1e-9:
        return float('nan'), 0.0
    best, best_corr = 0.0, -1.0
    for k in range(int(max_lag * fs) + 1):
        aa, bb = a[:len(a) - k], b[k:]
        corr = float(np.dot(aa, bb) / (np.linalg.norm(aa) * np.linalg.norm(bb) + 1e-12))
        if corr > best_corr:
            best, best_corr = k / fs, corr
    return best, best_corr


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv', help='registro log_csv de rpp_node')
    ap.add_argument('--wheelbase', type=float, default=0.33)
    ap.add_argument('--max-steering', type=float, default=0.524, help='rad del mando 1.0')
    args = ap.parse_args()

    d = np.genfromtxt(args.csv, delimiter=',', names=True)
    for col in ('steer_cmd', 'steer_fb', 'yaw_rate', 'v_tf'):
        if col not in d.dtype.names:
            raise SystemExit(f'{args.csv}: falta la columna {col} (registro de una version anterior)')
    t = d['t'] - d['t'][0]
    moving = d['v_tf'] > 0.2
    if moving.sum() < 20:
        raise SystemExit('el coche apenas se movio en este registro')
    lag_fb, c_fb = best_lag(t, d['steer_cmd'], d['steer_fb'])
    expected = d['v_tf'] * np.tan(d['steer_cmd'] * args.max_steering) / args.wheelbase
    lag_yaw, c_yaw = best_lag(t, expected, d['yaw_rate'])
    scale = (np.abs(d['steer_fb']).max() / max(np.abs(d['steer_cmd']).max(), 1e-6))
    print(f'{len(d)} ciclos, {t[-1]:.1f} s, cadencia {len(d) / t[-1]:.1f} Hz')
    print(f'mando -> realimentacion del servo: {lag_fb:.2f} s (correlacion {c_fb:.2f}; '
          f'realimentacion/mando = {scale:.2f})')
    print(f'mando -> guinada del IMU:          {lag_yaw:.2f} s (correlacion {c_yaw:.2f})')
    if not math.isnan(lag_fb) and not math.isnan(lag_yaw):
        print(f'Sugerencia: command_delay ~ {lag_fb:.1f} s, steering_lag ~ {max(lag_yaw - lag_fb, 0.0):.2f} s')
        print('(si la correlacion baja de 0.7 la pasada tiene pocas curvas: repetir)')


if __name__ == '__main__':
    main()

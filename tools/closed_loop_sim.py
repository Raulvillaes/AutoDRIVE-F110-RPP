#!/usr/bin/env python3
"""Simulacion cerrada del controlador sin AutoDRIVE.

Instancia el `RppNode` real (con un archivo de parametros) y le da poses a
la cadencia del puente desde un modelo de coche: bicicleta cinematica de
batalla 0.33 m, retardo puro entre el mando y su aplicacion, servo de
direccion de primer orden y el acelerador medido con tools/measure_speed.py
(5 m/s por unidad de mando, 90 % en ~1 s, frenada por inercia de 3 m/s^2).
No hay LiDAR: la regulacion por proximidad no actua.

Sirve para ver que aporta cada parametro antes de gastar una pasada en el
simulador, sobre todo `command_delay` y `steering_lag`: el error lateral
crece rapido si el retardo simulado no coincide con el configurado.

Uso (ROS 2 sourceado bajo bash, sin puente ni simulador):

  python3 tools/closed_loop_sim.py config/params.yaml --delay 0.5
  python3 tools/closed_loop_sim.py otro.yaml --delay 0.7 --servo 0.3 --laps 5

Imprime los tiempos de vuelta, el error lateral rms y maximo respecto al
CSV y la velocidad maxima alcanzada.
"""
import argparse
import collections
import math
import os
import sys

import numpy as np
import rclpy

sys.path.insert(0, __file__.rsplit('/tools/', 1)[0])
from rpp_f110.lap_counter import LapCounter  # noqa: E402
from rpp_f110.rpp_node import RppNode  # noqa: E402
from rpp_f110.vehicle_pose import VehiclePose  # noqa: E402

START = (0.745, 3.158, -math.pi / 2)
FINISH = ((1.47, 3.16), (0.29, 3.16), (0, -1))


def simulate(params_file, delay, servo, laps, rate, seed=0, verbose=True):
    rclpy.init(args=['--ros-args', '--params-file', os.path.abspath(params_file)])
    node = RppNode()
    cmds = {}
    node.publish_commands = lambda thr, st: cmds.update(thr=thr, st=st)
    node.publish_debug = lambda *a: None
    tr = node.traj
    rng = np.random.default_rng(seed)

    x, y, yaw = START
    v, delta = 0.0, 0.0
    dt, t, next_pose = 0.005, 0.0, 0.0
    queue = collections.deque()          # (instante de aplicacion, acelerador, direccion)
    thr_act, st_act = 0.0, 0.0
    counter = LapCounter(*FINISH, min_lap_time=5, rearm_distance=1.0)
    lap_times, err, v_max = [], [], 0.0
    while t < 60.0 * laps and len(lap_times) < laps:
        if t >= next_pose:
            next_pose += (1.0 + 0.1 * rng.standard_normal()) / rate
            node.on_pose(VehiclePose(t, x, y, yaw))
            if cmds:
                queue.append((t + delay, cmds['thr'], cmds['st']))
            i = tr.nearest_index(x, y)
            h = tr.heading(i)
            err.append(-math.sin(h) * (x - tr.x[i]) + math.cos(h) * (y - tr.y[i]))
            for e in counter.update(t, x, y):
                if e.kind == 'lap':
                    lap_times.append(e.lap_time)
                    if verbose:
                        print(f'vuelta {len(lap_times)}: {e.lap_time:.2f} s')
        while queue and queue[0][0] <= t:
            _, thr_act, st_act = queue.popleft()
        target = st_act * node.max_steering_angle
        delta += (target - delta) * dt / servo if servo > 0 else (target - delta)
        v += max(-3.0, min(4.0, (5.0 * thr_act - v) / 0.4)) * dt
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += v * math.tan(delta) / node.wheelbase * dt
        v_max = max(v_max, v)
        t += dt
    node.destroy_node()
    rclpy.shutdown()
    err = np.abs(err)
    return lap_times, float(np.sqrt((err ** 2).mean())), float(err.max()), v_max


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('params_file', help='YAML de parametros (formato de config/params.yaml)')
    ap.add_argument('--delay', type=float, default=0.5, help='s de retardo puro mando -> aplicacion')
    ap.add_argument('--servo', type=float, default=0.15, help='s, constante de tiempo del servo (0 = instantaneo)')
    ap.add_argument('--laps', type=int, default=3, help='vueltas a simular')
    ap.add_argument('--rate', type=float, default=4.5, help='Hz de la cadencia de poses')
    args = ap.parse_args()
    laps, rms, mx, v_max = simulate(args.params_file, args.delay, args.servo, args.laps, args.rate)
    print(f'retardo {args.delay} s + servo {args.servo} s: vueltas {["%.1f" % l for l in laps]}  '
          f'error lateral rms {rms:.2f} m, max {mx:.2f} m  v max {v_max:.2f} m/s')


if __name__ == '__main__':
    main()

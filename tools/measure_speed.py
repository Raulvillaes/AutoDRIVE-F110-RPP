#!/usr/bin/env python3
"""Mide la respuesta del acelerador del F1TENTH de AutoDRIVE.

El coche solo acepta un mando de acelerador normalizado [-1, 1] y no
publica su velocidad. Este script aplica un escalon de mando con la
direccion a cero, registra la velocidad derivada del TF (desplazamiento
entre poses / tiempo) y la velocidad angular de los encoders, corta el
mando cuando el LiDAR ve pared cerca, se agota la distancia o el tiempo, y
sigue grabando la frenada por inercia hasta que el coche se para.

Al final imprime: cadencia del puente, velocidad estacionaria, radio de
rueda implicito (v_tf / omega_encoder) y deceleracion en inercia, y guarda
un CSV con todas las muestras.

Uso (ROS 2 sourceado bajo bash, simulador en Autonomous, coche en la
salida mirando a la recta de 6.7 m):

  python3 tools/measure_speed.py 0.3                    # un escalon de 0.3
  python3 tools/measure_speed.py 0.15 0.5 --duration 2.5 # dos escalones seguidos,
                                                         # 2.5 s cada uno
"""
import argparse
import csv
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float32

sys.path.insert(0, __file__.rsplit('/tools/', 1)[0])
from rpp_f110.vehicle_pose import stamp_to_sec, subscribe_vehicle_pose  # noqa: E402

THROTTLE_TOPIC = '/autodrive/f1tenth_1/throttle_command'
STEERING_TOPIC = '/autodrive/f1tenth_1/steering_command'


class StepTest(Node):
    def __init__(self, args):
        super().__init__('measure_speed')
        self.args = args
        self.throttle_pub = self.create_publisher(Float32, THROTTLE_TOPIC, 10)
        self.steering_pub = self.create_publisher(Float32, STEERING_TOPIC, 10)
        self.pose_subs = subscribe_vehicle_pose(self, self.on_pose)
        self.create_subscription(LaserScan, '/autodrive/f1tenth_1/lidar', self.on_scan, 10)
        self.create_subscription(JointState, '/autodrive/f1tenth_1/left_encoder',
                                 lambda m: self.on_encoder('left', m), 10)
        self.create_subscription(JointState, '/autodrive/f1tenth_1/right_encoder',
                                 lambda m: self.on_encoder('right', m), 10)
        self.create_timer(0.05, self.tick)      # 20 Hz: el puente repite el ultimo mando

        self.phase = 'wait'          # wait -> run -> coast -> done
        self.throttle = 0.0
        self.t0 = None               # inicio del escalon (reloj del puente)
        self.t_cut = None            # corte del escalon
        self.pose0 = None
        self.prev_pose = None
        self.d_front = math.inf
        self.omega = {'left': 0.0, 'right': 0.0}
        self.enc_prev = {'left': None, 'right': None}
        self.rows = []
        self.cut_reason = ''
        self.wall_start = time.monotonic()

    def on_scan(self, msg):
        r = np.asarray(msg.ranges, dtype=float)
        ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        front = r[np.abs(ang) <= math.radians(10.0)]
        ok = np.isfinite(front) & (front >= msg.range_min)
        self.d_front = float(front[ok].min()) if ok.any() else math.inf

    def on_encoder(self, side, msg):
        if not msg.position:
            return
        t, a = stamp_to_sec(msg.header.stamp), msg.position[0]
        prev = self.enc_prev[side]
        self.enc_prev[side] = (t, a)
        if prev and t > prev[0] and abs(a - prev[1]) < math.pi:
            self.omega[side] = (a - prev[1]) / (t - prev[0])

    def on_pose(self, pose):
        if self.phase == 'wait':
            self.phase = 'run'
            self.step = 0
            self.throttle = self.args.throttle[0]
            self.t0 = pose.t
            self.t_step = pose.t
            self.pose0 = pose
            self.get_logger().info(f'Escalon de {self.throttle} aplicado.')
        v_tf = 0.0
        if self.prev_pose is not None and pose.t > self.prev_pose.t:
            dx, dy = pose.x - self.prev_pose.x, pose.y - self.prev_pose.y
            v_tf = (dx * math.cos(pose.yaw) + dy * math.sin(pose.yaw)) / (pose.t - self.prev_pose.t)
        dist = math.hypot(pose.x - self.pose0.x, pose.y - self.pose0.y)
        omega = 0.5 * (self.omega['left'] + self.omega['right'])
        self.rows.append({'t': pose.t - self.t0, 'phase': self.phase, 'step': getattr(self, 'step', -1),
                          'throttle': self.throttle,
                          'x': pose.x, 'y': pose.y, 'v_tf': v_tf, 'omega': omega,
                          'dist': dist, 'd_front': self.d_front})
        self.prev_pose = pose

        if self.phase == 'run':
            elapsed = pose.t - self.t0
            if self.d_front < self.args.abort_range:
                self.cut('pared a %.2f m' % self.d_front, pose.t)
            elif dist > self.args.max_distance:
                self.cut('distancia %.2f m' % dist, pose.t)
            elif pose.t - self.t_step > self.args.duration:
                if self.step + 1 < len(self.args.throttle):
                    self.step += 1
                    self.throttle = self.args.throttle[self.step]
                    self.t_step = pose.t
                    self.get_logger().info(f'Escalon de {self.throttle} aplicado.')
                else:
                    self.cut('tiempo %.1f s' % elapsed, pose.t)
        elif self.phase == 'coast':
            if abs(v_tf) < 0.03 or pose.t - self.t_cut > self.args.coast:
                self.phase = 'done'

    def cut(self, reason, t):
        self.phase = 'coast'
        self.throttle = self.args.brake
        self.t_cut = t
        self.cut_reason = reason
        self.get_logger().info(f'Corte por {reason}; grabando la frenada.')

    def tick(self):
        self.throttle_pub.publish(Float32(data=float(self.throttle)))
        self.steering_pub.publish(Float32(data=0.0))
        if self.phase == 'wait' and time.monotonic() - self.wall_start > self.args.wait:
            self.get_logger().error('No llega /tf: puente caido o simulador sin conectar.')
            self.phase = 'done'

    def report(self):
        rows = self.rows
        if len(rows) < 4:
            print('Muy pocas muestras.')
            return
        run = [r for r in rows if r['phase'] == 'run']
        coast = [r for r in rows if r['phase'] == 'coast']
        rate = (len(rows) - 1) / (rows[-1]['t'] - rows[0]['t'])
        print('\n' + '=' * 60)
        print(f'Escalones de mando {self.args.throttle}, corte por {self.cut_reason}')
        print(f'Cadencia del puente:      {rate:5.1f} Hz  ({len(rows)} muestras)')
        for k, thr in enumerate(self.args.throttle):
            step = [r for r in run if r['step'] == k]
            if not step:
                continue
            t_end = step[-1]['t']
            win = [r for r in step if r['t'] >= t_end - 0.8]
            v_ss = float(np.mean([r['v_tf'] for r in win]))
            v_max = max(r['v_tf'] for r in step)
            print(f'Mando {thr:4.2f}: velocidad final {v_ss:5.2f} m/s (max {v_max:.2f}) '
                  f'tras {t_end - step[0]["t"]:.1f} s; recorrido acumulado {step[-1]["dist"]:.2f} m')
            t90 = next((r['t'] - step[0]['t'] for r in step if r['v_tf'] >= 0.9 * v_ss), None)
            if t90 is not None and k == 0:
                print(f'   tiempo al 90 %:        {t90:5.2f} s')
        if len(coast) >= 3:
            t = np.array([r['t'] for r in coast]); v = np.array([r['v_tf'] for r in coast])
            slope = np.polyfit(t, v, 1)[0]
            print(f'Frenada con mando {self.args.brake}: {slope:+.2f} m/s^2 sobre {coast[-1]["t"] - coast[0]["t"]:.1f} s, '
                  f'{coast[-1]["dist"] - coast[0]["dist"]:.2f} m')
        print('=' * 60)
        if self.args.out:
            with open(self.args.out, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader(); w.writerows(rows)
            print(f'CSV: {self.args.out}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('throttle', type=float, nargs='+', help='mando(s) de acelerador de cada escalon [0, 1]')
    ap.add_argument('--duration', type=float, default=5.0, help='s con cada mando aplicado')
    ap.add_argument('--max-distance', type=float, default=5.0, help='m maximos recorridos con el mando')
    ap.add_argument('--abort-range', type=float, default=2.5, help='m: corta si el LiDAR frontal ve algo mas cerca')
    ap.add_argument('--brake', type=float, default=0.0, help='mando durante la frenada (0 = inercia, <0 = freno motor)')
    ap.add_argument('--coast', type=float, default=6.0, help='s maximos grabando la frenada')
    ap.add_argument('--wait', type=float, default=5.0, help='s de espera maxima a que llegue /tf')
    ap.add_argument('--out', default='', help='CSV de salida')
    args = ap.parse_args()

    rclpy.init()
    node = StepTest(args)
    try:
        while rclpy.ok() and node.phase != 'done':
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            node.throttle_pub.publish(Float32(data=0.0))
            node.steering_pub.publish(Float32(data=0.0))
            time.sleep(0.05)
        node.report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

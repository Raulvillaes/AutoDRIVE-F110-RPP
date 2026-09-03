#!/usr/bin/env python3
"""Mide el retardo entre publicar un mando de direccion y ver su efecto.

Aplica escalones de direccion alternos (+a, -a, +a, ...) y, en cada uno,
mide cuanto tarda la realimentacion `/autodrive/f1tenth_1/steering` del
puente en recorrer el 10 %, 50 % y 90 % del escalon. Si se pide un mando
de acelerador, mide ademas cuanto tarda la velocidad angular del IMU en
reaccionar: ese es el retardo completo mando -> puente -> simulador ->
sensor -> puente que sufre el controlador, y es el valor que va en
`command_delay` de config/params.yaml.

Uso (ROS 2 sourceado bajo bash, simulador conectado en Autonomous, coche
en la salida mirando a la recta):

  python3 tools/measure_delay.py                  # parado, solo el servo
  python3 tools/measure_delay.py --throttle 0.15  # en marcha, con la guinada

En marcha los escalones son pequenos (mando 0.15 = 4 m de radio) y cortos
(1 s), alternos para que el coche no se salga de la recta, y se corta si
el LiDAR ve pared a menos de `--abort-range`. Deja los mandos a cero al
salir. El escalon se manda con el reloj del sistema y los sensores se
fechan con su instante de llegada, asi todo va en la misma escala.

Otra forma de medirlo, con el coche dando vueltas: `tools/estimate_delay.py`
sobre el registro `log_csv` del controlador.
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32

sys.path.insert(0, __file__.rsplit('/tools/', 1)[0])

THROTTLE_TOPIC = '/autodrive/f1tenth_1/throttle_command'
STEERING_TOPIC = '/autodrive/f1tenth_1/steering_command'
STEERING_FEEDBACK = '/autodrive/f1tenth_1/steering'
IMU_TOPIC = '/autodrive/f1tenth_1/imu'


class DelayTest(Node):
    def __init__(self, args):
        super().__init__('measure_delay')
        self.args = args
        self.throttle_pub = self.create_publisher(Float32, THROTTLE_TOPIC, 10)
        self.steering_pub = self.create_publisher(Float32, STEERING_TOPIC, 10)
        self.create_subscription(Float32, STEERING_FEEDBACK, self.on_feedback, 10)
        self.create_subscription(Imu, IMU_TOPIC, self.on_imu, 10)
        self.create_subscription(LaserScan, '/autodrive/f1tenth_1/lidar', self.on_scan, 10)
        self.d_front = float('inf')
        self.create_timer(0.05, self.tick)      # 20 Hz: el puente repite el ultimo mando

        self.steer = 0.0
        self.step = -1
        self.t_step = None                     # reloj del sistema del ultimo escalon
        self.feedback = []                     # (t_llegada, valor)
        self.yaw_rate = []                     # (t_llegada, omega_z)
        self.steps = []                        # (t_escalon, mando)
        self.first_msg = None
        self.start = time.monotonic()
        self.done = False

    def on_feedback(self, msg):
        self.feedback.append((time.monotonic(), msg.data))
        if self.first_msg is None:
            self.first_msg = time.monotonic()

    def on_imu(self, msg):
        self.yaw_rate.append((time.monotonic(), msg.angular_velocity.z))

    def on_scan(self, msg):
        r = np.asarray(msg.ranges, dtype=float)
        ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        front = r[np.abs(ang) <= math.radians(15.0)]
        ok = np.isfinite(front) & (front >= msg.range_min)
        self.d_front = float(front[ok].min()) if ok.any() else float('inf')
        if self.args.throttle > 0.0 and self.d_front < self.args.abort_range and not self.done:
            self.get_logger().warning(f'Pared a {self.d_front:.2f} m: corto.')
            self.done = True

    def tick(self):
        now = time.monotonic()
        if self.first_msg is None:
            if now - self.start > self.args.wait:
                self.get_logger().error('No llega la realimentacion: puente caido o simulador sin conectar.')
                self.done = True
            return
        # Un segundo de asentamiento antes del primer escalon.
        if self.t_step is None and now - self.first_msg < 1.0:
            self.throttle_pub.publish(Float32(data=float(self.args.throttle)))
            self.steering_pub.publish(Float32(data=0.0))
            return
        hold = self.args.hold if self.args.hold else (1.0 if self.args.throttle > 0.0 else 3.0)
        if self.t_step is None or now - self.t_step > hold:
            self.step += 1
            if self.step >= self.args.steps:
                self.done = True
                return
            amplitude = self.args.amplitude if self.args.amplitude else (
                0.15 if self.args.throttle > 0.0 else 0.5)
            self.steer = amplitude * (1 if self.step % 2 == 0 else -1)
            self.t_step = now
            self.steps.append((now, self.steer))
            self.get_logger().info(f'Escalon {self.step + 1}/{self.args.steps}: direccion {self.steer:+.2f}')
        self.throttle_pub.publish(Float32(data=float(self.args.throttle)))
        self.steering_pub.publish(Float32(data=float(self.steer)))

    def report(self):
        fb = np.array(self.feedback)
        yr = np.array(self.yaw_rate) if self.yaw_rate else np.zeros((0, 2))
        if len(fb) < 5 or len(self.steps) < 2:
            print('Muy pocas muestras.')
            return
        rate = (len(fb) - 1) / (fb[-1, 0] - fb[0, 0])
        print('\n' + '=' * 64)
        amplitude = abs(self.steps[0][1])
        print(f'Cadencia del puente: {rate:4.1f} Hz.  Realimentacion de direccion: '
              f'min {fb[:, 1].min():+.3f}, max {fb[:, 1].max():+.3f} (mando +-{amplitude})')
        rows = []
        for k, (t0, cmd) in enumerate(self.steps):
            t1 = self.steps[k + 1][0] if k + 1 < len(self.steps) else fb[-1, 0]
            before = fb[fb[:, 0] <= t0][-1, 1] if (fb[:, 0] <= t0).any() else 0.0
            win = fb[(fb[:, 0] > t0) & (fb[:, 0] <= t1)]
            if len(win) < 2:
                continue
            final = win[-3:, 1].mean()
            span = final - before
            if abs(span) < 1e-3:
                print(f'Escalon {k + 1} ({cmd:+.2f}): la realimentacion no se movio.')
                continue
            times = {}
            for frac in (0.1, 0.5, 0.9):
                hit = win[np.sign(span) * (win[:, 1] - before) >= frac * abs(span)]
                times[frac] = hit[0, 0] - t0 if len(hit) else float('nan')
            t_yaw = float('nan')
            if len(yr) and self.args.throttle > 0.0:
                base = yr[(yr[:, 0] > t0 - 0.5) & (yr[:, 0] <= t0)]
                base_v = base[:, 1].mean() if len(base) else 0.0
                wy = yr[(yr[:, 0] > t0) & (yr[:, 0] <= t1)]
                sgn = np.sign(cmd - (self.steps[k - 1][1] if k else 0.0))
                # reaccion = mitad de la guinada esperada, v * tan(delta) / L
                expected = 5.0 * self.args.throttle * math.tan(amplitude * 0.524) / 0.33
                hit = wy[sgn * (wy[:, 1] - base_v) > 0.5 * expected]
                t_yaw = hit[0, 0] - t0 if len(hit) else float('nan')
            rows.append((cmd, times[0.1], times[0.5], times[0.9], t_yaw, final))
            print(f'Escalon {k + 1} ({cmd:+.2f}): servo al 10 % {times[0.1]:.2f} s, '
                  f'50 % {times[0.5]:.2f} s, 90 % {times[0.9]:.2f} s; '
                  f'guinada reacciona a {t_yaw:.2f} s; realimentacion final {final:+.3f}')
        if rows:
            a = np.array(rows)
            print('-' * 64)
            print(f'Mediana: servo 10 % {np.nanmedian(a[:, 1]):.2f} s, 50 % {np.nanmedian(a[:, 2]):.2f} s, '
                  f'90 % {np.nanmedian(a[:, 3]):.2f} s; guinada {np.nanmedian(a[:, 4]):.2f} s')
            t10, t90 = np.nanmedian(a[:, 1]), np.nanmedian(a[:, 3])
            print('Modelo retardo puro + servo de primer orden (t10 = T, t90 = T + 2.3 tau):')
            print(f'  command_delay ~ {t10:.2f} s   steering_lag ~ {(t90 - t10) / 2.2:.2f} s')
            print('  (mejor redondear hacia abajo; en marcha, contrastar con la guinada)')
        print('=' * 64)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--amplitude', type=float, default=0.0, help='mando de direccion de cada escalon (0 = 0.5 parado, 0.15 en marcha)')
    ap.add_argument('--steps', type=int, default=4, help='numero de escalones alternos')
    ap.add_argument('--hold', type=float, default=0.0, help='s con cada escalon aplicado (0 = 3 parado, 1 en marcha)')
    ap.add_argument('--throttle', type=float, default=0.0, help='acelerador durante la prueba (0 = parado)')
    ap.add_argument('--abort-range', type=float, default=0.8, help='m: en marcha, corta si el LiDAR frontal ve pared mas cerca')
    ap.add_argument('--wait', type=float, default=5.0, help='s de espera maxima a la realimentacion')
    args = ap.parse_args()

    rclpy.init()
    node = DelayTest(args)
    try:
        while rclpy.ok() and not node.done:
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

"""Regulated Pure Pursuit (RPP) para el F1TENTH del simulador AutoDRIVE.

Cada pose nueva del puente dispara un ciclo de control:

1. Punto mas cercano de la trayectoria y punto de lookahead a L_d metros,
   con L_d proporcional a la velocidad medida y acotado.
2. Curvatura del arco que lleva al lookahead, gamma = 2*y_L / L_d^2 (y_L es
   la coordenada lateral del lookahead en el marco del vehiculo), y angulo
   de direccion por bicicleta cinematica, delta = atan(L * gamma).
3. Velocidad objetivo: la maxima, regulada por (a) la curvatura del arco,
   con un suelo para que las curvas no dejen el coche parado, (b) la
   proximidad de obstaculos vista por el LiDAR, que si puede frenar del
   todo, y (c) el limite de aceleracion.
4. Conversion de la velocidad objetivo a mando de acelerador normalizado
   (prealimentacion medida mas correccion proporcional) y de delta al mando
   de direccion normalizado del simulador.

Referencia: S. Macenski, S. Singh, F. Martin, J. Gines, "Regulated Pure
Pursuit for Robot Path Tracking", Autonomous Robots, 2023 (es el
controlador RPP de Nav2).

Publica ademas, para depurar y para el video:
  /rpp/lookahead        visualization_msgs/Marker   punto objetivo en RViz
  /rpp/target_speed     std_msgs/Float32            velocidad objetivo (m/s)
  /rpp/measured_speed   std_msgs/Float32            velocidad medida (m/s)
"""
import csv
import math
import time

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker

from rpp_f110 import runner
from rpp_f110.trajectory import Trajectory, resolve_trajectory_path
from rpp_f110.vehicle_pose import stamp_to_sec, subscribe_vehicle_pose

THROTTLE_TOPIC = '/autodrive/f1tenth_1/throttle_command'
STEERING_TOPIC = '/autodrive/f1tenth_1/steering_command'
LIDAR_TOPIC = '/autodrive/f1tenth_1/lidar'
ENCODER_TOPICS = {'left': '/autodrive/f1tenth_1/left_encoder',
                  'right': '/autodrive/f1tenth_1/right_encoder'}

LOG_FIELDS = ['t', 'x', 'y', 'yaw', 'idx', 'lookahead', 'gamma', 'kappa_ahead', 'steer_cmd',
              'v_tf', 'v_enc', 'v_target', 'v_curv', 'v_prox', 'd_front',
              'throttle_cmd']


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class RppNode(Node):
    """Controlador Regulated Pure Pursuit sobre la trayectoria global."""

    def __init__(self):
        super().__init__('rpp_node')

        # ---------- Parametros (ver config/params.yaml) ----------
        self.declare_parameters('', [
            ('trajectory_csv', ''),
            ('wheelbase', 0.33),
            ('max_steering_angle', 0.524),
            ('lookahead_time', 0.6),
            ('lookahead_min', 0.6),
            ('lookahead_max', 1.3),
            ('max_speed', 1.0),
            ('min_speed', 0.3),
            ('regulated_min_radius', 1.3),
            ('curvature_lookahead', 1.5),
            ('proximity_distance', 0.8),
            ('proximity_gain', 1.0),
            ('proximity_fov', 30.0),
            ('max_accel', 1.0),
            ('max_decel', 2.0),
            ('throttle_offset', 0.0),
            ('throttle_per_mps', 0.2),
            ('speed_kp', 0.15),
            ('throttle_min', 0.0),
            ('throttle_max', 1.0),
            ('speed_source', 'tf'),
            ('wheel_radius', 0.058),
            ('speed_filter', 0.5),
            ('pose_source', 'sensors'),
            ('pose_timeout', 1.0),
            ('log_csv', ''),
        ])
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.wheelbase = p('wheelbase')
        self.max_steering_angle = p('max_steering_angle')
        self.lookahead_time = p('lookahead_time')
        self.lookahead_min = p('lookahead_min')
        self.lookahead_max = p('lookahead_max')
        self.max_speed = p('max_speed')
        self.min_speed = p('min_speed')
        self.regulated_min_radius = p('regulated_min_radius')
        self.curvature_lookahead = p('curvature_lookahead')
        self.proximity_distance = p('proximity_distance')
        self.proximity_gain = p('proximity_gain')
        self.proximity_half_fov = math.radians(p('proximity_fov')) / 2.0
        self.max_accel = p('max_accel')
        self.max_decel = p('max_decel')
        self.throttle_offset = p('throttle_offset')
        self.throttle_per_mps = p('throttle_per_mps')
        self.speed_kp = p('speed_kp')
        self.throttle_min = p('throttle_min')
        self.throttle_max = p('throttle_max')
        self.speed_source = p('speed_source')
        self.wheel_radius = p('wheel_radius')
        self.speed_filter = p('speed_filter')
        self.pose_timeout = p('pose_timeout')

        csv_path = resolve_trajectory_path(p('trajectory_csv'))
        self.traj = Trajectory.from_csv(csv_path)

        # ---------- Estado ----------
        self.idx = None            # indice del punto mas cercano (ciclo previo)
        self.prev_pose = None      # VehiclePose del ciclo previo
        self.v_tf = 0.0            # velocidad por desplazamiento de la pose
        self.v_enc = 0.0           # velocidad por encoders
        self.enc_prev = {'left': None, 'right': None}   # (t, angulo)
        self.enc_speed = {'left': 0.0, 'right': 0.0}
        self.v_target = 0.0        # velocidad objetivo del ciclo previo
        self.d_front = math.inf    # distancia libre en el sector frontal
        self.scan_mask = None      # rayos dentro del sector frontal
        self.last_pose_wall = None
        self.watchdog_stopped = False

        # ---------- Comunicacion ----------
        self.throttle_pub = self.create_publisher(Float32, THROTTLE_TOPIC, 10)
        self.steering_pub = self.create_publisher(Float32, STEERING_TOPIC, 10)
        self.marker_pub = self.create_publisher(Marker, '/rpp/lookahead', 1)
        self.target_pub = self.create_publisher(Float32, '/rpp/target_speed', 10)
        self.measured_pub = self.create_publisher(Float32, '/rpp/measured_speed', 10)

        self.pose_subs = subscribe_vehicle_pose(self, self.on_pose, p('pose_source'))
        self.scan_sub = self.create_subscription(
            LaserScan, LIDAR_TOPIC, self.on_scan, 10)
        self.enc_subs = [
            self.create_subscription(
                JointState, topic,
                lambda msg, side=side: self.on_encoder(side, msg), 10)
            for side, topic in ENCODER_TOPICS.items()]
        self.watchdog = self.create_timer(0.2, self.check_pose_timeout)

        # ---------- Registro opcional ----------
        self.log_writer = None
        if p('log_csv'):
            self.log_file = open(p('log_csv'), 'w', newline='')
            self.log_writer = csv.DictWriter(self.log_file, fieldnames=LOG_FIELDS)
            self.log_writer.writeheader()

        self.get_logger().info(
            f'RPP listo: {self.traj.n} puntos, vuelta de {self.traj.length:.2f} m '
            f'({csv_path}). max_speed={self.max_speed} m/s, '
            f'L_d en [{self.lookahead_min}, {self.lookahead_max}] m, '
            f'pose por "{p("pose_source")}", velocidad por "{self.speed_source}".')

    # ------------------------------------------------------------------
    # Sensores
    # ------------------------------------------------------------------
    def on_scan(self, msg: LaserScan):
        """Distancia libre minima dentro del sector frontal del LiDAR."""
        ranges = np.asarray(msg.ranges, dtype=float)
        if self.scan_mask is None or len(self.scan_mask) != len(ranges):
            angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
            self.scan_mask = np.abs(angles) <= self.proximity_half_fov
        front = ranges[self.scan_mask]
        valid = np.isfinite(front) & (front >= msg.range_min) & (front <= msg.range_max)
        self.d_front = float(front[valid].min()) if valid.any() else math.inf

    def on_encoder(self, side, msg: JointState):
        """Velocidad de una rueda derivando el angulo del encoder."""
        if not msg.position:
            return
        t = stamp_to_sec(msg.header.stamp)
        angle = msg.position[0]
        prev = self.enc_prev[side]
        self.enc_prev[side] = (t, angle)
        if prev is None:
            return
        dt = t - prev[0]
        dangle = angle - prev[1]
        # Un salto mayor que pi es un desbordamiento del encoder, no velocidad.
        if dt <= 0.0 or abs(dangle) > math.pi:
            return
        self.enc_speed[side] = dangle / dt * self.wheel_radius
        self.v_enc = 0.5 * (self.enc_speed['left'] + self.enc_speed['right'])

    def update_tf_speed(self, pose):
        """Velocidad longitudinal por desplazamiento entre poses consecutivas,
        con signo (positivo hacia delante) y filtro exponencial."""
        prev = self.prev_pose
        if prev is None:
            return
        dt = pose.t - prev.t
        if dt <= 0.0 or dt > 1.0:      # pose repetida o hueco largo
            return
        dx, dy = pose.x - prev.x, pose.y - prev.y
        v = (dx * math.cos(pose.yaw) + dy * math.sin(pose.yaw)) / dt
        a = self.speed_filter
        self.v_tf = a * v + (1.0 - a) * self.v_tf

    def measured_speed(self):
        return self.v_enc if self.speed_source == 'encoders' else self.v_tf

    # ------------------------------------------------------------------
    # Ciclo de control
    # ------------------------------------------------------------------
    def on_pose(self, pose):
        self.last_pose_wall = time.monotonic()
        self.watchdog_stopped = False
        self.update_tf_speed(pose)
        v_meas = self.measured_speed()

        # 1. Lookahead adaptativo y punto objetivo sobre la trayectoria.
        lookahead = clamp(self.lookahead_time * abs(v_meas),
                          self.lookahead_min, self.lookahead_max)
        self.idx = self.traj.nearest_index(pose.x, pose.y, hint=self.idx)
        lx, ly, _ = self.traj.lookahead_point(pose.x, pose.y, self.idx, lookahead)

        # 2. Curvatura del arco y angulo de direccion.
        dx, dy = lx - pose.x, ly - pose.y
        y_l = -math.sin(pose.yaw) * dx + math.cos(pose.yaw) * dy
        d2 = dx * dx + dy * dy
        gamma = 2.0 * y_l / d2 if d2 > 1e-6 else 0.0
        delta = math.atan(self.wheelbase * gamma)
        steer_cmd = clamp(delta / self.max_steering_angle, -1.0, 1.0)

        # 3. Velocidad objetivo con las tres regulaciones. La curvatura que
        #    regula es la mayor entre la del arco actual y la del camino que
        #    viene (curvature_lookahead metros): asi frena antes de la curva.
        kappa_ahead = self.traj.max_curvature_ahead(self.idx, self.curvature_lookahead)
        v_curv = max(self.regulate_curvature(self.max_speed, max(abs(gamma), kappa_ahead)),
                     self.min_speed)
        v_prox = self.regulate_proximity(v_curv)
        dt = pose.t - self.prev_pose.t if self.prev_pose else None
        v_target = self.limit_acceleration(v_prox, dt)

        # 4. Mandos normalizados al simulador.
        throttle_cmd = self.speed_to_throttle(v_target, v_meas)
        self.publish_commands(throttle_cmd, steer_cmd)
        self.publish_debug(lx, ly, v_target, v_meas)

        self.get_logger().info(
            f'v={v_meas:4.2f} obj={v_target:4.2f} m/s  L_d={lookahead:.2f} m  '
            f'giro={math.degrees(delta):+5.1f} deg  k_adel={kappa_ahead:4.2f}  libre={self.d_front:4.2f} m  '
            f'idx={self.idx}', throttle_duration_sec=1.0)

        if self.log_writer is not None:
            self.log_writer.writerow({
                't': pose.t, 'x': pose.x, 'y': pose.y, 'yaw': pose.yaw,
                'idx': self.idx, 'lookahead': lookahead, 'gamma': gamma,
                'kappa_ahead': kappa_ahead,
                'steer_cmd': steer_cmd, 'v_tf': self.v_tf, 'v_enc': self.v_enc,
                'v_target': v_target, 'v_curv': v_curv, 'v_prox': v_prox,
                'd_front': self.d_front, 'throttle_cmd': throttle_cmd})
        self.prev_pose = pose
        self.v_target = v_target

    # ------------------------------------------------------------------
    # Las tres regulaciones del RPP
    # ------------------------------------------------------------------
    def regulate_curvature(self, speed, gamma):
        """Regulacion por curvatura: si el radio (1/curvatura) baja del
        radio minimo regulado, la velocidad escala linealmente con el radio.
        Quien llama le pone el suelo min_speed: una curva nunca para el coche."""
        if abs(gamma) < 1e-9:
            return speed
        radius = 1.0 / abs(gamma)
        if radius < self.regulated_min_radius:
            return speed * radius / self.regulated_min_radius
        return speed

    def regulate_proximity(self, speed):
        """Regulacion por proximidad: con un obstaculo (o pared) mas cerca
        que proximity_distance en el sector frontal, la velocidad escala con
        la distancia libre, sin suelo: contra una pared se para."""
        if self.d_front < self.proximity_distance:
            factor = clamp(self.proximity_gain * self.d_front / self.proximity_distance,
                           0.0, 1.0)
            return speed * factor
        return speed

    def limit_acceleration(self, speed, dt):
        """Limite de aceleracion y de frenada respecto al ciclo anterior."""
        if dt is None or dt <= 0.0:
            return speed
        return clamp(speed, self.v_target - self.max_decel * dt,
                     self.v_target + self.max_accel * dt)

    # ------------------------------------------------------------------
    # Actuacion
    # ------------------------------------------------------------------
    def speed_to_throttle(self, v_target, v_meas):
        """Velocidad objetivo -> mando de acelerador normalizado: una
        prealimentacion (mapa medido mando->velocidad, invertido) mas una
        correccion proporcional con la velocidad medida."""
        throttle = (self.throttle_offset + self.throttle_per_mps * v_target
                    + self.speed_kp * (v_target - v_meas))
        return clamp(throttle, self.throttle_min, self.throttle_max)

    def publish_commands(self, throttle_cmd, steer_cmd):
        self.throttle_pub.publish(Float32(data=float(throttle_cmd)))
        self.steering_pub.publish(Float32(data=float(steer_cmd)))

    def publish_debug(self, lx, ly, v_target, v_meas):
        self.target_pub.publish(Float32(data=float(v_target)))
        self.measured_pub.publish(Float32(data=float(v_meas)))
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'rpp', 0, Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = lx, ly, 0.05
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.2, 1.0
        self.marker_pub.publish(m)

    def check_pose_timeout(self):
        """Sin poses nuevas durante pose_timeout, acelerador a cero: el
        puente repite el ultimo mando y el coche seguiria solo."""
        if self.last_pose_wall is None or self.watchdog_stopped:
            return
        if time.monotonic() - self.last_pose_wall > self.pose_timeout:
            self.publish_commands(0.0, 0.0)
            self.watchdog_stopped = True
            self.get_logger().warning(
                f'Sin pose del puente desde hace {self.pose_timeout} s: acelerador a cero.')

    def stop_vehicle(self):
        """Deja el coche parado antes de salir: el puente conserva el ultimo
        mando recibido, asi que hay que mandar ceros explicitamente."""
        for _ in range(5):
            self.publish_commands(0.0, 0.0)
            time.sleep(0.05)
        if self.log_writer is not None:
            self.log_file.close()


def main(args=None):
    runner.init(args)
    node = RppNode()
    try:
        runner.spin_until_stopped(node)
    finally:
        node.stop_vehicle()
        runner.shutdown(node)


if __name__ == '__main__':
    main()

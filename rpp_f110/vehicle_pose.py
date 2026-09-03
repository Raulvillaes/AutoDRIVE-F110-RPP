"""Pose del vehiculo en el marco `map`, tal como la publica el puente.

El puente de AutoDRIVE emite la pose verdadera del coche (IPS + IMU del
simulador) en el mismo marco `map` en el que esta la trayectoria, asi que
no hace falta localizacion ni cambio de marco (ver README). La publica de
dos formas y aqui se puede leer cualquiera de ellas:

- `sensors` (por defecto): la posicion de `/autodrive/f1tenth_1/ips`
  (geometry_msgs/Point) y la orientacion de `/autodrive/f1tenth_1/imu`
  (sensor_msgs/Imu). Son los mismos numeros que van al TF, publicados por
  dos publishers fijos, y el sello de tiempo del IMU fecha la pose.
- `tf`: la transformacion `map -> f1tenth_1` de `/tf`. El puente crea un
  TransformBroadcaster NUEVO por cada mensaje, y cada publisher nuevo tiene
  que ser descubierto por DDS antes de que el mensaje llegue: la entrega se
  retrasa y se pierden mensajes, cada vez mas conforme el puente lleva
  tiempo en marcha. Se conserva como alternativa.

En los dos casos cada mensaje dispara el ciclo de control en el instante
en que llega una pose nueva: no se consultan datos viejos ni se pierden
ciclos. El puente publica todo el sensado en un solo lote, asi que la
cadencia de la pose es la cadencia del lazo.
"""
import math
from dataclasses import dataclass

from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage

MAP_FRAME = 'map'
VEHICLE_FRAME = 'f1tenth_1'
IPS_TOPIC = '/autodrive/f1tenth_1/ips'
IMU_TOPIC = '/autodrive/f1tenth_1/imu'


@dataclass
class VehiclePose:
    t: float      # s, sello de tiempo que pone el puente
    x: float      # m, marco map
    y: float      # m, marco map
    yaw: float    # rad, rumbo antihorario desde +x
    yaw_rate: float = 0.0   # rad/s del IMU (0 con pose por tf)


def yaw_from_quaternion(qx, qy, qz, qw):
    """Angulo de guinada (rotacion sobre z) de un cuaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def subscribe_vehicle_pose(node, callback, source='sensors',
                           map_frame=MAP_FRAME, vehicle_frame=VEHICLE_FRAME):
    """Llama a `callback(VehiclePose)` con cada pose nueva del puente.

    `source` es 'sensors' (IPS + IMU) o 'tf'. Devuelve las suscripciones
    para que el nodo las conserve.
    """
    if source == 'tf':
        def on_tf(msg: TFMessage):
            for tr in msg.transforms:
                if (tr.header.frame_id == map_frame
                        and tr.child_frame_id == vehicle_frame):
                    p = tr.transform.translation
                    q = tr.transform.rotation
                    callback(VehiclePose(stamp_to_sec(tr.header.stamp), p.x, p.y,
                                         yaw_from_quaternion(q.x, q.y, q.z, q.w)))
        # Mismo QoS que el TransformBroadcaster del puente (fiable, cola de 100).
        return [node.create_subscription(TFMessage, '/tf', on_tf,
                                         QoSProfile(depth=100))]

    if source != 'sensors':
        raise ValueError(f"pose_source debe ser 'sensors' o 'tf', no '{source}'")

    latest = {'ips': None}

    def on_ips(msg: Point):
        latest['ips'] = (msg.x, msg.y)

    def on_imu(msg: Imu):
        # El puente publica el IPS justo antes que el IMU en el mismo lote,
        # asi que la ultima posicion recibida es la que acompana a esta
        # orientacion.
        if latest['ips'] is None:
            return
        q = msg.orientation
        x, y = latest['ips']
        callback(VehiclePose(stamp_to_sec(msg.header.stamp), x, y,
                             yaw_from_quaternion(q.x, q.y, q.z, q.w),
                             msg.angular_velocity.z))

    return [node.create_subscription(Point, IPS_TOPIC, on_ips, 10),
            node.create_subscription(Imu, IMU_TOPIC, on_imu, 10)]

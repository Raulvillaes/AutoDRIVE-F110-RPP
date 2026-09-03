#!/usr/bin/env python3
"""Simula el TF `map -> f1tenth_1` recorriendo la trayectoria del CSV.

Sirve para probar lap_node y rpp_node sin el simulador: publica /tf a la
cadencia del puente con el coche avanzando por los waypoints a velocidad
constante y dando las vueltas que se pidan.

  python3 tools/replay_pose.py --speed 1.5 --laps 2 --rate 4
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

sys.path.insert(0, __file__.rsplit('/tools/', 1)[0])
from rpp_f110.trajectory import Trajectory  # noqa: E402


class Replay(Node):
    def __init__(self, args):
        super().__init__('replay_pose')
        self.traj = Trajectory.from_csv(args.csv)
        self.args = args
        self.pub = self.create_publisher(TFMessage, '/tf', QoSProfile(depth=100))
        self.s = args.start_s
        self.t = 0.0
        self.total = args.laps * self.traj.length
        self.create_timer(1.0 / args.rate, self.tick)

    def pose_at(self, s):
        """Posicion y rumbo interpolados a la longitud de arco s (ciclica)."""
        s = s % self.traj.length
        i = int(min(self.traj.n - 1, max(0, (self.traj.s <= s).sum() - 1)))
        j = (i + 1) % self.traj.n
        seg = (self.traj.length - self.traj.s[i]) if j == 0 else (self.traj.s[j] - self.traj.s[i])
        u = 0.0 if seg <= 0 else (s - self.traj.s[i]) / seg
        x = self.traj.x[i] + u * (self.traj.x[j] - self.traj.x[i])
        y = self.traj.y[i] + u * (self.traj.y[j] - self.traj.y[i])
        return x, y, self.traj.heading(i)

    def tick(self):
        if self.s - self.args.start_s > self.total + 1.0:
            self.get_logger().info('Recorrido terminado.')
            raise SystemExit
        x, y, yaw = self.pose_at(self.s)
        tr = TransformStamped()
        tr.header.stamp = self.get_clock().now().to_msg()
        tr.header.frame_id, tr.child_frame_id = 'map', 'f1tenth_1'
        tr.transform.translation.x, tr.transform.translation.y = float(x), float(y)
        tr.transform.rotation.z, tr.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pub.publish(TFMessage(transforms=[tr]))
        self.s += self.args.speed / self.args.rate


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--csv', default=__file__.rsplit('/tools/', 1)[0] + '/config/trajectory.csv')
    ap.add_argument('--speed', type=float, default=1.5)
    ap.add_argument('--rate', type=float, default=4.0)
    ap.add_argument('--laps', type=float, default=2.0)
    ap.add_argument('--start-s', type=float, default=27.5, help='arco inicial (27.5 = casi en la meta)')
    args = ap.parse_args()
    rclpy.init()
    node = Replay(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

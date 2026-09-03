"""Publica la trayectoria como nav_msgs/Path y la linea de meta como Marker
para verlas en RViz. Solo visualizacion: el controlador no depende de esto.
"""
import math

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

from rpp_f110 import runner
from rpp_f110.trajectory import Trajectory, resolve_trajectory_path


class PathNode(Node):
    def __init__(self):
        super().__init__('path_node')
        self.declare_parameters('', [
            ('trajectory_csv', ''),
            ('finish_line', [1.47, 3.16, 0.29, 3.16]),
            ('frame_id', 'map'),
            ('period', 1.0),
        ])
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.frame_id = p('frame_id')
        self.traj = Trajectory.from_csv(resolve_trajectory_path(p('trajectory_csv')))
        self.finish_line = list(p('finish_line'))

        # transient_local: RViz recibe el ultimo mensaje aunque se abra despues.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/rpp/path', qos)
        self.line_pub = self.create_publisher(Marker, '/rpp/finish_line', qos)
        self.create_timer(p('period'), self.publish)
        self.get_logger().info(
            f'Publicando {self.traj.n} puntos en /rpp/path y la meta en /rpp/finish_line.')

    def build_path(self):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for i in range(self.traj.n):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(self.traj.x[i])
            ps.pose.position.y = float(self.traj.y[i])
            yaw = self.traj.heading(i)
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(ps)
        path.poses.append(path.poses[0])   # cierra el lazo en el dibujo
        return path

    def build_finish_marker(self):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'rpp', 1, Marker.LINE_STRIP, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.05
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 1.0, 1.0
        x1, y1, x2, y2 = self.finish_line
        m.points = [Point(x=x1, y=y1, z=0.02), Point(x=x2, y=y2, z=0.02)]
        return m

    def publish(self):
        self.path_pub.publish(self.build_path())
        self.line_pub.publish(self.build_finish_marker())


def main(args=None):
    runner.init(args)
    node = PathNode()
    try:
        runner.spin_until_stopped(node)
    finally:
        runner.shutdown(node)


if __name__ == '__main__':
    main()

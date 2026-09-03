"""Bucle de ejecucion comun a los tres nodos, con parada limpia.

Con Ctrl+C en un `ros2 launch` cada nodo recibe DOS SIGINT (uno del
terminal, por el grupo de procesos, y otro reenviado por launch). Si el
segundo llega durante la limpieza, rclpy la interrumpe con un traceback y
el controlador no llega a publicar el mando de parada. Por eso se desactiva
el manejador de senales de rclpy y se instala uno propio que solo levanta
una bandera e ignora las senales siguientes.
"""
import signal

import rclpy
from rclpy.signals import SignalHandlerOptions


def init(args=None):
    """rclpy.init sin el manejador de senales de rclpy."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)


def spin_until_stopped(node, done=lambda: False):
    """Gira el nodo hasta SIGINT/SIGTERM o hasta que `done()` sea True.
    Devuelve con el contexto de ROS todavia valido, para poder publicar en
    la limpieza."""
    stop = {'flag': False}

    def handler(signum, frame):
        stop['flag'] = True
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)

    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)
    while rclpy.ok() and not stop['flag'] and not done():
        rclpy.spin_once(node, timeout_sec=0.1)


def shutdown(node):
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

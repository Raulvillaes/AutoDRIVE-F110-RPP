"""Contador de vueltas y cronometro por vuelta, impresos en la terminal.

Lee la pose del TF `map -> f1tenth_1` y detecta el cruce orientado de la
linea de meta (ver lap_counter.py). Es independiente del controlador: se
puede lanzar solo, por ejemplo conduciendo con el teleop, para comprobarlo.
"""
from rclpy.node import Node

from rpp_f110 import runner
from rpp_f110.lap_counter import LapCounter
from rpp_f110.vehicle_pose import subscribe_vehicle_pose

RULE = '=' * 50


class LapNode(Node):
    def __init__(self):
        super().__init__('lap_node')
        self.declare_parameters('', [
            ('finish_line', [1.47, 3.16, 0.29, 3.16]),
            ('finish_direction', [0.0, -1.0]),
            ('min_lap_time', 5.0),
            ('rearm_distance', 1.0),
            ('total_laps', 0),
            ('progress_period', 0.0),
            ('pose_source', 'sensors'),
        ])
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        line = list(p('finish_line'))
        self.counter = LapCounter(line[0:2], line[2:4], list(p('finish_direction')),
                                  p('min_lap_time'), p('rearm_distance'))
        self.total_laps = int(p('total_laps'))
        self.last_t = None
        self.finished = False

        self.pose_subs = subscribe_vehicle_pose(self, self.on_pose, p('pose_source'))
        if p('progress_period') > 0.0:
            self.create_timer(p('progress_period'), self.print_progress)

        self.get_logger().info(
            f'Contador de vueltas listo. Meta: ({line[0]}, {line[1]}) -> '
            f'({line[2]}, {line[3]}), sentido {list(p("finish_direction"))}. '
            f'Esperando a que el coche se mueva...')

    def on_pose(self, pose):
        self.last_t = pose.t
        for event in self.counter.update(pose.t, pose.x, pose.y):
            if not self.finished:
                self.report(event)

    def report(self, event):
        log = self.get_logger().info
        if event.kind == 'reset':
            log(f'Reset del simulador detectado: contador a cero. Esperando a que el coche se mueva...')
        elif event.kind == 'start':
            log(f'Cronometro iniciado: el coche arranca en ({event.x:.2f}, {event.y:.2f}).')
        elif event.kind == 'sync':
            log('Meta cruzada al salir: el reloj de la vuelta 1 se sincroniza con la linea.')
        elif event.kind == 'lap':
            log('\n' + RULE + '\n'
                f'  VUELTA {event.lap} COMPLETADA\n'
                f'  Tiempo de vuelta:    {event.lap_time:7.2f} s\n'
                f'  Mejor vuelta:        {event.best_time:7.2f} s\n'
                f'  Tiempo acumulado:    {event.total_time:7.2f} s\n'
                + RULE)
            if self.total_laps > 0 and event.lap >= self.total_laps:
                self.finish(event)

    def print_progress(self):
        if self.last_t is None or self.finished:
            return
        t = self.counter.current_lap_time(self.last_t)
        if t is not None:
            self.get_logger().info(
                f'Vuelta {self.counter.laps + 1} en curso: {t:6.1f} s')

    def finish(self, event):
        self.finished = True
        times = '  '.join(f'V{i + 1}: {t:.2f} s' for i, t in enumerate(event.lap_times))
        self.get_logger().info(
            '\n' + '#' * 50 + '\n'
            f'  {event.lap} VUELTAS COMPLETADAS\n'
            f'  {times}\n'
            f'  Mejor vuelta:        {event.best_time:7.2f} s\n'
            f'  Tiempo total:        {event.total_time:7.2f} s\n'
            + '#' * 50)
        # `finished` saca del bucle de main(); al salir el proceso, el launch
        # apaga el resto de nodos (ver rpp.launch.py). No se llama a
        # rclpy.shutdown() desde el callback: en Humble deja el proceso colgado.


def main(args=None):
    runner.init(args)
    node = LapNode()
    try:
        runner.spin_until_stopped(node, done=lambda: node.finished)
    finally:
        runner.shutdown(node)


if __name__ == '__main__':
    main()

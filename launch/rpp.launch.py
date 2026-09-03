"""Lanza el controlador RPP con el contador de vueltas y la visualizacion.

Requiere el puente de AutoDRIVE ya en marcha
(`ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py` o
`..._rviz.launch.py`) y el simulador en modo Autonomous.

Si `lap_node` termina (parametro total_laps alcanzado) se apaga todo: el
controlador recibe la senal y deja el acelerador a cero antes de salir.

Argumentos:
  params_file:=/ruta/otro.yaml   otro archivo de parametros
  log_csv:=/tmp/rpp.csv          registro por ciclo del controlador
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    log_csv = LaunchConfiguration('log_csv')

    path_node = Node(
        package='rpp_f110', executable='path_node', name='path_node',
        parameters=[params_file], output='screen', emulate_tty=True)
    lap_node = Node(
        package='rpp_f110', executable='lap_node', name='lap_node',
        parameters=[params_file], output='screen', emulate_tty=True)
    rpp_node = Node(
        package='rpp_f110', executable='rpp_node', name='rpp_node',
        parameters=[params_file, {'log_csv': log_csv}],
        output='screen', emulate_tty=True)

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('rpp_f110'), 'config', 'params.yaml']),
            description='YAML con los parametros de los tres nodos'),
        DeclareLaunchArgument(
            'log_csv', default_value='',
            description='CSV donde rpp_node registra cada ciclo (vacio = no)'),
        path_node,
        lap_node,
        rpp_node,
        RegisterEventHandler(OnProcessExit(
            target_action=lap_node,
            on_exit=[EmitEvent(event=Shutdown(reason='vueltas completadas'))])),
    ])

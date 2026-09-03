"""Pruebas de la compensacion del retardo (sin ROS)."""
import math

import pytest

from rpp_f110.prediction import predict_pose


def test_no_delay_or_no_commands_returns_pose():
    assert predict_pose(1.0, 2.0, 0.3, 1.0, 0.33, [], 10.0, 0.5) == (1.0, 2.0, 0.3)
    assert predict_pose(1.0, 2.0, 0.3, 1.0, 0.33, [(9.0, 0.1)], 10.0, 0.0) == (1.0, 2.0, 0.3)
    assert predict_pose(1.0, 2.0, 0.3, 0.0, 0.33, [(9.0, 0.1)], 10.0, 0.5) == (1.0, 2.0, 0.3)


def test_straight_ahead_advances_speed_times_delay():
    x, y, yaw = predict_pose(0.0, 0.0, math.pi / 2, 2.0, 0.33, [(9.0, 0.0)], 10.0, 0.5)
    assert (x, y) == (pytest.approx(0.0), pytest.approx(1.0))
    assert yaw == pytest.approx(math.pi / 2)


def test_constant_steering_turns_at_bicycle_yaw_rate():
    delta, v, L, tau = 0.2, 1.0, 0.33, 0.5
    _, _, yaw = predict_pose(0.0, 0.0, 0.0, v, L, [(9.0, delta)], 10.0, tau)
    assert yaw == pytest.approx(v * math.tan(delta) / L * tau)


def test_only_commands_inside_the_window_act():
    # el mando viejo (0.4) solo vale hasta que llega el nuevo (0.0) a mitad
    # de la ventana: gira la mitad que con 0.4 durante toda la ventana
    v, L, tau = 1.0, 0.33, 0.5
    cmds = [(9.0, 0.4), (9.75, 0.0)]
    _, _, yaw = predict_pose(0.0, 0.0, 0.0, v, L, cmds, 10.0, tau)
    assert yaw == pytest.approx(v * math.tan(0.4) / L * 0.25)


def test_servo_lag_reduces_the_turn_and_converges():
    delta, v, L, tau = 0.2, 1.0, 0.33, 0.5
    full = v * math.tan(delta) / L * tau
    # ruedas que persiguen al mando: giran menos que con ruedas instantaneas
    _, _, yaw = predict_pose(0.0, 0.0, 0.0, v, L, [(9.0, 0.0), (9.5, delta)], 10.0, tau,
                             steering_lag=0.3)
    assert 0.0 < yaw < full
    # con un mando constante desde hace mucho el servo ya esta en el mando
    _, _, yaw2 = predict_pose(0.0, 0.0, 0.0, v, L, [(0.0, delta)], 10.0, tau,
                              steering_lag=0.3)
    assert yaw2 == pytest.approx(full, rel=1e-3)


def test_servo_model_uses_the_command_active_at_window_start():
    # mando viejo 0.3 vigente al abrir la ventana; el nuevo (0.0) llega a
    # mitad. Con un servo casi instantaneo debe dar lo mismo que sin servo.
    cmds = [(8.0, 0.3), (9.75, 0.0)]
    _, _, yaw_lag = predict_pose(0.0, 0.0, 0.0, 1.0, 0.33, cmds, 10.0, 0.5, steering_lag=1e-4)
    _, _, yaw_ref = predict_pose(0.0, 0.0, 0.0, 1.0, 0.33, cmds, 10.0, 0.5)
    assert yaw_ref > 0.1
    assert yaw_lag == pytest.approx(yaw_ref, rel=1e-2)

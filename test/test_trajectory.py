"""Pruebas de la geometria ciclica de la trayectoria (sin ROS)."""
import math
import os

import numpy as np
import pytest

from rpp_f110.trajectory import Trajectory

CSV = os.path.join(os.path.dirname(__file__), '..', 'config', 'trajectory.csv')


def circle(radius=2.0, n=200):
    a = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    return Trajectory(radius * np.cos(a), radius * np.sin(a))


def test_length_includes_closing_segment():
    t = circle()
    assert t.length == pytest.approx(2 * math.pi * 2.0, rel=1e-3)


def test_nearest_index_global_and_windowed():
    t = circle()
    i = t.nearest_index(2.0, 0.1)      # 0.05 rad = 1.6 pasos de 0.0314 rad
    assert i in (1, 2)
    # ventana ciclica: pista desde el final de la lista
    j = t.nearest_index(2.0, -0.05, hint=t.n - 3)
    assert j in (t.n - 1, 0)
    # salto grande: vuelve a buscar en toda la vuelta
    k = t.nearest_index(-2.0, 0.0, hint=0)
    assert k == t.n // 2


def test_lookahead_point_lies_at_distance():
    t = circle()
    px, py = 2.0, 0.0
    for ld in (0.5, 1.0, 1.5):
        lx, ly, _ = t.lookahead_point(px, py, 0, ld)
        assert math.hypot(lx - px, ly - py) == pytest.approx(ld, abs=2e-3)
        assert ly > 0        # avanza en sentido antihorario, como la lista


def test_lookahead_wraps_around_the_end():
    t = circle()
    lx, ly, seg = t.lookahead_point(2.0, -0.05, t.n - 2, 0.8)
    assert math.hypot(lx - 2.0, ly + 0.05) == pytest.approx(0.8, abs=2e-3)
    assert ly > 0


def test_lookahead_far_from_path_targets_nearest():
    t = circle()
    lx, ly, _ = t.lookahead_point(5.0, 0.0, 0, 0.8)
    assert (lx, ly) == (pytest.approx(2.0), pytest.approx(0.0))


def test_bundled_csv_loads():
    t = Trajectory.from_csv(CSV)
    assert t.n == 279
    assert t.length == pytest.approx(27.85, abs=0.01)
    assert abs(t.kappa).max() < 1.0
    # desde la pose de salida el punto mas cercano es el penultimo tramo
    assert t.nearest_index(0.745, 3.158) >= t.n - 4


def test_speed_profile_constant_on_a_circle():
    # circulo de r = 1 m con radio regulado 2 m: velocidad de curva = max/2
    a = np.linspace(0.0, 2 * math.pi, 100, endpoint=False)
    t = Trajectory(np.cos(a), np.sin(a), kappa=np.ones(100))
    v = t.speed_profile(2.0, 2.0, 0.3, 1.0, 1.0)
    assert v == pytest.approx(np.full(100, 1.0))


def test_speed_profile_respects_braking_and_acceleration():
    # recta de 100 puntos con una curva cerrada en el medio
    kappa = np.zeros(100)
    kappa[50:55] = 1.0
    t = Trajectory(np.arange(100) * 0.1, np.zeros(100), kappa=kappa)
    vmax, amax, adec = 3.0, 1.0, 2.0
    v = t.speed_profile(vmax, 2.0, 0.5, amax, adec)
    ds = t.length / t.n
    assert v.max() == pytest.approx(vmax)
    assert v[50:55] == pytest.approx(1.5)          # 3.0 * r / 2.0 con r = 1
    for i in range(t.n):
        j = (i + 1) % t.n
        # frenada: v_i^2 - v_j^2 <= 2 a ds; aceleracion: v_j^2 - v_i^2 <= 2 a ds
        assert v[i] ** 2 - v[j] ** 2 <= 2 * adec * ds + 1e-9
        assert v[j] ** 2 - v[i] ** 2 <= 2 * amax * ds + 1e-9
    # frena antes de la curva, no en ella, y acelera al salir
    assert v[45] < vmax and v[45] > v[49]
    assert v[60] > v[55]


def test_speed_profile_of_the_bundled_csv():
    t = Trajectory.from_csv(CSV)
    v = t.speed_profile(2.7, 2.5, 0.5, 2.0, 2.2)
    # la curva de 1.01 m de radio limita a 2.7 * 1.01 / 2.5
    assert v.min() == pytest.approx(2.7 * 1.01 / 2.5, abs=0.02)
    assert v.max() == pytest.approx(2.7)
    # en plena recta inicial va a tope y al final de la recta ya frena
    assert v[20] == pytest.approx(2.7)
    assert v[65] < 2.7


def test_speed_profile_lateral_acceleration_cap():
    a = np.linspace(0.0, 2 * math.pi, 100, endpoint=False)
    r = 2.0
    t = Trajectory(r * np.cos(a), r * np.sin(a), kappa=np.full(100, 1.0 / r))
    # sin limite lateral la regla lineal da max_speed * r / r_min = 2.7 * 2 / 2.5
    assert t.speed_profile(2.7, 2.5, 0.3, 1.0, 1.0).max() == pytest.approx(2.16, abs=1e-6)
    # con a_lat 1.8: sqrt(1.8 * 2) = 1.897, que es menor y manda
    v = t.speed_profile(2.7, 2.5, 0.3, 1.0, 1.0, max_lateral_accel=1.8)
    assert v == pytest.approx(np.full(100, math.sqrt(3.6)))

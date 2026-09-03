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


def test_max_curvature_ahead_previews_the_bend():
    t = Trajectory.from_csv(CSV)
    # en plena recta inicial no hay curvatura a 1 m, pero si a 3 m del final
    assert t.max_curvature_ahead(10, 1.0) < 0.05
    assert t.max_curvature_ahead(55, 3.0) > 0.3
    # ciclico: desde el ultimo punto sigue por el primero
    assert t.max_curvature_ahead(t.n - 1, 2.0) < 0.15

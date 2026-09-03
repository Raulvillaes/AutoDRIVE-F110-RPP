"""Pruebas del cruce orientado de meta y del cronometro (sin ROS)."""
import pytest

from rpp_f110.lap_counter import LapCounter

LINE_A, LINE_B = (1.47, 3.16), (0.29, 3.16)
DOWN = (0.0, -1.0)


def make(**kw):
    # Las muestras de estas pruebas estan muy separadas: se sube el umbral
    # de teletransporte para que no se confundan con un reset.
    kw.setdefault('teleport_distance', 10.0)
    kw.setdefault('min_lap_time', 1.0)
    kw.setdefault('rearm_distance', 0.5)
    return LapCounter(LINE_A, LINE_B, DOWN, **kw)


def run(counter, samples):
    return [e for s in samples for e in counter.update(*s)]


def test_start_event_when_vehicle_moves():
    c = make()
    events = run(c, [(0.0, 0.745, 3.158), (0.25, 0.745, 3.158), (0.5, 0.745, 3.0)])
    assert [e.kind for e in events] == ['start']
    assert events[0].t == 0.25       # ultima pose quieta


def test_lap_counted_with_interpolated_time():
    c = make()
    # arranca, se aleja, vuelve por arriba y cruza hacia -y entre t=10 y t=10.25
    samples = [(0.0, 0.7, 3.158), (0.25, 0.7, 3.0), (0.5, 0.7, 2.0),
               (5.0, 0.7, 4.0), (10.0, 0.7, 3.21), (10.25, 0.7, 3.11)]
    events = run(c, samples)
    assert [e.kind for e in events] == ['start', 'lap']
    lap = events[1]
    assert lap.lap == 1
    # el cruce cae a mitad del tramo 3.21 -> 3.11
    assert lap.t == pytest.approx(10.125)
    assert lap.lap_time == pytest.approx(10.125 - 0.0)


def test_slow_pass_counts_once():
    c = make()
    samples = [(0.0, 0.7, 3.158), (0.5, 0.7, 2.0), (5.0, 0.7, 4.0),
               (9.0, 0.7, 3.17), (9.5, 0.7, 3.16), (10.0, 0.7, 3.15),
               (10.5, 0.7, 3.155), (11.0, 0.7, 3.14), (11.5, 0.7, 3.0)]
    events = run(c, samples)
    assert sum(e.kind == 'lap' for e in events) == 1


def test_wrong_direction_does_not_count():
    c = make()
    samples = [(0.0, 0.7, 3.158), (0.5, 0.7, 2.0), (5.0, 0.7, 2.5),
               (6.0, 0.7, 3.1), (6.5, 0.7, 3.3)]    # cruza hacia +y
    events = run(c, samples)
    assert all(e.kind != 'lap' for e in events)


def test_crossing_outside_segment_does_not_count():
    c = make()
    samples = [(0.0, 0.7, 3.158), (0.5, 0.7, 2.0), (5.0, 2.5, 4.0),
               (6.0, 2.5, 3.3), (6.5, 2.5, 3.0)]    # cruza la recta fuera del muro
    events = run(c, samples)
    assert all(e.kind != 'lap' for e in events)


def test_start_before_line_syncs_clock():
    c = make()
    samples = [(0.0, 0.7, 3.4), (0.25, 0.7, 3.3), (0.5, 0.7, 3.1),
               (1.0, 0.7, 2.0), (12.0, 0.7, 4.0), (13.0, 0.7, 3.2), (13.5, 0.7, 3.1)]
    events = run(c, samples)
    kinds = [e.kind for e in events]
    assert kinds == ['start', 'sync', 'lap']
    sync, lap = events[1], events[2]
    assert sync.t == pytest.approx(0.25 + 0.25 * (0.14 / 0.2))
    assert lap.lap_time == pytest.approx(lap.t - sync.t)


def test_min_lap_time_rejects_early_crossing():
    c = make(min_lap_time=30.0)
    samples = [(0.0, 0.7, 3.158), (0.5, 0.7, 2.0), (5.0, 0.7, 4.0),
               (10.0, 0.7, 3.21), (10.25, 0.7, 3.11)]
    assert all(e.kind != 'lap' for e in run(c, samples))


def test_teleport_resets_counter():
    c = make(teleport_distance=1.0)
    samples = [(0.0, 0.7, 3.158), (0.5, 0.7, 2.5), (1.0, 0.7, 1.8), (5.0, 0.7, 2.6),
               (8.0, 0.7, 3.4), (10.0, 0.7, 3.21), (10.25, 0.7, 3.11),   # vuelta 1
               (20.0, 0.745, -1.5), (20.5, 0.745, -1.5), (21.0, 0.745, -1.7)]   # reset: salto de 4.6 m
    events = run(c, samples)
    assert [e.kind for e in events] == ['start', 'lap', 'reset', 'start']
    assert c.laps == 0
    assert events[-1].t == 20.5      # el reloj arranca tras el reset, no antes

"""Logica del contador de vueltas y cronometro, sin ROS para poder probarla.

Un cruce de meta es el paso ORIENTADO del segmento que la define: entre
dos poses consecutivas el signo de la distancia a la recta cambia, el punto
de cruce cae dentro del segmento (de muro a muro) y el desplazamiento va en
el sentido de la carrera. Con eso una pasada lenta, una parada sobre la
linea o un retroceso no cuentan doble ni cuentan al reves. Ademas hay que
alejarse `rearm_distance` de la meta antes de poder contar otra vez, y una
vuelta mas corta que `min_lap_time` se descarta.

El instante del cruce se interpola entre las dos poses, asi que el
cronometro no queda limitado a la cadencia (~4 Hz) del puente.
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class LapEvent:
    kind: str                     # 'reset', 'start', 'sync' o 'lap'
    t: float                      # instante del evento (interpolado)
    x: float = 0.0
    y: float = 0.0
    lap: int = 0                  # numero de vuelta completada
    lap_time: float = 0.0
    best_time: float = 0.0
    total_time: float = 0.0
    lap_times: List[float] = field(default_factory=list)


class LapCounter:
    def __init__(self, line_a, line_b, direction, min_lap_time=5.0,
                 rearm_distance=1.0, start_motion=0.05, teleport_distance=1.0):
        self.ax, self.ay = float(line_a[0]), float(line_a[1])
        dx, dy = float(line_b[0]) - self.ax, float(line_b[1]) - self.ay
        self.seg_len2 = dx * dx + dy * dy
        if self.seg_len2 <= 0.0:
            raise ValueError('la linea de meta necesita dos puntos distintos')
        self.dx, self.dy = dx, dy
        norm = math.sqrt(self.seg_len2)
        self.nx, self.ny = -dy / norm, dx / norm          # normal unitaria
        dnorm = math.hypot(direction[0], direction[1])
        self.dirx, self.diry = direction[0] / dnorm, direction[1] / dnorm
        self.min_lap_time = min_lap_time
        self.rearm_distance = rearm_distance
        self.start_motion = start_motion
        self.teleport_distance = teleport_distance

        self.prev: Optional[tuple] = None    # (t, x, y)
        self.start_pose: Optional[tuple] = None
        self.started = False
        self.armed = False
        self.t_start: Optional[float] = None
        self.t_lap_start: Optional[float] = None
        self.laps = 0
        self.lap_times: List[float] = []

    # ------------------------------------------------------------------
    def signed_distance(self, x, y):
        """Distancia con signo a la recta de meta."""
        return (x - self.ax) * self.nx + (y - self.ay) * self.ny

    def crossing(self, prev, now):
        """Si el tramo prev->now cruza el segmento de meta en el sentido de
        la carrera, devuelve (t_cruce, x, y); si no, None."""
        _, x0, y0 = prev
        _, x1, y1 = now
        s0, s1 = self.signed_distance(x0, y0), self.signed_distance(x1, y1)
        if s0 == s1 or (s0 < 0.0) == (s1 < 0.0):
            return None                                    # no cambia de lado
        if (x1 - x0) * self.dirx + (y1 - y0) * self.diry <= 0.0:
            return None                                    # va a contramano
        u = s0 / (s0 - s1)                                 # fraccion del tramo
        cx, cy = x0 + u * (x1 - x0), y0 + u * (y1 - y0)
        proj = ((cx - self.ax) * self.dx + (cy - self.ay) * self.dy) / self.seg_len2
        if not 0.0 <= proj <= 1.0:
            return None                                    # fuera del segmento
        t_cross = prev[0] + u * (now[0] - prev[0])
        return t_cross, cx, cy

    # ------------------------------------------------------------------
    def update(self, t, x, y) -> List[LapEvent]:
        """Procesa una pose. Devuelve los eventos que produce (normalmente
        ninguno): arranque del cronometro, sincronizacion con la meta o
        vuelta completada."""
        now = (t, x, y)
        if self.prev is None:
            self.prev = now
            self.start_pose = (x, y)
            return []

        # Un salto imposible entre dos poses es un reset del simulador: se
        # vuelve a empezar desde cero, sin contar el tramo ni el tiempo.
        if math.hypot(x - self.prev[1], y - self.prev[2]) > self.teleport_distance:
            self.reset()
            self.prev = now
            self.start_pose = (x, y)
            return [LapEvent('reset', t, x, y)]

        events = []
        if not self.started:
            moved = math.hypot(x - self.start_pose[0], y - self.start_pose[1])
            if moved <= self.start_motion:
                self.prev = now
                return []
            self.started = True
            self.t_start = self.prev[0]         # ultima pose quieta
            self.t_lap_start = self.t_start
            events.append(LapEvent('start', self.t_start, x, y))

        cross = self.crossing(self.prev, now)
        if cross is not None:
            t_cross, cx, cy = cross
            if self.armed and t_cross - self.t_lap_start >= self.min_lap_time:
                lap_time = t_cross - self.t_lap_start
                self.laps += 1
                self.lap_times.append(lap_time)
                self.t_lap_start = t_cross
                self.armed = False
                events.append(LapEvent('lap', t_cross, cx, cy, self.laps, lap_time,
                                       min(self.lap_times), t_cross - self.t_start,
                                       list(self.lap_times)))
            elif not self.armed and self.laps == 0:
                # Salida cruzando la meta: el reloj de la vuelta 1 se
                # sincroniza con la linea en vez de con el arranque.
                self.t_lap_start = t_cross
                events.append(LapEvent('sync', t_cross, cx, cy))
        if not self.armed and abs(self.signed_distance(x, y)) > self.rearm_distance:
            self.armed = True

        self.prev = now
        return events

    def reset(self):
        self.started = False
        self.armed = False
        self.t_start = None
        self.t_lap_start = None
        self.laps = 0
        self.lap_times = []

    def current_lap_time(self, t):
        return None if self.t_lap_start is None else t - self.t_lap_start

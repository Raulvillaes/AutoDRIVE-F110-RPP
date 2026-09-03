"""Trayectoria global ciclica en el marco `map`.

Carga el CSV que entrega AutoDRIVE-F110-Global-Planner (columnas x, y, s,
kappa) y resuelve la geometria que necesita el Pure Pursuit: punto mas
cercano y punto de lookahead. La lista es ciclica: tras el ultimo punto
viene el primero, asi que ningun indice se sale de rango y el seguimiento
no se interrumpe al cerrar la vuelta.

Este modulo no depende de ROS para poder probarlo con pytest.
"""
import math

import numpy as np


def resolve_trajectory_path(param_value):
    """Ruta del CSV: la del parametro o, si esta vacio, la copia que viaja
    en el paquete (config/trajectory.csv)."""
    if param_value:
        return param_value
    from ament_index_python.packages import get_package_share_directory
    import os
    return os.path.join(get_package_share_directory('rpp_f110'),
                        'config', 'trajectory.csv')


class Trajectory:
    """Poligonal cerrada con longitud de arco y curvatura por punto."""

    def __init__(self, x, y, s=None, kappa=None):
        self.x = np.asarray(x, dtype=float)
        self.y = np.asarray(y, dtype=float)
        self.n = len(self.x)
        if self.n < 3:
            raise ValueError('la trayectoria necesita al menos 3 puntos')
        if s is None:
            steps = np.hypot(np.diff(self.x), np.diff(self.y))
            s = np.concatenate([[0.0], np.cumsum(steps)])
        self.s = np.asarray(s, dtype=float)
        self.kappa = (np.zeros(self.n) if kappa is None
                      else np.asarray(kappa, dtype=float))
        # El tramo de cierre (ultimo -> primero) no esta en `s`.
        closing = math.hypot(self.x[0] - self.x[-1], self.y[0] - self.y[-1])
        self.length = float(self.s[-1] + closing)

    @classmethod
    def from_csv(cls, path):
        data = np.genfromtxt(path, delimiter=',', names=True)
        names = data.dtype.names or ()
        for col in ('x', 'y'):
            if col not in names:
                raise ValueError(f'{path}: falta la columna {col}')
        s = data['s'] if 's' in names else None
        kappa = data['kappa'] if 'kappa' in names else None
        return cls(data['x'], data['y'], s, kappa)

    def distance_to(self, i, px, py):
        """Distancia euclidiana del punto i a (px, py)."""
        return math.hypot(self.x[i] - px, self.y[i] - py)

    def nearest_index(self, px, py, hint=None, back=5, ahead=30,
                      max_jump=1.0):
        """Indice del punto mas cercano a (px, py).

        Con `hint` (el indice del ciclo anterior) solo mira la ventana
        ciclica [hint-back, hint+ahead]: en un ciclo de control el coche no
        puede haberse ido lejos, y asi no se confunde con otro tramo de
        pista que pase cerca. Si aun asi el mejor punto de la ventana queda
        a mas de `max_jump` metros, repite la busqueda sobre toda la vuelta.
        """
        if hint is None:
            idx = np.arange(self.n)
        else:
            idx = np.arange(hint - back, hint + ahead + 1) % self.n
        d2 = (self.x[idx] - px) ** 2 + (self.y[idx] - py) ** 2
        best = int(np.argmin(d2))
        if hint is not None and d2[best] > max_jump ** 2:
            return self.nearest_index(px, py, hint=None)
        return int(idx[best])

    def lookahead_point(self, px, py, start, distance):
        """Punto del camino a `distance` metros (euclidianos) del vehiculo.

        Avanza desde `start` hasta el primer punto que queda a `distance` o
        mas y devuelve, interpolando en ese segmento, el punto que esta a
        `distance`: asi el objetivo cae sobre la circunferencia de radio L_d
        y la formula de curvatura del Pure Pursuit es exacta. (Se interpola
        la distancia linealmente dentro del segmento; con paso de 0.1 m el
        error respecto a la interseccion exacta es despreciable.)

        Si el vehiculo esta mas lejos del camino que `distance`, devuelve el
        punto mas cercano para que vuelva hacia el.

        Returns:
            (x, y, i): el punto y el indice del segmento [i, i+1] donde cae.
        """
        i = start
        d_i = self.distance_to(i, px, py)
        if d_i >= distance:
            return float(self.x[i]), float(self.y[i]), i
        for _ in range(self.n):
            j = (i + 1) % self.n
            d_j = self.distance_to(j, px, py)
            if d_j >= distance:
                t = (distance - d_i) / (d_j - d_i)
                return (float(self.x[i] + t * (self.x[j] - self.x[i])),
                        float(self.y[i] + t * (self.y[j] - self.y[i])), i)
            i, d_i = j, d_j
        # Ningun punto llega a `distance`: lookahead mayor que la pista.
        return float(self.x[start]), float(self.y[start]), start

    def max_curvature_ahead(self, start, distance):
        """Mayor |kappa| del CSV en los proximos `distance` metros de camino
        a partir del punto `start` (ciclico). Sirve para frenar ANTES de
        entrar en una curva, no cuando el arco al lookahead ya se cierra."""
        step = self.length / self.n
        count = max(1, int(math.ceil(distance / step)) + 1)
        idx = (start + np.arange(count)) % self.n
        return float(np.abs(self.kappa[idx]).max())

    def heading(self, i):
        """Rumbo (rad) del segmento que sale del punto i, ciclico."""
        j = (i + 1) % self.n
        return math.atan2(self.y[j] - self.y[i], self.x[j] - self.x[i])

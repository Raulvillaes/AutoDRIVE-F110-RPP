"""Compensacion del retardo de mando.

Entre publicar un mando y ver su efecto en la pose pasan varios ciclos
(ver README, "Retardo y estabilidad"). Los mandos publicados en los ultimos
`delay` segundos todavia no han actuado: el coche los ejecutara a partir
de ahora. Este modulo integra el modelo de bicicleta cinematica con esos
mandos pendientes para estimar donde estara el coche cuando actue el mando
que se calcula en este ciclo. El servo de direccion tampoco es instantaneo:
se modela como un sistema de primer orden con constante de tiempo
`steering_lag`, asi que el angulo real de las ruedas persigue al mandado. El Pure Pursuit se aplica sobre esa pose
predicha en vez de sobre la medida, y asi puede usar un lookahead corto
(que recorta menos las curvas) sin oscilar en la recta.

Sin ROS, para poder probarlo con pytest.
"""
import math


def predict_pose(x, y, yaw, speed, wheelbase, commands, now, delay,
                 steering_lag=0.0, step=0.02):
    """Pose estimada `delay` segundos despues de `now`.

    `commands` es la lista de (instante, angulo de ruedas en rad) publicados,
    en orden. Se integra desde `now - delay` hasta `now` con el mando que
    estaba vigente en cada tramo (el ultimo publicado antes de ese instante)
    y la velocidad `speed` constante; lo recorrido en ese intervalo de
    mandos es lo que el coche recorrera en los proximos `delay` segundos.

    Con `steering_lag` > 0 el angulo real de las ruedas sigue al mandado
    como un primer orden, d(delta)/dt = (mando - delta) / steering_lag, y
    se integra en pasos de `step` segundos. Para conocer el angulo real al
    principio de la ventana se filtra tambien la historia anterior a ella
    (por eso conviene guardar mandos de unas cuantas constantes de tiempo).

    Con `delay` <= 0, sin mandos o parado devuelve la pose tal cual.
    """
    if delay <= 0.0 or not commands or abs(speed) < 1e-3:
        return x, y, yaw
    t_start = now - delay
    if steering_lag <= 0.0:
        return _integrate(x, y, yaw, speed, wheelbase, commands, t_start, now)
    # Angulo real de las ruedas en t_start: el filtro recorre la historia
    # anterior a la ventana; antes del primer mando se supone que las
    # ruedas ya lo tenian. `active` queda como el mando vigente en t_start.
    delta = commands[0][1]
    active = delta
    t = commands[0][0]
    for t_cmd, cmd in commands[1:]:
        if t_cmd > t_start:
            break
        delta += (active - delta) * (1.0 - math.exp(-(t_cmd - t) / steering_lag))
        t, active = t_cmd, cmd
    if t_start > t:
        delta += (active - delta) * (1.0 - math.exp(-(t_start - t) / steering_lag))
    # Ventana [t_start, now]: mando vigente en cada tramo y ruedas que lo persiguen.
    pending = [(t_c, c) for t_c, c in commands if t_c > t_start]
    t = t_start
    for t_next, delta_next in pending + [(now, None)]:
        while t < t_next - 1e-9:
            dt = min(step, t_next - t)
            delta += (active - delta) * (1.0 - math.exp(-dt / steering_lag))
            x += speed * math.cos(yaw) * dt
            y += speed * math.sin(yaw) * dt
            yaw += speed * math.tan(delta) / wheelbase * dt
            t += dt
        if delta_next is not None:
            active = delta_next
    return x, y, yaw


def _integrate(x, y, yaw, speed, wheelbase, commands, t_start, now):
    """Bicicleta con ruedas instantaneas: cada tramo usa el mando vigente."""
    active = commands[0][1]
    pending = []
    for t, delta in commands:
        if t <= t_start:
            active = delta
        else:
            pending.append((t, delta))
    t = t_start
    for t_next, delta_next in pending + [(now, None)]:
        dt = t_next - t
        if dt > 0.0:
            x += speed * math.cos(yaw) * dt
            y += speed * math.sin(yaw) * dt
            yaw += speed * math.tan(active) / wheelbase * dt
        t = t_next
        if delta_next is not None:
            active = delta_next
    return x, y, yaw

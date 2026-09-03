# AutoDRIVE-F110-RPP

Controlador **Regulated Pure Pursuit (RPP)** para el F1TENTH del simulador
AutoDRIVE, en ROS 2 Humble. Lee la trayectoria global suavizada de la
Parte 1, la sigue en pista y cuenta y cronometra cada vuelta en la terminal.

Segunda parte del proyecto final de Vehiculos no Tripulados. La primera parte
(mapeado con SLAM Toolbox, planificacion con LPA\* y suavizado) vive en
[AutoDRIVE-F110-Global-Planner](https://github.com/Raulvillaes/AutoDRIVE-F110-Global-Planner).

> **Video de evidencia:** PENDIENTE (enlace de YouTube).

## Indice

- [Resultado](#resultado)
- [Prerrequisitos](#prerrequisitos)
- [Instalacion](#instalacion)
- [Ejecucion](#ejecucion)
- [La trayectoria de entrada](#la-trayectoria-de-entrada)
- [Marcos de coordenadas: por que no hace falta localizacion](#marcos-de-coordenadas-por-que-no-hace-falta-localizacion)
- [El controlador](#el-controlador)
- [Contador de vueltas y cronometro](#contador-de-vueltas-y-cronometro)
- [Estructura del codigo](#estructura-del-codigo)
- [Interfaz ROS 2](#interfaz-ros-2)
- [Parametros](#parametros)
- [Sintonizacion](#sintonizacion)
- [Herramientas](#herramientas)

## Resultado

Vueltas consecutivas sin tocar el muro a 1.2 m/s de crucero, con frenado
anticipado a 0.5 m/s en las curvas de 1 m de radio:

| Vuelta | Tiempo |
|-------:|-------:|
| 1 | 25.42 s |
| 2 | 25.33 s |
| 3 | 25.38 s |
| 4 | 25.30 s |
| 5 | 25.34 s |

Vuelta de 27.85 m, velocidad media 1.10 m/s. Error lateral respecto a la
trayectoria de 7 cm (rms) y 31 cm maximo; holgura minima al muro vista por
el LiDAR de 0.47 m. El lazo corre a la cadencia del puente, unos 5 Hz.

## Prerrequisitos

- Ubuntu 22.04 y **ROS 2 Humble**.
- **Simulador AutoDRIVE** y su puente de ROS 2 (`autodrive_f1tenth`) ya
  compilados en `~/autodrive_ws`, siguiendo el
  [Tutorial 1](https://github.com/nabihandres/AUTODRIVE/blob/main/Tutorial%201%3A%20AutoDrive%20Installation%20and%20Setup.md)
  del curso. El puente necesita su propio entorno virtual en
  `~/autodrive_ws/venv`; este paquete solo anade `numpy`, que ya esta ahi.
- No hace falta el repositorio de la Parte 1: la trayectoria viaja dentro
  de este paquete (ver [La trayectoria de entrada](#la-trayectoria-de-entrada)).

## Instalacion

El repositorio es directamente un paquete `ament_python`. Se clona dentro
de `src` del workspace del simulador y se compila con colcon:

```bash
cd ~/autodrive_ws/src
git clone git@github.com:Raulvillaes/AutoDRIVE-F110-RPP.git rpp_f110

cd ~/autodrive_ws
source /opt/ros/humble/setup.bash
source venv/bin/activate
colcon build --packages-select rpp_f110 --symlink-install
```

`--symlink-install` es opcional: permite editar `config/params.yaml` sin
recompilar.

## Ejecucion

Hacen falta el simulador y dos terminales. **Todo se sourcea bajo `bash`**:
`/opt/ros/humble/setup.bash` no resuelve su propia ruta en `zsh` y falla
con `no such file or directory: .../setup.sh`.

**1. Simulador.** Abrir `AutoDRIVE Simulator`, pulsar *Connect* y poner el
modo de conduccion en **Autonomous**. El simulador arranca en *Manual* y en
ese modo **descarta en silencio** los comandos de ROS: el coche no se mueve
y la realimentacion de direccion se queda en 0.0. No hay topic para
cambiarlo; se hace en la interfaz.

**2. Puente** (terminal 1):

```bash
cd ~/autodrive_ws
source /opt/ros/humble/setup.bash
source venv/bin/activate
source install/setup.bash
export PYTHONUNBUFFERED=1
ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py
```

Con `simulator_bringup_rviz.launch.py` se abre ademas RViz. Si se reinicia
el simulador hay que relanzar el puente: los topics siguen anunciados pero
no fluye ningun dato.

**3. Controlador** (terminal 2):

```bash
cd ~/autodrive_ws
source /opt/ros/humble/setup.bash
source venv/bin/activate
source install/setup.bash
ros2 launch rpp_f110 rpp.launch.py
```

Arrancan tres nodos: `rpp_node` (control), `lap_node` (vueltas y
cronometro, imprime en esta terminal) y `path_node` (visualizacion). Para
verlo en RViz se anaden `/rpp/path` (Path), `/rpp/lookahead` (Marker) y
`/rpp/finish_line` (Marker) con el marco fijo en `map`.

Con `total_laps` mayor que cero en `config/params.yaml` (10 por defecto),
al completar esas vueltas `lap_node` imprime el resumen y el launch apaga
todo; el controlador deja el acelerador a cero antes de salir. Con Ctrl+C
pasa lo mismo.

Argumentos del launch:

```bash
ros2 launch rpp_f110 rpp.launch.py params_file:=/ruta/a/otro.yaml   # otros parametros
ros2 launch rpp_f110 rpp.launch.py log_csv:=/tmp/rpp.csv             # registro por ciclo
```

**Reset del simulador.** El boton *Reset* devuelve el coche a la salida
pero corta la conexion con el puente y deja el modo en *Manual*: hay que
pulsar *Connect* y *Autonomous* otra vez. No hace falta relanzar nada de
este paquete: el contador detecta el salto de pose, se pone a cero y
espera a que el coche se mueva.

**Carga de la maquina.** El lazo de control corre a la cadencia del
simulador (unos 5 Hz). Si el simulador pierde cuadros porque la maquina
esta ocupada con otras cosas, el puente publica mas lento, el retardo entre
mando y efecto crece y el coche subvira en las curvas cerradas. Para las
vueltas de evidencia conviene no tener nada mas abierto y usar el puente
*headless* si RViz no hace falta.

## La trayectoria de entrada

El controlador se alimenta de un CSV con la vuelta completa en el marco
`map`:

| Columna | Unidad | Significado                                     |
|---------|--------|-------------------------------------------------|
| `x`     | m      | posicion en el marco `map`                      |
| `y`     | m      | posicion en el marco `map`                      |
| `s`     | m      | longitud de arco acumulada desde el primer punto |
| `kappa` | 1/m    | curvatura con signo, positiva a izquierdas      |

La lista es **ciclica**: el ultimo punto no repite al primero, empalma con
el. El controlador la recorre en circulo y el seguimiento no se interrumpe
al cerrar la vuelta.

**Ya hay un archivo listo:** `config/trajectory.csv` es una copia de la
salida de la Parte 1 (279 puntos a 0.10 m, vuelta de 27.85 m, radio de
giro minimo 1.01 m, holgura minima al muro 0.40 m). Asi el paquete se
ejecuta y se revisa sin correr LPA\*. Para regenerarla o usar otra:

```bash
git clone https://github.com/Raulvillaes/AutoDRIVE-F110-Global-Planner.git
cd AutoDRIVE-F110-Global-Planner
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt
cd f1tenth && python f1tenth_map.py --no-anim
# resultado: f1tenth/resultados/trajectory.csv
```

y se apunta a ella con el parametro `trajectory_csv` de `rpp_node` y
`path_node` (vacio = la copia del paquete).

## Marcos de coordenadas: por que no hace falta localizacion

El puente de AutoDRIVE publica la transformacion `map -> f1tenth_1` a
partir del IPS y la IMU del simulador, es decir, la pose verdadera del
coche, con el marco del vehiculo en el centro del eje trasero. Ese `map` es
el mismo sistema de coordenadas del mapa de SLAM Toolbox sobre el que se
planifico la trayectoria: en la Parte 1 se comprobo proyectando la pose del
TF sobre el PGM (`tools/check_frame.py pose` en aquel repositorio), y cae
centrada en el carril.

Consecuencia: **los waypoints se consumen tal cual** y la pose se lee del
TF. No hay AMCL, ni filtro de particulas, ni cambio de marco.

Los nodos leen la pose de dos topics del puente, `/autodrive/f1tenth_1/ips`
(posicion) y `/autodrive/f1tenth_1/imu` (orientacion), y no del TF. Son los
mismos numeros que el puente mete en `map -> f1tenth_1`, pero el puente crea
un `TransformBroadcaster` **nuevo por cada mensaje** de `/tf`, y cada
publisher nuevo tiene que ser descubierto por DDS antes de que su mensaje
llegue: recien lanzado el puente la latencia es de 1 ms, pero con el tiempo
se degrada hasta perder la mitad de las poses (2 Hz efectivos tras media
hora). Los topics de IPS e IMU salen de publishers fijos y llegan siempre a
la cadencia del simulador, unos 4 a 5 Hz. El parametro `pose_source`
permite volver a `tf`.

Cada pose nueva dispara un ciclo de control en el instante en que llega: no
se calcula sobre datos viejos ni se pierden ciclos. La cadencia del puente
es la cadencia del lazo.

## El controlador

### Pure Pursuit

En cada ciclo se busca el punto de la trayectoria mas cercano al coche y,
avanzando desde el, el primer punto a distancia `L_d` (el *lookahead*),
interpolado dentro de su segmento para que quede exactamente sobre la
circunferencia de radio `L_d`. Ese punto se expresa en el marco del
vehiculo; su coordenada lateral `y_L` fija la curvatura del arco que pasa
por el eje trasero, tangente al rumbo actual, y llega al punto:

```
gamma = 2 * y_L / L_d^2
```

Con el modelo de bicicleta cinematica (el marco del vehiculo esta en el eje
trasero, que es donde vale la formula) el angulo de las ruedas es

```
delta = atan(L * gamma)          L = 0.33 m (batalla)
```

y el simulador recibe un mando normalizado. Se midio contra el simulador
que la relacion es lineal y simetrica: mando 1.0 da 0.524 rad (30°), asi que

```
steering_command = delta / 0.524      saturado a [-1, 1]
```

La busqueda del punto mas cercano usa una ventana ciclica alrededor del
indice del ciclo anterior: el coche no puede haberse ido lejos en un ciclo
y asi no se confunde con otro tramo de pista que pase cerca (el circuito
tiene tramos paralelos separados por un muro). Si aun asi el mejor punto
queda a mas de 1 m, repite la busqueda sobre toda la vuelta.

### Lookahead adaptativo

```
L_d = clamp(lookahead_time * v_medida, lookahead_min, lookahead_max)
```

Un lookahead corto sigue la trayectoria con precision pero oscila a
velocidad alta; uno largo es estable pero recorta las curvas. Escalarlo con
la velocidad da lo mejor de cada caso, y los limites evitan que a velocidad
cero apunte al punto mas cercano (inestable) o que en recta apunte
demasiado lejos.

### Las tres regulaciones

Lo que distingue al RPP del Pure Pursuit clasico es que la velocidad
objetivo no es fija: parte de `max_speed` y pasa por tres regulaciones.

1. **Curvatura.** Si el radio del arco `1/|gamma|` baja de
   `regulated_min_radius`, la velocidad escala linealmente con el radio:
   `v = v * r / r_min`. El coche frena en las curvas cerradas en proporcion
   a lo cerradas que son.
2. **Proximidad.** El LiDAR mira un sector frontal de `proximity_fov`
   grados. Si la distancia libre minima `d` baja de `proximity_distance`,
   `v = v * proximity_gain * d / proximity_distance`. En el circuito sin
   obstaculos actua sobre todo al encarar una pared en la entrada de una
   curva.
3. **Aceleracion.** La velocidad objetivo no puede cambiar mas de
   `max_accel * dt` al subir ni de `max_decel * dt` al bajar entre ciclos.
   Suaviza los escalones que dejan las dos anteriores.

Despues de las dos primeras se aplica un suelo `min_speed`, para que ninguna
regulacion deje el coche parado en mitad de la pista.

### De velocidad a mando de acelerador

El simulador no acepta una velocidad: acepta un mando de acelerador
normalizado en [-1, 1], y tampoco publica la velocidad del coche. Se midio
la velocidad estacionaria que da cada mando (ver
[Sintonizacion](#sintonizacion)) y se invierte esa curva como
prealimentacion, con una correccion proporcional sobre la velocidad medida:

```
throttle = throttle_offset + throttle_per_mps * v_objetivo
         + speed_kp * (v_objetivo - v_medida)
```

saturado a `[throttle_min, throttle_max]`. Con `throttle_min = 0` el coche
frena por inercia; un valor negativo permite freno motor.

### Medicion de la velocidad

El simulador tampoco publica la velocidad. Hay dos fuentes y un parametro
`speed_source` para elegir:

- `tf` (la que se usa): desplazamiento entre dos poses consecutivas
  dividido por el tiempo entre ellas, proyectado sobre el rumbo (con signo)
  y con un filtro exponencial `speed_filter`. A 5 Hz y con la pose exacta
  del simulador es limpia: oscila 0.05 m/s alrededor de la consigna.
- `encoders`: derivada del angulo de cada encoder de rueda por el radio de
  rueda `wheel_radius`. **Descartada:** el puente repite el mismo angulo en
  todos los mensajes aunque el coche vaya a 1.5 m/s, y la derivada sale
  constante (3.37 rad/s parado o en marcha).

### Retardo y estabilidad

Entre publicar un mando y ver su efecto en la pose pasan entre 0.5 y 1 s:
el puente saliente escribe el mando en un archivo que el puente entrante
lee en el siguiente lote, el simulador lo aplica y el servo de direccion
tiene su propia dinamica. Con ese retardo el Pure Pursuit oscila si el
tiempo de lookahead `L_d / v` se acerca al retardo: con `L_d / v = 1.0 s`
el coche entro en la recta con 5° de rumbo desviado y el zigzag crecio
(24, 31 y 37 cm) hasta el muro. Con `lookahead_time = 1.5 s` la correccion
se amortigua. Por eso el lookahead minimo es 1.0 m y no menos, aunque en
las curvas cerradas recorte un poco.

## Contador de vueltas y cronometro

`lap_node` es independiente del controlador: solo lee el TF. La linea de
meta es el segmento de muro a muro `(1.47, 3.16) -> (0.29, 3.16)`, el mismo
que en la Parte 1 hace de muro virtual para obligar a planificar la vuelta
entera; el coche aparece sobre ella mirando a `-y`.

Un cruce es el **paso orientado del segmento**: entre dos poses
consecutivas cambia el signo de la distancia a la recta, el punto de cruce
cae dentro del segmento, y el desplazamiento va en el sentido de la carrera
(`finish_direction`). No se detecta por distancia a un punto: con eso una
pasada lenta o una parada sobre la meta contarian doble. Dos protecciones
mas: hay que alejarse `rearm_distance` de la linea para rearmar el conteo,
y una vuelta mas corta que `min_lap_time` se descarta.

El **instante del cruce se interpola** entre las dos poses que abrazan la
linea, asi que el cronometro tiene mas resolucion que la cadencia del
puente. El reloj arranca cuando el coche empieza a moverse; si la salida
queda antes de la meta, el primer cruce lo sincroniza con la linea.

Por cada vuelta imprime en la terminal el numero, el tiempo de la vuelta,
la mejor y el acumulado:

```
==================================================
  VUELTA 3 COMPLETADA
  Tiempo de vuelta:      PENDIENTE s
  Mejor vuelta:          PENDIENTE s
  Tiempo acumulado:      PENDIENTE s
==================================================
```

## Estructura del codigo

```
rpp_f110/
  rpp_node.py       controlador RPP: /tf + LiDAR -> throttle_command, steering_command
  lap_node.py       contador de vueltas y cronometro: /tf -> terminal
  lap_counter.py    logica del cruce orientado y del cronometro (sin ROS, con pruebas)
  path_node.py      trayectoria como nav_msgs/Path y meta como Marker para RViz
  trajectory.py     carga del CSV y geometria ciclica (punto mas cercano, lookahead)
  vehicle_pose.py   pose del vehiculo desde /tf (map -> f1tenth_1) y guinada del cuaternion
config/
  params.yaml       parametros de los tres nodos
  trajectory.csv    copia de la trayectoria de la Parte 1
launch/
  rpp.launch.py     lanza los tres nodos; apaga todo cuando lap_node termina
tools/
  measure_speed.py  escalon de acelerador: velocidad estacionaria, cadencia, frenada
  replay_pose.py    publica /tf recorriendo el CSV, para probar sin simulador
test/
  test_trajectory.py, test_lap_counter.py   pruebas con pytest
```

Funciones principales de `rpp_node.py`:

| Funcion | Que hace |
|---|---|
| `on_pose` | ciclo de control completo, disparado por cada pose del TF |
| `regulate_curvature` | regulacion 1: velocidad proporcional al radio del arco |
| `regulate_proximity` | regulacion 2: velocidad proporcional a la distancia libre del LiDAR |
| `limit_acceleration` | regulacion 3: limite de cambio de velocidad por ciclo |
| `speed_to_throttle` | velocidad objetivo a mando de acelerador |
| `on_scan` | distancia libre minima en el sector frontal |
| `on_encoder`, `update_tf_speed` | las dos estimaciones de velocidad |
| `check_pose_timeout` | acelerador a cero si el puente deja de publicar |
| `stop_vehicle` | ceros al salir: el puente conserva el ultimo mando recibido |

En `trajectory.py`: `nearest_index` (ventana ciclica con rebusqueda global)
y `lookahead_point` (primer punto a `L_d`, interpolado). En
`lap_counter.py`: `crossing` (cruce orientado e instante interpolado) y
`update` (estado del contador).

## Interfaz ROS 2

| Dir | Topic | Tipo | Nodo | Descripcion |
|---|---|---|---|---|
| Sub | `/tf` | `tf2_msgs/TFMessage` | rpp, lap | pose `map -> f1tenth_1` del puente |
| Sub | `/autodrive/f1tenth_1/lidar` | `sensor_msgs/LaserScan` | rpp | regulacion por proximidad |
| Sub | `/autodrive/f1tenth_1/left_encoder`, `right_encoder` | `sensor_msgs/JointState` | rpp | velocidad por encoders |
| Pub | `/autodrive/f1tenth_1/throttle_command` | `std_msgs/Float32` | rpp | acelerador normalizado [-1, 1] |
| Pub | `/autodrive/f1tenth_1/steering_command` | `std_msgs/Float32` | rpp | direccion normalizada [-1, 1] |
| Pub | `/rpp/lookahead` | `visualization_msgs/Marker` | rpp | punto objetivo |
| Pub | `/rpp/target_speed`, `/rpp/measured_speed` | `std_msgs/Float32` | rpp | para `rqt_plot` |
| Pub | `/rpp/path` | `nav_msgs/Path` | path | trayectoria |
| Pub | `/rpp/finish_line` | `visualization_msgs/Marker` | path | linea de meta |

## Parametros

Todos en `config/params.yaml`. Los valores son los de las vueltas de
[Resultado](#resultado).

### `rpp_node`

| Parametro | Valor | Descripcion |
|---|---|---|
| `trajectory_csv` | `""` | CSV de la trayectoria; vacio = `config/trajectory.csv` |
| `wheelbase` | 0.33 m | batalla, del TF de la rueda delantera |
| `max_steering_angle` | 0.524 rad | angulo con mando 1.0, medido |
| `lookahead_time` | 1.5 s | `L_d = lookahead_time * v` |
| `lookahead_min`, `lookahead_max` | 1.0, 2.5 m | limites de `L_d` |
| `max_speed` | 1.2 m/s | velocidad de crucero |
| `min_speed` | 0.4 m/s | suelo de la regulacion por curvatura |
| `regulated_min_radius` | 1.5 m | radio por debajo del cual frena |
| `curvature_lookahead` | 1.5 m | camino por delante cuya curvatura tambien regula |
| `proximity_distance` | 1.0 m | distancia libre a la que empieza a frenar |
| `proximity_gain` | 1.0 | ganancia de la regulacion por proximidad |
| `proximity_fov` | 30° | abertura del sector frontal del LiDAR |
| `max_accel`, `max_decel` | 1.0, 2.0 m/s² | limite de aceleracion y frenada |
| `throttle_offset`, `throttle_per_mps` | 0.0, 0.2 | prealimentacion: 5 m/s por unidad de mando, medido |
| `speed_kp` | 0.1 | correccion proporcional |
| `throttle_min`, `throttle_max` | 0.0, 1.0 | saturacion del acelerador |
| `speed_source` | `tf` | `tf` o `encoders` |
| `wheel_radius` | 0.058 m | solo para `encoders` |
| `speed_filter` | 0.5 | peso de la muestra nueva en el filtro de velocidad |
| `pose_source` | `sensors` | `sensors` (IPS + IMU) o `tf` |
| `pose_timeout` | 1.0 s | sin poses -> acelerador a cero |
| `log_csv` | `""` | registro por ciclo para sintonizar |

### `lap_node`

| Parametro | Valor | Descripcion |
|---|---|---|
| `finish_line` | `[1.47, 3.16, 0.29, 3.16]` | meta de muro a muro, en `map` |
| `finish_direction` | `[0.0, -1.0]` | sentido de la carrera al cruzar |
| `min_lap_time` | 5.0 s | descarta cruces antes de este tiempo |
| `rearm_distance` | 1.0 m | distancia a la meta para rearmar |
| `total_laps` | 10 | resumen y apagado al completarlas; 0 = sin limite |
| `progress_period` | 0.0 s | >0: imprime la vuelta en curso cada tanto |
| `pose_source` | `sensors` | igual que en `rpp_node` |

## Sintonizacion

Todo lo que sigue se midio contra el simulador; nada sale de una hoja de
datos.

### Direccion

Con `tools/check_frame.py steering` de la Parte 1: el mando de direccion es
lineal y simetrico, sin zona muerta, y 1.0 da 0.524 rad. Positivo gira a
la izquierda, como en ROS (comprobado con el registro: mando positivo,
guinada creciente).

### Acelerador

`tools/measure_speed.py` aplica escalones de mando en la recta de salida y
mide la velocidad estacionaria por la pose:

| Mando | Velocidad estacionaria | Tiempo al 90 % |
|------:|-----------------------:|---------------:|
| 0.15 | 0.71 m/s | 2.0 s |
| 0.30 | 1.51 m/s | 1.0 s |
| 0.50 | 2.48 m/s | ~1 s |

Es lineal, unos 5 m/s por unidad de mando y sin zona muerta: de ahi
`throttle_per_mps = 0.2` y `throttle_offset = 0`. Con mando 0.0 el coche
frena solo a unos 3 m/s² (de 1.5 m/s a parado en 0.6 m), asi que
`throttle_min = 0` basta para frenar. La correccion proporcional
`speed_kp = 0.1` compensa el error residual: a consigna 1.2 m/s la
velocidad medida oscila entre 1.15 y 1.25.

### Pasadas de lazo cerrado

| Pasada | Crucero | `L_d` | Otros | Resultado |
|---|---|---|---|---|
| 1 | 1.0 m/s | 0.6 s, [0.6, 1.3] | pose por `/tf` | oscilacion creciente en la recta y muro a los 6 m; el lazo recibia 2 Hz |
| 2 | 1.0 m/s | 1.0 s, [1.0, 2.0] | pose por IPS + IMU | vuelta de 30.5 s; muro en la vuelta 3 al entrar en la curva cerrada con 29 cm de error arrastrado |
| 3 | 1.2 m/s | 1.0 s, [0.8, 2.0] | + curvatura anticipada 1.5 m, radio 1.5 m, proximidad 1.0 m | 4 vueltas de 26 s; zigzag en la recta en la vuelta 5 |
| 4 | 1.2 m/s | 1.5 s, [1.0, 2.5] | igual | vueltas de 25.3 s, sin tocar el muro |

Dos lecciones. La primera, el retardo del mando fija el lookahead minimo:
por debajo de `L_d / v = 1.5 s` el coche zigzaguea. La segunda, la
regulacion por curvatura del arco actual llega tarde: el arco al lookahead
solo se cierra cuando el coche ya esta en la curva. Mirar la curvatura del
CSV 1.5 m por delante frena antes de entrar, y con eso la curva de 1 m de
radio se toma a 0.5 m/s y con 0.47 m de holgura.

### Como sintonizar

1. Poner `log_csv` en `config/params.yaml` a una ruta; cada ciclo escribe
   pose, indice, lookahead, curvatura, mandos, velocidades y distancia libre.
2. Lanzar, dejar varias vueltas, y mirar el error lateral respecto a
   `config/trajectory.csv` y la distancia libre minima.
3. Subir `max_speed` de 0.2 en 0.2 m/s; si zigzaguea en recta, subir
   `lookahead_time`; si se abre en las curvas, subir `regulated_min_radius`
   o `curvature_lookahead`.

## Herramientas

- `tools/measure_speed.py MANDO`: aplica un escalon de acelerador con la
  direccion a cero, corta cuando el LiDAR ve pared cerca y graba la
  frenada. Imprime cadencia del puente, velocidad estacionaria, radio de
  rueda implicito y deceleracion, y guarda un CSV.
- `tools/replay_pose.py`: publica `/tf` recorriendo el CSV a velocidad
  constante. Sirve para probar `lap_node` y `rpp_node` sin simulador.
- Pruebas unitarias de la geometria y del contador:

```bash
cd ~/autodrive_ws/src/rpp_f110
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test
```

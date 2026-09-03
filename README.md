# AutoDRIVE-F110-RPP

Controlador **Regulated Pure Pursuit** para el F1TENTH del simulador AutoDRIVE.
Un nodo de ROS 2 lee la trayectoria global ya suavizada, la sigue en pista y
lleva la cuenta y el cronometraje de cada vuelta.

> **Estado:** en desarrollo. Este repositorio todavia no contiene el paquete de
> ROS 2; de momento fija el alcance y el contrato de entrada.

## Prerrequisito

La trayectoria no se genera aqui. Viene de
**[AutoDRIVE-F110-Global-Planner](https://github.com/Raulvillaes/AutoDRIVE-F110-Global-Planner)**,
que parte del mapa del circuito construido con SLAM Toolbox, planifica una
vuelta completa con LPA\* y entrega los waypoints suavizados en CSV.

Hay que generarlos alli **antes** de ejecutar nada de este repositorio:

```bash
git clone https://github.com/Raulvillaes/AutoDRIVE-F110-Global-Planner.git
cd AutoDRIVE-F110-Global-Planner
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt
cd f1tenth && python f1tenth_map.py
```

El CSV resultante es la unica entrada que este controlador necesita del
planificador.

## Contrato de entrada

Trayectoria en coordenadas del marco `map`, la misma en la que el puente de
AutoDRIVE publica la pose del vehiculo:

| Columna | Unidad | Significado                                  |
|---------|--------|----------------------------------------------|
| `x`     | m      | posicion en el marco `map`                   |
| `y`     | m      | posicion en el marco `map`                   |
| `s`     | m      | longitud de arco acumulada desde el inicio   |
| `kappa` | 1/m    | curvatura, para regular la velocidad         |

La lista es ciclica: el ultimo punto empalma con el primero, de modo que el
seguimiento no se interrumpe al cerrar la vuelta.

## Alcance previsto

- **`rpp_node`** — Regulated Pure Pursuit: lookahead adaptativo con la
  velocidad, y las tres regulaciones que dan nombre al metodo (por curvatura,
  por proximidad de obstaculo con el LiDAR y por limite de aceleracion).
  Publica `/autodrive/f1tenth_1/throttle_command` y
  `/autodrive/f1tenth_1/steering_command`.
- **`lap_node`** — cruce orientado de la linea de meta: contador de vueltas y
  tiempo por vuelta por terminal.
- **`path_node`** — publica la trayectoria como `nav_msgs/Path` para verla en
  RViz.

## Entorno

- ROS 2 Humble
- Simulador AutoDRIVE con el puente `autodrive_f1tenth`

# Lab 3 — Localización (IIC2685)

Paquete ROS 2 `lab3` con build type `ament_cmake`, correspondiente al
Laboratorio 3 del ramo **IIC2685 Robótica Móvil**.

El paquete está vacío: solo incluye el esqueleto y el material entregado por el
curso. Los nodos, launch files y la implementación se agregan durante el
desarrollo del laboratorio.

## Contenido

```text
Lab_3_IIC2685/
├── CMakeLists.txt
├── package.xml
├── Laboratorio_3.pdf      # Enunciado del laboratorio
├── config/
│   ├── mapa.yaml          # Metadata del mapa
│   └── mapa.pgm           # Imagen del laberinto
├── launch/                # Archivos launch (por ahora vacío)
└── nodes/                 # Nodos del laboratorio (por ahora vacío)
```

## Compilar

Dentro del contenedor ROS, desde la raíz del workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select lab3 --symlink-install
source install/setup.bash
```

## Referencias del enunciado

- El mapa se publica en `/world_map` al abrirlo desde el simulador.
- El LIDAR publica en `/scan` (`sensor_msgs/msg/LaserScan`).
- La exploración se inicia con mensajes `std_msgs/msg/Empty` en `/explore` y
  reporta en `/displacement` (`geometry_msgs/msg/Pose`).
- El localizador debe publicar las partículas en `/particles`
  (`geometry_msgs/msg/PoseArray`).
- El launch de la demostración debe llamarse `localization.xml`.

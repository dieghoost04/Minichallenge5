# MiniChallenge 5: Bug Algorithms Implementation

## Descripción del Proyecto

Este proyecto implementa los algoritmos Bug Two y Bug Zero para navegación robótica en ROS (Robot Operating System). Estos algoritmos permiten que un robot navegue hacia una meta mientras evita obstáculos en su camino. El proyecto está estructurado de la siguiente manera:


## Contenidos del Proyecto

### Archivos y Directorios

- **CMakeLists.txt**: Archivo de configuración de CMake para el proyecto.
- **include/minichallenge5**: Directorio para archivos de encabezado (headers) si es necesario.
- **launch**: Directorio que contiene los archivos de lanzamiento.
  - `bug_two.launch`: Archivo de lanzamiento para el algoritmo Bug Two.
  - `bug_zero.launch`: Archivo de lanzamiento para el algoritmo Bug Zero.
- **package.xml**: Archivo de configuración del paquete ROS.
- **scripts**: Directorio que contiene los scripts de Python para los algoritmos y otros módulos auxiliares.
  - `bug_two.py`: Implementación del algoritmo Bug Two.
  - `bug_zero.py`: Implementación del algoritmo Bug Zero.
  - `conditions.py`: Módulo con condiciones auxiliares utilizadas por los algoritmos.
  - `gtg.py`: Módulo para la funcionalidad "Go-To-Goal".
  - `move_to_goal.py`: Módulo para mover el robot hacia la meta.
  - `wf.py`: Módulo para la funcionalidad "Wall-Following".
  - `__pycache__`: Directorio que contiene archivos bytecode compilados de Python.

## Requisitos Previos

Antes de ejecutar el proyecto, asegúrate de tener instalado lo siguiente:

- ROS (Robot Operating System)
- Python 3.8 o superior
- Dependencias de ROS para navegación y control de robots

## Instalación

1. Clona este repositorio en tu espacio de trabajo de ROS:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/tu_usuario/minichallenge5.git
   cd ~/catkin_ws
   catkin_make

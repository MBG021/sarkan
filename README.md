# SARKAN
### Smart Autonomous Robotic Kinematic and Adaptive Navigation

## Descripción del Proyecto
SARKAN es un sistema de navegación autónoma y teleoperación para un rover, orientado a aplicaciones de exploración en entornos complejos. En este repositorio, se desarrollan y prueban algoritmos de percepción, navegación autónoma y control teleoperado usando **ROS2** y **Gazebo**. 

Este proyecto busca mejorar la adaptabilidad y precisión de la navegación autónoma, aprovechando herramientas avanzadas de simulación y control robótico.

---

## Clonación del Repositorio

Para comenzar, clona este repositorio en tu entorno local:

``
git clone https://github.com/MBG021/sarkan
``

## Preparación de Archivos
Incluir en la carpeta oculta `.gazebo` una carpeta llamada `sarkan`, dentro de la cual debes agregar la carpeta `meshes` y el archivo `model.config`.

## Instalación de Dependencias
Actualiza el sistema e instala las dependencias necesarias para ROS y Gazebo:

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-joint-state-publisher*
sudo apt install ros-humble-xacro
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-ackermann-*
sudo apt install ros-humble-rosgraph*
sudo apt install ros-humble-control*
sudo apt install ros-humble-ros2*
pip install modern_robotics
```
# Configuración del Entorno

Agrega la siguiente línea a tu archivo .bashrc para asegurarte de que el entorno ROS esté correctamente configurado:

source /usr/share/gazebo/setup.bash

source /usr/share/gazebo-11/setup.bash

echo $ROS_PACKAGE_PATH

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/ruta/a la carpeta/models/en tu work space/

# Compilación del Proyecto

Navega al directorio del proyecto y compila con colcon:

$ cd rosbot

$ colcon build

$ source install/setup.bash

# Visualización en RViz

Para visualizar el rover en RViz, ejecuta el siguiente comando:

$ ros2 launch rosbot rviz_robot.launch.py

# Simulación en Gazebo

Para simular el rover en Gazebo, usa el siguiente comando:

$ ros2 launch rosbot urdf_gz.launch.py

# Teleoperacion en gazebo

ventana 1: ros2 launch rosbot gz_controller.launch.py

ventana 2: ros2 run sarkan tepeop_twist_keyboard.py

ventana 3: ros2 run sarkan kinematic_client.py 0.2 0.18 0.3

Donde 0,2 es la x deseada, 0,18 la y deseada y 0,23 la z deseada.

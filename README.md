# SARKAN
### Smart Autonomous Robotic Kinematic and Adaptive Navigation

## Descripción del Proyecto
SARKAN es un sistema de navegación autónoma y teleoperación para un rover, orientado a aplicaciones de exploración en entornos complejos. En este repositorio, se desarrollan y prueban algoritmos de percepción, navegación autónoma y control teleoperado usando **ROS2** y **Gazebo**. 

Este proyecto busca mejorar la adaptabilidad y precisión de la navegación autónoma, aprovechando herramientas avanzadas de simulación y control robótico.

---

## Clonación del Repositorio

Para comenzar, clona este repositorio en tu entorno local:

```bash
git clone https://github.com/MBG021/sarkan
```

## Preparación de Archivos
Incluir en la carpeta oculta `.gazebo` una carpeta llamada `sarkan`, dentro de la cual debes agregar la carpeta `meshes` y el archivo `model.config`.

## Instalación de Dependencias
Actualiza el sistema e instala las dependencias necesarias para ROS y Gazebo:

```bash
sudo apt update
```
```bash
sudo apt upgrade
```
```bash
sudo apt install ros-humble-joint-state-publisher*
```
```bash
sudo apt install ros-humble-xacro
```
```bash
sudo apt install ros-humble-gazebo-*
```
```bash
sudo apt install ros-humble-ackermann-*
```
```bash
sudo apt install ros-humble-rosgraph*
```
```bash
sudo apt install ros-humble-control*
```
```bash
sudo apt install ros-humble-ros2*
```
```bash
pip install modern_robotics
```

## Configuración del Entorno

Agrega las siguientes líneas a tu archivo `.bashrc` para asegurarte de que el entorno de ROS y Gazebo esté correctamente configurado:

```bash
source /usr/share/gazebo/setup.bash
source /usr/share/gazebo-11/setup.bash
echo $ROS_PACKAGE_PATH
```
## Compilación del Proyecto

Navega al directorio del proyecto y compila con `colcon`:

```bash
cd sarkan
```
```bash
colcon build
```
```bash
source install/setup.bash
```
## Visualización en RViz

Para visualizar el rover en RViz, ejecuta el siguiente comando:

```bash
ros2 launch sarkan rviz_robot.launch.py
```
## Simulación en Gazebo

Para simular el rover en Gazebo, usa el siguiente comando:

```bash
ros2 launch sarkan urdf_gz.launch.py
```
## Teleoperación en Gazebo

Para teleoperar el rover en Gazebo, abre varias terminales y ejecuta los siguientes comandos:

1. **Ventana 1**: Inicia el controlador en Gazebo

   ```bash
   ros2 launch sarkan gz_controller.launch.py

2. **Ventana 2**: Controla el movimiento del rover con el teclado

   ```bash
   ros2 run sarkan tepeop_twist_keyboard.py

3. **ventana 3**: Control del Manipulador mediante Cinemática Inversa

   ```bash
   ros2 run sarkan kinematic_client.py 0.2 0.18 0.3
   ```
   Donde: 0,2 es la `X` deseada  |  0,18 es la `Y` deseada |  0,23 la `Z` deseada,

## Acknowledgments / Reconocimientos

Este proyecto utiliza parte del código de [JSOV2001](https://github.com/JSOV2001/phantomx_reactor.git). Agradezco a los autores por su contribución. Las codigo que se utilizaron se encuentran en las rutas: `sarkan/scripts/kinematics_client.py`, `sarkan/scripts/server.py`, `/sarkan/srv/KinematicGoal.srv`, `/sarkan/urdf/phantomx_reactor_core_wrist.xacro`.

Si estás interesado en explorar el repositorio completo, puedes encontrarlo en el siguiente enlace:

[https://github.com/JSOV2001/phantomx_reactor.git](https://github.com/JSOV2001/phantomx_reactor.git)


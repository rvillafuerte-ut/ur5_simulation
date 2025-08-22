# Simulación y Teleoperación de UR5 con Geomagic Touch

Este repositorio contiene el software necesario para teleoperar dos robots UR5/UR5e utilizando los dispositivos hápticos Geomagic Touch en un entorno ROS 2 Humble.

## Requisitos Previos

*   **Sistema Operativo:** Ubuntu 22.04
*   **Plataforma ROS:** ROS 2 Humble
*   **RAM:** Se recomienda un mínimo de 8 GB de RAM para la compilación.

## Guía de Instalación

Sigue estos pasos en orden para configurar tu entorno de desarrollo.

### 1. Instalación de ROS 2 Humble

Estos comandos instalarán la distribución de escritorio de ROS 2 Humble y las herramientas de desarrollo.

```bash
# Configurar la codificación de caracteres a UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen es_ES es_ES.UTF-8
sudo update-locale LC_ALL=es_ES.UTF-8 LANG=es_ES.UTF-8
export LANG=es_ES.UTF-8

# Habilitar los repositorios 'universe' y 'multiverse'
sudo apt install software-properties-common
sudo add-apt-repository universe

# Añadir la clave GPG y el repositorio de ROS 2
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Actualizar e instalar ROS 2 y herramientas de desarrollo
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop ros-dev-tools

# Añadir el script de configuración al .bashrc para cargarlo automáticamente
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
**Importante:** Cierra y vuelve a abrir tu terminal para que los cambios surtan efecto.

### 2. Instalación de Dependencias Adicionales

Instala todas las dependencias de paquetes de ROS y del sistema con un solo comando.

```bash
sudo apt update
sudo apt install \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-ros-gz \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-msgs \
  ros-humble-gazebo-plugins \
  ros-humble-ros-ign-bridge \
  ros-humble-teleop-twist-keyboard \
  ros-humble-ros2-control \
  ros-humble-controller-manager \
  ros-humble-ur \
  ros-humble-pinocchio \
  libeigen3-dev \
  libgoogle-glog-dev \
  libmodbus-dev
```

### 3. Instalación de Dependencias desde Código Fuente

`OSQP` y `Osqp-Eigen` se compilan desde el código fuente para asegurar la compatibilidad.

1.  **Instalar OSQP:**
    ```bash
    git clone --recursive https://github.com/osqp/osqp.git
    cd osqp
    git checkout v0.6.3
    git submodule update --init --recursive
    mkdir build && cd build
    cmake .. && make
    sudo make install
    cd ../..
    ```

2.  **Instalar Osqp-Eigen:**
    ```bash
    git clone https://github.com/robotology/osqp-eigen.git
    cd osqp-eigen
    git checkout v0.8.1
    mkdir build && cd build
    cmake .. && make
    sudo make install
    cd ../..
    ```
    **Nota:** Actualiza el caché del enlazador después de instalar librerías manualmente.
    ```bash
    sudo ldconfig
    ```

### 4. Compilación del Workspace

Una vez instaladas todas las dependencias, clona este repositorio y los paquetes adicionales en tu workspace de ROS 2 y compila.

```bash
# Crea y navega a tu workspace
mkdir -p ~/tesis_ws/src
cd ~/tesis_ws/src

# Clona los repositorios necesarios
git clone https://github.com/DavidValdezUtec/ur5_simulation.git
git clone https://github.com/stevens-armlab/Geomagic_Touch_ROS2.git
# Aquí iría el 'git clone' para el driver del UR si es necesario

# Regresa a la raíz del workspace y compila
cd ~/tesis_ws
colcon build --symlink-install
```
**Nota sobre la compilación:** Si encuentras un error de tipo `killed` o `Terminado`, significa que te has quedado sin memoria RAM. Intenta compilar de nuevo usando un solo núcleo:
`colcon build --parallel-workers 1`

### 5. Uso

Para ejecutar la simulación, asegúrate de haber "sourceado" tu workspace y luego utiliza el archivo de lanzamiento correspondiente.

```bash
# Carga la configuración de tu workspace (haz esto en cada nueva terminal)
source ~/tesis_ws/install/setup.bash

# Lanza el nodo de simulación (reemplaza con el nombre real de tu launch file)
ros2 launch ur5_simulation ur5_simulation.launch.py
```


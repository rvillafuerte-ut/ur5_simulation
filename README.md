# Simulación y Teleoperación de UR5 con Geomagic Touch

Este repositorio contiene el software necesario para teleoperar dos robots UR5/UR5e utilizando los dispositivos hápticos Geomagic Touch en un entorno ROS 2 Humble.

## Requisitos Previos

*   **Sistema Operativo:** Ubuntu 22.04
*   **Plataforma ROS:** ROS 2 Humble

## Guía de Instalación de Dependencias

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

# Añadir la clave GPG de ROS 2
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Añadir el repositorio de ROS 2 a la lista de fuentes
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Actualizar e instalar ROS 2
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools

# Añadir el script de configuración al .bashrc para cargarlo automáticamente
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
**Importante:** Cierra y vuelve a abrir tu terminal para que los cambios surtan efecto.

### 2. Instalación de Librerías Adicionales

#### Pinocchio
Librería de cinemática y dinámica para robótica.
```bash
sudo apt install ros-$ROS_DISTRO-pinocchio
```

#### libmodbus
Librería para la comunicación a través del protocolo Modbus.
```bash
sudo apt install libmodbus-dev
```

#### OSQP y Osqp-Eigen
Solucionador de optimización cuadrática y su interfaz para Eigen. Se compilan desde el código fuente para asegurar la compatibilidad.

1.  **Instalar OSQP:**
    ```bash
    git clone --recursive https://github.com/osqp/osqp.git
    cd osqp
    # Usamos la versión v0.6.3 por compatibilidad con osqp-eigen
    git checkout v0.6.3
    # ¡Paso clave! Actualizamos los submódulos para que coincidan con esta versión
    git submodule update --init --recursive
    mkdir build && cd build
    cmake ..
    make
    sudo make install
    cd ../..
    ```

2.  **Instalar Osqp-Eigen:**
    ```bash
    git clone https://github.com/robotology/osqp-eigen.git
    cd osqp-eigen
    # Usamos la versión v0.8.1, compatible con la versión de OSQP instalada
    git checkout v0.8.1
    mkdir build && cd build
    cmake ..
    make
    sudo make install
    cd ../..
    ```
    **Nota:** Es importante actualizar el caché del enlazador después de instalar librerías manualmente.
    ```bash
    sudo ldconfig
    ```

### 3. Otras Dependencias de Paquetes

Para la funcionalidad completa, necesitarás los siguientes paquetes de ROS 2:

*   **Driver de Universal Robots:**
    *   Instrucciones de instalación: [UR-Driver ROS2](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

*   **Paquete para Geomagic Touch:**
    *   Repositorio: `https://github.com/stevens-armlab/Geomagic_Touch_ROS2.git`

    *Nota del autor: "Moví omni_broadcaster por un error en mi primera instalación, pero actualmente podría funcionar bien".*


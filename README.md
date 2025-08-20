# ur5_simulation

Este repositorio permite teleoperar dos robots UR5/UR5e mediante los módulos hápticos Geomagic Touch


## Requirements
ROS2 Humble\n
Teleoperation of UR5/UR5e with geomagic touch and ROS2 Humble
Dependencies
Eigen
#------------ROS2 Humble ------------
sudo apt update && sudo apt install locales
sudo locale-gen es_ES es_ES.UTF-8
sudo update-locale LC_ALL=es_ES.UTF-8 LANG=es_ES.UTF-8
export LANG=es_ES.UTF-8

#Habilitar repositorios universe y multiverse
sudo apt install software-properties-common
sudo add-apt-repository universe

#Añadir la clave GPG de ROS 2
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

#Añadir el repositorio a la lista de fuentes
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade

sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

#------------Pinocchio--------------
sudo apt install ros-$ROS_DISTRO-pinocchio

#------------Osqp----------
git clone --recursive https://github.com/osqp/osqp
cd osqp
git checkout v0.6.3  # Versión compatible con osqp-eigen v0.8.1
mkdir build
cd build
cmake ..
make
sudo make install

#-------Osqp-Eigen----------
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
git checkout v0.8.1  # Versión estable compatible con OSQP v0.6.3
mkdir build
cd build
cmake ..
make

#-------libmodbus----------
sudo apt install libmodbus-dev

UR-Driver ROS2 Humble (Universal Robot)
geomagig package :https://github.com/stevens-armlab/Geomagic_Touch_ROS2.git {I move omni_bradcaster for an error in my fisrt installation, but actually may works well]


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
# para lanzar ejecutables:
from launch_ros.actions import Node

import os


def generate_launch_description():
    #usa la funcion IncludeLaunchDescription para incluir el archivo ur_sim_control.launch.py
    ur_sim_control_launch = IncludeLaunchDescription(
        #usa la funcion PythonLaunchDescriptionSource para especificar la ruta del archivo ur_sim_control.launch.py
        PythonLaunchDescriptionSource(
            os.path.join(
                #busca la ubicacion del paquete ur5_simulation y la junta con la
                # ruta relativa al archivo "launch" y a "ur_sim_control.launch.py"
                FindPackageShare('ur5_simulation').find('ur5_simulation'),
                'launch',
                'ur_sim_control.launch.py'
            )
        )
    )
    ur5_ik_node = Node(
        package='ur5_simulation',
        executable='ur5_ik_node',
        name='ur5_ik_node',
        output='screen'
    )
    ur5_impedance = Node(
        package='ur5_simulation',
        executable='ur5_impedance',
        name='ur5_impedance',
        output='screen'
    )

    return LaunchDescription([
        ur_sim_control_launch,
        ur5_impedance
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Declaraciones de Argumentos ---
    # Permite configurar desde la línea de comandos si se usa la simulación o no
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Encuentra la ruta al paquete para localizar los archivos de configuración
    pkg_share = get_package_share_directory('ur5_simulation')
    
    # --- Carga del Modelo del Robot (URDF/XACRO) ---
    # Es una buena práctica tener un launch file separado para el robot_description
    # Esto permite reutilizarlo en otros lanzamientos.
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'robot_description.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # --- Nodos Principales ---

    # 1. Nodo de Robot State Publisher
    # Publica las transformaciones estáticas del robot (TF2) a partir del URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': LaunchConfiguration('robot_description'),
                     'use_sim_time': use_sim_time}]
    )

    # 2. Nodo de Joint State Publisher (con GUI)
    # Publica el estado de las articulaciones. Útil para depuración y visualización.
    # En un sistema real, el driver del robot publicaría este topic.
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    # 3. Nodo de Cinemática Inversa (IK)
    # Tu ejecutable personalizado
    ur5_ik_node = Node(
        package='ur5_simulation',
        executable='ur5_ik_node',
        name='ur5_ik_node',
        output='screen',
        # Aquí podrías pasarle parámetros si los necesita
        # parameters=[{'param_name': 'param_value'}]
    )

    # 4. Nodo de Control de Impedancia
    # Tu ejecutable personalizado
    ur5_impedance_node = Node(
        package='ur5_simulation',
        executable='ur5_impedance',
        name='ur5_impedance_node',
        output='screen',
        # Aquí podrías pasarle parámetros
        # parameters=[{'param_name': 'param_value'}]
    )

    # 5. Nodo de RViz2 para Visualización
    # Carga la configuración de RViz desde un archivo .rviz
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'ur5_control.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- Creación de la Descripción del Lanzamiento ---
    # Se añaden todos los nodos y acciones a la lista
    nodes_to_launch = [
        # Argumentos
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        # Nodos
        # robot_description_launch, # Descomentar cuando crees robot_description.launch.py
        robot_state_publisher_node,
        joint_state_publisher_node,
        ur5_ik_node,
        ur5_impedance_node,
        rviz_node
    ]

    return LaunchDescription(nodes_to_launch)

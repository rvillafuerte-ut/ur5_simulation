# multi_ur_control.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Argumentos que podríamos querer cambiar desde la línea de comandos
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    use_fake_hardware_r1 = LaunchConfiguration("use_fake_hardware_r1")
    use_fake_hardware_r2 = LaunchConfiguration("use_fake_hardware_r2")
    r1_type = LaunchConfiguration("r1_type")
    r2_type = LaunchConfiguration("r2_type")
    r1_x_pos = LaunchConfiguration("r1_x_pos")
    r1_y_pos = LaunchConfiguration("r1_y_pos")
    r1_z_pos = LaunchConfiguration("r1_z_pos")
    r1_rot_x = LaunchConfiguration("r1_rot_x")
    r1_rot_y = LaunchConfiguration("r1_rot_y")
    r1_rot_z = LaunchConfiguration("r1_rot_z")
    r2_x_pos = LaunchConfiguration("r2_x_pos")
    r2_y_pos = LaunchConfiguration("r2_y_pos")
    r2_z_pos = LaunchConfiguration("r2_z_pos")
    r2_rot_x = LaunchConfiguration("r2_rot_x")
    r2_rot_y = LaunchConfiguration("r2_rot_y")
    r2_rot_z = LaunchConfiguration("r2_rot_z")

    # --- Configuración del UR5 ---
    ur5_launch = GroupAction(
        actions=[
            # Empuja todos los nodos de este grupo al namespace 'ur5'
            PushRosNamespace(r1_type),

            # Incluye el launch file de control original del driver
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"
                    ])
                ),
                launch_arguments={
                    "ur_type": r1_type,
                    "robot_ip": "192.168.10.113",
                    # Evitar f-strings con LaunchConfiguration: usar listas de sustituciones
                    "tf_prefix": [r1_type, "_"],
                    "controllers_file": ["ur_controllers_", r1_type, ".yaml"],
                    "kinematics_params_file": ["/home/david/my_robot_calibration_", r1_type, ".yaml"],
                    "use_fake_hardware": use_fake_hardware_r1,
                    "launch_dashboard_client": launch_dashboard_client,
                    "launch_rviz": "false", # Lanzaremos un RViz común si es necesario
                    # Asignación explícita de puertos para el hardware real
                    "reverse_port": "50001",
                    "script_sender_port": "50002",
                    "trajectory_port": "50003",
                    "script_command_port": "50004",
                    "pos_x": r1_x_pos,
                    "pos_y": r1_y_pos,
                    "pos_z": r1_z_pos,
                    "rot_x": r1_rot_x,
                    "rot_y": r1_rot_y,
                    "rot_z": r1_rot_z,
                }.items(),
            ),
        ]
    )

    # --- Configuración del robot 2: UR5e ---
    ur5e_launch = GroupAction(
        actions=[
            # Empuja todos los nodos de este grupo al namespace configurado para r2
            PushRosNamespace(r2_type),

            # Incluye el launch file de control original del driver
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"
                    ])
                ),
                launch_arguments={
                    "ur_type": r2_type,
                    "robot_ip": "192.168.10.103",
                    # Evitar f-strings con LaunchConfiguration: usar listas de sustituciones
                    "tf_prefix": [r2_type, "_"],
                    "controllers_file": ["ur_controllers_", r2_type, ".yaml"],
                    "kinematics_params_file": ["/home/david/my_robot_calibration_", r2_type, ".yaml"],
                    "use_fake_hardware": use_fake_hardware_r2,
                    "launch_dashboard_client": "false", # Solo lanzamos un dashboard client
                    "launch_rviz": "false",
                    # Asignación explícita de puertos DIFERENTES para el hardware real
                    "reverse_port": "50011",
                    "script_sender_port": "50012",
                    "trajectory_port": "50013",
                    "script_command_port": "50014",
                    "pos_x": r2_x_pos,
                    "pos_y": r2_y_pos,
                    "pos_z": r2_z_pos,
                    "rot_x": r2_rot_x,
                    "rot_y": r2_rot_y,
                    "rot_z": r2_rot_z,
                }.items(),
            ),
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "rviz", "view_robot.rviz"]
    )
    
    #lanzar  un solo rviz para ambos robots
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
    )
    
    
    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_dashboard_client", default_value="true", description="Launch Dashboard Client?"
        ),
        DeclareLaunchArgument(
            "r1_type", default_value="ur5", description="Robot 1 type"
        ),
        DeclareLaunchArgument(
            "r2_type", default_value="ur5e", description="Robot 2 type"
        ),
        DeclareLaunchArgument(
            "use_fake_hardware_r1",
            default_value="true",
            description="Use fake hardware for UR5.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware_r2",
            default_value="true",
            description="Use fake hardware for UR5e.",
        ),
        DeclareLaunchArgument(
            "r1_x_pos", default_value="0.0", description="Robot 1 X Position"
        ),
        DeclareLaunchArgument(
            "r1_y_pos", default_value="0.9", description="Robot 1 Y Position"
        ),
        DeclareLaunchArgument(
            "r1_z_pos", default_value="0.0", description="Robot 1 Z Position"
        ),
        DeclareLaunchArgument(
            "r1_rot_x", default_value="0.0", description="Robot 1 X Rotation"
        ),
        DeclareLaunchArgument(
            "r1_rot_y", default_value="0.0", description="Robot 1 Y Rotation"
        ),
        DeclareLaunchArgument(
            "r1_rot_z", default_value="0.0", description="Robot 1 Z Rotation"
        ),
        DeclareLaunchArgument(
            "r2_x_pos", default_value="0.0", description="Robot 2 X Position"
        ),
        DeclareLaunchArgument(
            "r2_y_pos", default_value="-0.9", description="Robot 2 Y Position"
        ),
        DeclareLaunchArgument(
            "r2_z_pos", default_value="0.0", description="Robot 2 Z Position"
        ),
        DeclareLaunchArgument(
            "r2_rot_x", default_value="0.0", description="Robot 2 X Rotation"
        ),
        DeclareLaunchArgument(
            "r2_rot_y", default_value="0.0", description="Robot 2 Y Rotation"
        ),
        DeclareLaunchArgument(
            "r2_rot_z", default_value="3.14", description="Robot 2 Z Rotation"
        ),  
        ur5_launch,
        ur5e_launch,
        rviz_node,
    ])
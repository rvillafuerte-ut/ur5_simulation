# launch para lanzar ros2 launch omni_common omni_state_iz.launch.py, ros2 run ur5_controller controller_backup --ros-args -p ur5_time:=0.5 -p control_topic:="/joint_trajectory_controller/joint_trajectory" -p geomagic:=true -p geomagic_topic:="/phantom2/phantom/pose" -p geomagic_button_topic:="/phantom2/phantom/button"
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

def generate_launch_description():
    #argumentos
    r1_simulation = LaunchConfiguration('r1_simulation')
    r2_simulation = LaunchConfiguration('r2_simulation')
    ur5_time = LaunchConfiguration('ur5_time', default='0.5')
    control_topic = LaunchConfiguration('control_topic', default='/joint_trajectory_controller/joint_trajectory')
    geomagic = LaunchConfiguration('geomagic', default='true')
    geomagic_topic = LaunchConfiguration('geomagic_topic', default='/phantom2/phantom/pose')
    geomagic_button_topic = LaunchConfiguration('geomagic_button_topic', default='/phantom2/phantom/button')
    robot1 = LaunchConfiguration('robot1')
    robot2 = LaunchConfiguration('robot2')
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
    
    
    laptop_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='laptop_camera',
        namespace='camera/laptop',  # Usando el argumento 'namespace'
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [1280, 720],
        }]
    )

    # Nodo para la cámara USB externa
    usb_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='usb_camera',
        namespace='camera/usb', # Namespace diferente
        parameters=[{
            'video_device': '/dev/video2',
            'image_size': [1280, 720], # Puede tener una resolución diferente
        }]
    )
    
    #Launch required components
    # 1.  Construir la ruta completa al archivo .launch.py
    omni_common_launch_path = os.path.join(
        get_package_share_directory('omni_common'),
        'launch',
        'dual_omni_state.launch.py'
    )
    
    robot_launch_path = os.path.join(
        get_package_share_directory('ur_robot_driver'),
        'launch',
        'dual_control.launch.py',
    )
    launch = []
    dual_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch_path),
        launch_arguments={
            'r1_type': robot1,
            'r2_type': robot2,
            'use_fake_hardware_r1': r1_simulation,
            'use_fake_hardware_r2': r2_simulation,
            'r1_x_pos': r1_x_pos,
            'r1_y_pos': r1_y_pos,
            'r1_z_pos': r1_z_pos,
            'r1_rot_x': r1_rot_x,
            'r1_rot_y': r1_rot_y,
            'r1_rot_z': r1_rot_z,
            'r2_x_pos': r2_x_pos,
            'r2_y_pos': r2_y_pos,
            'r2_z_pos': r2_z_pos,
            'r2_rot_x': r2_rot_x,
            'r2_rot_y': r2_rot_y,
            'r2_rot_z': r2_rot_z,
            }.items()
        )
    launch.append(dual_robot_launch)
    
    # 3. Crear la acción para incluir el otro launch file
    # if geomagic == 'true':
    #     omni_common_launch = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(omni_common_launch_path)
    #     )
    #     launch.append(omni_common_launch)
    # if simulation:
    #     print("Simulation is True")
    #     ur5_launch = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(ur5_simulation_launch_path),
    #         launch_arguments={'ur_type': 'ur5'}.items()
    #     )
    #     launch.append(ur5_launch)
    nodes = []
    
    nodes.append(
        Node(
            package='ur5_controller',
            executable='controller_backup',
            name='ur5_controller',
            output='screen',
            parameters=[
                {'control_topic': control_topic},
                {'geomagic': geomagic},
                {'geomagic_topic': geomagic_topic},
                {'geomagic_button_topic': geomagic_button_topic}
            ]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot1',
            default_value='ur5',
            description='Robot 1 type'
        ),
        DeclareLaunchArgument(
            'robot2',
            default_value='ur5e',
            description='Robot 2 type'
        ),
        DeclareLaunchArgument(
            'r1_simulation',
            default_value='true',
            description='Use simulation or real robot'
        ),
        DeclareLaunchArgument(
            'r2_simulation',
            default_value='true',
            description='Use simulation or real robot'
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
        )
        ] + launch + nodes + [laptop_camera_node, usb_camera_node]
    )
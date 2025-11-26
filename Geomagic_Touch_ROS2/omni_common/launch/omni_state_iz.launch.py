
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
 

def generate_launch_description():
    #Launch required components
    chmod_acm0 = ExecuteProcess(
        cmd=['sudo', 'chmod', '777', '/dev/ttyACM0'],
        shell=False
    )
    nodes = []

    nodes.append(
        Node(
            package="omni_common",
            executable="omni_state",
            output="screen",
            parameters=[                
                {"omni_name": "phantom2"},
                {"publish_rate": 1000},
                {"reference_frame": "/map"},
                {"units": "mm"},
                {"device_name": "phantom2"}
            ]
        )
    )


    return LaunchDescription(
        [chmod_acm0]+nodes 
        
    )
    
##SUBSYSTEMS=="usb", ATTRS{idVendor}=="2988", ATTRS{idProduct}=="0304", GROUP="users", MODE="0666"
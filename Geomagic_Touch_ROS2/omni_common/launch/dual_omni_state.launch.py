import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
 

def generate_launch_description():

    # --- Nodos (definidos al principio para claridad) ---

    # Nodo para el dispositivo DERECHO (phantom1)
    omni_der_node = Node(
        package="omni_common",
        executable="omni_state",
        output="screen",
        parameters=[{"omni_name": "phantom3"}, {"device_name": "phantom3"}]
    )

    # Nodo para el dispositivo IZQUIERDO (phantom2)
    omni_iz_node = Node(
        package="omni_common",
        executable="omni_state",
        output="screen",
        parameters=[{"omni_name": "phantom2"}, {"device_name": "phantom2"}]
    )

    # --- Secuencia de Comandos ---

    # Ruta al ejecutable del driver
    touch_driver_path = os.path.expanduser('~/Documentos/TouchDriver_2024_09_19/bin/Touch_HeadlessSetup')

    # 1. Pedir sudo y ocultar /dev/ttyACM0
    hide_acm0 = ExecuteProcess(
        cmd=['sudo', 'mv', '/dev/ttyACM0', '/dev/ttyACM0_bak'],
        shell=False,
        output='screen'
    )

    # 2. Calibrar el dispositivo izquierdo (se ejecuta cuando 'hide_acm0' termina)
    calibrate_iz = ExecuteProcess(
        cmd=[touch_driver_path],
        shell=False,
        output='screen'
    )

    # 3. Restaurar /dev/ttyACM0 (se ejecuta cuando 'omni_iz_node' se lanza)
    #    Usamos un truco: se lanza después de que el nodo IZ se inicie.
    restore_acm0 = ExecuteProcess(
        # Usamos bash -c para ejecutar múltiples comandos
        cmd=['bash', '-c', 'sudo mv /dev/ttyACM0_bak /dev/ttyACM0 && sudo chmod 666 /dev/ttyACM0'],
        shell=False,
        output='screen'
    )

    # 4. Calibrar el dispositivo derecho (se ejecuta cuando 'restore_acm0' termina)
    calibrate_der = ExecuteProcess(
        cmd=[touch_driver_path],
        shell=False,
        output='screen'
    )

    # --- Encadenamiento de Eventos ---

    return LaunchDescription([
        # Inicia la secuencia ocultando el primer dispositivo
        hide_acm0,

        # Cuando hide_acm0 termina, ejecuta la primera calibración
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=hide_acm0,
                on_exit=[calibrate_der]
            )
        ),

        # Cuando la primera calibración termina, lanza el primer nodo
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=calibrate_der,
                on_exit=[omni_der_node]
            )
        ),
        
        # Cuando el primer nodo se lanza, restaura el segundo dispositivo
        # NOTA: Esto se dispara al *terminar* el nodo. Para un flujo continuo,
        # lo lanzamos casi en paralelo con un pequeño retraso si fuera necesario,
        # pero para esta secuencia, lo encadenamos así.
        # Para que se ejecute DESPUÉS de que el nodo ARRANQUE, no hay un evento directo.
        # La mejor aproximación es encadenarlo a la acción anterior.
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=calibrate_der, # Se lanza después de la calibración, casi junto al nodo
                on_exit=[restore_acm0]
            )
        ),

        # Cuando el segundo dispositivo se restaura, ejecuta la segunda calibración
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=restore_acm0,
                on_exit=[calibrate_iz]
            )
        ),

        # Cuando la segunda calibración termina, lanza el segundo nodo
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=calibrate_iz,
                on_exit=[omni_iz_node]
            )
        )
    ])

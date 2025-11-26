import sys
import subprocess
import os
import signal
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QTextEdit,
    QGridLayout, QLabel, QCheckBox, QComboBox, QLineEdit, QHBoxLayout
)
from PySide6.QtCore import QProcess

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Panel de Control UR5")
        self.setGeometry(100, 100, 800, 600)

        # Proceso para el comando launch
        self.process = None

        # --- Layouts ---
        # Layout principal que contendrá todo
        main_layout = QVBoxLayout()

        # 1. Grid Layout para las opciones de configuración
        options_layout = QGridLayout()

        # Fila 0: Checkbox para simulación
        self.sim_checkbox = QCheckBox("Modo Simulación")
        self.sim_checkbox.setChecked(True)
        options_layout.addWidget(self.sim_checkbox, 0, 0, 1, 2) # Ocupa 2 columnas

        # Fila 1: Modo de operación
        self.mode_r1_label = QLabel("Modo de Operación Robot 1:")
        self.mode_r1_combo = QComboBox()
        self.mode_r1_combo.addItems(["Simulación", "Implementación"])
        options_layout.addWidget(self.mode_r1_label, 1, 0)
        options_layout.addWidget(self.mode_r1_combo, 1, 1)

        self.mode_r2_label = QLabel("Modo de Operación Robot 2:")
        self.mode_r2_combo = QComboBox()
        self.mode_r2_combo.addItems(["Simulación", "Implementación"])
        options_layout.addWidget(self.mode_r2_label, 2, 0)
        options_layout.addWidget(self.mode_r2_combo, 2, 1)

        self.r1_type_label = QLabel("Tipo de Robot 1:")
        self.r1_type_combo = QComboBox()
        self.r1_type_combo.addItems(["ur5", "ur5e",])
        options_layout.addWidget(self.r1_type_label, 1, 2)
        options_layout.addWidget(self.r1_type_combo, 1, 3)
        
        self.r2_type_label = QLabel("Tipo de Robot 2:")
        self.r2_type_combo = QComboBox()
        self.r2_type_combo.addItems(["ur5", "ur5e",])
        options_layout.addWidget(self.r2_type_label, 2, 2)
        options_layout.addWidget(self.r2_type_combo, 2, 3)

        # Fila 3: Tópico de la cámara (input + búsqueda + combo)
        self.cam_topic_label = QLabel("Tópico de Cámara:")
        options_layout.addWidget(self.cam_topic_label, 3, 0)

        cam_topic_row = QHBoxLayout()
        self.cam_topic_input = QLineEdit("/camera/image_raw")
        self.search_cam_button = QPushButton("Buscar tópicos")
        self.cam_topic_combo = QComboBox()
        self.cam_topic_combo.setEditable(False)
        self.cam_topic_combo.setPlaceholderText("Selecciona un tópico detectado")

        cam_topic_row.addWidget(self.cam_topic_input, 3)
        cam_topic_row.addWidget(self.search_cam_button, 1)
        cam_topic_row.addWidget(self.cam_topic_combo, 3)

        options_layout.addLayout(cam_topic_row, 3, 1, 1, 3)

        # Conexiones UI de cámara
        self.search_cam_button.clicked.connect(self.discover_camera_topics)
        self.cam_topic_combo.currentTextChanged.connect(lambda t: self.cam_topic_input.setText(t) if t else None)

        # Añadimos el grid de opciones al layout principal
        main_layout.addLayout(options_layout)

        # 2. Botones de control (en un layout horizontal para que estén juntos)
        control_layout = QHBoxLayout()
        self.launch_button = QPushButton("Lanzar Sistema")
        self.launch_button.clicked.connect(self.launch_ros)
        control_layout.addWidget(self.launch_button)

        self.stop_button = QPushButton("Detener")
        self.stop_button.clicked.connect(self.stop_ros)
        self.stop_button.setEnabled(False)
        control_layout.addWidget(self.stop_button)

        self.exit_button = QPushButton("Salir")
        self.exit_button.clicked.connect(self.close) # Llama al evento de cierre de la ventana
        control_layout.addWidget(self.exit_button)
        
        main_layout.addLayout(control_layout)

        # 3. Área de texto para la salida
        self.output_text = QTextEdit()
        self.output_text.setReadOnly(True)
        main_layout.addWidget(self.output_text)

        # Widget central
        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

    def discover_camera_topics(self):
        """Descubre tópicos ROS 2 de tipo sensor_msgs/msg/Image y CompressedImage y los muestra en el combo."""
        setup_cmd = "source /home/david/tesis_ws/install/setup.bash"
        types = [
            "sensor_msgs/msg/Image",
            "sensor_msgs/msg/CompressedImage",
        ]
        topics = set()
        errors = []
        for t in types:
            cmd = f"{setup_cmd} && ros2 topic find {t}"
            try:
                result = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    for line in result.stdout.strip().splitlines():
                        line = line.strip()
                        if line:
                            topics.add(line)
                else:
                    errors.append(result.stderr.strip())
            except Exception as e:
                errors.append(str(e))

        self.cam_topic_combo.blockSignals(True)
        self.cam_topic_combo.clear()
        if topics:
            for name in sorted(topics):
                self.cam_topic_combo.addItem(name)
            self.output_text.append(f"Tópicos de cámara detectados ({len(topics)}):\n" + "\n".join(sorted(topics)))
        else:
            self.cam_topic_combo.addItem("")
            self.output_text.append("No se detectaron tópicos de cámara. ¿Está corriendo el driver o simulación?")
            if errors:
                self.output_text.append("Detalles: \n" + "\n".join(e for e in errors if e))
        self.cam_topic_combo.blockSignals(False)

    def closeEvent(self, event):
        """
        Sobrescribe el evento de cierre para asegurar que el proceso ROS se detenga.
        """
        if self.process and self.process.state() == QProcess.ProcessState.Running:
            self.output_text.append("\nDeteniendo el proceso ROS antes de salir...")
            self.stop_ros() # Esta función ya espera a que el proceso termine.
        
        event.accept() # Aceptar el evento y cerrar la ventana.

    def launch_ros(self):
        self.output_text.clear()
        
        # --- Leer valores de la GUI ---
        is_simulation = self.sim_checkbox.isChecked()
        operation_mode = self.mode_r1_combo.currentText()
        camera_topic = self.cam_topic_input.text()

        self.output_text.append("--- Configuración de Lanzamiento ---")
        self.output_text.append(f"Simulación: {'Sí' if is_simulation else 'No'}")
        self.output_text.append(f"Modo: {operation_mode}")
        self.output_text.append(f"Tópico Cámara: {camera_topic}")
        self.output_text.append("-------------------------------------\n")
        self.output_text.append("Iniciando el sistema ROS 2...")

        # --- Construir el comando ---
        # Aquí es donde usarías los valores para pasar argumentos al launch file
        # Ejemplo: sim:={is_simulation} modo:={operation_mode}
        command = ["ros2", "launch", "ur5_bringup", "ur5_bringup.launch.py", 
                   f"r1_simulation:={'true' if self.mode_r1_combo.currentText() == 'Simulación' else 'false'}",
                   f"r2_simulation:={'true' if self.mode_r2_combo.currentText() == 'Simulación' else 'false'}",
                   f"r1_type:={self.r1_type_combo.currentText()}",
                   f"r2_type:={self.r2_type_combo.currentText()}"]

        # Usamos QProcess para tener mejor control en una app Qt
        self.process = QProcess()
        self.process.readyReadStandardOutput.connect(self.handle_stdout)
        self.process.readyReadStandardError.connect(self.handle_stderr)
        self.process.finished.connect(self.process_finished)

        self.process.start("bash", ["-c", f"source /home/david/tesis_ws/install/setup.bash && {' '.join(command)}"])

        self.launch_button.setEnabled(False)
        self.stop_button.setEnabled(True)

    def handle_stdout(self):
        data = self.process.readAllStandardOutput().data().decode()
        self.output_text.append(data)

    def handle_stderr(self):
        data = self.process.readAllStandardError().data().decode()
        self.output_text.append(f"[ERROR] {data}")

    def stop_ros(self):
        if self.process and self.process.state() == QProcess.ProcessState.Running:
            self.output_text.append("\nEnviando señal de interrupción (Ctrl+C) al proceso ROS...")
            
            # QProcess.terminate() envía SIGTERM, que no siempre es capturado por ros2 launch.
            # Enviar SIGINT (la señal de Ctrl+C) es la forma canónica de detener un launch file.
            # Esto le permite apagar los nodos de forma ordenada.
            pid = self.process.processId()
            if pid:
                # Enviamos la señal al grupo de procesos para asegurar que todos los hijos la reciban.
                os.killpg(os.getpgid(pid), signal.SIGINT)
            
            # Damos un tiempo para que termine de forma ordenada.
            if not self.process.waitForFinished(5000):
                self.output_text.append("El proceso no terminó de forma ordenada, forzando la detención.")
                self.process.kill() # Como último recurso, envía SIGKILL.
            else:
                self.output_text.append("El proceso se detuvo correctamente.")


    def process_finished(self):
        self.output_text.append("\nProceso terminado.")
        self.process = None
        self.launch_button.setEnabled(True)
        self.stop_button.setEnabled(False)

def main(args=None):
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()

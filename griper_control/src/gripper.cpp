#include "functions.hpp"

bool boton= false; // Variable global para el botón gris
bool apretado = false; // Variable para controlar el estado del botón


// gripper control

// Muestra bytes en hexadecimal
void printHex(const uint8_t* data, int len, const std::string& label) {
    std::cout << label;
    for (int i = 0; i < len; ++i)
        std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << (int)data[i] << " ";
    std::cout << std::dec << std::endl;
}

// Calcula CRC16-Modbus
uint16_t crc16_modbus(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t pos = 0; pos < length; pos++) {
        crc ^= data[pos];
        for (int i = 0; i < 8; i++) {
            if ((crc & 0x0001)) {
                crc >>= 1;
                crc ^= 0xA001;
            } else crc >>= 1;
        }
    }
    return crc;
}

// Enviar comando raw con CRC automático
bool sendRawWithCRC(modbus_t* ctx, uint8_t* request, int len_wo_crc) {
    uint16_t crc = crc16_modbus(request, len_wo_crc);
    request[len_wo_crc] = crc & 0xFF;
    request[len_wo_crc + 1] = (crc >> 8) & 0xFF;

    printHex(request, len_wo_crc + 2, "Request: ");
    uint8_t rsp[MODBUS_RTU_MAX_ADU_LENGTH];

    if (modbus_send_raw_request(ctx, request, len_wo_crc + 2) == -1 ||
        modbus_receive_confirmation(ctx, rsp) == -1) {
        std::cerr << "Error en comunicación Modbus\n";
        return false;
    }
    printHex(rsp, rsp[2] + 5, "Respuesta: ");
    return true;
}

// Activa el gripper si no está activado aún
bool activateGripper(modbus_t* ctx) {
    uint16_t status;
    if (modbus_read_registers(ctx, STATUS_REG, 1, &status) != 1)
        return false;

    if ((status & 0x3100) == 0x3100) return true;  // Ya activado

    // RESET
    uint8_t reset[] = { SLAVE_ID, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    sendRawWithCRC(ctx, reset, sizeof(reset) - 2);
    usleep(500000);

    // ACTIVAR
    uint8_t activate[] = { SLAVE_ID, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06,
                           0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    sendRawWithCRC(ctx, activate, sizeof(activate) - 2);

    // Esperar hasta activación
    for (int i = 0; i < 20; ++i) {
        usleep(200000);
        if (modbus_read_registers(ctx, STATUS_REG, 1, &status) == 1 &&
            (status & 0x3100) == 0x3100)
            return true;
    }
    return false;
}

// Controla el gripper (posición: 0-255, fuerza: 0-255)
void moveGripper(modbus_t* ctx, uint8_t position, uint8_t force) {
    if (!activateGripper(ctx)) {
        std::cerr << "No se pudo activar el gripper\n";
        return;
    }

    uint8_t cmd[] = {
        SLAVE_ID, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06,
        0x09, 0x00, 0x00,       // rACT=1, rGTO=0, rATR=0
        position,               // rPR
        0xFF,                   // rSP (velocidad máxima)
        force,                  // rFR
        0x00, 0x00              // CRC
    };
    sendRawWithCRC(ctx, cmd, sizeof(cmd) - 2);
}

Eigen::MatrixXd computeFullJacobianQuaternion(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    const pinocchio::FrameIndex& tool_frame_id,
    const Eigen::VectorXd& q,
    double delta = 1e-8)
{
    int nq = q.size();
    Eigen::MatrixXd J_full(7, nq); // 3 pos + 4 quat

    // Estado nominal
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacement(model, data, tool_frame_id);
    Eigen::Vector3d pos0 = data.oMf[tool_frame_id].translation();
    Eigen::Quaterniond quat0(data.oMf[tool_frame_id].rotation());

    for (int i = 0; i < nq; ++i) {
        Eigen::VectorXd q_perturbed = q;
        q_perturbed[i] += delta;

        pinocchio::forwardKinematics(model, data, q_perturbed);
        pinocchio::updateFramePlacement(model, data, tool_frame_id);
        Eigen::Vector3d pos1 = data.oMf[tool_frame_id].translation();
        Eigen::Quaterniond quat1(data.oMf[tool_frame_id].rotation());

        // Diferencia numérica para posición
        Eigen::Vector3d dpos = (pos1 - pos0) / delta;
        // Diferencia numérica para cuaternión (Eigen almacena como x, y, z, w)
        Eigen::Vector4d dquat = (quat1.coeffs() - quat0.coeffs()) / delta;

        // Guardar en la columna i
        J_full.block<3,1>(0, i) = dpos;
        J_full.block<4,1>(3, i) = dquat;
    }
    return J_full;
}




class PhantomButtonSubscriber : public rclcpp::Node
{
public:
    PhantomButtonSubscriber(): Node("phantom_button_subscriber")
    {
        sleep(2); // Espera 2 segundos para que el dispositivo se inicialice correctamente
        if (!ctx || modbus_set_slave(ctx, SLAVE_ID) == -1 || modbus_connect(ctx) == -1) {
            std::cerr << "Error al configurar Modbus\n";
        }

        subscription_ = this->create_subscription<omni_msgs::msg::OmniButtonEvent>(
            "/phantom/button",
            10,
            std::bind(&PhantomButtonSubscriber::button_callback, this, std::placeholders::_1)
        );
        timer_ = this->create_wall_timer( std::chrono::milliseconds(10), std::bind(&PhantomButtonSubscriber::control_loop, this));
        
    }

private:
    void button_callback(const omni_msgs::msg::OmniButtonEvent::SharedPtr msg)
    {
        if (msg->grey_button == 1) {
            RCLCPP_INFO(this->get_logger(), "Botón gris presionado");
            boton = !boton; 
            apretado = true; // Cambia el estado del botón
        }
        if (msg->white_button == 1) {
            RCLCPP_INFO(this->get_logger(), "Botón blanco presionado");
            apretado = true; // Cambia el estado del botón
        }
        
    }
    void control_loop()
    {
        // Aquí puedes agregar la lógica que deseas ejecutar periódicamente
        // Por ejemplo, podrías imprimir el estado del botón
        if (apretado){
            if (boton) {
            
            moveGripper(ctx, 0, 255); 
            cout << "cerrado con fuerza alta" << std::endl;
            } else {
                moveGripper(ctx, 250, 250); 
                cout << "abierto con fuerza media" << std::endl;
            }
            apretado = false; // Resetea el estado del botón
        }
    }

    rclcpp::Subscription<omni_msgs::msg::OmniButtonEvent>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    modbus_t* ctx = modbus_new_rtu("/dev/ttyUSB0", 115200, 'N', 8, 1);
};

int main(int argc, char * argv[])
{
    std::system("sudo chmod 666 /dev/ttyUSB0"); // En Linux, lista los archivos en el directorio actual

    modbus_t* ctx = modbus_new_rtu("/dev/ttyUSB0", 115200, 'N', 8, 1);
    if (!ctx || modbus_set_slave(ctx, SLAVE_ID) == -1 || modbus_connect(ctx) == -1) {
        std::cerr << "Error al configurar Modbus\n";
    }

    // Cambiar aquí la posición (0-255) y fuerza (0-255)
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PhantomButtonSubscriber>());
  
    rclcpp::shutdown();
    return 0;
}
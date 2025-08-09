#include "functions.hpp"

bool boton= false; // Variable global para el botón gris
bool apretado = false; // Variable para controlar el estado del botón




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
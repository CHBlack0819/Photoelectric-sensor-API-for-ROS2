#include "wheeltec_grayscale_c/gray_sensor_node_c.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);//初始化进程
    auto node = std::make_shared<GraySensorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;  
}

/*Declaration of members of GraySensorNode*/
GraySensorNode::GraySensorNode() : Node("gray_sensor_node_c")
{
    //动态数据清空
    memset(&receive_data_gray, 0, sizeof(receive_data_gray));
    memset(&send_data_gray, 0, sizeof(send_data_gray));

    //声明启动参数。函数的定义在Node类中
    this->declare_parameter<std::string>("usart_port_name", "/dev/ttyS1");
    this->declare_parameter<int>("serial_baud_rate", 115200);

    //获取启动参数
    this->get_parameter("usart_port_name", usart_port_name);
    this->get_parameter("serial_baud_rate", serial_baud_rate);

    //配置QoS参数
    rclcpp::QoS sensor_qos(10);
    sensor_qos.reliable().durability_volatile();

    //创建发布者
    gray_publisher = create_publisher<std_msgs::msg::UInt8>("gray_sensor_data_c", sensor_qos);

    //尝试打开串口
    try
    { 
        //Attempts to initialize and open the serial port //尝试初始化与开启串口
        Stm32_Serial.setPort(usart_port_name); 
        Stm32_Serial.setBaudrate(serial_baud_rate); 
        serial::Timeout _time = serial::Timeout::simpleTimeout(2000); 
        Stm32_Serial.setTimeout(_time);
        Stm32_Serial.open(); 
        Stm32_Serial.flushInput();
    }
    catch (serial::IOException& e)
    {
        RCLCPP_ERROR(this->get_logger(),"gray_sensor can not open serial port. Check the cable connection or your serial configuration. "); 
    } 
    if(Stm32_Serial.isOpen())
    {
        RCLCPP_INFO(this->get_logger(),"gray_sensor serial port opened");
    }

    //启动定时器，以便连续触发传感器的读取逻辑
    sensor_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&GraySensorNode::GetSensorData, this)
    );
}

GraySensorNode::~GraySensorNode()
{
    try{
        Stm32_Serial.close(); //Close the serial port //关闭串口  
        RCLCPP_INFO(this->get_logger(),"Shutting down"); //Prompt message //提示信息
    }catch( const std::exception& e ){
        RCLCPP_ERROR(this->get_logger(),"Unable to shutdown the serial!");
    }
    
}

void GraySensorNode::GetSensorData()
{
    uint8_t starter[] = {STARTER, ADDRESS};
    try{
        Stm32_Serial.write(starter, sizeof(starter));
        auto data = Stm32_Serial.read(3);
        memcpy(receive_data_gray.rx, data.data(), 3); 
    }catch(serial::IOException& e){
        RCLCPP_ERROR(this->get_logger(),"Unable to receive data through serial port");
    }

    if( receive_data_gray.rx[0] != FRAME_HEADER || receive_data_gray.rx[2] != FRAME_TAIL ){
        RCLCPP_ERROR(this->get_logger(),"Unable to parse the frame");
    }else{
        Publish_gray();
    }
    
}

void GraySensorNode::Publish_gray()
{
    auto msg = std::make_unique<std_msgs::msg::UInt8>();
    // 直接把 STM32 发上来的原始字节（rx[1]）发出去！
    // 极致高效，没有任何多余的 CPU 开销！
    msg->data = receive_data_gray.rx[1]; 
    gray_publisher->publish(std::move(msg));
}

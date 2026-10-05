#include "wheeltec_grayscale_c/line_follower_node_c.hpp"
// #define DEBUG
#define TURN_V 0.40
#define INERTIAL_COAST_MS 540
#define DWELL_MS 500 


int main( int argc, char** argv )
{
    rclcpp::init( argc, argv );
    auto node = std::make_shared<LineFollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

/*Declaration of members of LineFollowerNode*/
LineFollowerNode::LineFollowerNode(): Node("line_follower_node_c")
{
    memset(&parsed_sensor_data, 0, sizeof(uint8_t) * 7);
    memset(&sensor_history, 0, sizeof(uint8_t) * 3);

    this->declare_parameter<float>("kp", 0.5);
    this->declare_parameter<float>("ki", 0);
    this->declare_parameter<float>("kd", 0.12);
    this->declare_parameter<float>("amp", 1.2);    
    this->declare_parameter<float>("integral_limit", 10);
    this->declare_parameter<float>("az_output_limit", 2.0);
    this->declare_parameter<float>("base_vx", 0.17);
    //注意数组类成员的参数声明和传递方式，只能使用vector<T>类声明或传递数组
    // 1. 声明参数（保持你原来的写法）
    this->declare_parameter<std::vector<double>>(
        "sensor_weight_base", 
        std::vector<double>{0.65, 0.94, 0.50, 0, -0.50, -0.94, -0.65}
    );

    // 2. 直接获取并赋值（不再做 size 判断）
    sensor_weight_base = this->get_parameter("sensor_weight_base").as_double_array();
    bot_state_machine = on_course;
    right_T_cross_times = 0;
    left_T_cross_times = 0;
    both_side_T_cross_times = 0;
    history_index = 0;

    // 3. 加一行日志，看看拿到的到底有几个数
    //RCLCPP_INFO(this->get_logger(), "sensor_weight_base size: %zu", sensor_weight_base.size());

    //注意字符串里不能有空格，否则进程不会正确传参也不会报错（既没有编译错误也没有运行错误）。
    this->get_parameter("kp", kp);
    this->get_parameter("ki", ki);
    this->get_parameter("kd", kd);
    this->get_parameter("amp", amp);
    this->get_parameter("integral_limit", integral_limit);
    this->get_parameter("az_output_limit", az_output_limit);
    this->get_parameter("base_vx", base_vx);


    rclcpp::QoS follower_qos(10);
    follower_qos.reliable().durability_volatile();

    line_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", 
        follower_qos
    );

    gray_subscriber = this->create_subscription<std_msgs::msg::UInt8>(
        "gray_sensor_data_c", 
        follower_qos, // 传入和发布者一模一样的 QoS 配置
        std::bind(&LineFollowerNode::sensor_callback, this, std::placeholders::_1)
    );

    control_timer = this->create_wall_timer(
        std::chrono::milliseconds(20),  // 50Hz 的控制频率
        std::bind(&LineFollowerNode::ControlLoop, this) // 绑定到控制主循环
    );
}

LineFollowerNode::~LineFollowerNode(){
    //空的函数体，因为业务上没有什么额外的操作
}

//回调函数。只负责传递变量、翻转标志位等轻量级操作。
//它的作用和中断服务函数极其相似，区别是回调函数是操作系统级别，中断服务函数是硬件级别
//参数是订阅到的消息的引用
void LineFollowerNode::sensor_callback( const std::shared_ptr<const std_msgs::msg::UInt8>& msg )
{
    current_sensor_data = msg->data;
}

void LineFollowerNode::ParseSensorData()
{
    for( int i = 0; i < CHANNEL_COUNT; i++ ){
        parsed_sensor_data[i] = ( current_sensor_data >> i ) & 1;
    }
}

float LineFollowerNode::error_calculation()
{
    uint8_t channel_sum = 0;
    float error = 0.0f;

    for( int i = 0; i < CHANNEL_COUNT; i++ ){
        // 核心修复：用传感器的实际状态(0或1) 乘以 对应的权重
        if( parsed_sensor_data[i] ){
            error += parsed_sensor_data[i] * sensor_weight_base[i]; 
            channel_sum++;
        }
    }

    #ifdef DEBUG
    //RCLCPP_INFO(this->get_logger(), "error before: %.2f, channel_sum: %d, amp: %.2f", error, channel_sum, amp);
    #endif

    // 防止除以0，并且乘以放大系数 amp
    if (channel_sum > 0.001f) {
        error = (error / channel_sum) * amp;
    } else {
        error = 0.0f;
    }

    #ifdef DEBUG
    //RCLCPP_INFO(this->get_logger(), "error after: %.2f", error);
    #endif

    return error;
}

void LineFollowerNode::PID_calculation( float error )
{
    // 1. 比例项 (P)
    float p_term = kp * error;

    // 2. 积分项 (I) + 抗积分饱和
    integral += error;
    if( integral > integral_limit ) 
        integral = integral_limit;
    else if(integral < -integral_limit) 
        integral = -integral_limit;
        
    float i_term = ki * integral;

    // 3. 微分项 (D)
    float derivative = error - prev_error;
    float d_term = kd * derivative;
    prev_error = error; // 更新历史误差

    #ifdef DEBUG
    //RCLCPP_INFO(this->get_logger(), "PID: %.2f %.2f %.2f, error: %.2f", p_term, i_term, d_term, error);
    #endif

    float az_output = p_term + i_term + d_term;

    // 5. 输出限幅
    if (az_output > az_output_limit) az_output = az_output_limit;
    else if (az_output < -az_output_limit) az_output = -az_output_limit;

    // 6, 转向联动策略
    float vx_output = base_vx - abs( az_output * HALF_LENGTH ) ;
    // float vx_output = base_vx - 0.5 * ( az_output * HALF_LENGTH ) * ( az_output * HALF_LENGTH ) / base_vx;

    #ifdef DEBUG
    //RCLCPP_INFO(this->get_logger(), "vx: %.2f, az, %.2f", vx_output, az_output);
    #endif

    // 7, 装填与发送
    msg.linear.x = vx_output;
    msg.angular.z = az_output;
    line_publisher->publish(std::move(msg));
}

void LineFollowerNode::left_turn()
{
    msg.linear.x = 0;
    msg.angular.z = TURN_V;
    line_publisher->publish(std::move(msg));
}

void LineFollowerNode::right_turn()
{
    msg.linear.x = 0;
    msg.angular.z = -TURN_V;
    line_publisher->publish(std::move(msg));
}

void LineFollowerNode::brake()
{
    msg.linear.x = 0;
    msg.angular.z = 0;
    line_publisher->publish(std::move(msg));
}

void LineFollowerNode::ControlLoop()
{
    // 1. 解析传感器数据（如果 sensor_timer 还在，这步可以省略，直接在这里解析也行）
    ParseSensorData();

    uint8_t both_side_count = 0;
    uint8_t right_T_count = 0;
    uint8_t left_T_count = 0;
    uint8_t off_course_count = 0;

    uint8_t history_cumulation = 0;

    sensor_history[history_index] = current_sensor_data;
    history_index = (history_index + 1) % HISTORY_DEPTH; // 环形覆盖。现在的写入顺序是：0，1，2，0，1，2……

    for( int i = 0; i < HISTORY_DEPTH - 3; i++ ) history_cumulation = history_cumulation | sensor_history[i];

    // 极其纯粹的 for 循环，没有任何复杂的分支！
    for( int i = 0; i < HISTORY_DEPTH; i++ ) {
        uint8_t data = sensor_history[i];

        // 判断双侧 T 字路口（中间5路亮）
        if( (data & 62) == 62 || (history_cumulation & 127 ) == 127 ) {
            both_side_count++;
        }
        // 判断右侧 T 字路口（右侧3路亮，且中间路不亮）
        else if( (data & 112) == 112 
            || (data & 120) == 120 
            || (history_cumulation & 112) == 112 
            || (history_cumulation & 120) == 120) {
            right_T_count++;
        }
        // 判断左侧 T 字路口（左侧3路亮，且中间路不亮）
        else if( (data & 7) == 7 
            || (data & 15) == 15 
            || (history_cumulation & 7) == 7
            || (history_cumulation & 15) == 15) {
            left_T_count++;
        }
        // 判断脱线
        else if( data == 0 ) {
            off_course_count++;
        }
    }


    #ifdef DEBUG
    RCLCPP_INFO(this->get_logger(), "current state: %d", bot_state_machine);
    #endif

    switch( bot_state_machine ){
        case on_course:{
            // 只要 3 帧里有 2 帧及以上确认，就坚决切换！
            if( both_side_count >= 4 ){
                bot_state_machine = both_side_T_cross;
                state_enter_time_ = this->get_clock()->now();
                both_side_T_cross_times++; 
                break;
            }
            else if( right_T_count >= 3 ){
                bot_state_machine = right_T_cross;
                state_enter_time_ = this->get_clock()->now();
                right_T_cross_times++;             
                break;
            }
            else if( left_T_count >= 3 ){
                bot_state_machine = left_T_cross;
                state_enter_time_ = this->get_clock()->now();
                left_T_cross_times++;                 
                break;
            }
            else if( off_course_count >= 3 ){
                bot_state_machine = off_course;
                break;
            }
            else{
                float error = error_calculation();
                PID_calculation(error);
            }
            break;
        }
        case right_T_cross: {

            elapsed_ms = (this->now() - state_enter_time_).seconds() * 1000.0;

            if( elapsed_ms < INERTIAL_COAST_MS ) break;//让小车自然滑行INERTIAL_COAST_MS毫秒
            
            if( ((current_sensor_data & 64) == 64) ){//等到只有最外侧的传感器压线的时候才跳出去
                bot_state_machine = braking;
                state_enter_time_ = this->get_clock()->now(); 
                break;
            }else{
                right_turn();
            }

            break;
        }
        case left_T_cross: {

            elapsed_ms = (this->now() - state_enter_time_).seconds() * 1000.0;

            if( elapsed_ms < INERTIAL_COAST_MS ) break;//让小车自然滑行INERTIAL_COAST_MS毫秒
            
            if( ((current_sensor_data & 1) == 1) ){//同上
                bot_state_machine = braking;
                state_enter_time_ = this->get_clock()->now(); 
                break;
            }else{
                left_turn();
            }
            
            break;
        }
        case both_side_T_cross: {

            elapsed_ms = (this->now() - state_enter_time_).seconds() * 1000.0;

            if( both_side_T_cross_times == 1 ){
                bot_state_machine = on_course;
                break;
            }

            if( elapsed_ms < INERTIAL_COAST_MS ) break;//让小车自然滑行INERTIAL_COAST_MS毫秒
            // 判断小车到底碰到了多少个正面对着的丁字路口。
            // 按照比赛场地，小车应当第三次遇到正面对着的丁字路口时停车。
            
            if( both_side_T_cross_times > 1 && both_side_T_cross_times < 3 ){
                if( ((current_sensor_data & 64) == 64) || ((current_sensor_data & 1) == 1)) {
                    bot_state_machine = braking;
                    state_enter_time_ = this->get_clock()->now(); 
                    break;
                }else if( current_sensor_data < 8 ){
                    left_turn();
                }else if( current_sensor_data >= 8 ){
                    right_turn();
                }
            }else if( both_side_T_cross_times >= 3 ){
                bot_state_machine = braking;
                break;
            }
            break;
        }
        case off_course:{

            if( current_sensor_data != 0 ){
                bot_state_machine = on_course;
            }

            break;
        }
        case braking:{

            elapsed_ms = (this->now() - state_enter_time_).seconds() * 1000.0;

            if( both_side_T_cross_times < 3 ){
                if( elapsed_ms > DWELL_MS ){
                    bot_state_machine = on_course;
                    break;
                }else{
                    brake();
                }
            }else{// 如果第三次碰到正对的丁字路口就锁死在braking状态，并始终保持静止
                bot_state_machine = braking;// 制造死锁（latch），让小车强制停下
                brake();
            }
            break;
        }
    }
    
}
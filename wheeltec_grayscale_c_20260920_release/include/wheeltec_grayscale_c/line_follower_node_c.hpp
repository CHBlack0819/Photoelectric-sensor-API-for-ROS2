#ifndef _LINE_FOLLOWER_NODE_C_H
#define _LINE_FOLLOWER_NODE_C_H

#include <iostream>
#include <string.h>
#include <string>
#include <geometry_msgs/msg/twist.hpp>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/u_int8.hpp>
#include <cmath>

using namespace std;

#define CHANNEL_COUNT      7
#define HALF_LENGTH     0.17 //小车中部到车头的长度，单位：米
#define HISTORY_DEPTH      5

#pragma pack(push, 1) //强制以1为最小长度字节对齐，效果是所有变量在物理内存里都是挨着的

//小车状态机
typedef enum{
    on_course,
    off_course,
    right_T_cross,
    left_T_cross,
    both_side_T_cross,
    braking,
}BOT_STATE_MACHINE;

class LineFollowerNode: public rclcpp::Node{
public:
    LineFollowerNode();
    ~LineFollowerNode();
    void control();
private:
    rclcpp::TimerBase::SharedPtr control_timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr line_publisher;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr gray_subscriber;

    void sensor_callback( const std::shared_ptr<const std_msgs::msg::UInt8>& msg );
    void ParseSensorData();
    void PID_calculation( float error );
    void left_turn();
    void right_turn();
    void brake();
    float error_calculation();
    void ControlLoop();

    uint8_t current_sensor_data;
    uint8_t sensor_history[HISTORY_DEPTH];
    uint8_t history_index;
    uint8_t parsed_sensor_data[7];
    uint8_t right_T_cross_times;
    uint8_t left_T_cross_times;
    uint8_t both_side_T_cross_times;

    float kp;
    float ki;
    float kd;
    float amp;
    float base_vx;

    float integral;       // 积分累积量
    float prev_error;     // 上一次的误差
    float integral_limit; // 积分限幅（极其重要！）
    float az_output_limit;   // 输出限幅（防止小车过度打弯）
    
    vector<double> sensor_weight_base;
    geometry_msgs::msg::Twist msg;
    BOT_STATE_MACHINE bot_state_machine;

    uint32_t elapsed_ms;
    rclcpp::Time state_enter_time_; 
};


#endif
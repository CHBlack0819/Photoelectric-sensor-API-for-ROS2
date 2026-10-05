#ifndef _GRAY_SENSOR_NODE_C_H
#define _GRAY_SENSOR_NODE_C_H

#include <iostream>
#include <string.h>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h>
#include <std_msgs/msg/u_int8.hpp>
#include <cmath>

using namespace std;

#define STARTER         0x57
#define ADDRESS         0x01
#define FRAME_HEADER    0x75
#define FRAME_TAIL      0x02
#define RECEIVE_DATA_SIZE  3
#define SEND_DATA_SIZE     3
#define CHANNEL_COUNT      7

#pragma pack(push, 1) //强制以1为最小长度字节对齐，效果是所有变量在物理内存里都是挨着的

//从灰度传感器接收的数据
typedef struct {
    uint8_t rx[RECEIVE_DATA_SIZE];
    bool data[CHANNEL_COUNT];
}RECEIVE_DATA_GRAY;

//向gray_publisher发布的数据，供巡线节点订阅
typedef struct {
    uint8_t tx[SEND_DATA_SIZE];
    bool data[CHANNEL_COUNT];
}SEND_DATA_GRAY;

//传感器类
class GraySensorNode: public rclcpp::Node{
public:
    GraySensorNode();
    ~GraySensorNode();
    void control();
private:
    serial::Serial Stm32_Serial;//打开串口
    rclcpp::TimerBase::SharedPtr sensor_timer_;//打开定时器
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr gray_publisher;//话题发布者

    //打开串口的参数
    int serial_baud_rate;
    string usart_port_name;

    void Publish_gray();
    void GetSensorData();
    RECEIVE_DATA_GRAY receive_data_gray;
    SEND_DATA_GRAY send_data_gray;
};

#endif
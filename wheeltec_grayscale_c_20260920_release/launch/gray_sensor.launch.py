from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 原有的传感器启动参数
    uart_port_name_arg = DeclareLaunchArgument('usart_port_name', default_value='/dev/ttyS1')
    serial_baud_rate_arg = DeclareLaunchArgument('serial_baud_rate', default_value='115200')
    kp_arg = DeclareLaunchArgument('kp', default_value = '0.42')        
    ki_arg = DeclareLaunchArgument('ki', default_value = '0.00')
    kd_arg = DeclareLaunchArgument('kd', default_value = '0.14')
    amp_arg = DeclareLaunchArgument('amp', default_value = '1.1')
    base_vx_arg = DeclareLaunchArgument('base_vx', default_value = '0.16')
    integral_limit_arg = DeclareLaunchArgument('integral_limit', default_value = '10.0')
    az_output_limit_arg = DeclareLaunchArgument('az_output_limit', default_value = '1.5')
    #sensor_weight_base_arg = DeclareLaunchArgument('sensor_weight_base', default_value = '{0.70, 1.00, 0.64, 0, -0.64, -0.94, -0.65}')

    # 2. 原有的传感器节点
    gray_sensor_node_c = Node(
        package='wheeltec_grayscale_c',
        executable='gray_sensor_node_c',
        name='gray_sensor_node_c',
        output='screen',
        parameters=[{
            'usart_port_name': LaunchConfiguration('usart_port_name'),
            'serial_baud_rate': LaunchConfiguration('serial_baud_rate'),
        }],
    )

    # 3. 【新增】巡线控制节点
    line_follower_node_c = Node(
        package = 'wheeltec_grayscale_c',
        executable = 'line_follower_node_c',
        name = 'line_follower_node_c',
        output = 'screen',
        parameters = [{
            'kp': LaunchConfiguration('kp'),
            'ki': LaunchConfiguration('ki'),
            'kd': LaunchConfiguration('kd'),
            'amp': LaunchConfiguration('amp'),
            'base_vx': LaunchConfiguration('base_vx'),
            'integral_limit': LaunchConfiguration('integral_limit'),
            'az_output_limit': LaunchConfiguration('az_output_limit'),
            #'sensor_weight_base': LaunchConfiguration('sensor_weight_base'),
        }],
    )

    # 4. 把新节点也放进返回列表里
    return LaunchDescription([
        uart_port_name_arg,
        serial_baud_rate_arg,
        kp_arg,
        ki_arg,
        kd_arg,
        amp_arg,
        base_vx_arg,
        integral_limit_arg,
        az_output_limit_arg,
        #sensor_weight_base_arg,
        gray_sensor_node_c,
        line_follower_node_c,  #  别忘了加在这里！
    ])
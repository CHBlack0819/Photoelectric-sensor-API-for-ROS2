# gray_sensor_node.py
# Last edited by Chrono Black at 20260804
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
# '.' refers to relative path, meaning to serch for the class, function, etc. at this file
from .gray_sensor import gray_sensor  # 假设你的传感器类写在同目录的 gray_sensor.py 中

class GraySensorNode(Node):
    def __init__(self):
        super().__init__('gray_sensor_node')
        
        # 1. 声明参数（方便以后在 launch 文件里修改，不用改代码）
        __sensor_qos = QoSProfile(
                                    reliability=ReliabilityPolicy.BEST_EFFORT,
                                    history=HistoryPolicy.KEEP_LAST,
                                    depth=10
                                )
        self.declare_parameter('uart_dev', '/dev/ttyS1')
        self.declare_parameter('baudrate', 115200)
        
        uart_dev = self.get_parameter('uart_dev').value
        baudrate = self.get_parameter('baudrate').value
        
        # 2. 初始化传感器类
        try:
            self.sensor = gray_sensor(uart_dev, baudrate)
            self.get_logger().info(f"[I] successfully initialized the sensor.")
        except Exception as e:
            self.get_logger().error(f"[E] Initialization failed: {e}")
            raise
        
        # 3. 创建 ROS2 发布者
        self.publisher_ = self.create_publisher(Int32MultiArray, '/gray_sensor/data', __sensor_qos)
        
        # 4. 创建定时器（每 50ms 读取一次，即 20Hz 频率）
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        self.get_logger().info("[I] grayscale sensor launched.")

    def timer_callback(self):
        # 调用传感器类的接收方法
        data_list = self.sensor.serial_recv()
        
        # 如果读到了有效数据，就发布出去
        if data_list:
            msg = Int32MultiArray()
            msg.data = data_list
            self.publisher_.publish(msg)
            # 调试用：在终端打印当前数据
            # self.get_logger().info(f"发布数据: {data_list}")
        else:
            self.publisher_.info(f"[W] Invalid data type received and may result in unexpected errors.")

    def destroy_node(self):
        # 节点销毁时，安全关闭串口
        print(f"[I] Shutting down the serial...")
        self.sensor.serial_close()
        super().destroy_node()

def main(args=None):
    # 1, initialize the rclpy kit and the node object
    rclpy.init(args=args)
    node = GraySensorNode()
    # 2, the body of the main function
    try:# run the member function of the node endlessly unless keyboard interruption.
        rclpy.spin(node)
    except KeyboardInterrupt:# actions when KeyboardInterrupt was thrown
        node.get_logger().info("[W] User pressed CTRL+C, quitting the process...")
    finally:# write functions to destroy the node as well as the hardware object.
        node.destroy_node()
        #rclpy.shutdown()

if __name__ == '__main__':
    main()
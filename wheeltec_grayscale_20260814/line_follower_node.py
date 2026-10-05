import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# 👇 1. 必须导入这个模块，用于参数回调的返回值
from rcl_interfaces.msg import SetParametersResult 

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        
        # 1. 初始化成员变量
        self.current_gray_data = [0, 0, 0, 0, 0, 0, 0]
        # note: if you want to debug the gyroPID, you have to make sure that this two variables are the same.
        self.current_vx = 0.2
        self.target_vx = 0.2
        # note: 
        self.base_weight_sensor = [0.9, 1.5, 1.3, 0.0, -1.3, -1.5, -0.9]
        
        # 👇 2. 【核心修复】PID 状态变量必须加 self.，否则无法跨函数使用
        self.integral = 0.0
        self.error_last = 0.0
        
        # 2. 声明参数
        self.declare_parameter('kp', 0.55)
        self.declare_parameter('ki', 0.00)
        self.declare_parameter('kd', 0.12)
        self.declare_parameter('Amp', 1.3)
        
        # 【核心修复】注册参数回调（注意函数名不要和系统底层冲突，改叫 param_callback）
        self.add_on_set_parameters_callback(self.param_callback)

        # 3. 获取初始参数值
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.Amp = self.get_parameter('Amp').value
        
        # 4. 创建订阅者
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.gray_sub = self.create_subscription(
            Int32MultiArray, '/gray_sensor/data', self.gray_callback, sensor_qos
        )
        
        # 5. 创建发布者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 6. 创建定时器
        self.create_timer(0.02, self.control_loop)
        self.get_logger().info('[I] Line following controller has been launched.')

    # 👇 4. 【新增】参数回调函数（必须返回 SetParametersResult）
    def param_callback(self, parameters):
        for param in parameters:
            if param.name == 'kp':
                self.kp = param.value
                self.get_logger().info(f'[I] now Kp: {self.kp}')
            elif param.name == 'ki':
                self.ki = param.value
                self.get_logger().info(f'[I] now Ki: {self.ki}')
            elif param.name == 'kd':
                self.kd = param.value
                self.get_logger().info(f'[I] now Kd: {self.kd}')
            elif param.name == 'Amp':
                self.Amp = param.value
                self.get_logger().info(f'[I] now Amp: {self.Amp}')
        return SetParametersResult(successful=True)

    def gray_callback(self, msg):
        self.current_gray_data = msg.data

    def calculate_error(self):
        weights = self.base_weight_sensor
        total_weight = 0
        line_count = 0
        for i in range(7):
            if self.current_gray_data[i] == 1:
                total_weight += weights[i]
                line_count += 1
        if line_count == 0:
            return 0.0
        return total_weight * self.Amp / line_count

    def control_loop(self):
        # 1. 计算基础角速度（省略前面的 PID 计算过程）
        error = self.calculate_error()
        self.integral += error
        derivative = error - self.error_last
        self.integral = max(min(self.integral, 10.0), -10.0)
        
        base_angular_z = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # 2. 运动学速度补偿
        velocity_gain = max(self.current_vx, 0.01) / 0.1 
        final_angular_z = base_angular_z * velocity_gain
        
        # 3. 角速度限幅（防过冲）
        max_angular_z = 0.233
        final_angular_z = max(min(final_angular_z, max_angular_z), -max_angular_z)
        
        # 👇 4. 【核心】转向联动减速策略
        # 计算当前转向的绝对值比例（0.0 到 1.0）
        # 如果 final_angular_z 是 0.233，比例就是 1.0（满舵）
        # 如果是 0.1，比例大约是 0.43
        steer_ratio = abs(final_angular_z) / max_angular_z
        
        # 设定减速参数
        base_speed = 0.2       # 直道全速
        min_turn_speed = 0.15   # 极限打方向时的最低速度
        
        # 线性插值计算当前目标线速度：
        # 转向越大，线速度越接近 min_turn_speed
        # 转向越小，线速度越接近 base_speed
        target_vx = base_speed - (base_speed - min_turn_speed) * steer_ratio
        
        # 5. 打包并发布 Twist 消息
        twist = Twist()
        twist.linear.x = target_vx      # 动态减速后的线速度
        twist.angular.z = final_angular_z
        self.cmd_vel_pub.publish(twist)
        
        self.error_last = error

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户按下 Ctrl+C，正在优雅退出...')
        
        # 👇 【终极修复】用 try-except 包裹发布动作
        try:
            stop_twist = Twist()
            node.cmd_vel_pub.publish(stop_twist)
            node.get_logger().info('已发送零速指令，小车安全刹停。')
        except Exception as e:
            # 如果底层已经销毁了上下文，发布会失败。
            # 没关系，直接忽略它，让程序继续往下走。
            pass 
        
    finally:
        # 纯粹的资源清理
        node.destroy_node()
        #rclpy.shutdown()

if __name__ == '__main__':
    main()

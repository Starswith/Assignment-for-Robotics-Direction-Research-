import rclpy
import math

from rclpy.node import Node
from driver_msgs.msg import Target  # 导入自定义消息类型
from collections import deque
from std_msgs.msg import String
from geometry_msgs.msg import Vector3


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = Vector3()
        self.integral = Vector3()
    
    def compute(self, target, current):
        # 计算误差
        error = Vector3()
        error.x = target.x - current.x
        error.y = target.y - current.y
        error.z = target.z - current.z

        # 积分项
        self.integral.x += error.x
        self.integral.y += error.y
        self.integral.z += error.z

        # 微分项
        derivative = Vector3()
        derivative.x = error.x - self.prev_error.x
        derivative.y = error.y - self.prev_error.y
        derivative.z = error.z - self.prev_error.z

        # PID 输出
        output = Vector3()
        output.x = self.kp * error.x + self.ki * self.integral.x + self.kd * derivative.x
        output.y = self.kp * error.y + self.ki * self.integral.y + self.kd * derivative.y
        output.z = self.kp * error.z + self.ki * self.integral.z + self.kd * derivative.z

        # 保存当前误差作为下一次的前一误差
        self.prev_error = error

        return output

class ProcessNode(Node):

    def __init__(self):
        super().__init__('process_node')
        
        # 订阅 'target' 话题
        self.subscription = self.create_subscription(
            Target,
            'target',
            self.listener_callback,
            10)
        
        # 发布到 'command' 话题
        self.publisher = self.create_publisher(Target, 'command', 10)
        
        # 定时器以 100Hz 运行
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        # 使用 deque 保存最近接收到的 5 条消息
        self.messages = deque(maxlen=5)
        self.command_count = 0  # 发送命令计数器

        # 初始化 PID 控制器
        self.pid_controller = PIDController(kp=1.0, ki=0.1, kd=0.01)

        # 初始过程变量，代表系统的当前状态
        self.current_value = Vector3()

    def listener_callback(self, msg):
        # 收到消息时，将其添加到 deque
        self.messages.append(msg)
        self.get_logger().info(f'Received target message: count={msg.count}, time={msg.time:.2f}')

    def timer_callback(self):
        if self.messages:
            # 获取最近的一条目标消息
            last_message = self.messages[-1]
            target_value = last_message.target

            # 使用 PID 控制器计算新的控制输出
            control_output = self.pid_controller.compute(target_value, self.current_value)

            # 更新当前值（简单模拟系统响应，实际系统可能更加复杂）
            self.current_value.x += control_output.x
            self.current_value.y += control_output.y
            self.current_value.z += control_output.z

            # 创建并发布控制命令消息
            command_msg = Target()
            command_msg.name = 'command'
            command_msg.count = last_message.count
            command_msg.time = last_message.time
            command_msg.target = self.current_value  # 发送当前值作为控制输出

            self.publisher.publish(command_msg)
            self.command_count += 1
            self.get_logger().info(f'Sent command message: count={command_msg.count}, time={command_msg.time:.2f}, control_output=({control_output.x:.2f}, {control_output.y:.2f}, {control_output.z:.2f})')
        else:
            # 如果尚未接收到消息，则记录警告
            self.get_logger().warn('No messages received yet.')

def main(args=None):
    rclpy.init(args=args)
    node = ProcessNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
from chapt4_interfaces.srv import Patrol

class turtleservice(Node):
    def __init__(self, nodename):
        super().__init__(nodename)
        self.get_logger().info('success create node')
        
        # 创建服务、订阅者和发布者
        self.turtle_service = self.create_service(
            Patrol, 'patrol', self.service_callback
        )
        self.turtle_subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.turtle_callback, 10
        )
        self.turtle_publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10
        )
        
        # 初始化目标位置和控制参数
        self.target_x = 1.0
        self.target_y = 1.0
        self.k = 0.8
        self.max_speed = 2.0
    
    def service_callback(self, request, response):
        """处理目标位置设置请求"""
        if 0 < request.target_x < 12 and 0 < request.target_y < 12:
            self.target_x = request.target_x
            self.target_y = request.target_y
            response.result = Patrol.Response.SUCCESS
        else:
            response.result = Patrol.Response.FAIL
        return response
    
    def turtle_callback(self, pose):
        """根据当前位置计算并发布控制指令"""
        x = pose.x
        y = pose.y
        theta = pose.theta
        
        self.get_logger().info(f'当前姿态 x={x}, y={y}, theta={theta}')
        
        # 计算距离和角度差
        distance = math.sqrt((x - self.target_x)**2 + (y - self.target_y)**2)
        angle_diff = math.atan2(self.target_y - y, self.target_x - x) - theta
        
        msg = Twist()
        
        if distance > 0.1:  # 距离阈值
            if abs(angle_diff) > 0.1:  # 角度阈值
                msg.angular.z = angle_diff
            else:
                msg.linear.x = self.k * distance
                msg.linear.x = min(self.max_speed, msg.linear.x)
            
            self.turtle_publisher.publish(msg)
            self.get_logger().info(f'线速度: {msg.linear.x}, 角速度: {msg.angular.z}')

def main():
    rclpy.init()
    node = turtleservice("turtle_service")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
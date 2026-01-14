import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import Patrol
import random

class turtleclient(Node):
    def __init__(self,nodename):
        super().__init__(nodename)
        self.turtle_client=self.create_client(Patrol,'patrol')
        self.create_timer(10,self.callback)
    def callback(self):
        def request_callback(result_future):
            response=result_future.result()
            self.get_logger().info(f"result={response.result}")
        while self.turtle_client.wait_for_service(1) is False:
            self.get_logger().info(f"等待服务器响应")
        
        reuqest=Patrol.Request()
        reuqest.target_x=random.uniform(1,15)
        reuqest.target_y=random.uniform(1,15)
        self.get_logger().info(f"target_x={reuqest.target_x},target_y={reuqest.target_y}")
        future=self.turtle_client.call_async(reuqest)
        future.add_done_callback(request_callback)

def main():
    rclpy.init()
    node=turtleclient('turtle_client')
    rclpy.spin(node)
    rclpy.shutdown()
            
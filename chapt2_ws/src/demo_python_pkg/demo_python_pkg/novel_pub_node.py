import rclpy
import requests
from rclpy.node import Node
from example_interfaces.msg import String
from queue import Queue
# colcon build        source install/setup.bash    python3 -m http.server   ros2 run demo_python_pkg novel_pub_node

class NovelPubNode(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.get_logger().info(f"{node_name}节点，启动")
        self.novels_queue_=Queue()      #创建队列
        self.novel_publisher_=self.create_publisher(String,'novel',10)#创建发布者，用于发表话题信息
        self.create_timer(5,self.timer_callback)
    
    def timer_callback(self):
        if self.novels_queue_.qsize()>0:
            line=self.novels_queue_.get()
            msg=String() #组装
            msg.data=line
            self.novel_publisher_.publish(msg)
            self.get_logger().info(f'发布了：{msg}')

    def download(self,url):
        response=requests.get(url)
        response.encoding='utf-8'
        text=response.text
        for line in text.splitlines():
            self.novels_queue_.put(line)

def main():
    rclpy.init()
    node=NovelPubNode('novel_pub')
    node.download("http://127.0.0.1:8000/novel1.txt")
    rclpy.spin(node)
    rclpy.shutdown()
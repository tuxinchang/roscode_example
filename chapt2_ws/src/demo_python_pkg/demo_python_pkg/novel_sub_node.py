import espeakng
import rclpy
import requests
from rclpy.node import Node
from example_interfaces.msg import String
from queue import Queue
import threading
import time
# colcon build        source install/setup.bash    ros2 run demo_python_pkg novel_sub_node
class NovelSubNode(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.get_logger().info(f"{node_name}节点，启动")
        self.novels_queue_= Queue()
        self.create_subscription(String,'novel',self.novel_callback,10)
        self.speech_thread_=threading.Thread(target=self.speake_thread)
        self.speech_thread_.start()

    def novel_callback(self,msg): #回调函数把话题发布的文本信息放入队列
        self.novels_queue_.put(msg)

    def speake_thread(self):
        speaker=espeakng.Speaker()
        speaker.voice='zh'

        while rclpy.ok(): #检查当前ROS上下文是否ok
            if self.novels_queue_.qsize()>0:
                text=self.novels_queue_.get()
                self.get_logger().info(f'朗读：{text}')
                speaker.say(text)  #进行朗读
                speaker.wait() #等待朗读结束
            else:
                #让当前进程休眠
                time.sleep(1)

def main():
    rclpy.init()
    node=NovelSubNode('novel_sub')
    rclpy.spin(node)
    rclpy.shutdown() 
import rclpy
from rclpy.node import Node
class PersonNode(Node):
    def __init__(self,node_name,name:str,age:int) -> None:
        super().__init__(node_name)
        self.name=name
        self.age=age

    def eat(self,food:str):
        print(f"{self.name}{self.age}正在吃{food}")
        self.get_logger().info(f"get_logger:{self.name}{self.age}正在吃{food}")

def main():
    rclpy.init()
    node=PersonNode('lisi_node','里斯',25)
    node.eat('鱼香肉丝')
    rclpy.spin(node)
    rclpy.shutdown()
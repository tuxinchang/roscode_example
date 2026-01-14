#ros2 pkg create --build-type ament_python --license Apache-2.0 demo_python_pkg
#chapt2_ws的src存放功能包


"""
rqt rviz2 bag
"""
import rclpy
from rclpy.node import Node
def main():
    rclpy.init()#初始化工作，分配资源
    node=Node('python_node')
    node.get_logger().info('你好，pyhton节点')
    node.get_logger().warn('警告，pytho节点正在运行')
    rclpy.spin(node)
    rclpy.shutdown()
if __name__=="__main__":
    main()  
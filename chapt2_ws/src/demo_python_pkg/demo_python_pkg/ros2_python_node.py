#1.在根目录创建功能包：ros2 pkg create --build-type ament_python --license Apache-2.0 demo_python_pkg
#2.在功能包中同名的文件夹中加入节点的代码
#3.在setup.py文件中添加对应节点函数路径
#4.在package.xml添加对应节点代码需要依赖项声明，如rclpy包
#5.在功能包所在的文件夹上在终端执行colcon build命令，构建功能包，产生了build，install，log文件夹,它们与功能包同级
#install.lib.同名功能包名.节点代码名才算要真正要执行的文件，因此。修改代码一定要执行colcon build命令
"""
6.在终端的~/chapt2/install/demo_python_pkg/lib/demo_python_pkg目录下执行命令： export PYTHONPATH=/ho
me/tuxinchang/chapt2/install/demo_python_pkg/lib/python3.10/site-packages:$PYTHONPATH
配置配置好路径即可执行./python_node运行该节点。（缺点：换一个终端执行一次，就要修改一次环境变量）
"""
#7.为了防止执行一个节点就修改环境变量的缺点，直接执行install目录下setup.bash文件，一劳永逸。命令：source install/setup.bash

import rclpy
from rclpy.node import Node
def main():
    rclpy.init()#初始化工作，分配资源
    node=Node('python_node')
    node.get_logger().info('你好，pyhton节点')
    node.get_logger().warn('警告，pytho节点正在运行')
    rclpy.spin(node)
    rclpy.shutdown()

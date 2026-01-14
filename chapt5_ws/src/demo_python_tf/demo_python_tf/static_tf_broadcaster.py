import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster #静态坐标发布器
from geometry_msgs.msg import TransformStamped #消息接口
from tf_transformations import quaternion_from_euler  #欧拉角转四元函数
import math #角度转弧度函数

class  StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        self.static_broadcaster_ =StaticTransformBroadcaster(self)
        self.publish_static_tf()
    
    def publish_static_tf(self):
        """
        发布静态TF 从 camera_link 到 base_link 之间的坐标关系
        """
        transformer=TransformStamped()
        transformer.header.frame_id='base_link'
        transformer.child_frame_id='camera_link'
        transformer.header.stamp=self.get_clock().now().to_msg()

        transformer.transform.translation.x=0.5
        transformer.transform.translation.y=0.3
        transformer.transform.translation.z=0.6
        
         #math.radians(180)把角度转化为弧度

        q=quaternion_from_euler(math.radians(180),0,0) #q就是q就是四元组
        
        #旋转部分进行赋值
        transformer.transform.rotation.x=q[0]
        transformer.transform.rotation.y=q[1]
        transformer.transform.rotation.z=q[2]
        transformer.transform.rotation.w=q[3]

        #静态坐标关系发布出去
        self.static_broadcaster_.sendTransform(transformer)
        self.get_logger().info(f'发布静态TF：{transformer}')

def main():
    rclpy.init()
    node=StaticTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()




        





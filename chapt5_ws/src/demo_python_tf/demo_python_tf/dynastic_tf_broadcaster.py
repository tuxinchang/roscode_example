import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster #动态坐标发布器
from geometry_msgs.msg import TransformStamped #消息接口
from tf_transformations import quaternion_from_euler  #欧拉角转四元函数
import math #角度转弧度函数

class  TFBroadcaster(Node):
    def __init__(self):
        super().__init__('dynastic_tf_broadcaster')
        self.broadcaster_ =TransformBroadcaster(self)
        #self.publish_static_tf()
        self.timer_=self.create_timer(0.01,self.publish_tf)  #动态坐标要不停地发布
    
    def publish_tf(self):
        """
        发布动态TF 从 camera_link 到 base_link 之间的坐标关系
        """
        transformer=TransformStamped()
        transformer.header.frame_id='camera_link'
        transformer.child_frame_id='bottle_link'
        transformer.header.stamp=self.get_clock().now().to_msg()

        transformer.transform.translation.x=0.2
        transformer.transform.translation.y=0.3
        transformer.transform.translation.z=0.5
        
         #math.radians(180)把角度转化为弧度

        q=quaternion_from_euler(math.radians(180),0,0) #q就是q就是四元组
        
        #旋转部分进行赋值
        transformer.transform.rotation.x=q[0]
        transformer.transform.rotation.y=q[1]
        transformer.transform.rotation.z=q[2]
        transformer.transform.rotation.w=q[3]

        #动态坐标关系发布出去
        self.broadcaster_.sendTransform(transformer)
        self.get_logger().info(f'发布静态TF：{transformer}')

def main():
    rclpy.init()
    node=TFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()




        





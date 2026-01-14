import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener,Buffer #动态坐标发布器
from tf_transformations import euler_from_quaternion  #四元函数转欧拉角
import math #角度转弧度函数

class  TFBroadcaster(Node):
    def __init__(self):
        super().__init__('dynastic_tf_broadcaster')
        self.buffer_=Buffer()
        self.listener_ =TransformListener(self.buffer_,self)
        #self.publish_static_tf()
        self.timer_=self.create_timer(1.0,self.get_transformer)  #动态坐标要不停地发布
    
    def get_transformer(self):
        """
        实时查询坐标关系
        """
        try:
            result=self.buffer_.lookup_transform('base_link','bottle_link',
            rclpy.time.Time(seconds=0.0),rclpy.time.Duration(seconds=1.0))
            transformer=result.transform
            self.get_logger().info(f'平移：{transformer.translation}')
            self.get_logger().info(f'旋转：{transformer.rotation}')
            rotation_euler=euler_from_quaternion([
                transformer.rotation.x,
                transformer.rotation.y,
                transformer.rotation.z,
                transformer.rotation.w
            ])
            self.get_logger().info(f'旋转RPY：{rotation_euler}')
        except Exception as e:
            self.get_logger().warn(f'获取坐标变换失败：原因：{str(e)}')
        

def main():
    rclpy.init()
    node=TFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()




        





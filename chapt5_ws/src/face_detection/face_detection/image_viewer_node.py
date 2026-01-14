#!/usr/bin/env python3
"""
图像显示节点
订阅带有人脸框的图像并显示
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageViewerNode(Node):
    def __init__(self):
        super().__init__('image_viewer')
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            Image,
            'face_detection/image_with_faces',
            self.image_callback,
            10
        )
        
        # 创建CV桥
        self.bridge = CvBridge()
        
        # 创建显示窗口
        cv2.namedWindow('Face Detection', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Face Detection', 800, 600)
        
        self.get_logger().info("图像显示节点已启动，等待图像数据...")
        
    def image_callback(self, msg):
        """处理接收到的图像"""
        try:
            # 转换ROS图像消息为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 添加窗口标题
            window_title = 'Face Detection - ROS2'
            
            # 显示图像
            cv2.imshow('Face Detection', cv_image)
            
            # 等待1ms，允许窗口更新
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"处理图像失败: {e}")
    
    def destroy_node(self):
        """清理资源"""
        self.get_logger().info("关闭显示窗口...")
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        image_viewer = ImageViewerNode()
        rclpy.spin(image_viewer)
    except KeyboardInterrupt:
        pass
    finally:
        if 'image_viewer' in locals():
            image_viewer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
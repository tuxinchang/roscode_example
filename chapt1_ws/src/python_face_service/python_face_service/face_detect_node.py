import rclpy
from rclpy.node import Node
from python_face_interface.srv import FaceDetector  # 导入自定义的人脸检测服务接口
import face_recognition  # 人脸识别库
import cv2  # OpenCV图像处理库
from ament_index_python.packages import get_package_share_directory
import os
from cv_bridge import CvBridge  # ROS2与OpenCV图像转换桥接
import time
from rcl_interfaces.msg import SetParametersResult  # 参数设置结果消息

class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')  # 初始化ROS2节点
        
        # 创建人脸检测服务，服务名称为'face_detect'，回调函数为detect_face_callback
        self.service_ = self.create_service(FaceDetector, 'face_detect', self.detect_face_callback)
        self.bridge = CvBridge()  # 创建图像转换桥接对象
        
        # 声明并初始化节点参数[5,7](@ref)
        self.declare_parameter('number_of_times_to_upsample', 1)  # 人脸检测上采样次数参数
        self.declare_parameter('model', 'hog')  # 人脸检测模型参数（hog或cnn）
        
        # 获取参数当前值
        self.number_of_times_to_upsample = self.get_parameter('number_of_times_to_upsample').value
        self.model = self.get_parameter('model').value
        
        # 设置默认图像路径
        self.default_image_path = os.path.join(get_package_share_directory
        ('python_face_service'), 'resource/two_animal.jpg')
        
        self.get_logger().info("人脸检测服务已经启动！")
        
        # 添加参数设置回调函数，当参数改变时自动调用[7](@ref)
        self.add_on_set_parameters_callback(self.parameters_callback)

    def parameters_callback(self, parameters):
        """参数变更回调函数，处理动态参数更新[5,7](@ref)"""
        for parameter in parameters:
            self.get_logger().info(f"{parameter.name}->{parameter.value}")
            
            # 根据参数名更新对应的类属性
            if parameter.name == 'number_of_times_to_upsample':
                self.number_of_times_to_upsample = parameter.value
            if parameter.name == 'model':
                self.model = parameter.value
                
        # 返回设置成功的结果[7](@ref)
        return SetParametersResult(successful=True)

    def detect_face_callback(self, request, response):
        """人脸检测服务回调函数，处理客户端请求[4,5](@ref)"""
        # 检查请求中是否包含图像数据
        if request.image.data:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            # 如果请求图像为空，使用默认图像
            cv_image = cv2.imread(self.default_image_path)
            self.get_logger().info("传入图像为空，使用默认图像！")
        
        # 开始计时，测量人脸检测耗时
        start_time = time.time()
        self.get_logger().info("加载完成图像，开始识别！")
        
        # 调用face_recognition库进行人脸检测[4](@ref)
        face_locations = face_recognition.face_locations(
            cv_image,
            number_of_times_to_upsample=self.number_of_times_to_upsample,  # 使用动态参数
            model=self.model  # 使用动态参数选择检测模型
        )
        
        # 计算检测耗时并设置到响应中
        response.user_time = time.time() - start_time
        response.number = len(face_locations)  # 设置检测到的人脸数量
        
        # 遍历所有检测到的人脸位置信息
        for top, right, bottom, left in face_locations:
            # 将每个人脸的边界框坐标添加到响应数组中[5](@ref)
            response.top.append(top)    # 人脸上边界
            response.right.append(right)  # 人脸右边界
            response.bottom.append(bottom)  # 人脸下边界
            response.left.append(left)  # 人脸左边界
        
        return response  # 返回包含检测结果的响应

def main():
    """主函数：初始化并运行ROS2节点"""
    rclpy.init()  # 初始化ROS2
    node = FaceDetectNode()  # 创建人脸检测节点实例
    rclpy.spin(node)  # 保持节点运行，等待服务请求
    rclpy.shutdown()  # 关闭ROS2
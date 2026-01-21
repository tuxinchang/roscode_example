import rclpy
from rclpy.node import Node
from python_face_interface.srv import FaceDetector  # 导入自定义的人脸检测服务接口
import face_recognition  # 人脸识别库
import cv2  # OpenCV库，用于图像处理
from ament_index_python.packages import get_package_share_directory
import os
from cv_bridge import CvBridge  # ROS2和OpenCV图像转换桥接
import time
from rcl_interfaces.srv import SetParameters  # ROS2参数设置服务
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType  # ROS2参数相关消息类型

class FaceDetectClientNode(Node):
    def __init__(self):
        super().__init__('face_detect_client_node')  # 初始化ROS2节点，节点名为'face_detect_client_node'
        self.bridge = CvBridge()  # 创建CV桥接对象，用于ROS图像消息和OpenCV图像的转换
        # 获取默认图像路径[7](@ref)
        self.default_image_path = os.path.join(get_package_share_directory
        ('python_face_service'), 'resource/three_country.jpg')
        self.get_logger().info("人脸检测服务已经启动！")  # 日志输出，提示节点已启动
        # 创建人脸检测服务客户端，服务名称为'face_detect'[7](@ref)
        self.client = self.create_client(FaceDetector, 'face_detect')
        self.image = cv2.imread(self.default_image_path)  # 读取默认图像文件

    def call_set_parameters(self, parameters):
        """调用参数设置服务来更新节点参数[7](@ref)"""
        # 创建参数设置服务客户端，连接到指定节点的参数服务
        update_param_client = self.create_client(SetParameters, '/face_detect_node/set_parameters')
        # 等待参数更新服务端上线[8](@ref)
        while update_param_client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info('等待参数更新服务端上线')
        request = SetParameters.Request()  # 创建参数设置请求
        request.parameters = parameters  # 设置要更新的参数列表
        future = update_param_client.call_async(request)  # 异步调用参数设置服务
        rclpy.spin_until_future_complete(self, future)  # 等待服务端返回响应[8](@ref)
        response = future.result()  # 获取服务响应结果
        return response

    def update_detect_model(self, model='hog'):
        """更新人脸检测模型参数[7](@ref)
        
        Args:
            model (str): 检测模型类型，'hog'或'cnn'
        """
        param = Parameter()  # 创建参数对象
        param.name = 'model'  # 参数名称为'model'
        param_value = ParameterValue()  # 创建参数值对象
        param_value.string_value = model  # 设置参数值为指定的模型字符串
        param_value.type = ParameterType.PARAMETER_STRING  # 设置参数类型为字符串
        param.value = param_value  # 将参数值赋给参数对象
        # 调用参数设置服务，传入参数数组[7](@ref)
        response = self.call_set_parameters([param])
        # 遍历并记录每个参数设置结果
        for result in response.results:
            self.get_logger().info(f"设置参数结果：{result.successful} {result.reason}")

    def send_request(self):
        """发送人脸检测请求到服务端[7](@ref)"""
        # 等待人脸检测服务端上线
        while self.client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info('等待服务端上线！')
        request = FaceDetector.Request()  # 创建人脸检测请求
        # 将OpenCV图像转换为ROS2图像消息并设置到请求中[7](@ref)
        request.image = self.bridge.cv2_to_imgmsg(self.image)
        future = self.client.call_async(request)  # 异步调用人脸检测服务[8](@ref)
        
        def result_callback(result_future):
            """服务响应回调函数[7](@ref)"""
            response = result_future.result()  # 获取服务响应结果
            # 记录检测结果日志，包括检测到的人脸数量和耗时
            self.get_logger().info(f"接受响应，共检测到人脸{response.number}个，耗时为{response.user_time}秒")
            self.show_response(response)  # 显示检测结果
        
        future.add_done_callback(result_callback)  # 为异步调用添加完成回调函数

    def show_response(self, response):
        """可视化显示人脸检测结果
        
        Args:
            response: 人脸检测服务响应，包含人脸位置信息
        """
        # 遍历所有检测到的人脸框[7](@ref)
        for i in range(response.number):
            top = response.top[i]  # 人脸上边界
            right = response.right[i]  # 人脸右边界
            bottom = response.bottom[i]  # 人脸下边界
            left = response.left[i]  # 人脸左边界
            # 在图像上绘制人脸矩形框，蓝色，线宽4像素
            cv2.rectangle(self.image, (left, top), (right, bottom), (255, 0, 0), 4)
        # 显示带有人脸框的结果图像
        cv2.imshow('Face Detect Result', self.image)
        cv2.waitKey(0)  # 等待按键后关闭窗口

def main():
    """主函数，程序入口点[7](@ref)"""
    rclpy.init()  # 初始化ROS2 Python接口
    node = FaceDetectClientNode()  # 创建人脸检测客户端节点实例
    
    # 使用HOG模型进行人脸检测[7](@ref)
    node.update_detect_model('hog')  # 设置检测模型为HOG
    node.send_request()  # 发送人脸检测请求
    
    # 使用CNN模型进行人脸检测（对比测试）
    node.update_detect_model('cnn')  # 设置检测模型为CNN
    node.send_request()  # 再次发送人脸检测请求
    
    rclpy.spin(node)  # 保持节点运行，等待回调函数执行[7](@ref)
    rclpy.shutdown()  # 关闭ROS2 Python接口


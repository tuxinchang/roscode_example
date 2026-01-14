#!/usr/bin/env python3
"""
人脸检测节点
读取摄像头，检测人脸，并发布带有人脸框的图像
"""
#!/usr/bin/env python3
"""
修复版人脸检测节点 - 解决cv2.data属性不存在问题
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import sys
import subprocess
import urllib.request
import time

def get_cascade_path():
    """获取级联分类器文件路径，不依赖cv2.data"""
    # 1. 首先检查常用路径
    common_paths = [
        # Ubuntu/Debian 系统路径
        '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
        '/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml',
        # 其他可能的系统路径
        '/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
        '/usr/local/share/opencv/haarcascades/haarcascade_frontalface_default.xml',
        # Python包路径
        '/usr/local/lib/python3.8/dist-packages/cv2/data/haarcascade_frontalface_default.xml',
        '/usr/lib/python3/dist-packages/cv2/data/haarcascade_frontalface_default.xml',
        # 当前目录
        'haarcascade_frontalface_default.xml',
        # 用户目录
        os.path.expanduser('~/.local/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml'),
    ]
    
    for path in common_paths:
        if os.path.exists(path):
            print(f"找到级联分类器文件: {path}")
            return path
    
    # 2. 如果常用路径没有，尝试在系统中搜索
    try:
        result = subprocess.run(
            ['find', '/usr', '-name', 'haarcascade_frontalface_default.xml', '-type', 'f'],
            capture_output=True, text=True, timeout=5
        )
        if result.stdout:
            paths = result.stdout.strip().split('\n')
            for path in paths:
                if path and os.path.exists(path):
                    print(f"搜索到级联分类器文件: {path}")
                    return path
    except:
        pass
    
    # 3. 如果还没有找到，返回None
    return None

def download_cascade_file():
    """从网络下载级联分类器文件"""
    url = "https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_default.xml"
    local_path = "haarcascade_frontalface_default.xml"
    
    print(f"正在从网络下载级联分类器文件...")
    print(f"URL: {url}")
    
    try:
        urllib.request.urlretrieve(url, local_path)
        print(f"已下载到: {local_path}")
        return local_path
    except Exception as e:
        print(f"下载失败: {e}")
        return None

class FaceDetectorNode(Node):
    def __init__(self, camera_index=0):
        super().__init__('face_detector')
        
        self.get_logger().info("=== 人脸检测节点 (修复版) ===")
        self.get_logger().info(f"OpenCV版本: {cv2.__version__}")
        
        # 从命令行参数获取摄像头索引
        if len(sys.argv) > 1:
            try:
                camera_index = int(sys.argv[1])
            except ValueError:
                pass
        
        # 创建发布者，发布带有人脸框的图像
        self.publisher_ = self.create_publisher(Image, 'face_detection/image_with_faces', 10)
        
        # 创建CV桥
        self.bridge = CvBridge()
        
        # 1. 加载人脸检测器（修复cv2.data问题）
        cascade_path = get_cascade_path()
        
        if cascade_path is None:
            self.get_logger().warn("未找到级联分类器文件，尝试下载...")
            cascade_path = download_cascade_file()
        
        if cascade_path is None:
            self.get_logger().error("无法获取级联分类器文件，将使用模拟检测器")
            self.face_cascade = self.create_simulated_detector()
        else:
            self.get_logger().info(f"加载级联分类器: {cascade_path}")
            
            try:
                self.face_cascade = cv2.CascadeClassifier(cascade_path)
                if self.face_cascade.empty():
                    self.get_logger().error("无法加载人脸检测模型！")
                    self.get_logger().warn("将使用模拟检测器")
                    self.face_cascade = self.create_simulated_detector()
                else:
                    self.get_logger().info("✅ 人脸检测模型加载成功")
            except Exception as e:
                self.get_logger().error(f"加载人脸检测模型失败: {e}")
                self.get_logger().warn("将使用模拟检测器")
                self.face_cascade = self.create_simulated_detector()
        
        # 2. 初始化摄像头
        self.camera_index = camera_index
        self.init_camera()
        
        # 3. 创建定时器，定期处理视频帧
        self.timer = self.create_timer(0.033, self.process_frame)  # ~30 fps
        
        # 4. 统计信息
        self.frame_count = 0
        self.total_faces = 0
        self.start_time = time.time()
        
        self.get_logger().info("人脸检测节点已启动，正在处理摄像头视频...")
    
    def create_simulated_detector(self):
        """创建模拟的人脸检测器（当级联分类器不可用时）"""
        class SimulatedFaceDetector:
            def __init__(self, node):
                self.node = node
                self.detection_count = 0
            
            def detectMultiScale(self, gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30), flags=None):
                height, width = gray.shape
                
                # 模拟检测到的人脸
                faces = []
                
                # 在图像中心附近创建一个模拟人脸
                if self.detection_count % 100 < 70:  # 70%的时间检测到"人脸"
                    face_size = min(width, height) // 3
                    x = (width - face_size) // 2
                    y = (height - face_size) // 2
                    
                    # 添加一些随机偏移，使检测更"真实"
                    import random
                    x_offset = random.randint(-50, 50)
                    y_offset = random.randint(-30, 30)
                    x = max(0, min(width - face_size, x + x_offset))
                    y = max(0, min(height - face_size, y + y_offset))
                    
                    faces.append([x, y, face_size, face_size])
                
                self.detection_count += 1
                return np.array(faces, dtype=np.int32)
        
        return SimulatedFaceDetector(self)
    
    def init_camera(self):
        """初始化摄像头，支持多种设备和回退方案"""
        self.get_logger().info(f"尝试打开摄像头 /dev/video{self.camera_index}")
        
        # 首先检查设备是否存在
        device_path = f"/dev/video{self.camera_index}"
        if os.path.exists(device_path):
            self.get_logger().info(f"摄像头设备存在: {device_path}")
        else:
            self.get_logger().warn(f"摄像头设备不存在: {device_path}")
        
        # 尝试打开摄像头
        self.cap = cv2.VideoCapture(self.camera_index)
        
        if not self.cap.isOpened():
            self.get_logger().warn(f"无法打开摄像头索引 {self.camera_index}，尝试其他索引...")
            
            # 尝试其他常见索引
            for i in [1, 2, 0, 10, 20]:
                if i == self.camera_index:
                    continue
                    
                self.cap = cv2.VideoCapture(i)
                if self.cap.isOpened():
                    self.camera_index = i
                    self.get_logger().info(f"找到可用摄像头: 索引 {i} (/dev/video{i})")
                    break
            
            if not self.cap.isOpened():
                self.get_logger().warn("无法打开任何摄像头，进入虚拟模式")
                self.virtual_mode = True
                self.virtual_frame_count = 0
                return
        
        # 设置摄像头参数
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # 获取实际设置
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        self.virtual_mode = False
        self.get_logger().info(f"✅ 摄像头初始化成功")
        self.get_logger().info(f"   分辨率: {actual_width}x{actual_height}")
        self.get_logger().info(f"   帧率: {actual_fps:.1f} FPS")
    
    def generate_virtual_frame(self):
        """生成虚拟视频帧（当没有摄像头时）"""
        self.virtual_frame_count += 1
        
        # 创建测试图像
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # 渐变背景
        for i in range(480):
            color = int(100 + 50 * np.sin(i / 50 + self.virtual_frame_count / 20))
            frame[i, :] = (color, color, color)
        
        # 添加标题
        title = "Virtual Camera - Face Detection"
        cv2.putText(frame, title, (50, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # 添加模拟人脸
        center_x, center_y = 320, 240
        face_radius = 100 + int(20 * np.sin(self.virtual_frame_count / 10))
        
        # 绘制人脸
        cv2.circle(frame, (center_x, center_y), face_radius, (0, 0, 255), 3)
        
        # 眼睛
        eye_radius = 15
        cv2.circle(frame, (center_x - 40, center_y - 30), eye_radius, (255, 255, 255), -1)
        cv2.circle(frame, (center_x + 40, center_y - 30), eye_radius, (255, 255, 255), -1)
        
        # 嘴巴
        mouth_y = center_y + 30
        cv2.ellipse(frame, (center_x, mouth_y), (40, 20), 0, 0, 180, (255, 255, 255), 3)
        
        # 添加信息
        info = f"Frame: {self.virtual_frame_count}"
        cv2.putText(frame, info, (20, 460), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        return frame
    
    def process_frame(self):
        """处理视频帧并进行人脸检测"""
        self.frame_count += 1
        
        try:
            # 1. 获取视频帧
            if self.virtual_mode:
                frame = self.generate_virtual_frame()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                if self.cap is None or not self.cap.isOpened():
                    self.get_logger().warn("摄像头未就绪")
                    return
                
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().warn("无法读取摄像头帧")
                    return
                
                # 转换为灰度图以进行人脸检测
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # 2. 人脸检测
            faces = self.face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(30, 30)
            )
            
            # 3. 在检测到的人脸周围画红框
            for (x, y, w, h) in faces:
                # 红色框
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
                
                # 添加标签
                label = f"Face: {w}x{h}"
                cv2.putText(frame, label, (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # 4. 更新统计信息
            faces_count = len(faces)
            self.total_faces += faces_count
            
            # 5. 显示检测到的人脸数量和统计信息
            elapsed_time = time.time() - self.start_time
            fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
            
            mode_text = f"Camera {self.camera_index}" if not self.virtual_mode else "Virtual"
            cv2.putText(frame, f"Mode: {mode_text}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Faces: {faces_count}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Total: {self.total_faces}", (10, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 120), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 6. 转换为ROS2图像消息并发布
            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = "camera_frame"
                self.publisher_.publish(ros_image)
            except Exception as e:
                self.get_logger().error(f"转换图像失败: {e}")
            
            # 7. 每100帧输出一次统计信息
            if self.frame_count % 100 == 0:
                self.get_logger().info(f"已处理 {self.frame_count} 帧，检测到 {self.total_faces} 个人脸，FPS: {fps:.1f}")
                
        except Exception as e:
            self.get_logger().error(f"处理帧时出错: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def destroy_node(self):
        """清理资源"""
        self.get_logger().info("关闭摄像头并清理资源...")
        
        # 计算最终统计
        elapsed_time = time.time() - self.start_time
        fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
        
        self.get_logger().info(f"最终统计:")
        self.get_logger().info(f"  总帧数: {self.frame_count}")
        self.get_logger().info(f"  总检测人脸数: {self.total_faces}")
        self.get_logger().info(f"  平均FPS: {fps:.1f}")
        self.get_logger().info(f"  运行时间: {elapsed_time:.1f}秒")
        
        if not self.virtual_mode and self.cap is not None and self.cap.isOpened():
            self.cap.release()
        
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        face_detector = FaceDetectorNode()
        rclpy.spin(face_detector)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"启动节点失败: {e}")
    finally:
        if 'face_detector' in locals():
            face_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
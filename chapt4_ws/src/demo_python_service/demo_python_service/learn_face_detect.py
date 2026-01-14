import face_recognition
import cv2
#获取功能包share目录下的绝对路径
from ament_index_python.packages import get_package_share_directory 

def main():
    # 获取图片的真实路径:/home/tuxinchang/chapt/chapt4_ws/install/demo_python_service/share/demo_python_service
    default_image_path=get_package_share_directory("demo_python_service")+'/resource/two_animal.jpg'
    print(f'图片的真实路径：{default_image_path}')
    #使用vc2加载图片
    image=cv2.imread(default_image_path)
    #检测人脸
    face_locations=face_recognition.face_locations(image,number_of_times_to_upsample=1,
    model='hog')
    #绘制人脸框
    for top,right,bottom,left in face_locations:
        cv2.rectangle(image,(left,top),(right,bottom),(255,0,0),4)#最后一个参数是矩形框的宽度
    # 结果显示
    cv2.imshow('Face Detect Result',image)
    cv2.waitKey(0)







from setuptools import find_packages, setup
#为launch导入的
from glob import glob
package_name = 'python_face_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #添加的图片路径配置
        ('share/' + package_name+"/resource", ['resource/three_country.jpg']),#添加图片路径，到时候构建功能包才能把图片写入install文件夹中
        #配置launch文件路径
        ('share/' + package_name+"/launch", glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tuxinchang',
    maintainer_email='3031484694@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'face_detect_node=python_face_service.face_detect_node:main',
            'face_detect_client_node=python_face_service.face_detect_client_node:main',
        ],
    },
)

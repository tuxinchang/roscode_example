from setuptools import find_packages, setup

package_name = 'demo_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tuxinchang',
    maintainer_email='tuxinchang@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            #执行文件名字不一定非要和代码的文件保持一致
            'python_node=demo_python_pkg.ros2_python_node:main',#等号后面为：功能包名字.节点代码文件名字.实现节点功能的函数名
            'person_node=demo_python_pkg.person_node:main',
            'learn_thread=demo_python_pkg.learn_thread:main',
            'novel_pub_node=demo_python_pkg.novel_pub_node:main',
            'novel_sub_node=demo_python_pkg.novel_sub_node:main',
        ],
    },
)

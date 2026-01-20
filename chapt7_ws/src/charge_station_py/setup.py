from setuptools import find_packages, setup

package_name = 'charge_station_py'

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
    maintainer_email='3031484694@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'charge_action_server = charge_station_py.charge_action_server:main',
            'charge_action_client = charge_station_py.charge_action_client:main',
        ],
    },
)

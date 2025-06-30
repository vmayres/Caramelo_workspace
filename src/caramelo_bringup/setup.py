from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'caramelo_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='victor',
    maintainer_email='victoroliveiraayres@gmail.com',
    description='Pacote de bringup para o robô real Caramelo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'caramelo_hw_interface_node = caramelo_bringup.caramelo_hw_interface_node:main',
            'encoder_joint_state_node = caramelo_bringup.encoder_joint_state_node:main',
            'twist_converter_node = caramelo_bringup.twist_converter_node:main',
            'odom_tf_publisher_node = caramelo_bringup.odom_tf_publisher_node:main',
        ],
    },
)

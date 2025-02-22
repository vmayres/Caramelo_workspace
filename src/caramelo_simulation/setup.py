from setuptools import setup
import os
from glob import glob

package_name = 'caramelo_simulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seu Nome',
    maintainer_email='seuemail@example.com',
    description='Pacote do robô Caramelo convertido para ament_python',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Exemplo: 'nome_do_executavel = pacote.modulo:funcao'
            
        ],
    },
    data_files=[
        #('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'description'), glob('description/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        #(os.path.join('share', package_name, 'models'), glob('models/*')),
        (os.path.join('share', package_name, 'photos'), glob('photos/*')),
    ],
)


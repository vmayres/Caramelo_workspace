from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'caramelo_description'

# Function to get all files in a directory recursively
def get_files_recursive(directory):
    files = []
    for root, dirs, filenames in os.walk(directory):
        for filename in filenames:
            files.append(os.path.join(root, filename))
    return files

# Get all files in Kuka YouBot directory
youbot_files = []
if os.path.exists('Kuka YouBot'):
    for root, dirs, filenames in os.walk('Kuka YouBot'):
        for filename in filenames:
            file_path = os.path.join(root, filename)
            rel_dir = os.path.relpath(root, 'Kuka YouBot')
            if rel_dir == '.':
                target_dir = os.path.join('share', package_name, 'Kuka YouBot')
            else:
                target_dir = os.path.join('share', package_name, 'Kuka YouBot', rel_dir)
            youbot_files.append((target_dir, [file_path]))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'URDF'), glob('URDF/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ] + youbot_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='victor',
    maintainer_email='victoroliveiraayres@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Exemplo: 'nome_do_executavel = pacote.modulo:funcao'

        ],
    },
)

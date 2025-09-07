from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gazebo_simulation'

# Reutiliza find_packages para pacotes Python (se houver futuramente)

# Função genérica para coletar todos os arquivos dentro de um diretório
# preservando subdiretórios ao instalar em share/<package>/<dir>
def package_files(root_dir):
    paths = []
    if os.path.isdir(root_dir):
        for path in glob(os.path.join(root_dir, '**'), recursive=True):
            if os.path.isfile(path):
                install_dir = os.path.join('share', package_name, os.path.dirname(path))
                paths.append((install_dir, [path]))
    return paths

# Diretórios de dados que queremos instalar recursivamente
DATA_DIRS = ['launch', 'worlds', 'models']

# Base data_files entries obrigatórios para índice e package.xml
data_files = [
    ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
    (f'share/{package_name}', ['package.xml']),
]

for d in DATA_DIRS:
    data_files.extend(package_files(d))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],  # ROS dependencies handled via package.xml
    zip_safe=True,
    maintainer='linux24-04',
    maintainer_email='victoroliveiraayres@gmail.com',
    description='Gazebo simulation launch and resources',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

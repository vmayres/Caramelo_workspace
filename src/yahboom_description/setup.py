from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'yahboom_description'

def only_files(pattern):
    return [p for p in glob(pattern) if os.path.isfile(p)]

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        # launch / rviz
        (f'share/{package_name}/launch', only_files('launch/*')),
        (f'share/{package_name}/rviz',   only_files('rviz/*')),
        # URDF raiz
        (f'share/{package_name}/urdf', only_files('urdf/*')),
        # Subpastas URDF (repita/adapte conforme existir)
        (f'share/{package_name}/urdf/control',  only_files('urdf/control/*')),
        (f'share/{package_name}/urdf/mech',    only_files('urdf/mech/*')),
        (f'share/{package_name}/urdf/robots',  only_files('urdf/robots/*')),
        (f'share/{package_name}/urdf/sensors', only_files('urdf/sensors/*')),
        # Meshes raiz
        (f'share/{package_name}/meshes', only_files('meshes/*')),
        # Subpastas meshes existentes referenciadas pelos URDF/Xacros
        (f'share/{package_name}/meshes/robot_3d',          only_files('meshes/robot_3d/*')),
        (f'share/{package_name}/meshes/robot_3d/visual',   only_files('meshes/robot_3d/visual/*')),
        (f'share/{package_name}/meshes/robot_3d/collision', only_files('meshes/robot_3d/collision/*')),
        (f'share/{package_name}/meshes/rplidar',           only_files('meshes/rplidar/*')),
        (f'share/{package_name}/meshes/intel_realsense',   only_files('meshes/intel_realsense/*')),
        (f'share/{package_name}/meshes/intel_realsense/visual',  only_files('meshes/intel_realsense/visual/*')),
        (f'share/{package_name}/meshes/wheels',            only_files('meshes/wheels/*')),
        (f'share/{package_name}/meshes/chassis',           only_files('meshes/chassis/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linux24-04',
    maintainer_email='victoroliveiraayres@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

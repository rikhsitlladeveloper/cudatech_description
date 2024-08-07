import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cudatech_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.gazebo'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.stl'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cudatech',
    maintainer_email='cudatech@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamic_frame_tf2_broadcaster = cudatech_description.dynamic_frame_tf2_broadcaster:main',
            'state_publisher = cudatech_description.state_publisher:main',
            'serialport_controller = cudatech_description.serialport_controller:main',
            'go_to_pose = cudatech_description.go_to_pose:main',
        ],
    },
)

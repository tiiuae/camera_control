from setuptools import setup
import os
from glob import glob

package_name = 'camera_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        #('share/ament_index/resource_index/packages', ['resource/depthai_node' ]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('config/*.yaml')),
        (os.path.join(os.path.join('share', package_name),'urdf'), glob('urdf/*.urdf')),
        (os.path.join(os.path.join('share', package_name),'urdf'), glob('urdf/*.urdf.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sergey',
    maintainer_email='sergey.smirnov@unikie.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'depthai_node = resource.depthai_node:main'
            'px4_imu_converter = camera_control.px4_imu_converter:main'
        ],
    },
)

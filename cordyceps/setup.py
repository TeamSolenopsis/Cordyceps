from setuptools import setup
import os
from glob import glob

package_name = 'cordyceps'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cas',
    maintainer_email='466933@student.fontys.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = cordyceps.vs_controller:main',
            'manager = cordyceps.vs_manager:main',
            'planner = cordyceps.path_planner:main',
            'assembler = cordyceps.custom_assembler:main',
            'task_publisher = cordyceps.task_publisher:main',
        ],
    },
)

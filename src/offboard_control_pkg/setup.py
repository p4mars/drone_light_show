from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'offboard_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name, 'launch'), ['launch/offboard_control_launch_file.launch.py']),
        ],
    install_requires=['setuptools', 'px4_msgs'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='l.jordan@student.tudelft.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'offboard_control_node = offboard_control_pkg.offboard_control:main'
        ],
    },
)


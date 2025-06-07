from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'offboard_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=['offboard_control_py'],
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
    description='Meow meow meow',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_control_node = offboard_control_py.offboard_control_multi_vehicle:main',
            'Drone_One_Node = offboard_control_py.drone1:main',
            'Drone_Two_Node = offboard_control_py.drone2:main',
            'Drone_Three_Node = offboard_control_py.drone3:main',
        ],
    },
)


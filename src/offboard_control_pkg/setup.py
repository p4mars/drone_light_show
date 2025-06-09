from setuptools import setup

package_name = 'offboard_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lauren Jordan',
    maintainer_email='your@email.com',
    description='Drone offboard control package',
    license='MIT',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/offboard_control_pkg']),
        ('share/offboard_control_pkg', ['package.xml']),
        # ADD THIS:
        ('share/offboard_control_pkg/launch', [
            'launch/offboard_control_launch_file.launch.py'
        ]),
    ],
    entry_points={
        'console_scripts': [
            'control_node = offboard_control_pkg.control_node:main',
            'drone1 = offboard_control_pkg.drone1:main',
            'drone2 = offboard_control_pkg.drone2:main',
            'drone3 = offboard_control_pkg.drone3:main',
        ],
    },
)

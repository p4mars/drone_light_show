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
    ],
    entry_points={
        'console_scripts': [
            'offboard_control_node = offboard_control_pkg.control_node:main',
            'Drone_One_Node = offboard_control_pkg.drone1:main',
            'Drone_Two_Node = offboard_control_pkg.drone2:main',
            'Drone_Three_Node = offboard_control_pkg.drone3:main',
        ],
    },
)

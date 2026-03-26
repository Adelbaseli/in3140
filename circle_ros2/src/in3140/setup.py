from setuptools import setup

package_name = 'in3140'

setup(
    name=package_name,
    version='0.0.0',
    packages=['in3140'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adel',
    maintainer_email='adel.baselizadeh@gmail.com',
    description='A simple ROS 2 package for path planning',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'path_planner = in3140.path_planner_ros2:main',
        ],
    },
)

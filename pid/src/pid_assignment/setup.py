from setuptools import setup, find_packages
import os

package_name = 'pid_assignment'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},  # tells setuptools to look in src/
    data_files=[
        ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/pid.launch.py',
            'launch/crustcrawler.perspective',  # include perspective file
        ]),
        ('share/' + package_name + '/cfg', ['cfg/Pid.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adel Baselizadeh',
    maintainer_email='adel@example.com',
    description='PID controller package for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'node = pid_assignment.node:main',  # matches your node.py main()
        ],
    },
)

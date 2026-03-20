from setuptools import setup, find_packages

package_name = 'ros_services_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=[
        'setuptools',
        'tornado>=4.2.1',
    ],
    zip_safe=True,
    maintainer='Sverk',
    maintainer_email='sverk@example.com',
    description='HTTP API to list and call ROS 2 services',
    license='BSD',
    entry_points={
        'console_scripts': [
            'ros_services_bridge_node = ros_services_bridge.bridge_node:main',
        ],
    },
)

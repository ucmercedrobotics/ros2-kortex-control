import os
from glob import glob
from setuptools import setup

package_name = 'nanospec'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Mehrad Mortazavi',
    maintainer_email='smortazavi3@ucmerced.edu',
    description='ROS2 driver for nanoLambda NSP32 spectrometer',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nanospec_action_server = nanospec.nanospec_action_server:main',
            # Legacy nodes (kept for backwards compatibility)
            'NSP32_service_node = nanospec.NSP32_service_node:main',
            'NSP32_client_node = nanospec.NSP32_client_node:main',
            'NSP32_client_triggered = nanospec.NSP32_client_triggered:main',
        ],
    },
)

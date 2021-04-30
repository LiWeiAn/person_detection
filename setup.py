from setuptools import setup
import os

package_name = 'person_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vivi',
    maintainer_email='vli16@student.aau.dk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'detection_node = person_detection.detection_node',
         'person_remove = person_detection.person_remove:main',
         'sync = person_detection.sync:main'
        ],
    },
)

import os
from glob import glob

from setuptools import setup

package_name = 'button_analyzer'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "resource"),
            glob("resource/*.txt"),
        ),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='banana-killer',
    maintainer_email='sashagrachev2005@gmail.com',
    description='ROS2 node for processing button presses and managing motion\
        sequences.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'button_analyzer_without_hands = button_analyzer.button_analyzer_without_hands:main',
            'button_analyzer_with_hands = button_analyzer.button_analyzer_with_hands:main',
        ],
    },
)

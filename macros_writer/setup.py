from setuptools import setup

package_name = 'macros_writer'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='banana-killer',
    maintainer_email='sashagrachev2005@gmail.com',
    description='ROS 2 node for recording Unitree robot motion.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'macros_writer = macros_writer.macros_writer:main',
        ],
    },
)

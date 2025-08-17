from setuptools import setup

package_name = 'position_writer'

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
    description='ROS 2 GUI application for recording robot poses.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'position_writer_node = position_writer.position_writer_node:main'
        ],
    },
)

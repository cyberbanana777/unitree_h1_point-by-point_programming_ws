from setuptools import setup

package_name = 'buttons_server'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['resource/html_skeleton_with_hands.html']),
        ('share/' + package_name, ['resource/html_skeleton_without_hands.html']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='banana-killer',
    maintainer_email='sashagrachev2005@gmail.com',
    description='This package allows check a web page for robot control. ' \
    '"hear_server" in this package send to topic "button_status" which button is pressed. ' \
    'Buttons located on a web page. Web page is started by "server_stand_up',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "hear_server = buttons_server.hear_server:main",
            "server_stand_up_without_hands = buttons_server.server_stand_up_without_hands:main",
            "server_stand_up_with_hands = buttons_server.server_stand_up_with_hands:main",
        ],
    },
)

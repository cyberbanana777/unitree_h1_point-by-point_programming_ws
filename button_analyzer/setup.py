from setuptools import setup

package_name = 'button_analyzer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['resource/selfie_with_hands.txt']),
        ('share/' + package_name, ['resource/wave_with_hands.txt']),
        ('share/' + package_name, ['resource/attention_with_hands.txt']),
        ('share/' + package_name, ['resource/photo_with_hands.txt']),

        ('share/' + package_name, ['resource/selfie_without_hands.txt']),
        ('share/' + package_name, ['resource/wave_without_hands.txt']),
        ('share/' + package_name, ['resource/attention_without_hands.txt']),
        ('share/' + package_name, ['resource/photo_without_hands.txt']),
        
        ('share/' + package_name, ['resource/1_offer_hand_with_hands.txt']),
        ('share/' + package_name, ['resource/2_shake_hand_with_hands.txt']),
        ('share/' + package_name, ['resource/1_offer_docs_with_hands.txt']),
        ('share/' + package_name, ['resource/2_grip_docs_with_hands.txt']),
        ('share/' + package_name, ['resource/3_hold_docs_with_hands.txt']),
        ('share/' + package_name, ['resource/4_give_docs_with_hands.txt']),
        ('share/' + package_name, ['resource/5_release_docs_with_hands.txt'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='banana-killer',
    maintainer_email='sashagrachev@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'button_analyzer_without_hands = button_analyzer.button_analyzer_without_hands:main',
            'button_analyzer_with_hands = button_analyzer.button_analyzer_with_hands:main',
        ],
    },
)

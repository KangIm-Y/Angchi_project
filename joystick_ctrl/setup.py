from setuptools import find_packages, setup

package_name = 'joystick_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kangim',
    maintainer_email='tls3162@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_joy_sub = joystick_ctrl.test_joy_sub:main',
            'test_joy_echo = joystick_ctrl.test_joy_echo:main',
            'joy_pub_testmodel = joystick_ctrl.joy_pub_testmodel:main',
            'joy_sub_testmodel = joystick_ctrl.joy_sub_testmodel:main',
        ],
    },
)

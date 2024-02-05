from setuptools import find_packages, setup

package_name = 'pi_proj_mj'

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
    maintainer='meja',
    maintainer_email='mejariver@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	     'left_motor_node = pi_proj_mj.Motor_control:main',
	     'right_motor_node = pi_proj_mj.Motor_control:main',
	     'left_node = pi_proj_mj.left_node:main',
	     'right_node = pi_proj_mj.right_node:main',
        ],
    },
)

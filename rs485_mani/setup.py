from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rs485_mani'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        #(os.path.join('share', package_name), glob('rs485_mani/nuri_protocool.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ascroid',
    maintainer_email='pas123068@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mani_rs485 = rs485_mani.mani_rs485:main',
            'pub_joint = rs485_mani.pub_joint:main',
        ],
    },
)

from setuptools import find_packages, setup
import glob
import os

package_name = 'test_pkg_1'

setup(
    name=package_name,
    version='0.1.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),( glob.glob(os.path.join('launch', '*launch.py')))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skh',
    maintainer_email='skh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_talker1 =test_pkg_1.test_talker1:main',
            'test_talker2 = test_pkg_1.test_talker2:main',
            'test_listener = test_pkg_1.test_listener:main',
        ],
    },
)

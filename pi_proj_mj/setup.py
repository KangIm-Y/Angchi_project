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
	     'Left = pi_proj_mj.subscriber_left:main',
	     'Right = pi_proj_mj.subscriber_right:main',
	     'pubt = pi_proj_mj.pubt:main',
	     'subf = pi_proj_mj.subf:main',    
        ],
    },
)

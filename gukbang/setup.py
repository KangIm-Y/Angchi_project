from setuptools import find_packages, setup

package_name = 'gukbang'

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
    maintainer='skh',
    maintainer_email='tls3162@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ########### spring ##########
        'army_detection=gukbang.spring.army_detection:main',
        'center_army_detection=gukbang.spring.center_army_detection:main',
        
        
        ########## summer #############
        
        
        
        ########## autumn #############
        'aruco_detection=gukbang.autumn.aruco_detection:main',
        'center_aruco_detection=gukbang.autumn.center_aruco_detection:main',
        
        
        ########## winter #############
        
        
        
        ########## common #############
        'imu_node=gukbang.common.imu_node:main',
        'joy_drive=gukbang.common.joy_drive:main',
        
        ],
    },
)

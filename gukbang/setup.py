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
        'spring_col_checker=gukbang.spring.spring_col_checker:main',
        
        
        ########## summer #############
        'summer_track_checker=gukbang.summer.summer_track_checker:main',
        
        
        
        ########## autumn #############
        'aruco_detection=gukbang.autumn.aruco_detection:main',
        'center_aruco_detection=gukbang.autumn.center_aruco_detection:main',
        
        
        ########## winter #############
        
        
        
        ########## common #############
        'imu_node=gukbang.common.imu_node:main',
        'joy_drive=gukbang.common.joy_drive:main',
        'joy_drive_sub_cam=gukbang.common.joy_drive_sub_cam:main',
        'center_joy_drive=gukbang.common.center_joy_drive:main',
        'center_joy_drive_sub_cam=gukbang.common.center_joy_drive_sub_cam:main',
        
        ],
    },
)

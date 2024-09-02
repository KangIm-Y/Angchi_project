from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dynamixel_MJ'


setup(
    name=package_name,
    version='3.7.51',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rivermeja',
    maintainer_email='mejariver@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            ######################EXTEND POSITION###########################
            'pub_extend= dynamixel_MJ.pub_extend:main',
            'sub_extend = dynamixel_MJ.sub_extend:main',
            
             #####################Tripple Control############################
            'sub_trip = dynamixel_MJ.sub_trip:main',
            'pub_trip = dynamixel_MJ.pub_trip:main',
            'Sub_Trip_Oneport = dynamixel_MJ.Sub_Trip_Oneport:main',#4~6 Joint test

            #########################test #################################
            'serial_comm= dynamixel_MJ.serial_comm:main', #NURI Serial TEST

            ######################TEST CODE_BY INDIVIDUALLY################
            'test_pub = dynamixel_MJ.test_pub:main',
            'test_id4_sub = dynamixel_MJ.test_id4_sub :main', #ID4
            'test_id5_sub = dynamixel_MJ.test_id5_sub:main', #ID5
            'test_id6_sub = dynamixel_MJ.test_id6_sub:main', #ID6
                                    
            ####################Gripper TEST###############################
            'Gripper_kokomk4 = dynamixel_MJ.Gripper_kokomk4:main',
            'gripper_testcode = dynamixel_MJ.gripper_testcode:main',
            'Gripper_test2 = dynamixel_MJ.Gripper_test2:main',
                           
        ],
    },
)

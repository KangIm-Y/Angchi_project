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
             ##########################POSITION#############################
            'rw_pub = dynamixel_MJ.rw_pub:main',
            'rw_pub_user = dynamixel_MJ.rw_pub_user:main',
            'rw_sub = dynamixel_MJ.rw_sub :main',
            ######################EXTEND POSITION###########################
            'pub_extend= dynamixel_MJ.pub_extend :main',
            'sub_extend = dynamixel_MJ.sub_extend :main',
            ######################DOUBLE Control#############################
            'sub_double = dynamixel_MJ.sub_double :main',
            'pub_double = dynamixel_MJ.pub_double :main',
             #####################Tripple Control#############################
            'sub_trip = dynamixel_MJ.sub_trip :main',
            'pub_trip = dynamixel_MJ.pub_trip :main',
            'Sub_Trip_Oneport = dynamixel_MJ.Sub_Trip_Oneport :main',
            'Gripper = dynamixel_MJ.Gripper :main',
            #####################GRIPPER TORQUE TEST###########################
            'Grip_torque_test = dynamixel_MJ.Grip_torque_test :main',
            'Grip_torque_test_pub = dynamixel_MJ.Grip_torque_test_pub :main',
            ####################RS 485 Without SDK Library####################
            'test_0722 = dynamixel_MJ.test_0722 :main',
            ###########################test #################################
            'serial_comm= dynamixel_MJ.serial_comm:main', #NURI Serial TEST
            ####################Gripper final###############################
            'Gripper_kokomk4 = dynamixel_MJ.Gripper_kokomk4:main',
            'Gripper_test = dynamixel_MJ.Gripper_test:main',
            'Gripper_test2 = dynamixel_MJ.Gripper_test2:main',
            #'example = dynamixel_MJ.example:main'
            
            
             

            
                                 

        ],
    },
)

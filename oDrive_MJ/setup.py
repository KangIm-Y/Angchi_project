from setuptools import find_packages, setup

package_name = 'oDrive_MJ'

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
    maintainer='rivermeja',
    maintainer_email='mejariver@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            ###################HALL Sensor#########################
            'pub_hall = oDrive_MJ.pub_hall:main',
            'sub_hall = oDrive_MJ.sub_hall:main',
            'sub2_hall = oDrive_MJ.sub2_hall:main',
            ##################TEST_joystick##################
            'joy_controll = oDrive_MJ.joy_control_meja:main',
            'joy_test = oDrive_MJ.joy_test:main',
            #################MODE TEST###################
            'Middle = oDrive_MJ.pub_hall_mode_select:main',
            'Mode = oDrive_MJ.mode_control_meja:main',
            'ModeA = oDrive_MJ.ModeA:main',
            'ModeB = oDrive_MJ.ModeB:main',
            ################JOY_STICK###################
            'JOYPUB = oDrive_MJ.joy_pub_testmodel:main',
            'JOYSUB = oDrive_MJ.joy_sub_testmodel:main',
            ################TEST_CAR#################
            'Testcar_pub = oDrive_MJ.Testcar_pub:main',
            'Testcar_sub = oDrive_MJ.Testcar_sub:main',
            'Testcar_pub_mode = oDrive_MJ.Testcar_pub_mode:main',
            'Testcar_sub_mode = oDrive_MJ.Testcar_sub_mode:main' ,
            ################FINAL########################
            'trajectory_mode = oDrive_MJ.trajectory_mode:main',
            'velocity_mode = oDrive_MJ.velocity_mode:main' , 

            ######################## KOKOMK4 OPEN #################
            'car_kokomk4 = oDrive_MJ.car_kokomk4:main',
            'encoder_feedback = oDrive_MJ.encoder_feedback:main',
            'odrive_car = oDrive_MJ.odrive_car:main',


        ],
    },
)

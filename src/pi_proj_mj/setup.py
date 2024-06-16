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
    maintainer='rivermeja',
    maintainer_email='rivermeja@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            ###################meja#########################
            'pub_hall = pi_proj_mj.pub_hall:main',
            'sub_hall = pi_proj_mj.sub_hall:main',
            'sub2_hall = pi_proj_mj.sub2_hall:main',
            #############joystick########################
            'joy_controll = pi_proj_mj.joy_control_meja:main',
            'joy_test = pi_proj_mj.joy_test:main',
            ###########mode#############################
            'Middle = pi_proj_mj.pub_hall_mode_select:main',
            'Mode = pi_proj_mj.mode_control_meja:main',
            #########################
            'ModeA = pi_proj_mj.ModeA:main',
            'ModeB = pi_proj_mj.ModeB:main',
            ################JOY_STICK###############
            'JOYPUB = pi_proj_mj.joy_pub_testmodel:main',
            'JOYSUB = pi_proj_mj.joy_sub_testmodel:main',
            ################TEST_CAR#################
            'Testcar_pub = pi_proj_mj.Testcar_pub:main',
            'Testcar_sub = pi_proj_mj.Testcar_sub:main',
            'Testcar_pub_mode = pi_proj_mj.Testcar_pub_mode:main',
            'Testcar_sub_mode = pi_proj_mj.Testcar_sub_mode:main'        
        ],
    },
)

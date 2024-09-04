from setuptools import find_packages, setup

package_name = 'parabellum_gukbang'

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
            # ver2 autumn
            'aruco_detection = parabellum_gukbang.ver2.autumn.aruco_detection:main',
            'autumn_col_checker = parabellum_gukbang.ver2.autumn.autumn_col_checker:main',
            'center_aruco_detection = parabellum_gukbang.ver2.autumn.center_aruco_detection:main',

            # ver2 common
            'center_display_dep = parabellum_gukbang.ver2.common.center_display_dep:main',
            'center_display_dep_side = parabellum_gukbang.ver2.common.center_display_dep_side:main',
            'center_joy_drive = parabellum_gukbang.ver2.common.center_joy_drive:main',
            'center_joy_drive_sub_cam = parabellum_gukbang.ver2.common.center_joy_drive_sub_cam:main',
            'imu_node = parabellum_gukbang.ver2.common.imu_node:main',
            'joy_drive = parabellum_gukbang.ver2.common.joy_drive:main',
            'joy_drive_sub_cam = parabellum_gukbang.ver2.common.joy_drive_sub_cam:main',
            'joy_remapper = parabellum_gukbang.ver2.common.joy_remapper:main',
            'odrive = parabellum_gukbang.ver2.common.odrive:main',

            # ver2 spring
            'army_detection = parabellum_gukbang.ver2.spring.army_detection:main',
            'center_army_detection = parabellum_gukbang.ver2.spring.center_army_detection:main',
            'spring_col_checker = parabellum_gukbang.ver2.spring.spring_col_checker:main',

            # ver2 summer
            'center_summer_joy_drive = parabellum_gukbang.ver2.summer.center_summer_joy_drive:main',
            'summer_joy_drive = parabellum_gukbang.ver2.summer.summer_joy_drive:main',
            'summer_track_checker = parabellum_gukbang.ver2.summer.summer_track_checker:main',

            # ver2 winter
            'winter_coldep_checker = parabellum_gukbang.ver2.winter.winter_coldep_checker:main',

            # ver3 autumn
            'aruco_detection_ver3 = parabellum_gukbang.ver3.autumn.aruco_detection:main',
            'autumn_col_checker_ver3 = parabellum_gukbang.ver3.autumn.autumn_col_checker:main',
            'center_aruco_detection_ver3 = parabellum_gukbang.ver3.autumn.center_aruco_detection:main',

            # ver3 common
            'center_display_dep_ver3 = parabellum_gukbang.ver3.common.center_display_dep:main',
            'center_display_dep_side_ver3 = parabellum_gukbang.ver3.common.center_display_dep_side:main',
            'center_joy_drive_ver3 = parabellum_gukbang.ver3.common.center_joy_drive:main',
            'center_joy_drive_sub_cam_ver3 = parabellum_gukbang.ver3.common.center_joy_drive_sub_cam:main',
            'imu_node_ver3 = parabellum_gukbang.ver3.common.imu_node:main',
            'joy_drive_ver3 = parabellum_gukbang.ver3.common.joy_drive:main',
            'joy_drive_sub_cam_ver3 = parabellum_gukbang.ver3.common.joy_drive_sub_cam:main',
            'joy_remapper_ver3 = parabellum_gukbang.ver3.common.joy_remapper:main',
            'odrive_ver3 = parabellum_gukbang.ver3.common.odrive:main',

            # ver3 spring
            'army_detection_ver3 = parabellum_gukbang.ver3.spring.army_detection:main',
            'center_army_detection_ver3 = parabellum_gukbang.ver3.spring.center_army_detection:main',
            'spring_col_checker_ver3 = parabellum_gukbang.ver3.spring.spring_col_checker:main',

            # ver3 summer
            'center_summer_joy_drive_ver3 = parabellum_gukbang.ver3.summer.center_summer_joy_drive:main',
            'summer_joy_drive_ver3 = parabellum_gukbang.ver3.summer.summer_joy_drive:main',
            'summer_track_checker_ver3 = parabellum_gukbang.ver3.summer.summer_track_checker:main',

            # ver3 winter
            'winter_coldep_checker_ver3 = parabellum_gukbang.ver3.winter.winter_coldep_checker:main',
        ],
    },
)

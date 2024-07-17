from setuptools import find_packages, setup

package_name = 'track_test_pkg'

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
            'blue_ratio=track_test_pkg.usable_code.blue_ratio:main',
            'testcar_sub=track_test_pkg.usable_code.testcar_sub_mode:main',
            'testcar_img_indi=track_test_pkg.usable_code.testcar_img_indicater:main',
            
            'blue_ratio_imu=track_test_pkg.usable_code.blue_ratio_imu:main',
            'blue_ratio_imu_nonMulti=track_test_pkg.blue_ratio_imu_nonMulti:main',
            
            
            'imu_pub=track_test_pkg.sensor.imu_pub:main',
            
        ],
    },
)

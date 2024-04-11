from setuptools import find_packages, setup

package_name = 'image_processing'

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
    maintainer='kangim',
    maintainer_email='tls3162@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection=image_processing.hand_detector:main',
            'commander=image_processing.commander:main',
            'img_catch=image_processing.image_catcher:main',
            'img_indi=image_processing.image_indicater:main',
            'depth_cap=image_processing.depth_capture:main',
            'depth_indi=image_processing.depth_indicater:main',
            
            'test_cap=image_processing.test_capture:main',
            'test_indi=image_processing.test_indicator:main',
            
            'align_cap=image_processing.align_capture:main',
            'align_indi=image_processing.align_indi:main',
            
            'qos_align_cap=image_processing.dir_test.qos_align_cap:main',
            'qos_align_indi=image_processing.dir_test.qos_align_indi:main',
            'qos_test_cap=image_processing.dir_test.qos_test_cap:main',
            'qos_test_cap1=image_processing.dir_test.qos_test_cap1:main',
            'qos_test_indi=image_processing.dir_test.qos_test_indi:main',
            
            'person_dis_pub=image_processing.yolo_depth.person_distance_cap:main',
            'person_dis_sub=image_processing.yolo_depth.person_distance_indi:main',

        ],
    },
)

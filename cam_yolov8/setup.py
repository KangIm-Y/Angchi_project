from setuptools import find_packages, setup

package_name = 'cam_yolov8'

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
    maintainer_email='yunmo0501@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_cam_yolo = cam_yolov8.pub_cam_yolo:main',
            'sub_cam_yolo = cam_yolov8.sub_cam_yolo:main',
        ],
    },
)

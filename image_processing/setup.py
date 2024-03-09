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

        ],
    },
)

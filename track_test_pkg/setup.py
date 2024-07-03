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
    maintainer_email='=',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blue_ratio=track_test_pkg.blue_ratio:main',
            'testcar_sub=track_test_pkg.testcar_sub_mode:main',
            'testcar_img_indi=track_test_pkg.testcar_img_indicater:main',
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'jongsul'

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
            "joy_pub_testmodel=jongsul.joy_pub_testmodel:main",
            "joy_sub_testmodel=jongsul.joy_sub_testmodel:main",
            "auto_person_tracking=jongsul.auto_person_tracking:main",
        ],
    },
)

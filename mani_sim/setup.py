from setuptools import find_packages, setup

package_name = 'mani_sim'

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
            'pub_helloworld = mani_sim.pub_helloworld:main',
            'sub2RS485 = mani_sim.sub2RS485:main',
        ],
    },
)

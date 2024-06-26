from setuptools import find_packages, setup

package_name = 'pi_proj_kh'

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
            'motor = pi_proj_kh.motor:main',
            'cli_input = pi_proj_kh.cli_input:main',
            'aruco_pub = pi_proj_kh.aruco_pub:main',
            'lane_detector = pi_proj_kh.lane_detector:main',
        ],
    },
)

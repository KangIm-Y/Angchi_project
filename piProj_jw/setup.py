from setuptools import find_packages, setup

package_name = 'piProj_jw'

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
    maintainer='junwoo',
    maintainer_email='dlawnsdn0525@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frame_pub = piProj_jw.webcam_pub:main',
            'frame_sub = piProj_jw.webcam_sub:main',
        ],
    },
)

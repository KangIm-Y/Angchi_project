from setuptools import find_packages, setup

package_name = 'pi_proj_mj'

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
    maintainer='meja',
    maintainer_email='mejariver@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [


	     'pubt = pi_proj_mj.pubt:main',
	     'subF = pi_proj_mj.subF:main',
	     'sub3 = pi_proj_mj.subf:main',
	     'Aruco_kh = pi_proj_mj.Aruco_kh:main',
	     'Aruco_js = pi_proj_mj.Aruco_js1:main',
	     'Aruco_js2 = pi_proj_mj.Aruco_js2:main',	    
	          
        ],
    },
)

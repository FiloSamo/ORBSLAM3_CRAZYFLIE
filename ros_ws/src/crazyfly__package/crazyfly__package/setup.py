from setuptools import find_packages, setup

package_name = 'crazyfly__package'

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
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crazyfly_imu = crazyfly__package.crazyflie_imu:main',
            'crazyfly_img = crazyfly__package.crazyflie_img:main',
        ],
    },
)

from setuptools import setup

package_name = 'ros2_mpu6050'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Darren Poulson',
    maintainer_email='darren.poulson@gmail.com',
    description='MPU6050 over i2c',
    license='GNU GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpu6050 = ros2_mpu6050.publisher_mpu_6050:main',
        ],
    },
)

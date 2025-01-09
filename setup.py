from setuptools import find_packages, setup

package_name = 'icp_lidar_odometry'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='LiDAR odometry using ICP for SLAM pipelines.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_odometry = icp_lidar_odometry.lidar_odometry_pointcloud:main',
        ],
    },
)

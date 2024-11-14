from setuptools import setup

package_name = 'orne_box_lidar_with_mirror'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryusei-Baba',
    maintainer_email='babaryusei.kw@gmail.com',
    description='LiDAR-based anomaly detection using autoencoder with ROS 2 and PyTorch',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_with_mirror_node = orne_box_lidar_with_mirror.lidar_with_mirror_node:main'
        ],
    },
)

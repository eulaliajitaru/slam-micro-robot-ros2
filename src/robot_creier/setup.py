from setuptools import setup

package_name = 'robot_creier'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eulalia Jitaru',
    maintainer_email='eulaliajitaru15@gmail.com',
    description='Robot nodes: obstacle avoidance, motor driver, odometry/TF helpers',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_miscare = robot_creier.control_miscare:main',
            'odom_to_tf = robot_creier.odom_to_tf:main',
            'scan_frame_fix = robot_creier.scan_frame_fix:main',
            'evitare_obstacole = robot_creier.evitare_obstacole:main',
            'hardware_interface = robot_creier.hardware_interface:main',
        ],
    },
)

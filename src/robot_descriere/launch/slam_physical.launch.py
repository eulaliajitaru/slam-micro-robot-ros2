import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_descriere')
    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox_physical.yaml')

    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_core.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
        output='screen'
    )

    slam = TimerAction(period=2.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'
        )]),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'false',
        }.items()
    )])

    rviz = TimerAction(period=3.0, actions=[Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )])

    return LaunchDescription([rsp, slam, rviz])

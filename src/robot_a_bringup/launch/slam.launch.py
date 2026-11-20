"""로봇 A SLAM 지도 만들기 (실행 흐름 ①).

slam_toolbox(turtlebot4_navigation/slam.launch.py)와 RViz를 한 번에 띄운다.
지도 저장은 별도 터미널에서 map_saver_node를 실행해 Enter 키로 한다.

    ros2 launch robot_a_bringup slam.launch.py namespace:=/robot<n>
    ros2 launch robot_a_bringup slam.launch.py namespace:=/robot<n> rviz:=false
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

pkg_tb4_nav = get_package_share_directory('turtlebot4_navigation')
pkg_tb4_viz = get_package_share_directory('turtlebot4_viz')

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='로봇 네임스페이스 (예: /robot3). 시뮬레이션은 비워 둠'),
    DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
    DeclareLaunchArgument('sync', default_value='true', choices=['true', 'false'],
                          description='true: 동기 SLAM(스캔을 빠짐없이 처리), '
                                      'false: 비동기(노트북이 느릴 때)'),
    DeclareLaunchArgument('params',
                          default_value=PathJoinSubstitution([pkg_tb4_nav, 'config', 'slam.yaml']),
                          description='slam_toolbox 파라미터 파일'),
    DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
    DeclareLaunchArgument('saver', default_value='false', choices=['true', 'false'],
                          description='true면 map_saver_node도 함께 실행 (저장은 서비스 호출로)'),
    DeclareLaunchArgument('map_name', default_value='farm_map'),
]


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_nav, 'launch', 'slam.launch.py'])),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time),
            ('sync', LaunchConfiguration('sync')),
            ('params', LaunchConfiguration('params')),
        ],
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_viz, 'launch', 'view_navigation.launch.py'])),
        launch_arguments=[('namespace', namespace), ('use_sim_time', use_sim_time)],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    saver = Node(
        package='robot_a_mapping',
        executable='map_saver_node',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_name': LaunchConfiguration('map_name'),
            'keyboard': False,
        }],
        condition=IfCondition(LaunchConfiguration('saver')),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(LogInfo(msg=['[robot_a] SLAM 시작 · namespace=', namespace,
                               ' · 다른 터미널에서 teleop으로 천천히 주행하세요']))
    ld.add_action(slam)
    ld.add_action(rviz)
    ld.add_action(saver)
    return ld

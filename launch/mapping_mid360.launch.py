import os

from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare the RViz argument
    mid360_dev = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('livox_ros_driver2'), 'launch'),
            '/msg_MID360_launch.py'])
    )



     # 参数声明
    slam_arg = DeclareLaunchArgument( 
        'slam', 
        default_value='true',
        description='Enable/disable SLAM mode'
    )



    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Flag to launch RViz.')
    

    # Node parameters, including those from the YAML configuration file
    laser_mapping_params = [
        PathJoinSubstitution([
            FindPackageShare('lio'),
            'config', 'avia.yaml'
        ]),
    ]

    # Node definition for laserMapping with Point-LIO
    laser_mapping_node = Node(
        package='lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=laser_mapping_params,
    )

    # Conditional RViz node launch
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg', 'loam_livox.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice'
    )


    tf2_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='aft_to_base_broadcaster',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0.002', '--pitch', '0.146', '--roll', '0.022',
            '--frame-id', 'aft_mapped', '--child-frame-id', "base_link"
        ]
    )

    # Assemble the launch description
    ld = LaunchDescription([

        slam_arg,  # 声明参数
        # rviz_arg,
        # mid360_dev,
        laser_mapping_node,
        tf2_base_link
        # GroupAction(
        #     actions=[rviz_node],
        #     condition=IfCondition(LaunchConfiguration('rviz'))
        # ),
    ])

    return ld

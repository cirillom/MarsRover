from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = '' # your config file path

    return LaunchDescription([
        Node(namespace='rslidar_sdk', package='rslidar_sdk', executable='rslidar_sdk_node', output='screen', parameters=[{'config_path': config_file}]),
        # Static transform base_link -> lidar_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_tf',
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'rslidar']
        ),

        #  PointCloud2 -> LaserScan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pcl_to_scan',
            output='screen',
            parameters=[{
                'target_frame': 'rslidar',
                'min_height': -0.05,
                'max_height': 0.2
            }],
            remappings=[
                ('cloud_in', '/rslidar_points'),
                ('scan', '/scan')
            ]
        ),

        #  SLAM Toolbox
        TimerAction(
           period=2.0,
           actions=[
              Node(
                 package='slam_toolbox',
                 executable='async_slam_toolbox_node',
                 name='slam_toolbox',
                 output='screen',
                 parameters=[{
                    'use_sim_time': False,
                    'odom_topic': '/odom',
                    'scan_topic': '/scan',
                    'base_frame': 'base_link',
                    'odom_frame': 'odom',
                    'map_frame': 'map',
                   'tf_buffer_duration': 10.0
                 }]
              )
          ]
        )                                                                                                                    [ Read 57 lines ]
    ])

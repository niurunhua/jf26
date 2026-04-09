from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 参数声明
    livox_frame_arg = DeclareLaunchArgument(
        'livox_frame',
        default_value='livox_frame',
        description='Livox LiDAR frame ID'
    )

    base_link_arg = DeclareLaunchArgument(
        'base_link',
        default_value='base_link',
        description='Robot base link frame ID'
    )

    # 1. 启动 Livox MID-360 驱动
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch_ROS2',
                'msg_MID360_launch.py'  # Livox MID-360 启动文件
            ])
        ]),
        launch_arguments={
            'xfer_format': '1',  # 0-pointcloud2, 1-custom pointcloud, 2-both
            'multi_topic': '0',  # 0-all lidars share one topic, 1-one topic per lidar
            'data_src': '0',     # 0-lidar, 1-rosbag, 2-pcap
            'publish_freq': '10.0',
            'output_type': '0',
            'frame_id': LaunchConfiguration('livox_frame'),
            'lvx_file_path': '',  # 留空，不从文件读取
        }.items()
    )

    # 2. 启动 FAST-LIO
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('fast_lio'),
                'config',
                'mid360.yaml'  # FAST-LIO 配置文件
            ])
        ],
        remappings=[
            ('/cloud_registered', '/jf/cloud_registered'),
            ('/Odometry', '/jf/odometry'),
            # 如果FAST-LIO使用不同的话题名，可能需要更多remap
            ('/imu', '/jf/imu'),  # 如果需要IMU数据
        ]
    )

    # 3. TF 静态变换：base_link 到 livox_frame（根据实际安装调整）
    # Livox MID-360 通常安装在机器人中心上方
    static_base_to_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.0', '0.0', '0.15',  # x, y, z 偏移（米）
            '0.0', '0.0', '0.0',   # roll, pitch, yaw（弧度）
            LaunchConfiguration('base_link'),
            LaunchConfiguration('livox_frame')
        ],
        output='screen'
    )

    # 4. TF 桥接：静态的 map->odom 变换（初始位置）
    # 注：实际中可能需要动态的 map->odom 变换
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
        output='screen'
    )

    # 5. 点云滤波节点，滤除车体自身点云 - 针对3D地形优化
    filter_node = Node(
        package='cpp_lidar_filter',
        executable='lidar_filter_node',
        name='lidar_filter_node',
        output='screen',
        parameters=[{
            'input_topic': '/jf/cloud_registered',           # FAST-LIO配准后的点云
            'output_topic': '/livox/lidar_filtered',         # 统一话题名称，供Nav2使用
            'min_x': -0.4,                                   # 扩大滤波范围以完全覆盖车体
            'max_x': 0.4,
            'min_y': -0.4,
            'max_y': 0.4,
            'min_z': -0.2,                                   # 降低最小Z以检测坑洞
            'max_z': 0.6,                                    # 提高最大Z以检测阶梯
            'negative': True,                                # 移除框内的点（车体自身）
            'leaf_size': 0.05,                               # 体素滤波尺寸
        }]
    )

    return LaunchDescription([
        livox_frame_arg,
        base_link_arg,
        livox_launch,
        static_base_to_livox,
        fast_lio_node,
        static_map_to_odom,
        filter_node,
    ])
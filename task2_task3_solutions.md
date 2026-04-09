# 任务二：引入 FAST-LIO 替代 slam_toolbox 的完整 ROS2 部署代码

## 1. FAST-LIO 社区移植版克隆命令

推荐使用 `tech-shrimp/fast_lio_ros2` 仓库，该仓库已适配 ROS2 Humble 和 Livox MID-360。

```bash
cd ~/your_workspace/src  # 进入你的 ROS2 工作空间 src 目录
git clone https://github.com/tech-shrimp/fast_lio_ros2.git
cd ..
# 安装依赖（根据 fast_lio_ros2 的 README）
sudo apt install -y ros-humble-livox-ros-driver2 ros-humble-pcl-ros ros-humble-tf2-eigen
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select fast_lio_ros2
```

## 2. 完整的 `jf_mapping.launch.py` 文件代码

创建文件 `~/your_workspace/src/jf_navigation/launch/jf_mapping.launch.py`：

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 启动 Livox MID-360 驱动
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch',
                'mid360.launch.py'
            ])
        ]),
        launch_arguments={
            'xfer_format': '1',  # 0-pointcloud2, 1-custom pointcloud, 2-both
            'multi_topic': '0',  # 0-all lidars share one topic, 1-one topic per lidar
            'data_src': '0',     # 0-lidar, 1-rosbag, 2-pcap
            'publish_freq': '10.0',
            'output_type': '0',
            'rviz_enable': 'false',
            'frame_id': 'livox_frame',
        }.items()
    )

    # 2. 启动 FAST-LIO
    fast_lio_node = Node(
        package='fast_lio_ros2',
        executable='fast_lio_ros2_node',
        name='fast_lio',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('fast_lio_ros2'),
                'config',
                'mid360.yaml'  # 需要根据你的传感器创建此配置文件
            ])
        ],
        remappings=[
            ('/cloud_registered', '/jf/cloud_registered'),  # 可选：重命名话题
            ('/Odometry', '/jf/odometry'),                 # 可选：重命名里程计话题
        ]
    )

    # 3. TF 静态变换：base_link 到 livox_frame（根据实际安装调整）
    static_base_to_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.15', '0.0', '0.0', '0.0', 'base_link', 'livox_frame'],
        output='screen'
    )

    # 4. TF 桥接：FAST-LIO 输出 map->base_link，但 Nav2 需要 map->odom->base_link
    # 假设 FAST-LIO 发布 map->base_link 的 TF，我们需要将其拆分为 map->odom 和 odom->base_link
    # 使用 odom_to_map 转换节点（可选，如果 FAST-LIO 直接输出 odom->base_link 则不需要）
    # 这里提供一个静态的 map->odom 变换（如果地图原点固定）
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
        output='screen'
    )

    # 5. 如果 FAST-LIO 输出 odom->base_link，则不需要上述静态变换，而是需要发布 map->odom
    # 可以使用 robot_localization 的 navsat_transform_node 或自定义节点
    # 这里提供一个简单的 TF 重映射节点（示例，需要自定义）
    # tf_remap_node = Node(
    #     package='tf2_ros',
    #     executable='tf_remap',
    #     name='tf_remap',
    #     parameters=[{
    #         'mappings': [
    #             {
    #                 'from_frame': 'map',
    #                 'to_frame': 'odom',
    #                 'source_frame': 'base_link',
    #                 'target_frame': 'base_link'
    #             }
    #         ]
    #     }]
    # )

    return LaunchDescription([
        livox_launch,
        static_base_to_livox,
        fast_lio_node,
        static_map_to_odom,  # 如果 FAST-LIO 输出 odom->base_link，注释掉这一行
        # tf_remap_node,
    ])
```

## 3. FAST-LIO 的 TF 配置方案

### 情况 A：FAST-LIO 输出 `map` -> `base_link` 的 TF
这是 FAST-LIO 的默认行为（构建全局地图）。Nav2 期望 `map` -> `odom` -> `base_link` 树。解决方案：

1. **发布静态的 `odom` -> `base_link` 变换**（实际上不需要，因为 FAST-LIO 已经提供 `map`->`base_link`）：
   ```bash
   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
   ```
   这样 TF 树变为：`map` (static) -> `odom` -> `base_link`（来自 FAST-LIO 的 `map`->`base_link` 将被覆盖？）
   实际上，我们需要将 FAST-LIO 的 `map`->`base_link` 拆分为 `map`->`odom` + `odom`->`base_link`。

2. **更好的方案**：使用 `tf2_ros` 的 `transform_publisher` 节点动态计算 `map`->`odom` 变换：
   ```python
   # 在 launch 文件中添加
   map_to_odom_node = Node(
       package='tf2_ros',
       executable='static_transform_publisher',
       arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
       output='screen'
   )
   ```
   然后配置 FAST-LIO 使其发布 `odom`->`base_link` 而不是 `map`->`base_link`。

### 情况 B：配置 FAST-LIO 输出 `odom` -> `base_link`
在 FAST-LIO 的配置文件 `mid360.yaml` 中设置：
```yaml
# 发布 odom 到 base_link 的 TF
publish_odometry: true
publish_tf: true
tf_frame: "odom"
child_frame: "base_link"
```

然后添加静态的 `map` -> `odom` 变换（如果地图原点固定）：
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

### 推荐的 remap 话题：
- FAST-LIO 输出的点云：`/cloud_registered` -> `/jf/cloud_registered`（在 launch 中 remap）
- FAST-LIO 输出的里程计：`/Odometry` -> `/jf/odometry`
- 确保 Nav2 的 `odom_topic` 参数设置为 `/jf/odometry`

### 完整的 TF 树应为：
```
map (static) -> odom (来自 FAST-LIO 或静态) -> base_link
                                     ^
                                     |
                              (FAST-LIO 发布 odom->base_link 或 map->base_link)
```

建议采用 **情况 B**，因为 Nav2 更自然地处理 `odom`->`base_link` 的里程计。

# 任务三：针对 ROBOCON 3D 地形的 Nav2 确切 YAML 改造代码

## 1. 局部代价地图 (Local Costmap) 配置

修改 `local_costmap` 部分的 `stvl_voxel_layer` 插件配置，使其直接接收 FAST-LIO 滤除车体后的 3D 点云。

### 关键修改点：

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      # ... 其他参数保持不变 ...
      plugins: ["static_layer", "voxel_layer", "inflation_layer"]  # 将 stvl_voxel_layer 改为 voxel_layer 或保持
      
      # 移除 pointcloud_to_laserscan 的引用，直接使用 voxel_layer
      voxel_layer:
        plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
        enabled: true
        voxel_decay: 1.0          # 体素衰减时间（秒），线性衰减
        decay_model: 0            # 0=线性, 1=指数, 2=持久
        voxel_size: 0.05          # 体素大小（米），适用于阶梯地形
        track_unknown_space: true
        observation_persistence: 0.0
        max_obstacle_height: 2.5  # 最大障碍物高度，涵盖阶梯和坡道
        mark_threshold: 0         # 体素高度阈值，0 表示任何占据都标记为障碍
        update_footprint_enabled: true
        combination_method: 1     # 1=max（取最大值），0=override
        obstacle_range: 5.0       # 障碍物检测范围（米）
        origin_z: -0.5            # 体素网格的 Z 轴原点，负值以检测低于机器人的地形
        publish_voxel_map: false
        transform_tolerance: 0.2
        mapping_mode: false
        map_save_duration: 60.0
        
        # 观察源配置：使用 FAST-LIO 滤除车体后的点云
        observation_sources: fast_lio_source
        
        fast_lio_source:
          data_type: PointCloud2
          topic: /jf/cloud_registered  # FAST-LIO 输出的已注册点云
          # 如果需要滤除车体，可以添加预处理节点或使用带 ROI 滤波的 topic
          # topic: /jf/cloud_filtered  # 滤除车体后的点云
          transport_type: "raw"
          marking: true
          clearing: true
          obstacle_range: 8.0          # 障碍物检测范围
          raytrace_range: 9.0          # 射线追踪清除范围
          
          # 针对 3D 地形的关键参数
          min_obstacle_height: -0.3    # 负值以检测低于机器人的障碍物（下坡、坑洞）
          max_obstacle_height: 2.0     # 最大值以检测阶梯、上坡
          
          expected_update_rate: 20.0   # 期望更新频率（Hz）
          observation_persistence: 0.2 # 观察持久性（秒），平滑传感器数据
          inf_is_valid: false
          clear_after_reading: true
          filter: "voxel"              # 体素滤波降采样
          voxel_min_points: 0
          model_type: 1                # 1=Lidar（激光雷达模型）
          
          # 视场角配置（Livox MID-360 为 360°x70°）
          vertical_fov_angle: 1.22     # 70° ≈ 1.22 rad
          vertical_fov_offset: -0.61   # 偏移使 FOV 对称（-35° to +35°）
          horizontal_fov_angle: 6.28   # 360° ≈ 6.28 rad
```

### 额外建议：添加预处理节点滤除车体点云

创建过滤节点，移除属于车体自身的点云（例如，半径 0.3 米内的点）：

```cpp
// 示例：使用 PCL 的 CropBox 或 RadiusOutlierRemoval
// 发布到 /jf/cloud_filtered
```

然后在 launch 文件中添加该节点，并将 voxel_layer 的 topic 改为 `/jf/cloud_filtered`。

## 2. 全局地图 (Global Map) 配置

纯 `.pgm` 2D 地图无法处理阶梯地形。推荐引入 **OctoMap** 作为 3D 全局地图表示，并投影到 2D 供 Nav2 使用。

### 方案：使用 `octomap_server2`（ROS2 Humble）

```bash
# 安装 octomap_server2
sudo apt install ros-humble-octomap-server2
```

### 全局代价地图 YAML 配置：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      # ... 基本参数保持不变 ...
      plugins: ["octomap_layer", "inflation_layer"]  # 移除 static_layer，使用 octomap_layer
      
      octomap_layer:
        plugin: "octomap_server2::OctomapLayer"
        enabled: true
        octomap_topic: "/octomap_full"  # octomap_server 发布的完整 octomap
        max_obstacle_height: 2.5        # 最大障碍物高度
        min_obstacle_height: -0.3       # 最小障碍物高度（负值检测坑洞）
        origin_z: 0.0
        resolution: 0.05                # 2D 投影的分辨率
        projection_mode: 0              # 0=最大高度，1=概率求和，2=计数
        occupied_threshold: 0.7         # 占据概率阈值
        filter_ground: false            # 不平坦地形不过滤地面
        ground_filter_distance: 0.04    # 如果启用地面过滤的距离
        ground_filter_angle: 0.15       # 地面滤波角度（弧度）
        ground_filter_plane_distance: 0.07
        transform_tolerance: 0.2
        observation_sources: fast_lio_global_source
        
        fast_lio_global_source:
          data_type: PointCloud2
          topic: /jf/cloud_registered
          marking: true
          clearing: true
          obstacle_range: 15.0           # 全局地图需要更大范围
          raytrace_range: 16.0
          min_obstacle_height: -0.3
          max_obstacle_height: 2.5
          expected_update_rate: 5.0      # 全局地图更新可以慢一些
          observation_persistence: 1.0   # 持久性更长
          filter: "voxel"
          voxel_size: 0.1                # 全局地图体素可以更大
          
      inflation_layer:
        # ... 保持原有配置 ...
```

### 启动 octomap_server 的 launch 配置：

在 mapping launch 文件中添加：

```python
octomap_server_node = Node(
    package='octomap_server2',
    executable='octomap_server_node',
    name='octomap_server',
    output='screen',
    parameters=[{
        'frame_id': 'map',
        'resolution': 0.05,
        'sensor_model.max_range': 15.0,
        'height_map': False,           # 不生成高度图
        'color_map': False,
        'filter_ground': False,        # 不平坦地形不过滤地面
        'ground_filter.distance': 0.04,
        'ground_filter.angle': 0.15,
        'ground_filter.plane_distance': 0.07,
        'pointcloud_min_z': -0.5,      # 包含负高度点
        'pointcloud_max_z': 2.5,
        'occupancy_min_z': -0.3,
        'occupancy_max_z': 2.0,
        'compress_map': True,
        'incremental_2D_projection': True,
    }],
    remappings=[
        ('cloud_in', '/jf/cloud_registered'),
    ]
)
```

### 替代方案：使用 `grid_map` 生成 2.5D 高程图

如果需要更精细的地形高程信息，可以使用 `grid_map` 包：

```bash
sudo apt install ros-humble-grid-map ros-humble-grid-map-costmap-2d
```

YAML 配置示例：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["grid_map_layer", "inflation_layer"]
      
      grid_map_layer:
        plugin: "grid_map_costmap_2d::GridMapLayer"
        grid_map_topic: "/elevation_map"
        layer_name: "elevation"          # grid_map 中的图层名称
        min_height: -0.5
        max_height: 2.0
        height_threshold: 0.15           # 高度变化阈值，超过视为障碍
        slope_threshold: 0.3             # 坡度阈值（弧度）
        # ... 其他参数 ...
```

## 总结

1. **局部代价地图**：直接使用 `spatio_temporal_voxel_layer` 处理 3D 点云，调整 `min_obstacle_height` 和 `max_obstacle_height` 以涵盖阶梯和坡道。
2. **全局地图**：推荐使用 `octomap_server2` 构建 3D 占据地图，并通过 `OctomapLayer` 插件集成到 Nav2 的 global_costmap。
3. **预处理**：添加点云过滤节点，移除车体自身点云，避免自遮挡。
4. **TF 配置**：确保 FAST-LIO 输出 `odom->base_link`，并添加静态的 `map->odom` 变换。

这些修改将使 Nav2 能够处理 ROBOCON "武林探秘" 比赛中的梅花桩阶梯和上下坡道等 2.5D/3D 复杂地形。
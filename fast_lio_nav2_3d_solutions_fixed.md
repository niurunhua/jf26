# 修正版：任务二和三的完整解决方案

## 任务二：引入 FAST-LIO 替代 slam_toolbox 的完整 ROS2 部署代码

### 1. 可用的 FAST-LIO ROS2 仓库（多个选项）

**选项1：Ericsii/FAST_LIO_ROS2**（推荐）
```bash
cd ~/JF26/src
git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive
```

**选项2：se7oluti0n/FAST_LIO_ROS2**
```bash
cd ~/JF26/src
git clone https://github.com/se7oluti0n/FAST_LIO_ROS2.git
```

**选项3：从原版FAST-LIO的ROS2分支**
```bash
cd ~/JF26/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git checkout ROS2
```

### 2. 安装依赖和编译

```bash
cd ~/JF26
# 安装系统依赖
sudo apt install -y \
    ros-humble-livox-ros-driver2 \
    ros-humble-pcl-ros \
    ros-humble-tf2-eigen \
    libeigen3-dev \
    libpcl-dev

# 安装ROS包依赖
rosdep install --from-paths src --ignore-src -r -y

# 编译FAST-LIO（根据选择的仓库）
# 对于 Ericsii/FAST_LIO_ROS2：
colcon build --packages-select fast_lio_ros2

# 对于原版FAST-LIO ROS2分支：
colcon build --packages-select fast_lio
```

### 3. 完整的 `jf_mapping.launch.py` 文件代码

创建文件 `~/JF26/src/jf_-rm2026_-navigation/launch/jf_mapping.launch.py`：

```python
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
                'launch',
                'MID360.launch.py'  # 注意：MID360 不是 mid360
            ])
        ]),
        launch_arguments={
            'xfer_format': '1',  # 0-pointcloud2, 1-custom pointcloud, 2-both
            'multi_topic': '0',  # 0-all lidars share one topic, 1-one topic per lidar
            'data_src': '0',     # 0-lidar, 1-rosbag, 2-pcap
            'publish_freq': '10.0',
            'output_type': '0',
            'rviz_enable': 'false',
            'frame_id': LaunchConfiguration('livox_frame'),
            'lvx_file_path': 'path/to/your/lvx/file.lvx',  # 如果使用录制数据
        }.items()
    )

    # 2. 启动 FAST-LIO（根据不同仓库调整可执行文件名）
    fast_lio_node = Node(
        package='fast_lio_ros2',  # 或 'fast_lio'，根据实际包名
        executable='fast_lio_ros2_node',  # 或 'fast_lio_node'
        name='fast_lio',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('fast_lio_ros2'),  # 根据实际包名调整
                'config',
                'mid360.yaml'  # 需要创建此配置文件
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

    # 5. 可选的：点云预处理节点，滤除车体自身点云
    filter_node = Node(
        package='jf_-rm2026_-navigation',  # 需要创建此包
        executable='pointcloud_filter',
        name='pointcloud_filter',
        output='screen',
        parameters=[{
            'filter_radius': 0.3,  # 滤除半径（米）内的点
            'input_topic': '/jf/cloud_registered',
            'output_topic': '/jf/cloud_filtered',
            'frame_id': 'livox_frame',
        }]
    )

    return LaunchDescription([
        livox_frame_arg,
        base_link_arg,
        livox_launch,
        static_base_to_livox,
        fast_lio_node,
        static_map_to_odom,
        filter_node,  # 可选
    ])
```

### 4. FAST-LIO 配置文件示例 `mid360.yaml`

创建文件 `~/JF26/src/fast_lio_ros2/config/mid360.yaml`：

```yaml
common:
  lid_topic: "/livox/lidar"  # Livox驱动输出的点云话题
  imu_topic: "/jf/imu"       # IMU话题（如果需要）
  time_sync_en: false        # 是否启用硬件时间同步
  time_offset_lidar_to_imu: 0.0
  
  # 坐标系设置
  map_frame: "map"
  odometry_frame: "odom"
  lidar_frame: "livox_frame"
  imu_frame: "imu_link"
  
  # 发布设置
  publish_odometry: true
  publish_tf: true
  publish_point_cloud: true
  publish_path: false
  
  # 输出话题
  output_point_type: 0  # 0: PointXYZI, 1: PointXYZINormal

preprocess:
  lidar_type: 1        # 1: Livox
  point_filter_num: 1  # 降采样率
  feature_enabled: false
  max_range: 100.0
  min_range: 1.0

mapping:
  # 滤波器参数
  acc_cov: 0.1
  gyr_cov: 0.1
  b_acc_cov: 0.0001
  b_gyr_cov: 0.0001
  
  # 地图管理
  max_iteration: 4
  cube_len: 1000.0
  det_range: 300.0
  filter_size_surf: 0.5
  filter_size_map: 0.5
  
  # 外参（需要标定）
  extrinsic_T: [0.0, 0.0, 0.0]
  extrinsic_R: [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]

# Livox MID-360 特定参数
livox:
  # MID-360 参数
  blind: 0.1          # 盲区（米）
  scan_rate: 10       # 扫描频率（Hz）
  timestamp_unit: 1   # 时间戳单位：1=ns
```

### 5. TF 配置方案

**推荐配置**：FAST-LIO 输出 `odom` -> `base_link` 变换

1. **在 FAST-LIO 配置中设置**：
   ```yaml
   common:
     odometry_frame: "odom"
     lidar_frame: "livox_frame"
     publish_tf: true
   ```

2. **静态变换**：
   - `map` -> `odom`：初始位置（可通过SLAM更新）
   - `base_link` -> `livox_frame`：传感器安装位置

3. **完整的TF树**：
   ```
   map (static/dynamic) -> odom (FAST-LIO) -> base_link -> livox_frame
   ```

4. **话题重映射**：
   ```python
   remappings=[
       ('/cloud_registered', '/jf/cloud_registered'),
       ('/Odometry', '/jf/odometry'),
       ('/tf', '/jf/tf'),  # 如果需要隔离TF
   ]
   ```

## 任务三：针对 ROBOCON 3D 地形的 Nav2 确切 YAML 改造代码

### 1. 局部代价地图配置（直接使用 3D 点云）

修改 `~/JF26/src/jf_-rm2026_-navigation/params/singlenav2_params.yaml` 中的 `local_costmap` 部分：

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 20.0
      publish_frequency: 20.0
      global_frame: odom
      robot_base_frame: base_link_fake
      use_sim_time: false
      rolling_window: true
      width: 12.0  # 增大以处理3D地形
      height: 12.0
      resolution: 0.05
      robot_radius: 0.1
      footprint: "[ [0.1, 0.1], [0.1, -0.1], [-0.1, -0.1], [-0.1, 0.1] ]"
      
      # 插件配置：使用 voxel_layer 处理3D点云
      plugins: ["static_layer", "voxel_layer", "inflation_layer"]
      
      # 静态层（可选，如果使用全局地图）
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
        enabled: false  # 3D地形中可禁用，或使用octomap
        
      # 体素层 - 关键配置
      voxel_layer:
        plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
        enabled: true
        voxel_decay: 1.0          # 体素衰减时间（秒）
        decay_model: 0            # 0=线性衰减
        voxel_size: 0.05          # 体素大小（米）
        track_unknown_space: true
        observation_persistence: 0.2  # 观察持久性（秒）
        
        # 3D地形关键参数
        max_obstacle_height: 2.5      # 最大障碍物高度（涵盖阶梯）
        min_obstacle_height: -0.5     # 最小障碍物高度（负值检测坑洞）
        origin_z: -0.5                # 网格Z轴原点
        
        mark_threshold: 0             # 体素高度阈值
        update_footprint_enabled: true
        combination_method: 1         # 1=max（取最大值）
        obstacle_range: 8.0           # 障碍物检测范围
        raytrace_range: 9.0           # 射线追踪清除范围
        
        publish_voxel_map: false
        transform_tolerance: 0.2
        mapping_mode: false
        map_save_duration: 60.0
        
        # 观察源：FAST-LIO 点云（滤除车体后）
        observation_sources: fast_lio_filtered
        
        fast_lio_filtered:
          data_type: "PointCloud2"
          topic: "/jf/cloud_filtered"  # 滤除车体后的点云
          marking: true
          clearing: true
          obstacle_range: 8.0
          raytrace_range: 9.0
          
          # 3D地形滤波参数
          min_obstacle_height: -0.3    # 检测下坡、坑洞
          max_obstacle_height: 2.0     # 检测阶梯、上坡
          min_z: -0.5                  # 点云Z轴最小值
          max_z: 2.5                   # 点云Z轴最大值
          
          expected_update_rate: 20.0
          observation_persistence: 0.2
          inf_is_valid: false
          clear_after_reading: true
          filter: "voxel"              # 体素滤波降采样
          voxel_min_points: 0
          model_type: 1                # 1=激光雷达模型
          
          # Livox MID-360 视场角（360°×70°）
          vertical_fov_angle: 1.22     # 70° ≈ 1.22 rad
          vertical_fov_offset: -0.61   # -35° 偏移使FOV对称
          horizontal_fov_angle: 6.28   # 360° ≈ 6.28 rad
          
          # 可选：ROI滤波（滤除车体自身）
          roi_filter_enabled: true
          roi_min_x: -0.3
          roi_max_x: 0.3
          roi_min_y: -0.3
          roi_max_y: 0.3
          roi_min_z: -0.2
          roi_max_z: 0.5
      
      # 膨胀层
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0
        inflation_radius: 0.75  # 增大以适应3D地形
```

### 2. 点云滤波节点（滤除车体自身）

创建 `~/JF26/src/jf_-rm2026_-navigation/src/pointcloud_filter.cpp`：

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

class PointCloudFilter : public rclcpp::Node {
public:
    PointCloudFilter() : Node("pointcloud_filter") {
        // 参数
        this->declare_parameter("filter_radius", 0.3);
        this->declare_parameter("input_topic", "/jf/cloud_registered");
        this->declare_parameter("output_topic", "/jf/cloud_filtered");
        this->declare_parameter("frame_id", "livox_frame");
        
        double radius = this->get_parameter("filter_radius").as_double();
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        
        // 设置ROI滤波框（滤除车体自身）
        roi_min_ = Eigen::Vector4f(-radius, -radius, -0.2, 1.0);
        roi_max_ = Eigen::Vector4f(radius, radius, 0.5, 1.0);
        
        // 订阅和发布
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10,
            std::bind(&PointCloudFilter::cloudCallback, this, std::placeholders::_1));
            
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic, 10);
            
        RCLCPP_INFO(this->get_logger(), "PointCloud Filter initialized");
        RCLCPP_INFO(this->get_logger(), "Filter ROI: [%f, %f, %f] to [%f, %f, %f]",
                   roi_min_[0], roi_min_[1], roi_min_[2],
                   roi_max_[0], roi_max_[1], roi_max_[2]);
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 转换为PCL点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);
        
        // 应用ROI滤波（移除车体自身点云）
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::CropBox<pcl::PointXYZI> box_filter;
        box_filter.setMin(roi_min_);
        box_filter.setMax(roi_max_);
        box_filter.setNegative(true);  // 移除框内的点
        box_filter.setInputCloud(cloud);
        box_filter.filter(*filtered_cloud);
        
        // 转换回ROS消息
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_msg);
        filtered_msg.header = msg->header;
        filtered_msg.header.frame_id = frame_id_;
        
        // 发布
        publisher_->publish(filtered_msg);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    Eigen::Vector4f roi_min_, roi_max_;
    std::string frame_id_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFilter>());
    rclcpp::shutdown();
    return 0;
}
```

### 3. 全局地图配置（3D地图方案）

#### 方案A：使用 OctoMap（推荐）

```bash
# 安装 octomap_server2
sudo apt install ros-humble-octomap-server2
```

**全局代价地图配置**：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_link_fake
      update_frequency: 5.0  # 全局地图更新可以慢一些
      publish_frequency: 5.0
      width: 30.0
      height: 30.0
      resolution: 0.05
      rolling_window: false  # 全局地图不滚动
      plugins: ["octomap_layer", "inflation_layer"]
      
      # OctoMap 层
      octomap_layer:
        plugin: "octomap_server2::OctomapLayer"
        enabled: true
        octomap_topic: "/octomap_full"
        max_obstacle_height: 2.5
        min_obstacle_height: -0.3
        resolution: 0.05
        projection_mode: 0  # 0=最大高度投影
        occupied_threshold: 0.7
        filter_ground: false  # 不平坦地形不过滤地面
        
        # 可选：地面滤波参数（如果启用）
        ground_filter_distance: 0.04
        ground_filter_angle: 0.15
        ground_filter_plane_distance: 0.07
        
        transform_tolerance: 0.5
        observation_sources: fast_lio_global
        
        fast_lio_global:
          data_type: "PointCloud2"
          topic: "/jf/cloud_filtered"
          marking: true
          clearing: true
          obstacle_range: 15.0
          raytrace_range: 16.0
          min_obstacle_height: -0.3
          max_obstacle_height: 2.5
          expected_update_rate: 5.0
          observation_persistence: 1.0
          filter: "voxel"
          voxel_size: 0.1  # 全局地图可以使用更大的体素
      
      # 膨胀层
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0
        inflation_radius: 1.0  # 全局地图使用更大的膨胀半径
```

**启动 octomap_server**（在 mapping launch 中添加）：

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
        'sensor_model.hit': 0.7,
        'sensor_model.miss': 0.4,
        'sensor_model.min': 0.12,
        'sensor_model.max': 0.97,
        
        # 3D地形处理
        'height_map': False,
        'color_map': False,
        'filter_ground': False,  # 关键：不平坦地形不过滤地面
        
        # 点云滤波
        'pointcloud_min_z': -0.5,
        'pointcloud_max_z': 2.5,
        'occupancy_min_z': -0.3,
        'occupancy_max_z': 2.0,
        
        # 性能
        'compress_map': True,
        'incremental_2D_projection': True,
        
        # 发布设置
        'publish_2d_map': True,
        'publish_3d_map': True,
        'publish_free_space': False,
    }],
    remappings=[
        ('cloud_in', '/jf/cloud_filtered'),
        ('octomap_full', '/octomap_full'),
        ('projected_map', '/projected_map'),
    ]
)
```

#### 方案B：使用 Grid Map（2.5D高程图）

```bash
# 安装 grid_map
sudo apt install ros-humble-grid-map ros-humble-grid-map-costmap-2d
```

**Grid Map 配置**：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["grid_map_layer", "inflation_layer"]
      
      grid_map_layer:
        plugin: "grid_map_costmap_2d::GridMapLayer"
        grid_map_topic: "/elevation_map"
        layer_name: "elevation"  # grid_map 中的高程图层
        min_height: -0.5
        max_height: 2.0
        height_threshold: 0.15   # 高度变化阈值（米），超过视为障碍
        slope_threshold: 0.3     # 坡度阈值（弧度），超过视为障碍
        robot_radius: 0.1
        inflation_radius: 0.5
        unknown_cost_value: 255
        lethal_cost_value: 254
        transform_tolerance: 0.5
        
        # 高程图处理参数
        use_min_max_height: true
        normalize: false
        apply_smoothing: true
        smoothing_radius: 0.3
```

### 4. Nav2 其他配置调整

#### 全局规划器调整（处理3D地形）：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["Smac3d"]
    Smac3d:
      plugin: "nav2_smac_planner/SmacPlanner3D"
      tolerance: 0.5
      allow_unknown: true
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 5.0
      analytic_expansion_max_length: 3.0
      minimum_turning_radius: 0.05
      
      # 3D搜索参数
      search_info.cost_penalty: 2.0
      search_info.reverse_penalty: 1.0
      search_info.change_penalty: 0.0
      search_info.non_straight_penalty: 0.0
      
      # 高程处理
      use_elevation_map: true
      elevation_map_topic: "/elevation_map"
      max_elevation: 2.0
      min_elevation: -0.3
      elevation_threshold: 0.15
```

#### 控制器调整（3D地形速度限制）：

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      # 根据地形坡度调整速度
      terrain_adaptive_velocity: true
      max_slope: 0.3  # 最大允许坡度（弧度）
      
      # 坡度-速度映射
      slope_velocity_map: [
        {slope: 0.0, velocity: 2.5},
        {slope: 0.1, velocity: 1.5},
        {slope: 0.2, velocity: 0.8},
        {slope: 0.3, velocity: 0.3}
      ]
```

## 部署步骤总结

1. **克隆并编译 FAST-LIO**：
   ```bash
   cd ~/JF26/src
   git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive
   cd ~/JF26
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --packages-select fast_lio_ros2
   ```

2. **创建配置文件**：
   - `mid360.yaml` - FAST-LIO 配置文件
   - `jf_mapping.launch.py` - 启动文件
   - 更新 `singlenav2_params.yaml` - Nav2 配置

3. **实现点云滤波节点**：
   - 编译 `pointcloud_filter` 节点
   - 测试点云滤波效果

4. **安装并配置 3D 地图**：
   ```bash
   sudo apt install ros-humble-octomap-server2
   ```
   - 配置 octomap_layer
   - 启动 octomap_server

5. **测试 3D 导航**：
   - 启动 mapping: `ros2 launch jf_-rm2026_-navigation jf_mapping.launch.py`
   - 启动 Nav2: `ros2 launch nav2_bringup navigation_launch.py`
   - 测试阶梯和坡道地形

## 故障排除

1. **FAST-LIO 无法启动**：
   - 检查 Livox 驱动是否正确安装
   - 确认点云话题名称匹配
   - 检查 IMU 数据（如果使用）

2. **TF 树错误**：
   - 使用 `ros2 run tf2_tools view_frames` 检查 TF 树
   - 确保所有帧都正确连接

3. **3D 代价地图问题**：
   - 检查点云是否包含有效数据
   - 调整 `min_obstacle_height` 和 `max_obstacle_height`
   - 验证体素层是否正确订阅点云

4. **规划失败**：
   - 检查全局地图是否包含有效障碍信息
   - 调整规划器参数以适应 3D 地形
   - 确保机器人可以物理通过规划路径

## 参考资料

- [Ericsii/FAST_LIO_ROS2 GitHub Repository](https://github.com/Ericsii/FAST_LIO_ROS2)
- [OctoMap ROS2 Package](http://wiki.ros.org/octomap_server)
- [Nav2 Voxel Layer Documentation](https://navigation.ros.org/plugins/voxel_layer.html)
- [ROS2 Grid Map](https://github.com/ANYbotics/grid_map)
- [ROBOCON 2026 武林探秘比赛规则](https://www.robocon.org)
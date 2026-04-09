# FAST-LIO + Nav2 缝合系统 - Robocon 2026 终极指南

## 📁 文件结构概览

```
~/JF26/
├── src/
│   ├── FAST_LIO_ROS2/                    # FAST-LIO 源码
│   └── jf_-rm2026_-navigation/           # Nav2 导航包
│       └── src/
│           └── jf_bringup/
│               ├── launch/
│               │   ├── singlenav_launch.py               # 已修改，接入FAST-LIO
│               │   ├── pcd2pgm.launch.py                 # 新增：PCD转2D地图
│               │   └── singlenav_fastlio_launch.py       # 备份副本
│               ├── scripts/
│               │   └── pcd_to_gridmap.py                 # 新增：转换脚本
│               └── params/
│                   └── singlenav2_params.yaml            # 已更新：Voxel Layer参数
├── scans.pcd                              # 你的3D点云地图
└── FAST_LIO_NAV2_INTEGRATION_GUIDE.md    # 本指南
```

## 🚀 快速开始

### 第1步：安装依赖

```bash
# 安装 open3d（用于PCD转换）
pip install open3d

# 确保已安装所有ROS2包依赖
cd ~/JF26
rosdep install --from-paths src --ignore-src -r -y
```

### 第2步：编译工作空间

```bash
cd ~/JF26
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 第3步：转换3D点云为2D栅格地图（可选）

如果你需要2D地图用于可视化或备用导航：

```bash
# 方法1：使用launch文件
ros2 launch jf_bringup pcd2pgm.launch.py \
  pcd_file:=~/JF26/scans.pcd \
  output_dir:=~/JF26/maps \
  min_z:=0.4 \
  max_z:=1.2 \
  resolution:=0.05

# 方法2：直接运行脚本
python3 src/jf_-rm2026_-navigation/src/jf_bringup/scripts/pcd_to_gridmap.py \
  ~/JF26/scans.pcd \
  --output-dir ~/JF26/maps \
  --min-z 0.4 \
  --max-z 1.2 \
  --resolution 0.05
```

输出文件：`~/JF26/maps/scans.pgm` 和 `~/JF26/maps/scans.yaml`

### 第4步：启动完整缝合系统

**比赛现场启动顺序：**

```bash
# 终端1：雷达驱动
source ~/JF26/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 终端2：FAST-LIO 里程计 + 定位
source ~/JF26/install/setup.bash
ros2 launch jf_bringup singlenav_launch.py \
  use_sim_time:=false \
  fastlio_config:=mid360.yaml \
  fastlio_map_file:=~/JF26/scans.pcd

# 终端3：Nav2 导航（如果需要单独启动）
source ~/JF26/install/setup.bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=false \
  params_file:=src/jf_-rm2026_-navigation/src/jf_bringup/params/singlenav2_params.yaml \
  map:=~/JF26/maps/scans.yaml

# 终端4：RViz 可视化
source ~/JF26/install/setup.bash
rviz2 -d src/jf_-rm2026_-navigation/src/jf_bringup/rviz/jf_nav.rviz
```

### 第5步：验证系统状态

```bash
# 检查TF树
ros2 run tf2_tools view_frames

# 检查里程计话题
ros2 topic echo /fast_lio_odom

# 检查定位话题
ros2 topic echo /fast_lio_localization_odom

# 检查代价地图
ros2 topic echo /local_costmap/costmap
```

## 🔧 关键配置说明

### 1. FAST-LIO 双节点策略

| 节点 | 功能 | TF变换 | 输出话题 |
|------|------|--------|----------|
| `fast_lio_odom` | 纯里程计 | `odom → base_link` | `/fast_lio_odom` |
| `fast_lio_localization` | 3D定位 | `map → odom` | `/fast_lio_localization_odom` |

**TF树结构：**
```
map (来自FAST-LIO定位)
  ↓
odom (来自FAST-LIO定位)
  ↓
base_link (来自FAST-LIO里程计)
```

### 2. Voxel Layer 参数（防爬坡误检）

配置文件：`src/jf_-rm2026_-navigation/src/jf_bringup/params/singlenav2_params.yaml`

**关键修改：**
```yaml
# 梅林台阶过滤 (台阶最高0.35m)
min_obstacle_height: 0.4    # 过滤台阶底面
max_obstacle_height: 1.2    # 过滤过高障碍
min_z: 0.4                  # 点云高度过滤下限
max_z: 1.2                  # 点云高度过滤上限
origin_z: 0.0               # 体素网格起点
```

**效果：**
- 机器人爬坡/上台阶时，车身倾斜扫到的地面（z < 0.4m）**不会**计入代价地图
- 只有真实高度的障碍物（0.4m ≤ z ≤ 1.2m）才会被检测
- 防止Nav2因虚假障碍物而瘫痪

### 3. PCD转2D地图参数

**高度过滤逻辑：**
- `min_z=0.4`: 过滤掉台阶底面（梅林台阶最高0.35m）
- `max_z=1.2`: 过滤过高障碍，保留关键障碍物
- 输出2D地图只包含**绝对障碍物**，适合平面导航

## 🐛 故障排除

### 常见问题1：TF树不完整

**症状：** Nav2报错"Transform failed"或"Unable to obtain robot pose"

**解决：**
```bash
# 检查TF树
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link

# 如果缺少map->odom，检查FAST-LIO定位节点是否启动
ros2 node list | grep fast_lio
```

### 常见问题2：代价地图无更新

**症状：** 局部/全局代价地图显示空白或过时

**解决：**
```bash
# 检查点云话题
ros2 topic echo /livox/lidar_filtered --once | head -5
ros2 topic echo /cloud_registered_loc --once | head -5

# 检查Voxel Layer是否启用
ros2 param get /local_costmap/stvl_voxel_layer enabled
```

### 常见问题3：机器人无法移动

**症状：** MPPI控制器报"no valid trajectories"或"all trajectories invalid"

**解决：**
1. 检查代价地图是否有障碍物
2. 确认`min_obstacle_height`和`max_obstacle_height`设置正确
3. 检查雷达滤波器参数（车身ROI过滤）

## ⚙️ 高级调优

### 1. 调整FAST-LIO参数

编辑：`src/FAST_LIO_ROS2/config/mid360.yaml`

**关键参数：**
```yaml
mapping:
  acc_cov: 0.1        # 加速度计协方差
  gyr_cov: 0.1        # 陀螺仪协方差
  fov_degree: 360.0   # 视场角
  det_range: 100.0    # 探测范围
```

### 2. 调整MPPI控制器

编辑：`singlenav2_params.yaml` 中的 `FollowPath` 部分

**比赛场景建议：**
```yaml
FollowPath:
  approach_distance: 1.5      # 更早开始减速
  time_steps: 60              # 缩短预测时域
  vx_max: 2.0                 # 降低最大速度
  ax_min: -4.0                # 增强制动能力
```

### 3. 实时调整高度过滤

**应对不同地形：**
```bash
# 动态更新Voxel Layer参数
ros2 param set /local_costmap/stvl_voxel_layer min_obstacle_height 0.3
ros2 param set /local_costmap/stvl_voxel_layer max_obstacle_height 1.5
```

## 📋 比赛现场检查清单

### 赛前准备
- [ ] 雷达驱动正常 (`/livox/lidar`话题有数据)
- [ ] FAST-LIO里程计输出稳定 (`/fast_lio_odom`)
- [ ] FAST-LIO定位收敛 (`/fast_lio_localization_odom`)
- [ ] TF树完整 (`map → odom → base_link`)
- [ ] 代价地图正常更新（RViz中可见障碍物）
- [ ] MPPI控制器能生成轨迹

### 赛中监控
- [ ] 实时检查定位残差（FAST-LIO终端输出）
- [ ] 监控代价地图，确保无虚假障碍物
- [ ] 观察MPPI轨迹质量（RViz中`/mpc_trajectory`）
- [ ] 准备紧急停止命令：`ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'`

### 赛后调试
- [ ] 保存运行日志：`ros2 bag record -a -o robocon_run`
- [ ] 分析定位漂移：检查FAST-LIO的`path_en`输出路径
- [ ] 评估导航性能：记录实际轨迹vs规划轨迹

## 🔄 回滚方案

如果新系统出现问题，可快速切换回原方案：

```bash
# 恢复原启动文件
cp src/jf_-rm2026_-navigation/src/jf_bringup/launch/singlenav_launch.py.backup \
   src/jf_-rm2026_-navigation/src/jf_bringup/launch/singlenav_launch.py

# 恢复原参数文件（如果有备份）
cp src/jf_-rm2026_-navigation/src/jf_bringup/params/singlenav2_params.yaml.backup \
   src/jf_-rm2026_-navigation/src/jf_bringup/params/singlenav2_params.yaml

# 重新编译
colcon build --symlink-install --packages-select jf_bringup
```

## 📞 技术支持

### 关键日志位置
```bash
# FAST-LIO日志
ros2 topic echo /fast_lio_odom/logger

# Nav2日志
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server

# TF错误
ros2 run tf2_ros tf2_monitor
```

### 紧急联系人
- FAST-LIO问题：检查终端2的输出
- Nav2问题：检查终端3的输出
- 硬件问题：检查雷达和IMU连接

---

**祝 Robocon 2026 比赛顺利！🎯**

*本指南由 Claude Code 生成，基于 JF26 工作空间实际配置。*
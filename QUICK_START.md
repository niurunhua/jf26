# FAST-LIO + Nav2 快速启动

## 🚀 一键启动命令（比赛用）

```bash
# 终端1：雷达
source ~/JF26/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 终端2：FAST-LIO + Nav2 完整系统
source ~/JF26/install/setup.bash
ros2 launch jf_bringup singlenav_launch.py \
  use_sim_time:=false \
  fastlio_config:=mid360.yaml \
  fastlio_map_file:=~/JF26/scans.pcd
```

## 🔧 编译与安装

```bash
# 安装依赖
pip install open3d
rosdep install --from-paths src --ignore-src -r -y

# 编译
cd ~/JF26
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 📊 系统验证

```bash
# 检查TF树
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link

# 检查话题
ros2 topic list | grep -E "(odom|lidar|costmap)"

# RViz可视化
rviz2 -d src/jf_-rm2026_-navigation/src/jf_bringup/rviz/jf_nav.rviz
```

## ⚙️ 关键参数（梅林台阶过滤）

**Voxel Layer 配置：**
- `min_obstacle_height: 0.4`  # 过滤台阶底面
- `max_obstacle_height: 1.2`  # 过滤过高障碍
- `min_z: 0.4`  # 点云高度下限
- `max_z: 1.2`  # 点云高度上限

**文件位置：** `src/jf_-rm2026_-navigation/src/jf_bringup/params/singlenav2_params.yaml`

## 🆘 紧急情况

```bash
# 紧急停止
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# 切换回原系统
cp src/jf_-rm2026_-navigation/src/jf_bringup/launch/singlenav_launch.py.backup \
   src/jf_-rm2026_-navigation/src/jf_bringup/launch/singlenav_launch.py
colcon build --symlink-install --packages-select jf_bringup
```

## 📞 详细文档

完整指南见：`FAST_LIO_NAV2_INTEGRATION_GUIDE.md`
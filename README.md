# lidar_nav2_installer

一键搭建基于 **ROS 2 + Livox MID-360 + 奥比中光深度相机** 的 3D LiDAR 自主导航环境。
按组件**拆分到多个工作空间**，结构参考开源项目 [Ikunio/Lidar_nav2_ws](https://github.com/Ikunio/Lidar_nav2_ws)。

## 特点

- ✅ 交互式选择：**工作空间命名**、**奥比中光相机型号系列**、**SLAM 算法多选**
- ✅ 相机驱动与雷达驱动**分开放在不同工作空间**
- ✅ 支持 FAST-LIO2 / Point-LIO 两种 LIO 可选（可同时装）
- ✅ 建图后端：**LIO-LoopClosure**（Scan Context 回环 + GTSAM iSAM2 位姿图优化，生成全局一致 `.pcd` 先验地图）
- ✅ 全局定位：**FAST_LIO_LOCALIZATION_ROS2**（FAST-LIO 里程计 + Open3D ICP 扫描-地图匹配）
- ✅ 支持 ROS 2 Humble / Jazzy / Iron，自动检测或 `-r` 指定
- ✅ 可选国内镜像加速
- 📄 完整话题 / 数据类型 / TF 树说明见 [`docs/architecture.md`](docs/architecture.md)

## 环境要求

| ROS 2 版本 | Ubuntu 版本 | 状态 |
|-----------|------------|------|
| `humble` | 22.04 LTS | ✅ 推荐（与 Lidar_nav2_ws 一致） |
| `jazzy` | 24.04 LTS | ✅ 推荐 |
| `iron` | 22.04 | ⚠️ 已停止维护 |

> 前置条件：已安装对应的 ROS 2。

## 一键安装

```bash
git clone https://github.com/Camille-123-z/lidar_nav2_installer.git
cd lidar_nav2_installer
chmod +x install.sh

# 交互式安装（会询问工作空间名、相机系列、SLAM 算法、建图后端、定位）
./install.sh

# 或全程指定参数（非交互）
./install.sh -n myrobot -r humble -c g3 -s fastlio2,pointlio -p loopclosure -l fastlio_loc
```

### 参数说明

| 参数 | 含义 | 取值 |
|------|------|------|
| `-n <name>` | 工作空间名称前缀 | 如 `myrobot` → `myrobot_orbbec_ws` / `myrobot_lidar_ws` / `myrobot_loc_ws` |
| `-r <distro>` | ROS 版本 | `humble` / `jazzy` / `iron` |
| `-c <camera>` | 奥比中光相机系列 | `g2`（Gemini2/Astra/DaBai，main 分支）· `g3`（Gemini3 330/430，v2-main 分支）· `none` |
| `-s <slam>` | SLAM 算法（逗号分隔） | `fastlio2` / `pointlio`，也可 `all` / `none` |
| `-p <pgo>` | 建图后端（回环+位姿图优化） | `loopclosure`（LIO-LoopClosure，自动装 GTSAM）· `none` |
| `-l <loc>` | 全局定位 | `fastlio_loc`（FAST_LIO_LOCALIZATION_ROS2）· `none` |
| `-m` | 启用国内镜像代理 | — |
| `-y` | 非交互，缺失项用默认值 | — |

### 国内加速

```bash
USE_MIRROR=1 ./install.sh -n myrobot -r humble -c g3 -s fastlio2
# 或自定义代理前缀
GH_PROXY=https://ghproxy.net/https://github.com USE_MIRROR=1 ./install.sh
```

## 安装内容与来源

| 组件 | 来源 | 位置 |
|------|------|------|
| 奥比中光深度相机驱动 `orbbec_camera` | [orbbec/OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2) | `<name>_orbbec_ws` |
| Livox MID-360 底层驱动 `Livox-SDK2` | [Livox-SDK/Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) | 系统级安装 |
| Livox ROS2 封装 `livox_ros_driver2` | [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | `<name>_lidar_ws` |
| **FAST-LIO2** (ROS2 移植) | [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) | `<name>_lidar_ws` |
| **Point-LIO** (ROS2 移植, MID-360) | [Innovative-Physics/9000_point_lio_ros2_Mid-360](https://github.com/Innovative-Physics/9000_point_lio_ros2_Mid-360) | `<name>_lidar_ws` |
| **LIO-LoopClosure** (建图后端, Scan Context + GTSAM) | [Linlinqiu/LIO-LoopClosure](https://github.com/Linlinqiu/LIO-LoopClosure) | `<name>_lidar_ws` |
| **FAST_LIO_LOCALIZATION_ROS2** (全局定位) | [myeongw002/FAST_LIO_LOCALIZATION_ROS2](https://github.com/myeongw002/FAST_LIO_LOCALIZATION_ROS2) | `<name>_loc_ws` |
| GTSAM（LIO-LoopClosure 依赖） | [borglab PPA](https://launchpad.net/~borglab/+archive/ubuntu/gtsam-release-4.1) `libgtsam-dev`（失败时源码编译 [borglab/gtsam](https://github.com/borglab/gtsam)） | 系统级安装 |
| 3D 点云 → 2D 激光扫描 | [ros-perception/pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan) | `<name>_lidar_ws` |
| 3D 点云 → 2D 栅格地图 | [jkk-research/pointcloud_to_grid](https://github.com/jkk-research/pointcloud_to_grid) | `<name>_lidar_ws` |
| Nav2 / slam_toolbox | apt 二进制安装 | 系统（`/opt/ros`） |

## 工作空间结构（以 `-n myrobot` 为例）

```
~/myrobot_orbbec_ws/            # 相机工作空间
└── src/OrbbecSDK_ROS2/         # 奥比中光相机 ROS2 驱动

~/myrobot_lidar_ws/             # 雷达 + SLAM 工作空间
├── src/
│   ├── livox_ros_driver2/       # Livox MID-360 ROS2 驱动
│   ├── FAST_LIO2/               # FAST-LIO2（-s fastlio2）
│   ├── Point-LIO/               # Point-LIO（-s pointlio）
│   ├── LIO-LoopClosure/         # 建图后端（-p loopclosure）
│   ├── pointcloud_to_laserscan/
│   └── pointcloud_to_grid/
└── third_party/Livox-SDK2/      # Livox 底层 SDK（已 make install 到系统）

~/myrobot_loc_ws/               # 全局定位工作空间（-l fastlio_loc）
└── src/FAST_LIO_LOCALIZATION_ROS2/
```

## 使用示例

```bash
source /opt/ros/humble/setup.bash
source ~/myrobot_orbbec_ws/install/setup.bash   # 用到相机时
source ~/myrobot_lidar_ws/install/setup.bash    # 雷达 / SLAM / 建图后端
source ~/myrobot_loc_ws/install/setup.bash      # 全局定位
```

```bash
# 奥比中光深度相机
ros2 launch orbbec_camera gemini.launch.py       # Gemini 系列
ros2 launch orbbec_camera astra.launch.py        # Astra 系列

# Livox MID-360（先改 config/MID360_config.json 里的主机/雷达 IP）
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# SLAM 建图（先改对应 config 里的雷达/IMU 话题与标定）
ros2 launch fast_lio mapping.launch.py           # FAST-LIO2
ros2 launch point_lio mapping.launch.py          # Point-LIO

# 建图后端（Scan Context 回环 + GTSAM 位姿图优化）
ros2 launch lio_loopclosure pointlio_mid360.launch.py   # 与 Point-LIO + MID-360 配对
ros2 service call /save_map std_srvs/srv/Empty          # 保存 corrected_map.pcd 先验地图

# 全局定位（先改 config/*.yaml 的 map_file_path 指向上面的 .pcd）
ros2 launch fast_lio_localization velodyne_localization.launch.py

# 3D 点云 -> 2D 激光扫描 / 栅格地图
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node
ros2 run pointcloud_to_grid pointcloud_to_grid_node

# Nav2 导航
ros2 launch nav2_bringup bringup_launch.py
```

### 两阶段工作流

1. **建图**：`livox_ros_driver2` + `fast_lio`（或 `point_lio`）+ `lio_loopclosure`，绕场一圈覆盖回环区域后 `/save_map`，得到全局一致 `corrected_map.pcd` + 优化位姿 `optimized_poses.txt`。
2. **定位 + 导航**：`livox_ros_driver2` + `fast_lio_localization`（加载先验 `.pcd` 做 ICP 匹配）+ Nav2。注意还需一个**里程计适配节点**把 FAST-LIO2 的 `camera_init→body` 桥接为 Nav2 的 `odom→base_footprint` + `map→odom`（详见 [`docs/architecture.md`](docs/architecture.md) §6）。

## 常见问题

- **FAST-LIO2 与 Point-LIO 能同时装吗？** 能。两者包名不同（`fast_lio` / `point_lio`），可共存于同一个雷达工作空间。
- **Livox 驱动找不到 `liblivox_lidar_sdk_shared.so`**：执行 `sudo ldconfig`，或 `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib`。
- **相机打不开（权限不足）**：确认已执行奥比中光 udev 规则安装，并重新插拔 USB。
- **Gemini 3 系列不工作**：请用 `-c g3`（走 `v2-main` 分支）重新安装。
- **FAST-LIO 收不到雷达/IMU 话题**：检查 config YAML 的 `lidar_topic`/`imu_topic`、QoS，回放 bag 加 `--clock`。
- **为什么全局定位不是 FAST-LIO-Localization-SC-QN（Quatro+Nano-GICP）？** engcang 的 SC-QN 目前仅 ROS1（catkin/roslaunch）。本仓库集成的是其 ROS2 等价替代 `FAST_LIO_LOCALIZATION_ROS2`（FAST-LIO2 + Open3D ICP 扫描-地图匹配），Humble+ 可用。若坚持用 Quatro+Nano-GICP 需自行移植或回退 ROS1。
- **GTSAM 安装失败**：脚本会先试 borglab PPA（`libgtsam-dev`），失败则源码编译 4.2.0（较慢）。也可手动 `sudo apt install libgtsam-dev` 或按需换版本。
- **LIO-LoopClosure 编译报找不到 GTSAM**：确认 `libgtsam-dev` 已装，或源码编译后 `sudo ldconfig`；再 `colcon build --packages-select lio_loopclosure`。
- **定位配准精度差**：先验地图质量决定上限——建图阶段务必走回环并 `/save_map`；再调 `map_voxel_size`/`scan_voxel_size`/`localization_th`，确保初始位姿接近真值（RViz2 给 2D Pose Estimate）。
- **clone 慢/失败**：加 `-m` 走镜像代理，或手动设置 `GH_PROXY`。

## 参考资料

- 参考工作空间：[Ikunio/Lidar_nav2_ws](https://github.com/Ikunio/Lidar_nav2_ws)
- Livox：[Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) · [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
- 奥比中光：[OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2) · [文档](https://orbbec.github.io/OrbbecSDK_ROS2_Docs/)
- SLAM：[Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) · [Innovative-Physics/9000_point_lio_ros2_Mid-360](https://github.com/Innovative-Physics/9000_point_lio_ros2_Mid-360)
- 建图后端：[Linlinqiu/LIO-LoopClosure](https://github.com/Linlinqiu/LIO-LoopClosure)（Scan Context + GTSAM）
- 全局定位：[myeongw002/FAST_LIO_LOCALIZATION_ROS2](https://github.com/myeongw002/FAST_LIO_LOCALIZATION_ROS2) · 上游 [HViktorTsoi/FAST_LIO_LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION) · [engcang/FAST-LIO-Localization-SC-QN](https://github.com/engcang/FAST-LIO-Localization-SC-QN)（ROS1）
- GTSAM：[borglab/gtsam](https://github.com/borglab/gtsam) · [PPA](https://launchpad.net/~borglab/+archive/ubuntu/gtsam-release-4.1)
- 点云转换：[pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan) · [pointcloud_to_grid](https://github.com/jkk-research/pointcloud_to_grid)
- 话题 / TF 说明：[docs/architecture.md](docs/architecture.md)

# lidar_nav2_installer

一键搭建基于 **ROS 2 + Livox MID-360 + 奥比中光深度相机** 的 3D LiDAR 自主导航环境。
按组件**拆分到多个工作空间**，结构参考开源项目 [Ikunio/Lidar_nav2_ws](https://github.com/Ikunio/Lidar_nav2_ws)。

## 特点

- ✅ 交互式选择：**工作空间命名**、**奥比中光相机型号系列**、**SLAM 算法多选**
- ✅ 相机驱动与雷达驱动**分开放在不同工作空间**
- ✅ 支持 FAST-LIO2 / Point-LIO 两种 LIO 可选（可同时装）
- ✅ 支持 ROS 2 Humble / Jazzy / Iron，自动检测或 `-r` 指定
- ✅ 可选国内镜像加速

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

# 交互式安装（会询问工作空间名、相机系列、SLAM 算法）
./install.sh

# 或全程指定参数（非交互）
./install.sh -n myrobot -r humble -c g3 -s fastlio2,pointlio
```

### 参数说明

| 参数 | 含义 | 取值 |
|------|------|------|
| `-n <name>` | 工作空间名称前缀 | 如 `myrobot` → `myrobot_orbbec_ws` / `myrobot_lidar_ws` |
| `-r <distro>` | ROS 版本 | `humble` / `jazzy` / `iron` |
| `-c <camera>` | 奥比中光相机系列 | `g2`（Gemini2/Astra/DaBai，main 分支）· `g3`（Gemini3 330/430，v2-main 分支）· `none` |
| `-s <slam>` | SLAM 算法（逗号分隔） | `fastlio2` / `pointlio`，也可 `all` / `none` |
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
│   ├── pointcloud_to_laserscan/
│   └── pointcloud_to_grid/
└── third_party/Livox-SDK2/      # Livox 底层 SDK（已 make install 到系统）
```

## 使用示例

```bash
source /opt/ros/humble/setup.bash
source ~/myrobot_orbbec_ws/install/setup.bash   # 用到相机时
source ~/myrobot_lidar_ws/install/setup.bash
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

# 3D 点云 -> 2D 激光扫描 / 栅格地图
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node

# Nav2 导航
ros2 launch nav2_bringup bringup_launch.py
```

## 常见问题

- **FAST-LIO2 与 Point-LIO 能同时装吗？** 能。两者包名不同（`fast_lio` / `point_lio`），可共存于同一个雷达工作空间。
- **Livox 驱动找不到 `liblivox_lidar_sdk_shared.so`**：执行 `sudo ldconfig`，或 `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib`。
- **相机打不开（权限不足）**：确认已执行奥比中光 udev 规则安装，并重新插拔 USB。
- **Gemini 3 系列不工作**：请用 `-c g3`（走 `v2-main` 分支）重新安装。
- **FAST-LIO 收不到雷达/IMU 话题**：检查 config YAML 的 `lidar_topic`/`imu_topic`、QoS，回放 bag 加 `--clock`。
- **clone 慢/失败**：加 `-m` 走镜像代理，或手动设置 `GH_PROXY`。

## 参考资料

- 参考工作空间：[Ikunio/Lidar_nav2_ws](https://github.com/Ikunio/Lidar_nav2_ws)
- Livox：[Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) · [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
- 奥比中光：[OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2) · [文档](https://orbbec.github.io/OrbbecSDK_ROS2_Docs/)
- SLAM：[Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) · [Innovative-Physics/9000_point_lio_ros2_Mid-360](https://github.com/Innovative-Physics/9000_point_lio_ros2_Mid-360)
- 点云转换：[pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan) · [pointcloud_to_grid](https://github.com/jkk-research/pointcloud_to_grid)

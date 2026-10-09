# 建图 + 全局定位 + Nav2 导航：话题 / 数据类型 / TF 树

本文档说明整套方案（FAST-LIO2 + Scan Context + GTSAM 建图 → FAST-LIO Localization 全局定位 → Nav2 导航）涉及的 **ROS2 话题、消息类型、TF 树**，以及各组件之间的数据流与坐标变换约定。

> 组件来源见根目录 `README.md`；一键安装见 `install.sh`。

---

## 0. 系统总览

```
                ┌────────────────── 建图阶段（离线/在线） ──────────────────┐
                │                                                          │
 Livox MID-360 ─┤→ livox_ros_driver2 ──┐                                  │
   (点云+IMU)    │                      ├→ FAST-LIO2 (前端 LIO, 10Hz)      │
                │                      │      ├─ /Odometry                │
                │                      │      └─ /cloud_registered        │
                │                      │              │                    │
                │                      │              └→ LIO-LoopClosure   │
                │                      │                 (Scan Context     │
                │                      │                  回环 + GTSAM     │
                │                      │                  iSAM2 位姿图优化)│
                │                      │                   └─ /aft_pgo_*   │
                │                      │                      │            │
                │                      │                 /save_map         │
                │                      │                   └─ corrected_map.pcd
                └──────────────────────────────────────────────────────────┘
                                    │ 先验地图 (.pcd) + 关键帧位姿
                                    ▼
                ┌────────────────── 定位阶段（运行时） ─────────────────────┐
                │                                                          │
 Livox MID-360 ─┤→ livox_ros_driver2 → FAST-LIO2 (里程计)                   │
                │                          │                              │
                │        FAST_LIO_LOCALIZATION_ROS2                        │
                │          ├─ global_map_publisher (加载 .pcd → /global_map)│
                │          ├─ global_localization (ICP 扫描-地图 → 修正量)  │
                │          └─ transform_fusion (融合 → /localization + TF)  │
                │                          │                              │
                └──────────────────────────┼──────────────────────────────┘
                                           ▼
                ┌────────────────── Nav2 导航 ─────────────────────────────┐
                │  里程计适配节点(自定义)                                   │
                │    FAST-LIO2 /Odometry  →  odom→base_footprint          │
                │    localization 修正量   →  map→odom                     │
                │  3D 点云地图 → 2D 栅格 (pointcloud_to_grid)              │
                └──────────────────────────────────────────────────────────┘
```

---

## 1. 驱动层：`livox_ros_driver2`（MID-360）

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| 发布 | `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | MID-360 原始点云（驱动内部由 `livox_ros_driver2/msg/CustomMsg` 转换得到） |
| 发布 | `/livox/imu` | `sensor_msgs/msg/Imu` | MID-360 内置 IMU（200Hz） |

> FAST-LIO2 的 `config/*.yaml` 中 `lid_topic` / `imu_topic` 默认即指向这两个话题；换 MID-360 型号需同步修改。

---

## 2. 前端 LIO：FAST-LIO2（`fast_lio` 包）

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| 订阅 | `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | 点云输入 |
| 订阅 | `/livox/imu` | `sensor_msgs/msg/Imu` | IMU 输入 |
| 发布 | `/Odometry` | `nav_msgs/msg/Odometry` | 紧耦合里程计，**10Hz**，位姿在 `camera_init` 坐标系 |
| 发布 | `/path` | `nav_msgs/msg/Path` | 轨迹（可选，`path_en`） |
| 发布 | `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | 去畸变/配准后点云（`camera_init` 系） |
| 发布 | `/cloud_registered_body` | `sensor_msgs/msg/PointCloud2` | 去畸变点云（`body`/IMU 系） |
| 广播 | TF `camera_init` → `body` | `tf2_msgs/msg/TFMessage` | LIO 自身世界系 → 机体系 |

**关键帧约定**：FAST-LIO2 用 `frame_id: camera_init`（自身世界原点）和 `body_frame: body`（机体）。这与 Nav2 的 `map/odom/base_footprint` 约定不同，见 §6。

---

## 3. 建图后端：LIO-LoopClosure（Scan Context + GTSAM，`lio_loopclosure` 包）

纯后端、松耦合：前端 LIO 独立运行，本包只订阅前端输出做回环 + 位姿图优化。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| 订阅 | `/aft_mapped_to_init` | `nav_msgs/msg/Odometry` | 前端里程计位姿（由 FAST-LIO2 的 `/Odometry` remap） |
| 订阅 | `/velodyne_cloud_registered_local` | `sensor_msgs/msg/PointCloud2` | 前端配准点云，用于关键帧存储（由 `/cloud_registered` remap） |
| 订阅 | `/cloud_for_scancontext` | `sensor_msgs/msg/PointCloud2` | （可选）Scan Context 用降采样点云，缺省回退到全分辨率 |
| 订阅 | `/gps/fix` | `sensor_msgs/msg/NavSatFix` | （可选）GPS 高度因子，默认关闭 |
| 发布 | `/aft_pgo_odom` | `nav_msgs/msg/Odometry` | iSAM2 优化后的最新里程计位姿 |
| 发布 | `/aft_pgo_path` | `nav_msgs/msg/Path` | 优化后的完整轨迹（所有关键帧位姿） |
| 发布 | `/aft_pgo_map` | `sensor_msgs/msg/PointCloud2` | 体素滤波后的全局点云地图（回环后更新） |
| 发布 | `/loop_scan_local` | `sensor_msgs/msg/PointCloud2` | 当前关键帧点云（回环验证可视化） |
| 发布 | `/loop_submap_local` | `sensor_msgs/msg/PointCloud2` | 匹配到的历史子图（回环验证可视化） |
| 广播 | TF `camera_init` → `aft_pgo` | `tf2_msgs/msg/TFMessage` | 优化后位姿变换 |
| 服务 | `/save_map` | `std_srvs/srv/Empty` | 按需保存 `corrected_map.pcd` 等 |

**建图产出**（`/save_map` 后）：
- `corrected_map.pcd` — 全局一致先验点云地图（**后续重定位的精度上限**）
- `optimized_poses.txt` — KITTI 格式优化位姿（可作为定位的关键帧位姿库）
- `corrected_map_dense.pcd` / `uncorrected_map.pcd` — 供对比

**启动**（与前端配对）：
```bash
ros2 launch lio_loopclosure pointlio_mid360.launch.py      # Point-LIO + MID-360
ros2 launch lio_loopclosure fastlio_ouster64.launch.py     # FAST-LIO2 + Ouster64（MID-360 需改 launch）
```

---

## 4. 全局定位：FAST_LIO_LOCALIZATION_ROS2（`fast_lio_localization` 包）

FAST-LIO2 里程计 + Open3D ICP 扫描-地图匹配（先验 `.pcd`）。是 ROS1 `FAST-LIO-Localization(-SC-QN)` 的 ROS2 移植；Quatro+Nano-GICP 的 SC-QN 版目前仍为 ROS1，本 ROS2 版用 ICP 做同等精配准。

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| 发布 | `/global_map` | `sensor_msgs/msg/PointCloud2` | 由 `map_file_path` 加载的静态先验地图（PCD→PointCloud2，周期发布） |
| 发布 | `/submap` | `sensor_msgs/msg/PointCloud2` | 当前 ICP 用局部裁剪地图 |
| 发布 | `/cur_scan_in_map` | `sensor_msgs/msg/PointCloud2` | 配准到 `map` 系的当前扫描 |
| 发布 | `/map_to_odom` | `nav_msgs/msg/Odometry` | 全局修正量（里程计在 `map` 系的表达） |
| 发布 | `/localization` | `nav_msgs/msg/Odometry` | **融合后的最终里程计**（高频里程计 + 低频全局修正） |
| 广播 | TF `map` → `camera_init`、`map` → `body` | `tf2_msgs/msg/TFMessage` | 由 `transform_fusion` 发布 |

**配置**（`config/*.yaml`）：`map_file_path`（先验地图绝对路径）、`map_voxel_size`/`scan_voxel_size`、`freq_localization`、`localization_th`（配准接受阈值）。

---

## 5. Nav2 集成：3D 点云 → 2D 栅格

| 组件 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| `pointcloud_to_grid` | 订阅 `/aft_pgo_map`（或先验 `.pcd` 转发的 PointCloud2） | `sensor_msgs/msg/PointCloud2` | 输入 3D 地图点云 |
| `pointcloud_to_grid` | 发布 `/grid_map`（或 `/occupancy_grid`） | `grid_map_msgs/msg/GridMap` / `nav_msgs/msg/OccupancyGrid` | 2D 栅格，供 Nav2 全局代价地图 |
| `pointcloud_to_laserscan` | 订阅 `/cloud_registered_body` | `sensor_msgs/msg/PointCloud2` | 实时 3D→2D 扫描 |
| `pointcloud_to_laserscan` | 发布 `/scan` | `sensor_msgs/msg/LaserScan` | 局部代价地图/避障 |

> Nav2 的 `map_server` 也可直接加载 `.pgm`/`.yaml` 栅格；若用点云地图则用 `pointcloud_to_grid` 在线转 2D 栅格，或离线用 `octomap_server`（`ros-<distro>-octomap-mapping`）投影。

---

## 6. TF 树与坐标约定转换

### 6.1 两套坐标约定

| 语义 | SLAM/Localization 内部约定 | Nav2 标准约定 |
|------|---------------------------|---------------|
| 全局世界（先验地图系） | `camera_init`（建图）/ `map`（定位） | `map` |
| 里程计参考系 | `camera_init`（纯里程计漂移系） | `odom` |
| 机体系 | `body` | `base_link` / `base_footprint` |
| 传感器系 | `/livox`（点云） | `/livox`、`/imu_link`（static TF） |

### 6.2 完整 TF 树（导航运行时）

```
map ──(map→odom，全局定位修正量)──> odom ──(odom→base_footprint，FAST-LIO2 里程计)──> base_footprint
                                                                                          │
                                                              base_link ──(static)──> lidar / imu_link
```

- `map → odom`：由全局定位修正维持（FAST_LIO_LOCALIZATION_ROS2 的 `transform_fusion`，或里程计适配节点）。定位正常时近似恒定，丢失定位时冻结。
- `odom → base_footprint`：由 FAST-LIO2 高频里程计维持（10Hz）。
- `base_footprint → base_link → lidar/imu_link`：机器人描述文件 `URDF` 里的 `static` 变换。

### 6.3 里程计适配节点（需自定义，本仓库不含）

FAST-LIO2 输出的 `/Odometry`（`camera_init`→`body`）不能直接给 Nav2 用，需要一个小适配节点做转换：

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| 订阅 | `/Odometry` | `nav_msgs/msg/Odometry` | FAST-LIO2 里程计（`camera_init`→`body`） |
| 订阅 | `/localization`（或 `/aft_pgo_odom`） | `nav_msgs/msg/Odometry` | 全局修正量 / 优化位姿 |
| 发布 | `/odom`（或直接 TF） | `nav_msgs/msg/Odometry` | 重映射为 `odom`→`base_footprint` |
| 广播 | TF `map`→`odom`、`odom`→`base_footprint` | `tf2_msgs/msg/TFMessage` | 桥接两套坐标约定 |

核心逻辑：把 `body` 当作 `base_footprint`，把 FAST-LIO2 的 `camera_init` 当作 `odom`；再用全局修正量把 `camera_init` 对齐到 `map`，发布 `map→odom`。可参考定位包的 `transform_fusion.py` 改造。

### 6.4 各阶段 TF 对比

| 阶段 | TF 链 |
|------|-------|
| 建图（纯 LIO） | `camera_init` → `body`（FAST-LIO2） |
| 建图（后端优化） | `camera_init` → `aft_pgo`（LIO-LoopClosure 叠加） |
| 定位 | `map` → `camera_init` → `body`（FAST_LIO_LOCALIZATION_ROS2） |
| 导航 | `map` → `odom` → `base_footprint` → `base_link` → `lidar`（适配节点桥接） |

---

## 7. 数据流时序（以 MID-360 为例）

```
livox_ros_driver2 ──(PointCloud2 /livox/lidar, Imu /livox/imu)──> FAST-LIO2
FAST-LIO2 ──(Odometry 10Hz /Odometry, PointCloud2 /cloud_registered)──> LIO-LoopClosure
LIO-LoopClosure ──(Scan Context 回环候选 → ICP 精化 → GTSAM iSAM2)──> /aft_pgo_odom, /aft_pgo_map
   └─ /save_map ──> corrected_map.pcd + optimized_poses.txt

【定位阶段】
FAST_LIO_LOCALIZATION_ROS2:
   global_map_publisher ──(PCD)──> /global_map
   FAST-LIO2 ──(/Odometry + 当前扫描)──> global_localization ──(ICP 扫描-地图)──> /map_to_odom
   transform_fusion ──(融合)──> /localization + TF(map→camera_init→body)

【导航阶段】
里程计适配节点 ──> odom→base_footprint (TF) + map→odom (TF)
pointcloud_to_grid ──(/aft_pgo_map)──> /grid_map ──> Nav2 全局代价地图
```

#!/usr/bin/env bash
# =============================================================================
#  lidar_nav2_installer — 一键安装脚本（多工作空间拆分版）
#  ---------------------------------------------------------------------------
#  在 Ubuntu + ROS 2 上搭建 3D LiDAR 自主导航环境，按组件拆分为多个工作空间，
#  结构参考开源工作空间 Ikunio/Lidar_nav2_ws。
#
#  安装内容（分工作空间）：
#    ~/<name>_orbbec_ws  奥比中光深度相机驱动 (orbbec/OrbbecSDK_ROS2)
#    ~/<name>_lidar_ws   Livox MID-360 驱动 + SLAM 建图 + 点云转换
#      ├─ Livox-SDK2 (系统级安装)  ├─ livox_ros_driver2 (ROS2 封装)
#      ├─ SLAM: FAST-LIO2 / Point-LIO (可选)
#      ├─ pointcloud_to_laserscan  └─ pointcloud_to_grid
#    Nav2 + slam_toolbox 通过 apt 二进制安装（不属于任何工作空间）
#
#  用法：
#    ./install.sh                       # 交互式选择
#    ./install.sh -n myrobot -r humble -c g2 -s fastlio2
#    ./install.sh -h                    # 查看全部参数
# =============================================================================
set -euo pipefail

# -----------------------------------------------------------------------------
# 颜色与日志
# -----------------------------------------------------------------------------
if [ -t 1 ]; then
  C_RED=$'\033[31m'; C_GRN=$'\033[32m'; C_YEL=$'\033[33m'; C_BLU=$'\033[34m'; C_RST=$'\033[0m'
else
  C_RED=""; C_GRN=""; C_YEL=""; C_BLU=""; C_RST=""
fi

log()  { printf '%s\n' "$*"; }
info() { printf '%s[INFO]%s %s\n' "$C_BLU" "$C_RST" "$*"; }
ok()   { printf '%s[ OK ]%s %s\n' "$C_GRN" "$C_RST" "$*"; }
warn() { printf '%s[WARN]%s %s\n' "$C_YEL" "$C_RST" "$*" >&2; }
die()  { printf '%s[ERR ]%s %s\n' "$C_RED" "$C_RST" "$*" >&2; exit 1; }

# -----------------------------------------------------------------------------
# 组件仓库定义
# -----------------------------------------------------------------------------
# 奥比中光相机（按系列选分支）
ORBBEC_URL="https://github.com/orbbec/OrbbecSDK_ROS2.git"
# 相机系列 -> 分支
declare -A ORBBEC_BRANCH=(
  [g2]="main"      # Gemini 2 / Astra / DaBai / Femto 等 (v1.x)
  [g3]="v2-main"   # Gemini 3 系列 330/430/335/336 等 (v2.x)
)

# Livox
LIVOX_SDK2_URL="https://github.com/Livox-SDK/Livox-SDK2.git"
LIVOX_ROS_DRIVER2_URL="https://github.com/Livox-SDK/livox_ros_driver2.git"

# SLAM 算法（可选）
declare -A SLAM_URL SLAM_BRANCH SLAM_DIR SLAM_LABEL
SLAM_URL[fastlio2]="https://github.com/Ericsii/FAST_LIO_ROS2.git"
SLAM_BRANCH[fastlio2]=""
SLAM_DIR[fastlio2]="FAST_LIO2"
SLAM_LABEL[fastlio2]="FAST-LIO2"

SLAM_URL[pointlio]="https://github.com/Innovative-Physics/9000_point_lio_ros2_Mid-360.git"
SLAM_BRANCH[pointlio]=""
SLAM_DIR[pointlio]="Point-LIO"
SLAM_LABEL[pointlio]="Point-LIO"

# 点云 -> 2D
PCL_TO_LASERSCAN_URL="https://github.com/ros-perception/pointcloud_to_laserscan.git"
PCL_TO_GRID_URL="https://github.com/jkk-research/pointcloud_to_grid.git"
PCL_TO_GRID_BRANCH="ros2"

CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
USE_MIRROR="${USE_MIRROR:-0}"
GH_PROXY="${GH_PROXY:-https://ghproxy.com/https://github.com}"

# git clone，可选走镜像代理
gh_clone() {
  local url="$1"; shift
  if [ "$USE_MIRROR" = "1" ]; then
    git clone "${GH_PROXY}/${url}" "$@"
  else
    git clone "${url}" "$@"
  fi
}

usage() {
  cat <<EOF
用法: $0 [选项]

选项:
  -n <name>     工作空间名称前缀（默认交互询问，如 myrobot -> myrobot_orbbec_ws）
  -r <distro>   ROS 版本: humble | jazzy | iron（默认自动检测）
  -c <camera>   奥比中光相机系列: g2 | g3 | none（默认交互选择）
                  g2  = Gemini 2 / Astra / DaBai 等 (v1.x, main 分支)
                  g3  = Gemini 3 系列 330/430     (v2.x, v2-main 分支)
  -s <slam>     SLAM 算法，逗号分隔: fastlio2 | pointlio（也可 all/none）
  -m            启用国内镜像代理（git clone）
  -y            非交互模式，缺失项使用默认值
  -h            显示本帮助

示例:
  $0 -n myrobot -r humble -c g3 -s fastlio2,pointlio
  $0 -y                                  # 全部默认，非交互
EOF
}

# 把 SLAM 编号/名称解析为 key 列表（输出到 SLAM_SELECTED）
SLAM_SELECTED=""
resolve_slam() {
  local input; input=$(echo "$1" | tr -d ' ')
  SLAM_SELECTED=""
  case "$input" in
    ""|0|none|NONE) SLAM_SELECTED="" ;;
    3|all|ALL)      SLAM_SELECTED="fastlio2 pointlio" ;;
    *)
      # 允许直接传 key（如 fastlio2,pointlio）
      for key in fastlio2 pointlio; do
        local n
        case "$key" in fastlio2) n=1;; pointlio) n=2;; esac
        if echo ",${input}," | grep -qE ",(${key}|${n}),"; then
          SLAM_SELECTED="${SLAM_SELECTED} ${key}"
        fi
      done
      ;;
  esac
}

# -----------------------------------------------------------------------------
# 解析命令行参数
# -----------------------------------------------------------------------------
WS_NAME=""; ROS_DISTRO=""; CAMERA_SEL=""; SLAM_SEL=""; NONINTERACTIVE=0
while getopts "n:r:c:s:myh" opt; do
  case "$opt" in
    n) WS_NAME="$OPTARG" ;;
    r) ROS_DISTRO="$OPTARG" ;;
    c) CAMERA_SEL="$OPTARG" ;;
    s) SLAM_SEL="$OPTARG" ;;
    m) USE_MIRROR=1 ;;
    y) NONINTERACTIVE=1 ;;
    h) usage; exit 0 ;;
    *) usage; exit 1 ;;
  esac
done
shift $((OPTIND - 1))
ROS_DISTRO="${ROS_DISTRO:-${1:-}}"

# -----------------------------------------------------------------------------
# 0. 前置检查
# -----------------------------------------------------------------------------
[ "$(id -u)" -eq 0 ] && die "请用普通用户运行本脚本（脚本内部会按需使用 sudo）"
command -v git >/dev/null 2>&1 || { info "安装 git..."; sudo apt-get install -y git; }
command -v lsb_release >/dev/null 2>&1 || sudo apt-get install -y lsb-release

# -----------------------------------------------------------------------------
# 1. 确定 ROS 版本
# -----------------------------------------------------------------------------
if [ -z "$ROS_DISTRO" ]; then
  ROS_DISTRO=$(ls /opt/ros 2>/dev/null | sort -V | tail -n1 || true)
  [ -z "$ROS_DISTRO" ] && die "未检测到已安装的 ROS。请用 -r 指定：humble|jazzy|iron"
fi

UBUNTU_VER=$(lsb_release -rs)
case "$ROS_DISTRO" in
  humble) REQ_UBUNTU="22.04" ;;
  iron)   REQ_UBUNTU="22.04"; warn "ROS 2 Iron 已停止维护，建议 Humble/Jazzy" ;;
  jazzy)  REQ_UBUNTU="24.04" ;;
  *)      die "不支持的 ROS 版本: ${ROS_DISTRO}（支持 humble / jazzy / iron）" ;;
esac
[ "$UBUNTU_VER" = "$REQ_UBUNTU" ] || warn "当前 Ubuntu ${UBUNTU_VER}，${ROS_DISTRO} 官方对应 ${REQ_UBUNTU}，可能不兼容"
[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ] || die "未找到 /opt/ros/${ROS_DISTRO}/setup.bash，请先安装 ROS 2 ${ROS_DISTRO}"
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# -----------------------------------------------------------------------------
# 2. 交互式 / 默认值确定参数
# -----------------------------------------------------------------------------
if [ "$NONINTERACTIVE" = "1" ]; then
  WS_NAME="${WS_NAME:-lidar_nav2}"
  CAMERA_SEL="${CAMERA_SEL:-g2}"
  SLAM_SEL="${SLAM_SEL:-fastlio2 pointlio}"
else
  if [ -z "$WS_NAME" ]; then
    printf '请输入工作空间名称前缀 (默认 lidar_nav2): '
    read -r REPLY; WS_NAME="${REPLY:-lidar_nav2}"
  fi

  if [ -z "$CAMERA_SEL" ]; then
    echo
    echo "选择奥比中光深度相机系列:"
    echo "  1) Gemini 2 / Astra / DaBai 等   (v1.x, main 分支)"
    echo "  2) Gemini 3 系列 330/430/335/336 (v2.x, v2-main 分支)"
    echo "  0) 不安装相机驱动"
    printf '请输入编号 (默认 1): '
    read -r REPLY
    case "${REPLY:-1}" in
      1) CAMERA_SEL=g2 ;;
      2) CAMERA_SEL=g3 ;;
      0) CAMERA_SEL=none ;;
      *) CAMERA_SEL=g2 ;;
    esac
  fi

  if [ -z "$SLAM_SEL" ]; then
    echo
    echo "选择要安装的 SLAM 建图算法 (可多选，逗号分隔，如 1,2):"
    echo "  1) FAST-LIO2  (社区 ROS2 移植 Ericsii, MID-360 适配)"
    echo "  2) Point-LIO  (社区 ROS2 移植, MID-360 专用)"
    echo "  3) 全部"
    echo "  0) 不安装"
    printf '请输入编号 (默认 1,2): '
    read -r REPLY
    resolve_slam "${REPLY:-1,2}"
    SLAM_SEL="$SLAM_SELECTED"
  fi
fi

# 解析最终选择
resolve_slam "$SLAM_SEL"
SLAM_SELECTED="$(echo "$SLAM_SELECTED" | xargs)"   # 去掉首尾空格

# 校验相机选择
case "$CAMERA_SEL" in g2|g3|none) ;; *) CAMERA_SEL=g2; warn "未知相机参数，回退为 g2" ;; esac

ORBBEC_WS="${HOME}/${WS_NAME}_orbbec_ws"
LIDAR_WS="${HOME}/${WS_NAME}_lidar_ws"

echo
info "========== 安装配置 =========="
log "  ROS 版本   : ${ROS_DISTRO} (Ubuntu ${UBUNTU_VER})"
log "  工作空间   : ${ORBBEC_WS}"
log "             : ${LIDAR_WS}"
log "  相机驱动   : $([ "$CAMERA_SEL" = none ] && echo '不安装' || echo "${CAMERA_SEL} (${ORBBEC_BRANCH[$CAMERA_SEL]})")"
log "  SLAM 算法  : $([ -n "$SLAM_SELECTED" ] && echo "$SLAM_SELECTED" || echo '不安装')"
log "  国内镜像   : $([ "$USE_MIRROR" = 1 ] && echo '开启' || echo '关闭')"
log "=============================="

# -----------------------------------------------------------------------------
# 3. 安装系统依赖（含 Nav2 与 slam_toolbox）
# -----------------------------------------------------------------------------
info "安装系统依赖..."
sudo apt-get update
sudo apt-get install -y \
  git cmake build-essential \
  python3-pip python3-colcon-common-extensions python3-vcstool python3-rosdep \
  libeigen3-dev libpcl-dev libyaml-cpp-dev libboost-all-dev \
  libgflags-dev libgoogle-glog-dev nlohmann-json3-dev \
  ros-${ROS_DISTRO}-navigation2 \
  ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-slam-toolbox \
  ros-${ROS_DISTRO}-pcl-ros \
  ros-${ROS_DISTRO}-pcl-conversions \
  ros-${ROS_DISTRO}-tf-transformations \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-camera-info-manager \
  ros-${ROS_DISTRO}-diagnostic-updater
sudo apt-get install -y ros-${ROS_DISTRO}-grid-map-msgs 2>/dev/null || true

# -----------------------------------------------------------------------------
# 4. rosdep
# -----------------------------------------------------------------------------
info "初始化 rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init || warn "rosdep init 失败（可能已初始化）"
fi
rosdep update || rosdep update || warn "rosdep update 失败，已手动安装主要依赖，可忽略"

# -----------------------------------------------------------------------------
# 5. 相机工作空间
# -----------------------------------------------------------------------------
if [ "$CAMERA_SEL" != "none" ]; then
  info "===== 搭建相机工作空间 ${ORBBEC_WS} ====="
  mkdir -p "${ORBBEC_WS}/src"
  cd "${ORBBEC_WS}/src"
  if [ ! -d OrbbecSDK_ROS2 ]; then
    gh_clone "${ORBBEC_URL}" -b "${ORBBEC_BRANCH[$CAMERA_SEL]}"
  fi

  info "安装奥比中光相机 udev 规则..."
  if [ -f "${ORBBEC_WS}/src/OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh" ]; then
    sudo bash "${ORBBEC_WS}/src/OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh"
    sudo udevadm control --reload-rules && sudo udevadm trigger
  else
    warn "未找到 Orbbec udev 脚本，跳过"
  fi

  info "安装相机工作空间依赖..."
  ( cd "${ORBBEC_WS}" && rosdep install --from-paths src --ignore-src -r -y ) || warn "相机依赖 rosdep 安装有遗漏"

  info "编译相机工作空间..."
  ( cd "${ORBBEC_WS}" && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" )
  ok "相机工作空间完成"
else
  info "跳过相机驱动安装"
fi

# -----------------------------------------------------------------------------
# 6. Livox-SDK2（系统级安装，供 livox_ros_driver2 与 SLAM 链接）
# -----------------------------------------------------------------------------
info "===== 编译安装 Livox-SDK2 (系统级) ====="
mkdir -p "${LIDAR_WS}/third_party"
cd "${LIDAR_WS}/third_party"
if [ ! -d Livox-SDK2 ]; then
  gh_clone "${LIVOX_SDK2_URL}"
fi
cd Livox-SDK2
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
make -j"$(nproc)"
sudo make install
sudo ldconfig
ok "Livox-SDK2 已安装"

# -----------------------------------------------------------------------------
# 7. 雷达 + SLAM + 点云转换 工作空间
# -----------------------------------------------------------------------------
info "===== 搭建雷达工作空间 ${LIDAR_WS} ====="
mkdir -p "${LIDAR_WS}/src"
cd "${LIDAR_WS}/src"

# 7.1 livox_ros_driver2
if [ ! -d livox_ros_driver2 ]; then
  gh_clone "${LIVOX_ROS_DRIVER2_URL}"
fi

# 7.2 SLAM 算法（可选）
for key in $SLAM_SELECTED; do
  if [ -d "${SLAM_DIR[$key]}" ]; then
    info "已存在 ${SLAM_LABEL[$key]}，跳过 clone"
    continue
  fi
  info "拉取 ${SLAM_LABEL[$key]} ..."
  if [ -n "${SLAM_BRANCH[$key]}" ]; then
    gh_clone "${SLAM_URL[$key]}" -b "${SLAM_BRANCH[$key]}" --recursive "${SLAM_DIR[$key]}"
  else
    gh_clone "${SLAM_URL[$key]}" --recursive "${SLAM_DIR[$key]}"
  fi
done

# 7.3 点云 -> 2D 转换
if [ ! -d pointcloud_to_laserscan ]; then
  gh_clone "${PCL_TO_LASERSCAN_URL}" -b "${ROS_DISTRO}" || gh_clone "${PCL_TO_LASERSCAN_URL}"
fi
if [ ! -d pointcloud_to_grid ]; then
  gh_clone "${PCL_TO_GRID_URL}" -b "${PCL_TO_GRID_BRANCH}"
fi

# 7.4 livox_ros_driver2 启用 ROS2 package.xml
info "配置 livox_ros_driver2（启用 ROS2 package.xml）..."
cd "${LIDAR_WS}/src/livox_ros_driver2"
if [ ! -f package.xml ]; then
  [ -f package_ROS2.xml ] && cp package_ROS2.xml package.xml || die "未找到 package_ROS2.xml"
fi

# 7.5 依赖 + 编译
info "安装雷达工作空间依赖..."
( cd "${LIDAR_WS}" && rosdep install --from-paths src --ignore-src -r -y ) || warn "雷达依赖 rosdep 安装有遗漏"

info "编译雷达工作空间..."
( cd "${LIDAR_WS}" && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" )
ok "雷达工作空间完成"

# -----------------------------------------------------------------------------
# 8. 完成
# -----------------------------------------------------------------------------
echo
ok "全部安装完成！"
echo
echo "================================================================"
echo "  加载环境（按需 source 对应工作空间）："
echo "    source /opt/ros/${ROS_DISTRO}/setup.bash"
[ "$CAMERA_SEL" != none ] && echo "    source ${ORBBEC_WS}/install/setup.bash   # 相机"
echo "    source ${LIDAR_WS}/install/setup.bash    # 雷达/SLAM"
echo
echo "  启动示例："
[ "$CAMERA_SEL" != none ] && echo "    ros2 launch orbbec_camera gemini.launch.py        # 相机(按型号选 launch)"
echo "    ros2 launch livox_ros_driver2 msg_MID360_launch.py  # 雷达(先改 config IP)"
for key in $SLAM_SELECTED; do
  case "$key" in
    fastlio2) echo "    ros2 launch fast_lio mapping.launch.py              # FAST-LIO2 建图" ;;
    pointlio) echo "    ros2 launch point_lio mapping.launch.py             # Point-LIO 建图" ;;
  esac
done
echo "    ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node  # 3D->2D 扫描"
echo "    ros2 launch nav2_bringup bringup_launch.py          # Nav2 导航"
echo "================================================================"
echo
ok "FAST-LIO2 与 Point-LIO 包名不同（fast_lio / point_lio），可同时安装"

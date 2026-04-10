#! /bin/bash
# ============================================================
# 启动 ROS2 Navigation Stack 与 RViz2 可视化
# 使用相对路径加载地图与 RViz 配置
# ============================================================

# 当前脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 项目根目录（上上级：script/bash -> script -> 项目根目录）
PROJECT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# ROS2 工作空间路径
WORKSPACE_DIR="${PROJECT_DIR}"

# 地图与RViz配置文件路径
MAP_PATH="${PROJECT_DIR}/map/map.yaml"
RVIZ_CONFIG="${PROJECT_DIR}/src/navigation/src/robot_navigo/rviz/rviz2_config.rviz"

on_interrupt() {
  echo "[INFO] Caught interrupt, stopping child processes..."
  pkill -P $$ >/dev/null 2>&1 || true
  exit 130
}

trap on_interrupt INT TERM

# 检查 setup.bash 是否存在
if [ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  echo "【ERROR】找不到 ${WORKSPACE_DIR}/install/setup.bash"
  echo "请先执行: colcon build"
  exit 1
fi

print_usage() {
  echo "Usage: $0 {nav|rviz|all|print}"
  echo "  nav   : 启动导航栈（当前终端前台运行）"
  echo "  rviz  : 启动 RViz2（当前终端前台运行）"
  echo "  all   : 有 gnome-terminal 时自动开两个终端（nav+rviz）；MATRiX 仿真需单独启动"
  echo "  print : 打印 UE 推荐的三终端命令（sim+nav+rviz）"
}

start_nav() {
  source "${WORKSPACE_DIR}/install/setup.bash"
  exec ros2 launch robot_navigo navigation_bringup.launch.py \
    platform:=UE \
    mc_controller_type:=RL_TRACK_VELOCITY \
    communication_type:=UDP \
    map:="${MAP_PATH}"
}

start_rviz() {
  source "${WORKSPACE_DIR}/install/setup.bash"
  exec rviz2 -d "${RVIZ_CONFIG}"
}

print_split_commands() {
  echo "# 终端0：启动 MATRiX 仿真（提供 odom）"
  echo "cd ../matrix && bash run_sim.sh 1 3"
  echo
  echo "# 终端1：启动导航栈"
  echo "bash script/bash/start_navigation.sh nav"
  echo
  echo "# 终端2：启动 RViz2"
  echo "bash script/bash/start_navigation.sh rviz"
}

if [ $# -lt 1 ]; then
  print_usage
  exit 1
fi

case "$1" in
  nav)
    echo ">>> Launching Navigation Stack..."
    start_nav
    ;;
  rviz)
    echo ">>> Launching RViz2..."
    start_rviz
    ;;
  print)
    print_split_commands
    ;;
  all)
    if command -v gnome-terminal >/dev/null 2>&1; then
      gnome-terminal -- bash -c "cd '${PROJECT_DIR}' && bash script/bash/start_navigation.sh nav; exec bash"
      gnome-terminal -- bash -c "cd '${PROJECT_DIR}' && bash script/bash/start_navigation.sh rviz; exec bash"
    else
      echo "[INFO] gnome-terminal not found. 请手动在两个终端分别执行："
      print_split_commands
    fi
    ;;
  *)
    print_usage
    exit 1
    ;;
esac
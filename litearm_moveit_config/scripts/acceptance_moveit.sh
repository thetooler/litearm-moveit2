#!/usr/bin/env bash
# acceptance_moveit.sh — MoveIt 规划链路验收（无硬件）。
#
#   起 move_group（含控制栈 dry-run）→ 规划到 ready 位形 → 可选执行 → 收栈
#
# 用法:
#   scripts/acceptance_moveit.sh                # 只规划（不驱动机械臂）
#   scripts/acceptance_moveit.sh --execute      # 规划并执行
#   scripts/acceptance_moveit.sh --execute --real   # 真机
#
# 环境注意（踩过的坑）：ROS 2 默认域 0 且组播发现是全网的。同网段若有其他机器人
# 在跑 ROS 2，会看到**多个 /move_action server**，规划目标可能被陌生节点的
# move_group 接管 —— 它没有你的控制器，于是返回 FAILURE；而你的 move_group
# 同时也在执行，表现为"报失败但臂确实动了"这种极难定位的现象。
# 所以这里强制用独立域名 + 只走本机。

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 工作区根：脚本可能从源码树（<ws>/src/<pkg>/scripts/）运行，也可能从安装树
# （<ws>/install/<pkg>/lib/<pkg>/）运行，两种深度不同。这里向上找同时含
# src/ 与 install/ 的目录，而不是靠固定层数推算。
_find_ws() {
    local d="$SCRIPT_DIR"
    while [ "$d" != "/" ]; do
        if [ -f "$d/install/setup.bash" ] && [ -d "$d/src" ]; then
            echo "$d"
            return 0
        fi
        d="$(dirname "$d")"
    done
    return 1
}

if ! WS_DIR="$(_find_ws)"; then
    echo "找不到工作区根（需要同时存在 src/ 与 install/setup.bash）：$SCRIPT_DIR" >&2
    exit 2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

LOG="${TMPDIR:-/tmp}/litearm_acceptance_moveit.log"
PROBE="$SCRIPT_DIR/moveit_probe.py"

cd "$WS_DIR"
set +u
# ROS 环境：不写死发行版，用 $ROS_DISTRO 或默认 humble 定位安装前缀。
# 若已 source 过其他发行版，请显式导出 ROS_DISTRO 再运行本脚本。
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
# shellcheck disable=SC1091
source install/setup.bash
set -u

PROBE_ARGS=()
DRY="dry_run:=true"
for arg in "$@"; do
    case "$arg" in
        --execute) PROBE_ARGS+=("--execute") ;;
        --real)    DRY="dry_run:=false"
                   echo "[accept] ★ 真机模式：机械臂将实际运动，确认空间开阔、已扶好急停" ;;
        *) echo "未知参数: $arg" >&2; exit 2 ;;
    esac
done

rm -f "$LOG"
setsid ros2 launch litearm_moveit_config litearm_moveit.launch.py \
    "$DRY" use_rviz:=false >"$LOG" 2>&1 </dev/null &
LAUNCH_PID=$!
echo "[accept] MoveIt 栈已启动 pid=$LAUNCH_PID domain=$ROS_DOMAIN_ID log=$LOG"

cleanup() {
    local pgid
    pgid="$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ')"
    if [ -n "$pgid" ]; then
        kill -TERM -"$pgid" 2>/dev/null
        sleep 5
        kill -KILL -"$pgid" 2>/dev/null
    fi
    wait "$LAUNCH_PID" 2>/dev/null
    echo "[accept] 已收栈"
}
trap cleanup EXIT

deadline=$((SECONDS + 120))
while (( SECONDS < deadline )); do
    if grep -q "You can start planning now" "$LOG" 2>/dev/null; then
        echo "[accept] move_group 就绪"
        break
    fi
    if grep -qiE "\[FATAL\]|Failed to load|error loading" "$LOG" 2>/dev/null; then
        echo "[accept] move_group 启动失败："
        grep -iE "\[FATAL\]|Failed to load|error" "$LOG" | head -20
        exit 1
    fi
    sleep 1
done
if ! grep -q "You can start planning now" "$LOG" 2>/dev/null; then
    echo "[accept] 超时未就绪，日志尾部："
    tail -40 "$LOG"
    exit 1
fi

python3 "$PROBE" ${PROBE_ARGS[@]+"${PROBE_ARGS[@]}"}
RC=$?

echo "[accept] ===== 日志中的问题项 ====="
# octomap 那条是 MoveIt 在没有 3D 传感器时的固定告警，本配置无深度相机，属预期
grep -iE "\[ERROR\]|\[FATAL\]" "$LOG" | grep -v "occupancy_map_monitor" | head -10 || echo "（无）"
echo "[accept] 探针退出码=$RC"
exit $RC

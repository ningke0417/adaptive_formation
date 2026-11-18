#!/usr/bin/env bash
# Start this before launching the scene; it waits for ROS master if necessary.
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == "--help" ]]; then
  echo "Usage: $0 [run_name]"
  echo "Start before the scene. Ctrl+C stops recording and finalizes the bag."
  echo "Output: bags/eight_fixed_wall/<run_name>/"
  exit 0
fi
run_name="${1:-$(date +%Y%m%d_%H%M%S)}"
if [[ ! "$run_name" =~ ^[a-zA-Z0-9_-]+$ ]]; then
  echo 'Run name must contain only letters, numbers, underscores or hyphens.' >&2
  exit 1
fi
if [[ ! -f "$workspace_dir/devel/setup.bash" ]]; then
  echo 'Build the workspace first: catkin_make -j2 -l2' >&2
  exit 1
fi
set +u
source "$workspace_dir/devel/setup.bash"
set -u
output_dir="$workspace_dir/bags/eight_fixed_wall/$run_name"
mkdir -p -- "$(dirname -- "$output_dir")"
mkdir -- "$output_dir"  # Refuse to overwrite an existing trial.
git -C "$workspace_dir" rev-parse HEAD > "$output_dir/git_head.txt"
git -C "$workspace_dir" diff -- . > "$output_dir/source_changes.patch"
cp "$workspace_dir/src/planner/plan_manage/launch/rectangle_8_fixed_wall.launch" "$output_dir/"
cp "$workspace_dir/src/planner/plan_manage/config/rectangle_8_fixed_wall.yaml" "$output_dir/"
printf 'Recording directory: %s\nWaiting for ROS master; now launch the scene in another terminal.\n' "$output_dir"
monitor_pid=''
snapshot_pid=''
cleanup() {
  for child_pid in "$monitor_pid" "$snapshot_pid"; do
    if [[ -n "$child_pid" ]]; then
      kill "$child_pid" 2>/dev/null || true
      wait "$child_pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
until rosparam list > /dev/null 2>&1; do sleep 1; done
(
  until rosparam get /drone_7_ego_planner_node/size_swarm > /dev/null 2>&1; do sleep 1; done
  rosparam dump "$output_dir/params.yaml"
  rosnode list > "$output_dir/nodes.txt"
) &
snapshot_pid=$!
if command -v vmstat > /dev/null 2>&1; then
  vmstat -w -t 1 > "$output_dir/resources.log" &
  monitor_pid=$!
fi
topics=(/goal /map_generator/global_cloud /map_generator/wall_geometry
        /leader_traj_from_planner /leader_traj_to_planner
        /broadcast_traj_from_planner /broadcast_traj_to_planner /rosout_agg)
for ((i=0; i<8; i++)); do
  topics+=("/drone_${i}_visual_slam/odom"
           "/drone_${i}_planning/pos_cmd"
           "/drone_${i}_planning/trajectory"
           "/drone_${i}_planning/start"
           "/drone_${i}_planning/finish"
           "/drone_${i}_planning/data_display")
done
printf 'Press Ctrl+C when the run is finished; wait for run.bag to be finalized.\n'
rosbag record --lz4 --buffsize=64 --min-space=1G \
  -O "$output_dir/run.bag" "${topics[@]}" __name:=eight_fixed_wall_recorder

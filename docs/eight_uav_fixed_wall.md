# 八机 2×4 固定墙对比场景

参考本机 `~/Codes/EGO-Planner-criterion-validation/swarm-playground/formation_ws/src/planner/plan_manage/launch/rectangle_8uav_fixed_split_merge_wall.launch`，对应参考方法的 `coordination_mode:=distributed`。

本工程保留 adaptive_formation 的图编队优化、虚拟领航轨迹和本地规划；没有移植参考方法的分裂判据、分布式协调者或换槽机制。`formation/fixed_members` 表示成员集合固定为八机，编队形变仍由 adaptive 算法处理，并非强制保持刚性矩形。

## 启动

请在单独终端加载本工作空间；两个工程都提供 `ego_planner`，不要在同一 ROS master 中同时启动。

```zsh
cd ~/Codes/adaptive_formation
source /opt/ros/noetic/setup.zsh
catkin_make -j2 -l2
source devel/setup.zsh
roslaunch ego_planner rectangle_8_fixed_wall.launch
```

等待八架飞机显示后，RViz 使用 **3D Nav Goal** 点击目标中心 `(10, 0, 1.5)`。新场景会使用消息中的真实坐标。需要可重复的默认目标时使用：

```zsh
roslaunch ego_planner rectangle_8_fixed_wall.launch auto_start:=true
```

无界面运行加 `rviz:=false`。`auto_start` 默认关闭；开启后被动显示节点等待八个里程计和八个目标订阅连接，再发送一次目标。亦可手动发送：

```zsh
rostopic pub -1 /goal geometry_msgs/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 10.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}'
```

## 场景对齐

| 项目 | 数值 |
|---|---|
| 编号 | 0–7，前排 0–3，后排 4–7 |
| 编队偏移 x | 前排 +0.5 m，后排 −0.5 m |
| 每排偏移 y | −1.5、−0.5、+0.5、+1.5 m |
| 起始中心 / 目标中心 | (−6, 0, 1.5) / (10, 0, 1.5) m |
| 地图尺寸 | 24 × 14 × 3 m |
| 第一堵墙 | 中心 (−2, 0)，厚 0.5 m、宽 4 m |
| 第二堵墙 | x=6.5，厚 0.5 m、总宽 10 m，中间净开口 1.8 m |
| 墙高度 / 点云间隔 | z∈[−0.1, 3] m / 0.1 m |
| 名义最大速度 | 0.5 m/s |
| 感知 | 半径 5 m、水平 360°，保留原 CPU renderer 的竖直角度过滤 |

墙体生成器直接复制自参考项目，保持相同的采样规则及实体边界。 RViz 同样使用参考配置：`simulation_map` 为淡紫色（RGB 178/178/206）的 0.1 m 体素方块，`wall_geometry` 叠加地图生成器发布的半透明墙面。墙体不预先按安全距离膨胀。`safety_distance=0.2` 与代价安全距离对齐，但不同方法的地图膨胀、避障代价、速度实际执行值仍需分别记录，不能只凭相同参数名称声称物理模型完全相同。

可调整 `start_x`、`goal_x`、`height`、`max_vel`、`wall_x`、`wall_size_y`、`second_wall_gap_size_y` 等 launch 参数。固定墙原始参考设置保持上述默认值。

## 八机适配和资源保护

- 使用 YAML 中的八机槽位，不执行原三十机演示中按位置触发的增减成员。
- 虚拟领航位置解释为编队中心，保留每架飞机的固定槽位编号。
- 轻量 RViz 只显示地图、八机和最近 120 秒轨迹；不启动原有无限增长的 `odom_visualization`。视角复制参考项目的 `default.rviz` 当前 Orbit 配置，白色背景；无人机使用同一 `fake_drone.dae` 模型，黑色、0.35 缩放，姿态来自各机里程计。
- 拒绝失败、非有限或过长的虚拟领航轨迹；接收端再次检查消息长度、时长与系数。
- 本地轨迹在生成标记和采样缓存前检查时长。原配置默认上限 20 秒；八机低速配置通过 `optimization/max_local_duration` 提高到 60 秒（每 0.2 秒采样，最多 301 点），虚拟领航局部轨迹上限也是 60 秒。这些是失控保护，不是整段任务的时间限制。
- 这些保护可能使数值失控的试验停止前进或被记为失败，不应把“没有崩溃”视为成功避障或成功恢复编队。

## 对比数据

两个方法分开运行，使用相同起终点、墙体和目标触发，分别记录结果：

```zsh
rosparam dump /tmp/adaptive_8_fixed_wall_params.yaml
rosbag record -O /tmp/adaptive_8_fixed_wall.bag \
  /goal /map_generator/global_cloud \
  /drone_0_visual_slam/odom /drone_1_visual_slam/odom \
  /drone_2_visual_slam/odom /drone_3_visual_slam/odom \
  /drone_4_visual_slam/odom /drone_5_visual_slam/odom \
  /drone_6_visual_slam/odom /drone_7_visual_slam/odom \
  /broadcast_traj_from_planner /leader_traj_from_planner
```

至少比较：八机到达率、完成时间、总航程、最小机间距离、到实体墙的最小距离、槽位误差和资源峰值。发生碰撞、停滞或触发数值保护都保留为结果；当前适配并不证明 adaptive 方法一定能穿过两堵墙。控制/感知模型差异、槽位交换是否开启以及各方法参数应在实验报告中明确列出。

## 当前试跑结果（2026-11-18）

已完成编译、八机配置与参考墙体生成代码的一致性检查。一次受内存限制的无界面试跑中，八机均运动，约 96 秒观察期内无节点退出，相关进程 RSS 峰值总和约 1559 MiB。**该次未全员到达，点位置进入了实体墙（最小墙距离为 0），最小机间距离约 0.199 m，因此判为碰撞/未完成的失败试次。**

配置适配完成不等于基线算法已经标定正确：原优化器在 `optimizeTrajectory_lbfgs()` 中计算碰撞结果后仍有直接 `return true` 的历史逻辑，未改变该逻辑来伪造成功结果；这也是正式比较前需要审查的问题。原算法可能执行未通过碰撞检查的轨迹。当前结果仅用于暴露问题，不应直接用于论文中的成功率或性能结论。

原始报告见 [eight_fixed_wall_smoke.json](validation/eight_fixed_wall_smoke.json)。原始采样 CSV 暂存 `/tmp/adaptive_eight_checked/trajectory.csv`；正式试验请按上文录制 rosbag。此次未验证 RViz 长时间运行或硬件飞行。

## 一键录制实际试验

先在终端 A 运行（每次换一个试次名，已存在目录会拒绝覆盖）：

```zsh
cd ~/Codes/adaptive_formation
./scripts/record_8_fixed_wall.sh run01
```

它会等待 ROS master，然后记录目标、实体墙体、八机 odom / pos_cmd / trajectory / start / finish / data_display、轨迹广播和 rosout_agg，并保存 ROS 参数快照及系统资源日志。使用 LZ4 压缩和 64 MiB 录制缓冲区；不录制八机高频感知点云。

接着在终端 B 运行：

```zsh
cd ~/Codes/adaptive_formation
source devel/setup.zsh
roslaunch ego_planner rectangle_8_fixed_wall.launch auto_start:=true 2>&1 | tee bags/eight_fixed_wall/run01/console.log
```

结束时先在终端 A 按 Ctrl+C，等 rosbag 正常保存为 `run.bag` 后再停止仿真。文件位于 `bags/eight_fixed_wall/run01/`，下一次将两个终端的试次名都改为 `run02`。录包订阅不再影响自动触发：观察节点现在检查八个实际规划节点的目标订阅是否齐全。

碰撞分析应同时报告实体墙穿入、设定安全半径下的间距违规和数据覆盖范围。当前可视化模型大小（0.35 缩放）不是碰撞半径；离散记录也不能无条件证明连续时间内没有碰撞。规划消息数和输出频率不能代替真实求解耗时；耗时以实际日志字段为准，缺失的数据明确标注不可判定。


录完后可离线分析（不启动仿真）：

```zsh
source devel/setup.zsh
python3 scripts/analyze_8_fixed_wall.py bags/eight_fixed_wall/run01
```

输出在试次目录的 `analysis/`：`report.json`、`agents.csv`、`wall_crossings.csv` 和 `overview.png`。单个 bag 必须包含一次目标、八机世界系里程计和实体墙几何。到达按目标槽位0.3 m容差计算；机间最小距离在对齐的分段线性里程计上求解，超过0.2 s数据缺口不插值；墙体穿入检查包含段与实体盒相交。完整已发布轨迹的预测碰撞采样包含被后续重规划替换的尾段，不能代替执行结果。run01 的人工解读另见该次 `analysis/summary.md`。

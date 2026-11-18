# Adaptive Formation

ROS-based code and experiment assets for adaptive multi-UAV formation planning and control.

## Dataset

Only the rosbag files are hosted on Hugging Face:

https://huggingface.co/datasets/mmm1124/adaptive_formation

The `bags/` directory is excluded from this GitHub repository.

## Repository Structure

- `src/`: ROS packages for planning, simulation, localization, utilities, and the virtual-structure-based planner.
- `plot/`: plotting scripts and generated figures.
- `other_exp/`: supplementary experiment archives.
- `bag_record.sh`: bag recording helper script.

## GitHub Repository

Source code repository:

https://github.com/yzhou-zju/adaptive_formation

## Eight-UAV fixed-wall comparison

八机 2×4 编队、与 criterion-validation 分布式场景相同的两道固定墙：

```bash
roslaunch ego_planner rectangle_8_fixed_wall.launch auto_start:=true
```

编译、RViz 触发、几何参数、资源保护和对比记录见 [八机固定墙说明](docs/eight_uav_fixed_wall.md)。

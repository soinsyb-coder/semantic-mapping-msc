# 基于 RTAB-Map 与 YOLO 的语义建图系统

本仓库是曼彻斯特大学机器人学硕士毕业论文项目的代码冻结版本。

系统集成 ROS 2 Jazzy、Gazebo Harmonic、TurtleBot3、RTAB-Map、Nav2、
explore_lite 与 YOLO11n，实现 RGB-D 目标三维定位、地图坐标转换、时空语义
对象融合、SLAM 修正量诊断以及结构化实验记录。

## 正式实验基线

标签 `experiment-exact-20260816` 保存以下正式实验使用的精确源码：

- `20260813-220017-E2E01`
- `20260816-121939-AD04-R1`
- `20260816-203531-AD04-R2`
- `20260816-212856-AD04-R3`
- `20260816-225455-CMP01-SLAMONLY`

五次运行的源码 SHA-256 清单均与冻结前工作区完全匹配。

## 目录结构

- `src/semantic_perception/`：YOLO、RGB-D 定位、语义融合及漂移诊断。
- `src/semantic_mapping_bringup/`：仿真、SLAM、Nav2 与探索启动文件。
- `tools/`：实验记录、确定性速度回放与离线分析工具。
- `patches/`：外部依赖的实验修改补丁。
- `overlays/`：TurtleBot3 自定义 URDF 与工作区排除文件。
- `provenance/`：源码清单、上游 commit、归档及补丁哈希。

## 参考环境

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3.12
- RTAB-Map、Nav2、explore_lite
- Ultralytics YOLO11n（CPU 推理）

## 构建

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select semantic_perception semantic_mapping_bringup
source install/setup.bash
```

## 实验数据

大型 rosbag、RTAB-Map 数据库和实验归档不上传 GitHub。归档校验值位于
`provenance/formal_archive_checksums.sha256`，上游依赖版本位于
`provenance/upstream_revisions.tsv`。详细验证边界参见 `VALIDATION.md`。

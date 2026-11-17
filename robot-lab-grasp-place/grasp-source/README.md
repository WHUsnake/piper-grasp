# 抓取算法源码快照

此目录保存网页实际使用的 A2 抓取算法快照，共 44 个源码、消息和配置文件。
`manifest.json` 记录逐文件 SHA-256、上游仓库与基准提交，是本分支的算法内容清单。
`manifests/sources.lock.json` 锁定平台基础依赖，不能单独复现此处的本地改动。
运行时不自动覆盖使用者已有 A2 工作区。

- `a2_source/src/perception/perception_node/`：RGB-D 目标提取与点云处理。
- `a2_source/src/manipulation/grasp_generation/`：抓取候选生成与筛选。
- `a2_source/src/manipulation/robot_planner/`：Piper 机械臂规划。
- `a2_source/src/manipulation/task_executor/`：执行和反馈。
- `a2_source/src/interfaces/ros2/grasp_msgs/`：消息接口。

- `a2_source/config/robots/a2_piper/observation.json`：原观察姿态来源；网页实际角度由平台 profile 控制。

网页、世界控制和放置程序在 `integration/`，不在此处重复维护。
快照不含底盘控制、CAN 驱动、环境、权重、编译产物或日志，不是完整 A2 仓库的替代品。

## 为网页适配器构建

先按主 README 准备平台锁定源码（包含 Piper 描述与消息源码）、ROS Humble、MoveIt、
MTC、colcon，以及安装好 GraspNet 编译扩展的独立 Python 环境和所需权重。
以下命令不下载权重、不安装系统依赖、不启动硬件。

在仓库根目录执行。使用一个**不存在且全为 ASCII 的仓库外路径**保存源码和 ROS 构建，
避免 ROS 消息生成工具对中文路径的限制；已有路径须先检查，不覆盖。

```bash
set -e
LAB_REPO="$PWD"
GRASP_SOURCE="$HOME/robot-lab-grasp-source"
test ! -e "$GRASP_SOURCE"
cp -a "$LAB_REPO/grasp-source/a2_source" "$GRASP_SOURCE"
source /opt/ros/humble/setup.bash
mkdir -p "$GRASP_SOURCE/workspaces/pipeline"
cd "$GRASP_SOURCE/workspaces/pipeline"
/usr/bin/python3 -m colcon --log-base log build \
  --base-paths "$GRASP_SOURCE/src/interfaces/ros2/grasp_msgs" \
  --packages-select grasp_msgs \
  --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3
cd "$LAB_REPO"
```

适配器直接从快照的四个 Python 包目录加载算法，此处仅需生成 `grasp_msgs` overlay；
`piper_msgs` 由下一步的 `grasp prepare` 单独构建，不构建或启动硬件驱动。
然后按实际路径配置（`/absolute/path` 须替换为本机路径）：

```bash
./bin/lab --profile hssd-go2-piper grasp configure \
  --source "$GRASP_SOURCE" \
  --worker-python /absolute/path/to/graspnet-venv/bin/python \
  --baseline /absolute/path/to/graspnet-baseline \
  --checkpoint /absolute/path/to/checkpoint-rs.tar
./bin/lab --profile hssd-go2-piper grasp prepare
./bin/lab --profile hssd-go2-piper doctor --development
```

`graspnet.json` 使用环境变量定位 baseline 与权重，权重必须满足其中 SHA-256。
`grasp configure` 保存本机绑定到 Git 忽略的 `config.local.json`。
平台启动和网页操作见主 README；上述准备不执行抓取。
本次发布验证了干净副本消息构建和算法导入，尚未在另一台机器完成部署验收。

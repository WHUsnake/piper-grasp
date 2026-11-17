# 腕部 RGB-D 抓取与放置（Robot Lab 版本）

这里保存当前网页使用的抓取和放置代码，源版本锁在 `SOURCE_MANIFEST.json`，
对应 Robot Lab 提交 `1ed97b03530d00d6a53e6e4f4f97f8a0d394e9c9`（2026-11-17）。
本目录是面向 **Robot Lab 的抓放集成源码**，不是替换仓库根目录旧 A2 平台的独立启动器。
旧版 `A2_loco_manipulation/simulation/a2_table_grasp/start_web.sh` 不会自动加载这里的新放置。

## 内容与职责

- `grasp-source/a2_source/`：当前感知、GraspNet 候选、MoveIt 规划、执行器、消息和配置，44 文件逐项 SHA-256 锁定。
- `integration/integration/go2_grasp.py`：任务状态、持物／故障恢复、桌面感知、小步几何下降、松爪和回零编排。
- `integration/integration/go2_grasp_ros.py`：RGB-D 与原 ROS 算法桥接、执行前复核及瓶底几何绑定。
- `integration/mtc_place/`：MTC 接近、撤离、回零轨迹规划包。
- `integration/integration/static/go2/`：RGB-D 点击抓取／放置、六桌面入口、状态和恢复面板。
- `arm_trajectory.py`、`rpc.py`、`go2.py`、`go2_piper.py`、`viewer.py`：轨迹、通信、仿真执行和 HTTP 接入所需的宿主适配文件；不是可独立调用的硬件驱动。
- `lab/go2_grasp.py`：宿主配置与消息构建入口；由完整 Robot Lab 的 `bin/lab` 调用。
- `profiles/hssd-go2-piper.json`：腕部姿态、桌面分割先验及六桌面／测试瓶配置。

完整任务边界、正常／故障流程见 [架构说明](docs/grasp-place.md)，实际验证范围见 [验证记录](docs/validation.md)。

## 放置依赖的信息

使用 link6 腕部 RGB-D 测量桌面、空闲区域和目标世界坐标；用相机标定、关节位置／正运动学
及抓取前观测的直立瓶底偏移计算下降目标。机身相机只提供 RGB 观察。
MTC 规划到桌面上方，末段最大约 3 mm 小步修正、约 6 mm 几何间隙后松爪、撤离、回零，
再用新深度检查结果。当前不是受力确认支撑；目标被遮挡后沿用最后观测的静态桌面。
持物／掉落与碰撞保护仍有仿真状态依赖，不是纯 RGB-D 全流程或实机即用方案。

## 在完整 Robot Lab 中使用

1. 先获取完整宿主，固定到同一版本，避免把新适配器装入不匹配的旧任务协议：

   ```bash
   git clone --branch feature/she https://github.com/WHUBaoZi/robot-lab.git robot-lab
   cd robot-lab
   git checkout 1ed97b03530d00d6a53e6e4f4f97f8a0d394e9c9
   git lfs install
   git lfs pull
   ```

2. 上述提交已含本目录相同的运行文件。`SOURCE_MANIFEST.json` 可逐文件核对；
   对应路径以 `robot-lab/` 为根，不应覆盖到本仓库的旧 A2 根目录。
   如使用其他宿主版本，先检查差异并保留已有修改，不直接递归覆盖。
3. 按宿主 README 准备独立工作区、MuJoCo 4.0、锁定机器人／场景资产和 Python 依赖。
   六桌面和测试瓶由 profile 在资产构建时装配，旧派生资产须先归档再构建。
4. 按本目录 [算法构建说明](grasp-source/README.md)，从当前 `grasp-source/a2_source` 创建
   仓库外 ASCII 源码目录并构建 `grasp_msgs`。命令中的 `LAB_REPO` 使用完整宿主目录；
   也可使用宿主同版本的 `grasp-source`（字节一致）。准备 ROS Humble、MoveIt、MTC、
   `piper_msgs`、colcon 和带 GraspNet 扩展的独立 Python 环境。
5. 在完整宿主根目录绑定实际路径，然后启动：

   ```bash
   ./bin/lab --profile hssd-go2-piper grasp configure \
     --source /absolute/path/to/grasp-source \
     --worker-python /absolute/path/to/graspnet-venv/bin/python \
     --baseline /absolute/path/to/graspnet-baseline \
     --checkpoint /absolute/path/to/checkpoint-rs.tar
   ./bin/lab --profile hssd-go2-piper grasp prepare
   ./bin/lab --profile hssd-go2-piper doctor --development
   ./bin/lab --profile hssd-go2-piper preview --development
   ```

   `/absolute/path` 都须替换为实际目录；预览 Python 及资产首次准备见宿主 README。
   打开启动命令输出的 HTTP 地址，在腕部 RGB 中选目标抓取，持物后选空白桌面放置。

## 资源和版本边界

本次只新增抓放集成源码与说明，保留本仓库已有 A2 平台和历史验证资料。
没有重新上传个人配置、Python 环境、ROS 构建缓存、运行现场、录像或新增模型权重。
本仓库原有 GraspNet 基线／权重能否复用，须按当前配置的版本和 SHA-256 核对，不能仅凭文件名。
完整 HSSD/Go2 场景、步态资源、宿主其余部署模块留在上面的 Robot Lab 固定版本。
本目录没有新增许可授权；各来源代码和模型继续遵守原许可，源仓库资源许可不自动扩展到这里。

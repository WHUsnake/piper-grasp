# Robot Lab 抓取与放置：GraspNet / GraspGenX

这是两套可切换的**抓取候选生成方法**和一条共用的抓取执行、RGB-D 放置链路。两套方法分别见 [GraspNet](methods/graspnet/README.md) 与 [GraspGenX](methods/graspgenx/README.md)；共享代码在 `grasp-source/`（感知、候选筛选、MoveIt、执行）和 `integration/`（网页、仿真、MTC 放置）。选择方法只改变抓取候选的模型与适配；放置、11 cm TCP、碰撞检查和任务状态机相同，便于公平比较，也避免两份放置实现分叉。

**范围**：本机 Go2 + Piper MuJoCo 仿真；不连接真机。`agent_client.py` 是面向后续 agent 的稳定命令入口，当前调用的是 Lab HTTP 服务，不绕过服务端的任务状态和运动保护。API 与失败处理见 [Agent 接口](docs/agent-api.md)，感知与规划细节见 [架构](docs/grasp-place.md)。

**本轮抓取提速（2026-11-23）**：只更新三份抓取运行文件；网页、场景配置、放置和本机配置不变。仿真空闲时预热固定场景碰撞网格，任务只重建当前机器人与目标关系；底盘已稳定时复用最近姿态窗口，点击后仍复核。关节轨迹去掉低于模型额定值的第二层软件限速，保留仿真模型的 1.2 rad/s 关节上限、关节范围、碰撞检查和超时；加快预抓取与持物回零，抬升仍按原计划速度。同快照的 GraspNet 无录屏单轮对照，选点帧到首次真实关节运动为 6.167→5.735 秒，选点帧到抓起并持物回零为 39.347→29.331 秒。完整分段、边界和已部署版本的增量更新命令见 [提速报告](docs/speed-report.md)。GraspGenX 复用同一仿真轨迹代码，但本轮未重测其完整抓放。

## 真机迁移边界

当前两套方法切换的是**抓取候选生成模型**，其余感知、确认、执行和放置逻辑共用。以下仿真判定不能直接移植到真机：

- `go2_grasp.py` 的 `hold_confirmed` 用 MuJoCo 瓶子世界高度减去初始高度，要求抬升至少 5 cm；持物回零、滑落检测和 `hold_valid` 又读取瓶子相对 link6 的精确位置。网页的“允许放置”依赖 `hold_valid`。真机既没有瓶子的世界真值，也不能把深度缺失当作瓶子掉落。
- 抓取前，`go2_grasp_ros.py` 的目标分割要求至少 **100 个有效目标深度像素**、**6 cm 可见高度**，点击像素也必须有有效深度。用户提供的真实腕部图像中，RGB 能看到透明瓶，但瓶身局部仅约 **28/9589** 像素有深度，夹爪遮住下部。这一帧会在 GraspNet／GraspGenX 推理之前被拒绝；不能认为瓶身“全部可见”，也不能用少量瓶盖点直接推断精确瓶底。
- 当前网页 v4 放置下降与松爪根据已测腕部 RGB-D 桌面平面和关节正运动学运行，不以 MuJoCo 接触力决定松爪。保留的旧接触控制分支读取仿真接触力／重量，但不在 v4 网页流程上。放置仍以前述仿真持物判定作为准入，不能据此说整条链路已经能上真机。

**真机适配顺序**：先标定实际相机到 link6 和 11 cm TCP；用 RGB 目标区域、可靠的稀疏深度、桌面平面及多帧/多视角观测估计目标位姿与不确定度；再用真实夹爪开度、电流或已安装的触觉／力反馈，结合关节状态和可见区域变化，替代 MuJoCo 的抬升、持物、掉落判定。遮挡导致证据不足时，应维持夹持并报告“无法确认”，不能宣称抓稳或掉落。最后在真机按不同瓶型、透明度、视角和放置点重新统计完整抓放成功率。当前仓库没有完成这些真机适配。

## 从干净主机部署

本目录是完整 Robot Lab 的可安装 overlay，不是独立仿真器。宿主固定在已知兼容的 `WHUBaoZi/robot-lab` 提交 `1ed97b03530d00d6a53e6e4f4f97f8a0d394e9c9`。`install_overlay.py` 默认只检查，`--apply` 要求干净宿主并先备份被覆盖文件。不要直接覆盖自己已经修改过的宿主。

```bash
git clone --branch feature/she https://github.com/WHUBaoZi/robot-lab.git robot-lab
cd robot-lab
git checkout 1ed97b03530d00d6a53e6e4f4f97f8a0d394e9c9
git lfs install && git lfs pull
cd ..
python3 piper-grasp/robot-lab-grasp-place/install_overlay.py --host robot-lab
python3 piper-grasp/robot-lab-grasp-place/install_overlay.py --host robot-lab --apply
```

之后按宿主 README 准备独立工作区、场景包、预览 Python、ROS Humble、MoveIt 与 MTC。按 [抓取源码构建说明](grasp-source/README.md)把 `grasp-source/a2_source` 放到仓库外 ASCII 路径，构建 `grasp_msgs`，并用 `./bin/lab ... grasp configure` 绑定路径与 GraspNet 权重。`config.local.json` 属于本机配置，不上传。

GraspGenX 另需官方源码 `NVlabs/GraspGenX` 提交 `44eb1d77d6d27bc97ee361e3afb717a2115e30a9`、官方 release 生成器／判别器权重、PiPER hand 资产及独立的 CUDA/Python 依赖环境。把本目录 `backends/graspgenx/lab_worker.py` 的绝对路径写入宿主 `config.local.json` 的 `go2_grasp.graspgenx.script`；并配置 `root` 和 `python`，例如：

```json
{"go2_grasp": {"tcp_m": 0.11, "backend": "graspnet", "graspgenx": {
  "root": "/absolute/path/to/GraspGenX",
  "python": "/absolute/path/to/graspgenx-venv/bin/python",
  "script": "/absolute/path/to/piper-grasp/robot-lab-grasp-place/backends/graspgenx/lab_worker.py"
}}}
```

上面是**已有** `config.local.json` 中相应字段的示例，不能用这个片段覆盖整个文件。准备完成后，分别用 `methods/graspnet/launch.sh` 与 `methods/graspgenx/launch.sh` 启动；切换方法需要停止上一实例。同一时刻只应有一个实例占用同一世界与端口。页面和 agent 均在腕部 RGB-D 选目标抓取，持物后再次采图选空白桌面放置。服务器结果应标记真实 `backend`。

## 发布边界与优化方向

`SOURCE_MANIFEST.json` 锁定此目录的源码哈希；模型权重、虚拟环境、个人路径、视频、任务日志和授权凭据均不在此发布。权重与场景资产由各自合法来源取得并核对校验和。独立机器安装与真机运行尚未验收；当前实测见 [验证记录](docs/validation.md)。

发布前在同一 MuJoCo 初始场景独立重置 5 轮／方法，GraspNet、GraspGenX 均为 **5/5 完整抓放**；分别有 **2/5**、**4/5** 轮需在初次放置选点被拒绝后重新采图选点。两种方法观测到的单场景通过率均超过 80%，但样本仅覆盖一种瓶子和桌面，也没有真实支撑力确认；不能声称一般场景或真机达到 80%。

优先改进：让感知、MoveIt 与推理 worker 在任务间常驻；把 agent 任务请求加上幂等 ID、取消确认和结构化错误码；扩大同一初始场景、不同物体与不同桌面落点的双算法样本量，再比较抓稳率、放置率和各阶段耗时。当前小样本时间不能当成总体精度结论。

# Robot Lab 抓取与放置：GraspNet / GraspGenX

这是两套可切换的**抓取候选生成方法**和一条共用的抓取执行、RGB-D 放置链路。两套方法分别见 [GraspNet](methods/graspnet/README.md) 与 [GraspGenX](methods/graspgenx/README.md)；共享代码在 `grasp-source/`（感知、候选筛选、MoveIt、执行）和 `integration/`（网页、仿真、MTC 放置）。选择方法只改变抓取候选的模型与适配；放置、11 cm TCP、碰撞检查和任务状态机相同，便于公平比较，也避免两份放置实现分叉。

**范围**：本机 Go2 + Piper MuJoCo 仿真；不连接真机。`agent_client.py` 是面向后续 agent 的稳定命令入口，当前调用的是 Lab HTTP 服务，不绕过服务端的任务状态和运动保护。API 与失败处理见 [Agent 接口](docs/agent-api.md)，感知与规划细节见 [架构](docs/grasp-place.md)。

**真机迁移阻断项**：当前仿真抓取确认、持物回零、掉落检测和放置准入直接读取 MuJoCo 瓶子高度／世界位姿。这些不是相机或真实夹爪反馈，真机拿不到。当前 v4 放置下降和松爪走腕部 RGB-D 平面与关节正运动学；旧接触控制分支读取仿真接触力，但不在这条 v4 网页路径上。抓取入口还要求至少 100 个有效目标深度像素及 6 cm 可见高度；透明瓶、夹爪遮挡下可能达不到。两种模型共用这些门槛，因此下文通过率只说明仿真；接入真机前须补充稀疏深度与遮挡处理，并以实际可用的夹爪／视觉反馈重建持物判定，重新验收。

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

优先改进：让感知、MoveIt 与推理 worker 在任务间常驻；把 agent 任务请求加上幂等 ID、取消确认和结构化错误码；扩大同一初始场景、不同物体与不同桌面落点的双算法样本量，再比较抓稳率、放置率和各阶段耗时。当前小样本时间不能当成总体精度结论。

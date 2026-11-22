# GraspGenX 抓取到放置

`backend.json` 固定本方法的候选生成后端。输入为 link6 腕部 RGB-D 选点后的目标点云；GraspGenX 只生成候选抓取姿态。候选经过同一几何筛选、11 cm TCP 适配和 MoveIt 规划，执行接近、闭爪、抬升、持物回零。放置使用共用的腕部 RGB-D 桌面选点、MTC 路径、小步下降、释放及回零。

- 算法依赖：NVlabs GraspGenX generator/discriminator + PiPER hand asset。
- 本机配置需要：GraspGenX source, release checkpoints, PiPER asset and dedicated Python/CUDA environment。路径写入宿主 Git 忽略的 `config.local.json`，不要提交凭据、权重、虚拟环境或运行数据。
- 启动：先完成 [共用部署说明](../../README.md)，然后 `LAB_REPO=/absolute/path/to/robot-lab ./methods/graspgenx/launch.sh`。端口被其他 Lab 占用时先停止本人的实例，不要强制杀掉别人的进程。
- Agent 操作：[接口说明](../../docs/agent-api.md)；请求经同一状态机、感知与碰撞检查，不直接向模型或电机发送命令。
- 结果中的 `backend` 应为 `graspgenx`；请逐次核对 `result.json`、`grasp-screening.jsonl` 和放置结果，不能仅凭候选数判断成功。

## 模型准备检查

按 [NVlabs/GraspGenX 官方安装说明](https://github.com/NVlabs/GraspGenX#installation)在独立 Python/CUDA 环境安装源码，固定到 `44eb1d77d6d27bc97ee361e3afb717a2115e30a9`。当前适配器期待 `root/ext/graspgenx_checkpoints/release/gen`、`dis` 和 `root/assets`；官方模型下载与授权以原仓库说明为准。本目录只提供 PiPER 坐标轴与 17 列候选适配，不复制官方仓库、模型或环境。先在独立环境确认官方 sampler 能加载，再启动 Lab；缺少权重或 CUDA 时服务应报告初始化失败，不能悄悄退回 GraspNet。

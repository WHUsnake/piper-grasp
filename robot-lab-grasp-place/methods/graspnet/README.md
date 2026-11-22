# GraspNet 抓取到放置

`backend.json` 固定本方法的候选生成后端。输入为 link6 腕部 RGB-D 选点后的目标点云；GraspNet 只生成候选抓取姿态。候选经过同一几何筛选、11 cm TCP 适配和 MoveIt 规划，执行接近、闭爪、抬升、持物回零。放置使用共用的腕部 RGB-D 桌面选点、MTC 路径、小步下降、释放及回零。

- 算法依赖：GraspNet baseline + SHA-256 pinned checkpoint。
- 本机配置需要：GRASPNET_PYTHON, GRASPNET_BASELINE, GRASPNET_CHECKPOINT。路径写入宿主 Git 忽略的 `config.local.json`，不要提交凭据、权重、虚拟环境或运行数据。
- 启动：先完成 [共用部署说明](../../README.md)，然后 `LAB_REPO=/absolute/path/to/robot-lab ./methods/graspnet/launch.sh`。端口被其他 Lab 占用时先停止本人的实例，不要强制杀掉别人的进程。
- Agent 操作：[接口说明](../../docs/agent-api.md)；请求经同一状态机、感知与碰撞检查，不直接向模型或电机发送命令。
- 结果中的 `backend` 应为 `graspnet`；请逐次核对 `result.json`、`grasp-screening.jsonl` 和放置结果，不能仅凭候选数判断成功。

## 模型准备检查

此仓库历史 `search-grasp2/models/graspnet/checkpoint-rs.tar` 的 SHA-256 为 `60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868`，与本快照 `graspnet.json` 一致；部署时仍须由 `grasp configure` 显式绑定并核验。GraspNet baseline 源码固定到 `280c215129f759ed8649cb4e89fc5dfee55f4f80`，Python 扩展须在其独立环境构建。启动后以结果的 `backend=graspnet` 验证没有误用另一套模型。

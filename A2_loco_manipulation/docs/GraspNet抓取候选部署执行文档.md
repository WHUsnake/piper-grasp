# A2 + PiPER：GraspNet 抓取候选接入执行文档

**交给 6Astra 的实施任务书**  
编写日期：2026-11-05  
项目：`/home/xubo/桌面/she/A2_loco_manipulation`

## 1. 目标

把当前基于可见点云 PCA/包围盒的抓取候选生成，替换为 GraspNet Baseline 候选生成，并把输出接回项目已有候选消息和 MoveIt 预览链路。第一阶段目标是离线数据和 `plan_only` 预览中确认：候选位姿坐标正确、夹爪开口可用、预抓取点可规划、失败原因可诊断。

本任务不包含重新训练 GraspNet、改造底盘导航策略、绕过控制权互锁或执行实机抓取。模型输出是候选，不是可直接发送给 PiPER 的关节命令；是否执行仍由已有监督与安全流程管理。

附件截图只用于说明问题现象，不提供可用的三维位姿或接口约定。仓库文档中的 CAN、互锁、停稳和执行门控要求是项目边界，不能把它们当作可跳过的操作步骤。

## 2. 代码现状与必须先核对的事项

当前主目录里的相关文件：

- `src/manipulation/grasp_generation/grasp_generation/geometry.py`：PCA/包围盒候选生成；从点云估计轴和中心，按轴排列候选，分数主要来自夹爪宽度，不是抓取成功概率。
- `src/manipulation/grasp_generation/grasp_generation/node.py`：订阅 `/perception/target_cloud`，发布 `/grasp/candidates`；当前将点云头复制到输出，设置固定 0.10 m 预抓取距离和 0.12 m 抬升距离。
- `src/interfaces/ros2/grasp_msgs/msg/GraspCandidate.msg`：位姿定义为候选抓取中心，数组头给出坐标系和采集时间；项目约定工具 `+Z` 为进近方向、`+Y` 为夹爪闭合方向。
- `src/bringup/robot_bringup/launch/pipeline.launch.py`：当前启动 `grasp_generator`、`robot_planner`、`task_executor`；感知 device 默认参数为 `cpu`。
- `scripts/unified/arm_adapter.py`：将身份绑定的 `/grasp_perception/target` 点云转发为 `/perception/target_cloud`，要求同一观测身份、帧和时间戳，并拒绝旧数据。
- `config/scenarios/pick_and_hold/moveit.yaml`：规划组 `arm`、末端 `link6`、TCP 平移 `[0,0,0.105]` m，执行默认关闭。

**运行代码位置尚需核实。** 当前顶层 `src` 没有 `robot_planner` 和 `task_executor` 源包；`pipeline.launch.py` 仍会启动这两个包。`data/unified-inspection/earlier-project/work/...` 下有历史快照，但它不是默认的权威工作区。实施前先用 ROS 包索引、launch 路径和实际安装前缀确认运行中的 planner/executor 来源与版本；把修改落到真实维护源，不要只改历史快照或 `install/` 生成物。若权威源不在此仓库，先记录其路径、版本和部署关系，再决定如何纳入本项目。

已有历史 planner 快照最多尝试 `/grasp/candidates` 前 12 个候选，并对预抓取目标做 MoveIt `plan_only`；它没有完整记录每个候选的拒绝原因。历史 task executor 使用固定 `pregrasp_distance_m=0.10`、`gripper_open_m=0.075`、`gripper_close_m=0.050`，没有消费候选的 `gripper_width_m`、`pregrasp_distance_m`、`lift_distance_m`。部署前必须核对当前安装版是否相同；若相同，需要明确传递所选候选参数，不能假设消息字段会自动被执行器使用。

项目当前运行文档记录底盘控制权互锁和安全行走姿态尚未完成，真实任务保持未就绪。模型接入和所有阶段验证都必须保持 `execute=false`、`auto_enable=false`，不启动底盘运动或机械臂执行。

## 3. 推荐系统结构

```text
身份绑定的目标点云
  /grasp_perception/target
          │ arm_adapter 校验身份、帧、采集时间后转发
          ▼
  /perception/target_cloud
          │ GraspNet Baseline 推理
          ▼
  去重 / 开口过滤 / 点云碰撞过滤 / 坐标转换
          │ 保留原始 header 与采集时间
          ▼
  /grasp/candidates  (既有 GraspCandidateArray)
          │ MoveIt IK、碰撞和预抓取路径 plan_only
          ▼
  候选诊断 + selected pregrasp（只预览，不执行）
```

第一阶段只更换候选生成器，不并行发布 PCA 和 GraspNet 的候选。需要做 A/B 时，使用配置选择器或离线回放分别运行，避免两个 publisher 混发同一 ROS topic。

## 4. GraspNet 版本、依赖与设备策略

使用官方 `graspnet/graspnet-baseline`，在文档编写时固定到已核对提交 `280c215129f759ed8649cb4e89fc5dfee55f4f80`；GraspNet API 固定到 `eb57dd2092d8dbe05312a29c3d0c22f3226efbfc`。6Astra 若选择其他提交，须说明原因，保存完整 commit、模型权重来源及 SHA-256，不能依赖随时变化的 `main`/`master`。

官方 Baseline 列出的依赖包括 Python、PyTorch、Open3D、TensorBoard、NumPy、SciPy、Pillow、tqdm，并要求编译 `pointnet2` 和 `knn` CUDA 扩展；官方提供 RealSense 与 Kinect 预训练权重。旧依赖说明中的 PyTorch 版本较早，因此必须先按目标 GPU/CUDA/Python 验证组合，不要直接把旧 requirements 安装进 ROS、PiPER 驱动或 Isaac Lab 环境。[官方安装与权重说明](https://github.com/graspnet/graspnet-baseline/blob/main/README.md)

实施前确认推理运行在哪里：开发机 GPU、A2 机载 GPU，还是独立推理主机。记录 `nvidia-smi`、GPU 型号、驱动/CUDA、Python/Torch、点云单帧推理耗时和显存；如果目标运行设备没有兼容 CUDA，先测 CPU 方案的延迟，再决定部署位置。禁止仅因开发机有 RTX GPU 就假定机载端也有 GPU。

建议把网络和扩展安装在独立、可复现的 Python 环境；ROS adapter 保持 ROS2 消息环境稳定。需要跨 Python 环境时，将 GraspNet 作为独立 worker/节点接入 ROS2，不要污染 PiPER 驱动的 Python 环境。记录环境锁定文件、构建日志和启动命令。

## 5. 输入点云要求

1. 输入仍取自 `/perception/target_cloud`，这是上游按目标身份绑定的目标点云；不能改成不受限的全场景点云，也不能混入其他瓶子或桌面点云作为目标实例。
2. 读取 PointCloud2 时保留 `header.frame_id`、原始采集 `header.stamp` 和 XYZ 单位；NaN/Inf 清除，坐标转为米和 `float32`。不要把推理完成时间伪装成采集时间。
3. 对点数设置配置项和边界检查。官方 demo 使用单帧 RGB-D/内参构造米制点云，Baseline 测试配置示例使用 20,000 点；项目现有点云约 4,000 点时，应验证采样/补点策略，不要无条件重复点导致模型误判。[官方 demo 输入说明](https://github.com/graspnet/graspnet-baseline/blob/main/README.md)
4. 点云过期、时间戳在未来、点数不足、坐标帧为空、模型未加载或推理异常时，发布可诊断的失败状态并输出空候选；不得沿用上一个目标的候选。
5. 输出数组必须沿用输入点云的采集时间戳和 frame。planner 应在该时间戳查询 TF，并检查年龄；当前系统绑定点云链路要求不超过 0.5 s，GraspNet 的推理耗时必须计入预算。不得为了凑候选而放宽时间门槛。

## 6. 坐标系与位姿适配（高风险，必须单独验证）

GraspNet API 的抓取坐标系约定是 **X 轴进近、Y 轴夹持闭合**；项目 `GraspCandidate` 约定是 **Z 轴进近、Y 轴夹持闭合**。二者不能直接把 GraspNet 的旋转矩阵转成四元数后发布。官方 API 还说明：`translation` 是抓取框架的位置，末端指尖位置与 `depth` 有关；夹爪坐标系需按实际机械手转换。[GraspNet API 位姿字段与进近轴说明](https://github.com/graspnet/anygrasp_sdk/blob/main/grasp_detection/USAGE.md)（该页面介绍的 GraspNet 表示约定同样适用于该 API 的 grasp frame，使用 Baseline 时仍须按实际版本源码复核。）

若定义 GraspNet 帧为 `G`、项目候选帧为 `P`，并确认两个坐标系原点一致，则一个满足轴约定的旋转转换是：

```text
R_G_to_P = [[ 0, 0,  1],
            [ 0, 1,  0],
            [-1, 0,  0]]
R_cloud_P = R_cloud_G @ R_G_to_P
R_base_P  = R_base_cloud @ R_cloud_G @ R_G_to_P
```

这令 `P +Z = G +X`（进近方向）、`P +Y = G +Y`（闭合方向），并保持右手坐标系。**原点通常不能仅凭轴转换假设相同。** 必须检查 GraspNet `depth`、手掌/指尖参考位置、PiPER 实际夹爪 TCP 和 `link6 -> TCP` 变换，定义并标定 `T_GraspNet_to_project_TCP`。把变换写成明确函数和配置，不要散落硬编码；不确定的 GraspNet `depth` 语义先用官方 gripper 可视化和已知几何例子确认。

坐标链路应明确写为：

```text
GraspNet 相机帧候选
  -> 原始采集时间的 TF (camera frame -> base_link)
  -> GraspNet gripper frame -> 项目 grasp-center/TCP frame 的标定变换
  -> MoveIt link6 目标（应用已标定的 link6 -> TCP = 0.105 m）
```

同一个 0.105 m TCP 变换只能应用一次。用人工构造的轴向测试（纯 `G+X`、纯 `G+Y`、纯 `G+Z`）验证预览夹爪模型的方向；再用已知相机/基座点验证平移和 TF。检查旋转矩阵正交、`det(R)=+1`、四元数归一化；无效变换拒绝，不做“看起来差不多”的补偿。

## 7. 候选过滤、排序和 ROS 消息

GraspNet 的分数只用于候选相对排序，不应标注为成功率或概率。按以下次序处理：

1. NMS/去重，限制返回候选数量；保留网络分数和原始序号用于诊断。
2. 按 PiPER 实测最大/最小开口、夹爪宽度单位和限位过滤。当前历史配置中最大值为 0.075 m，但部署前必须从实际夹爪配置、URDF/SDK 和现场标定核对，不能把旧值当成已验证机械限位。
3. 运行 GraspNet 点云碰撞过滤作为初筛。它只用输入点云估算夹爪碰撞；不能替代完整 MoveIt 场景中的机械臂、自碰撞、桌面、底盘和障碍物检查。
4. 可按任务约束限定进近方向；瓶子初期可优先测试瓶身中段侧向/竖直夹持与可用的上方进近，不把瓶盖、瓶肩或桌面方向不合适的候选直接送 planner。约束方向必须在同一相机坐标系变换后使用。
5. 将合格候选变为既有 `GraspCandidateArray`；`generator="graspnet_baseline"`，`score` 写模型排序分，`gripper_width_m` 写米制开口，`pose` 按第 6 节转换，header 沿用输入。
6. planner 对候选逐个进行 MoveIt `plan_only`；检查抓取位和预抓取位的 IK、碰撞与完整进近路径，不以 GraspNet 分数代替可达性。记录每个候选的拒绝原因；避免只报告一个模糊的 “No reachable candidate”。
7. 给候选排序时先应用硬约束（帧/年龄/姿态/开口/碰撞/IK/路径），再在可行集内综合 GraspNet 分数、关节限位余量和路径代价。不要因为分数高而保留不可达候选。

在已确认运行中的 planner/executor 版本前提下，新增一个明确的“所选抓取”接口（建议 `SelectedGrasp.msg` 或等价接口），至少携带选中 grasp pose、对应 pregrasp pose、开口宽度、预抓取距离、抬升参数、采集 header 和候选 ID/网络分数。当前单独发布的 `PoseStamped /planner/selected_pregrasp` 无法把这些候选参数完整传给执行器。task executor 必须读取**同一个被选中的候选**参数；不得重算成旧固定 0.10 m 或丢弃宽度后仍宣称使用了 GraspNet 候选。

候选网络参数不能决定夹爪闭合控制器的 `gripper_close_m` 或 effort。实施者需要基于 PiPER 的实测夹爪映射定义安全的开口、接近、闭合和接触判定；先在仿真/离线模式验证参数传递。保持现有接触阈值，不得为提高成功显示而降低阈值。

## 8. 实施阶段与验收门槛

### 阶段 A：运行链路审计

- 找到真正运行的 `robot_planner`、`task_executor`、感知与抓取节点源码、安装前缀和版本。
- 画出 `/grasp_perception/target -> /perception/target_cloud -> /grasp/candidates -> /planner/preview -> /planner/selected_pregrasp` 的实际 publisher/subscriber 图。
- 核实活动 ROS2 图中抓取候选 topic 只有一个有效 publisher；保存 `ros2 node info`、`ros2 topic info -v` 与包版本记录。
- 交付：源代码位置/commit、消息字段消费情况、现有宽度和预抓取值使用路径、TF/header 时间语义。

### 阶段 B：隔离安装和官方 demo

- 固定上游 commit、Python/Torch/CUDA/Open3D/编译器版本和预训练 checkpoint SHA-256。
- 在独立环境中构建 pointnet2/knn 扩展；先用官方 demo 跑项目真实相机或录制数据，确认输出候选与夹爪模型可视化一致。
- 至少使用多种瓶子姿态、尺寸、距离、点云稀疏度和遮挡样例；记录空候选率、候选生成延迟、GPU/CPU占用。
- 交付：可复现环境、模型和 license/source 信息、demo 截图/视频、延迟统计、失败样例。

### 阶段 C：ROS2 适配和候选可视化

- 新建 GraspNet adapter 或将模型封装为可配置 generator；PCA 仅保留为可切换对照，不同时发布。
- 复用当前身份绑定点云输入，输出格式符合现有 `GraspCandidateArray`；记录推理耗时、候选数和逐项过滤计数。
- 在 RViz 里显示夹爪开口、进近轴、抓取中心、预抓取位、深度/指尖位置；必须能辨别平移/轴转换错误。
- 交付：录制 rosbag 回放时的 topic echo、RViz 图、无效/过期帧拒绝日志、消息接口测试。

### 阶段 D：MoveIt 预览筛选

- 对候选执行 plan-only，不触发 task start；遍历配置上限内的 Top-N 候选。
- 输出每个候选的模型分、开口、`grasp_depth`、TF 年龄、预抓取 pose、MoveIt error code、路径点数/长度和拒绝原因。
- 被选中接口携带同一候选的全部执行参数；检查候选宽度和固定抓取参数是否被 task executor 消费。
- 交付：离线/仿真场景下可重放的规划报告；没有候选可达时清楚区分 IK 无解、碰撞、进近路径失败、候选过期或输入无效。

### 阶段 E：对照评估

用同一批保存的目标点云分别运行 PCA 和 GraspNet，保持相同 TF、MoveIt 规划场景、候选数量上限、目标身份和观测时间。至少统计：

- 每帧候选数、有效开口率、点云碰撞过滤率、MoveIt 可达率（Top-1/Top-5/Top-N）。
- 预抓取路径规划成功率、平均及 P95 推理/规划延迟、各类失败原因数量。
- 若在可控仿真里做物理抓取，分别记录抓取成功、滑落、碰撞、空抓；不能用模型分数或规划成功代替物理成功。

先用至少 50 个不同目标观测/摆放做离线筛选统计；同一段视频相邻帧不应都算独立物体样本。阈值按真实数据设定，不预先声称特定百分比即代表可部署。相同测试集必须同时报告 PCA 基线。

### 阶段 F：部署准备审查

部署前先在仿真和 `execute=false` 预览通过，确认模型部署主机、模型文件校验、GPU/延迟、TF 标定、夹爪参数、MoveIt 场景完整性和失败停机行为。当前项目文档记录互锁/行走姿态尚未达到实机放行条件，因此这份任务书的验收终点是“候选生成 + 规划预览 + 报告”，不是实机执行。

**禁止事项：**不设置 `execute=true`；不打开 `auto_enable`；不发布 `/task/start`；不发 PiPER/CAN 命令；不向底盘发送导航命令；不移除、绕过或伪造互锁、停稳、反馈年龄、目标身份、接触阈值和恢复锁；不自动重试抓取。任何实机试验都需要另行完成项目安全条件和现场验收。

## 9. 最终交付清单

6Astra 完成后提交：

1. 本文档各阶段完成/未完成清单及实际源码路径。
2. GraspNet 与 GraspNet API commit、checkpoint 来源与哈希、环境锁定文件、安装日志。
3. 位姿/坐标变换推导、GraspNet depth 到 PiPER grasp center/TCP 的验证证据、夹爪开口实测映射。
4. 新旧候选同包络 A/B 指标，包含 Top-K 可达率、推理/规划耗时和失败原因。
5. 离线/仿真运行命令、rosbag 或测试输入、RViz 截图、结构化报告。
6. 明确声明是否仅完成离线/规划预览；在没有额外批准和安全验收前，不得报告“实机抓取成功”。

## 10. 上游参考

- [GraspNet Baseline 官方仓库](https://github.com/graspnet/graspnet-baseline)
- [GraspNet Baseline 安装、权重和 demo 说明](https://github.com/graspnet/graspnet-baseline/blob/main/README.md)
- [GraspNet API grasp 数据结构](https://github.com/graspnet/graspnetAPI/blob/master/graspnetAPI/grasp.py)
- [GraspNet 抓取坐标系、depth 与进近方向说明](https://github.com/graspnet/anygrasp_sdk/blob/main/grasp_detection/USAGE.md)
- 项目链路与实机边界：`docs/unified-flow.md`、`docs/搜索靠近抓取使用说明.md`、`config/scenarios/pick_and_hold/moveit.yaml`

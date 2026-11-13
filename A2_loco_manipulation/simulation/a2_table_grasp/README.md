# A2 + PiPER 可交互桌面多目标 MuJoCo 实验

当前网页可选择水瓶、苹果、橙子、鸡蛋、易拉罐和水杯的参数化刚体模型。苹果、橙子、易拉罐、水杯已在一组默认位置完成接触抓取验收；鸡蛋完成了规划与执行，但持物阶段滑落，网页保留失败录像。具体尺寸、感知方式和验证边界见 [目标清单](TARGET_CATALOG.md)。非水瓶目标使用已知目标位置辅助的深度 ROI，不属于 YOLO 自主识别；纸巾盒和其余 76 个 YOLO 类别尚未建成可执行模型。

网页的“放回桌面原位”复选框开启后，仿真会在持物验证后沿已执行的抬升轨迹回落、确认目标靠近桌面、张开夹爪、检查目标由桌面支撑，然后退出并返回零位。放置结果与抓取保持结果分开记录。该仿真路径没有调用真机代码中的 `place_left_and_return()`；原函数尚未接入主任务状态机，且按固定左转角松爪前没有桌面支撑验证。易拉罐默认场景已通过抓取和放回的完整物理测试。

固定站立的 A2、从六关节零位出发的 PiPER、前方桌子和自由运动水瓶。网页重放实际 MuJoCo 物理轨迹，支持旋转视角、瓶子点云与抓取位姿显示、阶段跳转、时间拖动和完整录像。


## 当前运行后端：主工程 ROS2 / MoveIt

网页“运行抓取”现已切换至 `../ros_pipeline/run.sh`，不再使用本目录旧 `simulate.py` 的独立 SciPy IK 动作脚本。真实链路为 YOLO 掩膜 → 深度点云 → GraspNet/共享几何适配 → MoveIt → ROS 执行器 → MuJoCo。旧脚本保留用于对照，历史结果在网页明确标注。首次需运行主工程 `scripts/graspnet/build_pipeline.sh`；详细命令和边界见 `../ros_pipeline/README.md`。

默认主链路成功样例可用 `http://127.0.0.1:8766/?sample=1&autoplay=1` 查看，来自真实 ROS 节点与 MuJoCo 接触结果，不是旧脚本录像冒充。最新复验 `../ros_pipeline/end_to_end07/` 最低保持抬升约 10.2 cm，双指接触覆盖100%，指尖余量约30 mm。保留用户当前参数，未强行把未通过路径检查的参数改成成功。

部分旧段落描述独立脚本的历史实现，不能当作当前主链路证明。完整修复/未解决问题见主工程 `docs/抓取代码审计与MuJoCo接入_20261106.md`。

## 水瓶左右移动

新增“水瓶左右位置”滑条和数字框：−25～25 cm，相对 A2 朝向，正值向左（世界 +Y），负值向右。左右调节只移动瓶子，桌子和 A2 不移动；前后距离仍按原约定带动桌子一起移动。改动后旧点云和结果失效，点击运行重新采集/推理/计算。虚拟深度相机自动朝向新瓶子位置，近景镜头随瓶子调整。

CLI 新增 `--bottle-y`（米），API 新增 `bottle_y`；旧请求和历史记录缺少该字段时按 0 处理。左右各偏移 8 cm 的真实物理抓取验证均通过，最低保持抬升约 11.4 cm；浏览器已检查左右预览、回中和手机布局。10 项测试通过。

## 当前抓取策略：瓶身中段 + 有效夹持面（v2）

修复原策略的指尖浅夹和低位抓取。`grasp_refinement.py` 从渲染目标点云拟合直立瓶身圆柱中心及可见高度，在观测高度的 50% 处选择夹持中心，保留 GraspNet 候选方向。位置已做几何修正，不再宣称完整使用原始网络位姿；原始候选、原位置与修正量保留在 `proposals.json` 中。

PiPER 有效夹持面中点取当前 MJCF 的 link6 +Z 0.105 m，指尖取 0.13503 m，二者与用户可调的 TCP 是不同概念。目标 TCP 为 `contact_center_world + approach_axis * (tcp_m - 0.105)`，保证瓶身落在有效夹持面中段，期望指尖余量约 30 mm。改变 TCP 时不改实体夹爪长度，不自动覆盖用户参数。该夹持面定义适用于现有仿真模型，未当作真实 PiPER 实测标定。

网页显示“实际指尖余量”和“瓶底以上接触高度”，由 MuJoCo 实际接触点分别换算到 link6 和瓶子局部坐标得出，随回放更新。成功判据在原抬升/双指保持之外，新增保持期接触余量中位数至少 8 mm 的检查，避免指尖边缘夹持仍被显示为通过。

物理验证：默认场景、61 cm 距离/63 cm 桌高/8.5 cm TCP，以及原失败的 61 cm 距离/60 cm 桌高/8.5 cm TCP 均重新计算成功。原失败样例接触高度从约 5.5 cm 提高到约 10.5 cm，指尖余量从约 0–0.4 mm 提高到约 30 mm，最低保持抬升约 11.5 cm，双指接触覆盖 100%。原始物理文件仍保留，结果见 `pad_refinement_validation.json`。`validate.py` 现在检查 `output_pad_middle_default`；8 项模型/参数/几何适配测试通过。

该适配针对当前直立、近似圆柱水瓶；点云几何无法支持拟合时会报错，不用仿真物体真值位姿替代感知。尚未推广到任意形状、倾倒瓶子或真实透明瓶深度数据。

## 当前版本：95 mm TCP + 可调实验台

仿真 TCP 已改为 link6 局部 `+Z = 0.095 m`（9.5 cm）。仅修改此独立实验环境，未更改实机 ROS 的标定配置。

启动：

```bash
cd /home/xubo/桌面/she/A2_loco_manipulation/simulation/a2_table_grasp
./start_web.sh
```

浏览器打开 `http://127.0.0.1:8766/`。现在必须使用此脚本提供的本地 API；单独启动 `python -m http.server` 只支持只读回放，不能运行新实验。

操作步骤：

1. 用滑条或数字框修改**水瓶前后距离**（50–110 cm，相对 A2 机身原点沿世界 +X）和**桌面高度**（50–105 cm，相对地面）。桌子随水瓶前后移动，瓶底始终置于桌面。TCP 默认 9.5 cm，也可在 7–13 cm 范围内调整。
2. 三维场景立即预览新位置；旧点云、抓取标记与成功数据隐藏，旧相机图像标记为待更新。预览不等于已做物理计算。
3. 点击 **运行抓取**，服务重新构建 MuJoCo 场景、渲染深度、运行真实 GraspNet 权重、筛选 IK、计算接触动力学并生成回放/录像。无需终端操作。
4. 运行结束后自动载入该次结果。不可达时显示无候选；物理抓取未通过时如实记录失败，不沿用上次成功。可以再次调参，或恢复默认场景。
5. **停止计算**只停止本次仿真进程及其模型 worker。允许旋转/缩放视角；本服务没有机器人通信。完成结果和日志保存在 `runs/<id>/`，最新结果通过 `current_run.json` 在刷新/服务重启后恢复。

TCP 变动时同步调整原生候选到 TCP 的平移：`tcp_from_tip_m = 0.135 - tcp_m`。这改变参考点，不改变手指实体几何。`link6 -> TCP` 只在 MuJoCo FK/IK 链中应用一次。原始 MJCF 指尖参考约 0.13503 m，此处沿用原适配器 0.135 m 的近似，与真实硬件标定仍分开。

验证：`output_tcp95/metrics.json` 为 95 mm 默认场景真实物理结果，最低保持抬升约 10.3 cm、双指接触覆盖 100%。网页端另实测了 74 cm 距离/78 cm 桌高的新成功样例，及 110 cm 距离/105 cm 桌高的不可达样例，之后恢复默认并成功重新计算。`interactive_validation.json`、`interactive_failure_validation.json` 保存浏览器检查结果；4 项参数/模型一致性测试通过。接口已验证参数越界拒绝、并发运行拒绝和停止计算。

新增 `server.py` 使用 Python 标准库启动仅绑定 127.0.0.1 的服务。命令由固定脚本和受限数字参数组成，不接受任意 shell 命令。`export_web.py` 支持按实验导出数据，共享未变化的网格以避免每次重复存储。静态成品包只能作为只读浏览，交互计算需要本地服务及现有仿真/GPU 环境。

## 历史版本结果（TCP 105 mm）

`output_v2/metrics.json` 为成功记录：水瓶 200 g、半径 26 mm、桌高 0.76 m；真实 GraspNet 权重对渲染深度点云推理，保留原始网络候选方向，经预抓取/抓取/抬升 IK 筛选后执行。保持阶段最低瓶子参考中心抬升 0.10299 m，双指接触比例 100%，持续 2 秒。没有焊接、吸附或回放期间强行设置瓶子位姿。

这只是一个仿真样例，不是泛化成功率。A2 底座固定、腿关节冻结于站立姿态，不测试全身平衡。PiPER 安装位姿复用现有 CAD 估计 `[0.0899, 0, 0.1416, 0, 0, 0]`，未实物标定。夹爪从 70 mm 开口开始；原始候选预测宽度 71.47 mm 超过仿真夹爪 70 mm 的最大开口，因此实际预张开固定为模型极限 70 mm，不能把预测宽度当作已消费的执行值。物体直径 52 mm 可通过该开口。此处独立 MuJoCo 程序没有修改 ROS 节点的夹爪控制。

点云来自虚拟 RGB-D 相机的真实深度渲染与物体 ID 分割（oracle segmentation），水瓶用不透明刚体近似；未模拟透明塑料的真实深度缺失。模型推理约 66 ms，未计入环境预热和规划。未使用 RL、行为克隆或教师轨迹。IK 只做目标可达性，未实现完整 MoveIt 路径规划或跨场景碰撞规划验收。

## 文件

- `simulate.py`：合并已有 A2/PiPER MJCF 网格与运动学，站立场景、深度采集、隔离 GraspNet worker、候选筛选、IK、物理抓取与视频。
- `export_web.py`：导出 MuJoCo 网格与每帧真实刚体变换供 Three.js 回放。网页的模型运动来自 `mj_forward` 还原的 MuJoCo 状态，不是另画的示意机器狗。
- `site/dist`：静态网页、几何网格、运动记录、点云、候选、指标、全景/近景视频。
- `output_v2`：通过的实验。`output` 是第一次未通过候选可达性筛选的诊断，不作为成功结果。
- `validate.py`：独立核对当前 95 mm TCP 默认样例的接触/抬升、场景固定方式、零位和导出一致性。

## 复现

在 she 根目录：

```bash
supermimic训练机械臂抓取/.venv/bin/python \
  A2_loco_manipulation/simulation/a2_table_grasp/simulate.py \
  --bottle-x .72 --table-z .76 \
  --output A2_loco_manipulation/simulation/a2_table_grasp/output_new

supermimic训练机械臂抓取/.venv/bin/python \
  A2_loco_manipulation/simulation/a2_table_grasp/export_web.py \
  A2_loco_manipulation/simulation/a2_table_grasp/output_new

python3 -m http.server 8766 --bind 127.0.0.1 \
  --directory A2_loco_manipulation/simulation/a2_table_grasp/site/dist
```

浏览器访问 `http://127.0.0.1:8766/`。重导出会更新网页展示数据，场景和视频保留在指定输出目录。

使用现有 MuJoCo Python 环境和 `search-grasp2/.venv-graspnet`；GPU 必须可用。源码/权重 SHA256 验证由上一阶段的独立 worker 执行。没有引入 ROS/CAN/底盘通信。

模型来源：A2 为 `A2_Piper_WBC/vendor/unitree_ros/robots/a2_description`（包含同源 URDF）；PiPER 为 `PiPER抓取完整项目/PiPER抓取项目/vendor/agilex_piper`。安装估计参见 `isaac lab A2+机械臂/logs/a2_piper_assembly.json`。许可证保留于网页 `assets/licenses`。

## 网页交付与浏览器验证

本地交互启动：`./start_web.sh`，或用 `./start_web.sh 端口号`。浏览器访问 `http://127.0.0.1:8766/?autoplay=1` 自动播放；不带参数时从零位暂停开始。运行中的本地服务保留供本次查看。

Chromium 桌面 1440px、手机 390px 浏览器检查通过，包含模型加载、播放、时间拖动到保持阶段、点云/位姿开关、录像/设置切换，无横向页面溢出和脚本错误。结果在 `web_validation.json`，预览图在 `preview.png`。

已按 Sites 技能创建 owner-private 站点并保留 `.openai/hosting.json` 身份，但源代码服务 `git.chatgpt-team.site` 的 TLS 握手失败，尚未 push 或部署，因此没有可声明已上线的云端链接。未关闭证书校验。静态成品包在 `site-deploy.tar.gz`，本地网页不依赖该服务，也不需要外部 CDN 或在线字体。

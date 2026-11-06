# 瓶子靠近与一次抓取统一入口

> 更新：按用户要求保留原底盘服务，新增现场监督的两段入口。当前操作请先读 [两步操作](两步操作.md)。本文中的独占模式及候选驱动说明不等于监督模式；没有删除原有独占检查。

2026-10-28。主工程 `/home/unitree/A2_loco_manipulation`；旧 `/home/unitree/a2_piper_project` 仅作为核对来源。本次没有修改 `/home/unitree/a2_nav`、底盘服务或设置开机启动。

**当前交付是已部署、可诊断的启动入口和受阻保护的任务实现，不是已打通的实机自动抓取。** 底盘独占控制互锁尚未接入，真实 run / pick-once 不具备放行条件。不能通过删除任务锁、伪造 authority 响应或手工转发速度绕过。

## 命令

在机器人上执行：

```bash
cd /home/unitree/A2_loco_manipulation
./a2ctl up
./a2ctl status
./a2ctl run --task-id bottle-001 --goal bottle --timeout 180
./a2ctl pick-once --task-id bottle-002 --timeout 120
./a2ctl abort
./a2ctl down
```

`up` 只启动环境，不调用导航 start、机械臂 enable 或 task/start。返回码 3 表示未就绪，不能继续执行；2 表示命令错误。进程被分离到后台，SSH 断开不会重新提交任务。`status` 区分进程存活、组件健康和真实任务可用性。`nodes` 保留原 ROS2 节点列表功能。

`run` 的 goal 为项目已有瓶子目标配置 `bottle`，不是地图坐标。采用现有跟踪器、目标管理器、object_navigation_agent。使用主工程 YAML 的 stop_distance=0.52、distance_tolerance=0.02，即相机光学 Z<=0.54m 触发到达。该值来自已有现场参考，未在本次验证真实制动距离。不能宣称已完成实机自主寻找/视觉靠近。

`run` / `pick-once` 的 accepted 只表示接收任务；用 status 查看最终结果。ID 持久去重，同一 ID 不再执行。失败、超时、取消和持物均保留恢复锁，不重试、不松爪、不回零、不返回导航。

`abort` 分别报告本入口导航取消和抓取软件中止是否确认；没有提交过的任务不调用外部执行器中止。软件中止不是硬件急停。

## 实现与环境

代码：`scripts/unified/`。总控使用标准 Python、Unix socket JSON RPC；ROS1、ROS2 适配器分别在干净 shell 加载本工程环境。只有专用 ros1_bridge 进程按工程既有方式加载两种 ROS 环境。

ROS1 项目图使用 11349；独立只读适配器从现有 11311 读取 `/a2/odom`，不向该图发布运动命令。导航请求保留在 `/a2/navigation_velocity` 和 `/a2/navigation_mode_request`，没有连接 `/cmd_vel`。旧模拟 motion_guard 不被当作实机互锁。

ROS2 使用 `single_view.launch.py → pipeline.launch.py`，只启动一套驱动、相机、抓取分割、候选、规划和任务执行器；auto_enable=false、execute=false。导航跟踪器与按需抓取分割是工程已有的两个不同阶段，抓取分割启动时保持 IDLE。没有另起旧 single-pick-session、旧 piper_vision_grasp 抓取入口或额外抓取分割进程。相机 launch 的复用不等于启用旧抓取入口。

身份交接复用 target_exporter 和已有 String 协议。一次抓取绑定 tracker_session / track_id / class_id；只转发停稳后、身份匹配且新鲜的 BoundTargetCloud 给旧候选生成器。此接线有离线测试，尚无实机端到端验证。

抓取调用顺序：检查控制权和反馈 → 连续实测停稳及导航零指令 → 检查观察姿态 → 绑定新观测 → 新 preview → enable → execute=true → task/start → 等待最终状态。等待预览期间继续监测停止反馈。预览选择的原始观测过期则拒绝，不通过放宽年龄或接触阈值强行继续。

TCP：主工程 `config/scenarios/pick_and_hold/moveit.yaml` 已从 0.110 改为用户要求的 0.105；运行节点也读回 `[0,0,0.105]`。旧项目原值 0.105 未改动。接触阈值未修改。

HOLDING 显示“执行器报告持物保持”，附接触反馈；`object_grasp_confirmed=false`，不等于视觉或现场确认抓住瓶子。

## 当前阻碍和现场边界

- 现有底盘驱动接受多个运动入口，没有可验证的独占租约／抓取期间底盘禁止运动接口。`authority_socket=null`，任务拒绝执行。外部 arbiter 只定义了扩展契约，尚未实现或验证；不能把接口存在视为互锁已经完成。
- 当前观察姿态取自工程已记录的 `[0,0,0,0,5,0]°`。该记录不证明它适合底盘行走，`travel_joint_deg=null`，run 拒绝。没有自动姿态切换路径。
- ROS 反馈的持续发布不完全等于 SDK 原始反馈时间有效；实体驱动反馈可靠性仍需现场完整验收。
- 启动发现已有外部同类节点/进程或 11349 master 时拒绝，不自动停掉、不猜测复用。不支持在未知旧持物状态下启动新的执行器。
- 初次启动检查发现 can0 DOWN；后续读回已 UP、1Mbps。没有发送电机使能或导航/抓取命令。
- 单一 ROS 导航 String 状态没有原生 task_id，适配器要求本轮 running 后再接受终态；真正避免外部客户端并发仍依赖尚未接入的控制权互锁。

## 日志与恢复

`data/logs/unified/` 下：supervisor.log、每个组件 `.log`、state.json（持久锁和去重）、events.jsonl、owned.json（本入口拥有的 PID 和启动标识）、last-status.json。

启动前备份：`data/logs/unified-install-20261028/a2ctl.before` 和 `moveit.yaml.before`。新模块是 Python 脚本，不需要重新构建原 ROS 包；现有构建和安装 launch 通过原路径使用。

持物/恢复锁存在时 down 拒绝。现场处理物体、确认机械臂和底盘停稳后，且反馈适配器仍在线，可记录恢复：

```bash
./a2ctl recover --task-id bottle-001 --note '现场已处理物体，夹爪无物，机械臂及底盘停稳，已确认故障原因'
./a2ctl down
./a2ctl up
```

此命令不移动、不松爪、不使能；只是现场记录加反馈检查。它不会删除旧任务 ID。若监督进程崩溃，重启保留 RECOVERY_REQUIRED；若反馈适配器不能恢复，恢复命令仍拒绝，需工程人员检查原进程及物理状态。当前不提供离线强制解锁。不要删除 state.json 或重启机器来当作持物处理。

## 验证结果

- 14 项总控模拟：到达/失败/取消/超时、抓取终态/失联、控制权缺失、未停稳、姿态不符、重复任务、崩溃持久锁、恢复后重启要求等。
- 6 项 ROS2 环境内无节点测试：旧采集时间、停稳前数据、未来时间、缺身份点云、另一瓶子身份拒绝。
- 6 项 ROS1 环境内无节点测试：连续实测低速和零命令、旧反馈、短窗口及 master 响应解析。
- 无运动启动已运行相机、跟踪、按需分割待机、规划和任务节点；实际 TCP 读回 0.105。初次驱动因 CAN DOWN 未就绪，启动返回未就绪，不报告成功。
- CAN 恢复 UP/1Mbps 后再次无运动启动：arm/nav errors 均为空，environment_ready=true，TCP=0.105，executor=WAITING_FOR_PREGRASP，navigation=idle，stopped=true。ready 仍为 false（缺独占互锁和行走姿态）。随后复查出现 `can0 is loss`，驱动退出，且相机发生 USB 传输错误和重连；就绪状态退回 false。该故障未被自动重试。最后关闭本次启动的进程，CAN 接口仍保持 UP。全程没有调用使能或任务开始。
- 重复 up 未重复拉起组件；down 只关闭本入口创建的进程。原有底盘服务保留。

以上是模拟和无运动验证。未执行真实靠近、姿态切换或抓取，完整实机流程尚未验收。


## 2026-10-28：已有相机的复用修复

`up` 现在会只读检查已有 `/camera/camera` 的对齐参数、新鲜 RGB-D／内参、尺寸、坐标系和发布者唯一性。通过后以 `start_camera:=false` 启动 single_view，status 显示 `camera.reused=true`、`ownership=external`。只读预览节点不再仅因名称含 camera 被误判为冲突；未通过兼容检查的相机仍拒绝复用。down 不关闭外部相机和预览。

如果上次仅预检失败、尚未启动任何子进程和任务，也没有恢复锁，重复 up 会退出空的失败监督进程并重新检查；不会重试已经提交的任务。未启动的适配器显示明确原因，不再显示误导性的 Connection refused。

本次机载验证：复用相机成功，boot_error 为空，arm/nav errors 为空，environment_ready=true，TCP=0.105，导航 idle；没有使能或执行运动。ready 仍为 false，底盘互锁与行走姿态问题未因这次修复而解决。


## 两段式操作更新

按用户要求，`run --task-id ID --goal bottle` 现在只完成第一段靠近和停稳检查，然后进入 AWAITING_PICK。需要用户再执行 `pick-once --task-id ID` 才继续第二段抓取；必须是同一 ID。第二段开始前重新检查停止反馈，并重新获取观测和规划，不沿用等待前的数据。

等待第二段也计入 run 的总 timeout。等待期间中止、超时或失去停止/控制权证据均不会抓取；down 不能越过活动任务锁。独立 pick-once（用于其他方式已到位）仍不导航，也仍要求控制权和停稳证据。新增 3 项回归后总控测试为 17 项，本地和机载均通过。

底盘互锁驱动候选已放在 `src/hardware/a2_guarded_base` 并编译，尚未启动或接入总控。旧底盘服务未修改。实际运行条件未因两段式更新而解除。

# 验证状态

## 2026-11-23 双方法发布验证

两套方法使用同一个静止仿真初始快照 `anygrasp-baseline-start.json`、11 cm TCP、同一 Robot Lab 物理与放置代码，各执行一轮完整网页 API 抓放。GraspNet 与 GraspGenX 均抓起并持物回零、在腕部 RGB-D 重新选空白桌面点、松爪、MTC 撤离回零，最终任务状态为 `idle`。放置结果中的 `support_confirmed=false` 表示没有力传感器确认支撑；RGB-D 可见瓶体直立，不能把它写成真机受力验证。两种方法的开始落点选取不同，放置耗时不可归因于抓取模型本身。

- GraspNet：`go2-grasp-1795363474437276608` 和 `go2-place-1795363519583944948`；抓取约 44.32 秒，首次可执行候选约 8.52 秒，模型推理中位约 0.050 秒，放置约 26.46 秒，均通过。
- GraspGenX：`go2-grasp-1795363580249905511` 和 `go2-place-1795363634358281369`；抓取约 46.07 秒，首次可执行候选约 10.40 秒，模型推理中位约 0.104 秒，放置约 39.36 秒，均通过。
- 两次初选放置点在新的腕部深度帧中被障碍检查拒绝，没有开始机械臂运动；重新采图并选可见桌面点后执行成功。这是正常的选点重试路径，Agent 必须按错误信息重新观察，不能盲重发。
- 测试中还发现释放时保留旧下降关节目标会使末端下沉，已改成在松爪开始前固定实测关节；释放后 MTC 撤离曾因“必须仍持物”而拒绝，已允许已释放但仍属于同一放置任务的模型导出。两项修复已包含在本快照的 `integration/go2_grasp.py`，复测后两方法均通过。
- `agent_client.py` 的 `status` 和 `capture` 在隔离网页服务上验证，RGB PPM 为 640×480，深度文件 1,228,800 字节、遮挡掩码 307,200 字节。完整 overlay 在固定 Robot Lab 基版的独立工作树中试装，67 个目标文件与发布清单逐一 SHA-256 一致。

以上各只有一轮成功样本，不能推断统计成功率；未做独立机器安装、不同物体/落点批量验收、真机接触/急停验收。推理性能只统计持久 worker 已就绪之后的单帧模型调用，端到端时间包含感知、启动 ROS/MoveIt、规划和物理运动。

## 2026-11-18 TCP 调整

网页抓取请求及命令行默认 TCP 从 0.105 m 调为 0.11 m，沿 link6 局部 Z 轴。Python 编译与 diff 检查通过，未进行运动验收。在线仍保留失败任务现场，本次未重启服务，旧进程尚未加载新值；未推送 GitHub。

## 2026-11-14 抓取页面单次规划与执行

网页不再要求先运行独立预览才能执行。点击「规划并执行抓取」后，现有 `run` 流程在同一次任务内完成新图采集、GraspNet 候选、MoveIt 预览与执行；独立预览保留为可选诊断。后端仍要求明确执行确认、有效像素和新鲜 RGB-D，并保留底盘位姿、碰撞与任务占用检查。未改变 CLI 和机器人运动控制。此项只消除用户主动先预览再运行造成的重复整链计算，尚未实现 ROS 节点常驻和抓取速度调参。

## 2026-11-13 本地抓取代码开发适配

当前保留 feature/Lin 和依赖锁；本地覆盖读取使用者的 A2_loco_manipulation 源码，
未修改原源码，也未迁移A2底盘、导航、WBC或硬件驱动。入口为 bin/lab grasp。
适配连接同一个 Go2＋Piper 物理世界的真实RGB-D、臂关节反馈和轨迹目标，
导出当时场景的URDF/SRDF供MoveIt规划；夹爪碰撞链接映射为源规划器的p_link7/p_link8。
不通过传送物体、weld或第二个旧A2世界制造成功。

已完成：真实相机采集、当前模型导出、CLI帮助、无execute执行拒绝、Python编译检查、
只读verify；外部工作区 local-tools/test_go2_grasp.py 的9项隔离物理测试通过，
覆盖稳定性门槛、跨会话拒绝、非法关节/NaN、轨迹完成、心跳停止、底盘漂移、导出路径及夹爪别名；
导出URDF在三组臂关节位置下的link6正运动学与MuJoCo一致（位置/旋转绝对容差1e-8）。
这些测试不向正在运行的世界发送抓取轨迹。

当前真实相机图中未找到瓶子，preview以 selected_pixels=0 明确失败，未启动执行。
完整ROS节点协作、MoveIt规划及接触抓取尚未端到端验证，不宣称抓取成功。
需用户先将机器人停到目标可见且可达的位置，再选像素预览；本轮未自动驱动靠近或执行抓取。
证据位于外部工作区 runs/go2-grasp-*，源码指纹随每次请求记录。
本地适配所依赖的ROS构建目录和GraspNet环境尚非从锁文件自动构建的发布链路。
本轮源码修正没有强制重启当前预览，新的物理服务改动需下次正常停启后生效。


代码整理阶段仅做离线检查；后续已按用户要求完成关闭、启动及网页视频验证，见文末。未执行实际导航或抓取。历史实验流水和自动化测试已备份至仓库外，不能用旧结果代替整理后的平台验收。

## 本轮检查

| 检查 | 本轮实际结果 |
| --- | --- |
| 宿主 CLI、帮助、配置与参数替身 | 36 项通过 |
| 集成、语义记忆、地形、恢复保护、仿真与诊断 | 162 项通过，4 组依赖环境跳过 |
| ROS2 抓取批次、任务、IK 及在线诊断包装 | 95 项通过 |
| ROS1 导航回调 | 7 项通过 |
| 网页后端语义、抓取交互与在线诊断 | 40 项通过 |

共 340 项通过。集成中跳过的四组已在对应 ROS1/ROS2 环境补跑；后端有既有的依赖弃用提示。另在 Python 3.8 标准库模式验证全部 15 个内部帮助入口，不重复计入上述总数。

全仓旧入口引用检查、源码语法检查和差异空白检查通过。帮助不加载 ROS、数据库、GPU 模型或渲染库。扫描返回与恢复参数、当前配置消费、进程身份和停止保护使用本地替身检查，不连接在线运动。

测试与详细日志位于用户外部工作区，正式源码不再维护自动化测试目录。

## 待验证

- 平地导航抓取整链路；平台启动、网页画面与视频解码已在后续实测验证。
- 新工作区完整构建和同伴独立机器部署。
- 楼梯仅保留现有三维导航验证配置；不宣称通用地形能力或真实四足爬楼。

未执行提交、推送或源码锁更新，稳定版标记保持关闭。详细本地检查命令、日志及四仓库待提交清单随外部交接交付。

## 后续启动失败修复

用户实际执行 up --development，预检 deployment_ready 为 true，但导航服务启动失败。navigation.log 记录组件退出，bridge.log 明确报 IndentationError。原因是合并 navigation_service 时，把嵌入 python -c 的桥接程序一并缩进；帮助和既有回归没有覆盖生成程序的语法。

已在生成桥接程序时去除公共缩进，保持原程序语义；启动失败现在指明具体服务、退出码和日志路径。新增的两项本地检查捕获 service_main 实际生成的子进程参数，编译其中的 Python 程序，并验证组件失败日志定位；所有 Docker/ROS 操作使用替身。

本次修复后重跑：宿主36项通过，集成164项通过、4组依赖环境跳过；生成桥接程序另在ROS1 Python3.8中通过语法检查，与整理前程序的AST一致。其他ROS/后端结果仍是上一轮快照记录，没有重新计作本次通过。本次未重启平台、未发在线运动；完整启动等待再次实际运行。

## 完整关闭与启动实测

按用户要求依次执行 `./bin/lab down`、`./bin/lab up --development`，两条命令均以0退出。导航桥接缩进修复后的完整启动通过；没有追加运动、场景重置或任务解锁操作。

`./bin/lab status` 确认platform、world、viewer、broker、navigation、grasp、gateway、agent均运行，UE signalling与streamer连接。OpenVox加载完成，持续处理RGB-D并发布观测，last_error为空。

默认 `./bin/lab verify` 以0退出，报告 `runs/20261109T021316Z/acceptance.json` 中passed=true。机身/腕部RGB-D各640×480，采集年龄分别约0.061/0.049秒，深度有效，物理世界和导航地图身份一致，抓取组件environment_ready=true。

实际登录本地工作台，页面显示设备在线、三维地图已连接。浏览器video的readyState=4，1280×720，paused=false，播放时间从11.89秒增长到38.01秒，已看到场景画面。视频证据为本次页面实测，不仅依据UE streamer连接状态判断。

当前会话b9f715239401497d9931b3acfee0009e。平台保留上一轮抓取任务占用、恢复及搜索姿态限制，没有通过自动清锁或驱动机械臂使ready变真；启动通过不等于已完成新一轮导航抓取。服务保持运行，原生观察窗口未额外打开。尚未执行完整运动验收、全新工作区或独立机器部署。


## 2026-11-09 全图语义（本轮最新快照）

新增 `bin/lab semantic build`，在独立渲染模型上完成 1,104 个采样视角，1,047 帧有效、57 帧因可见深度不足等拒绝。实际使用锁定 OpenVox 模型推理；最终发布 250 个至少双视角支持的语义候选、1,116 条物体照片引用（同一原图可以支持多个物体）。家具与小物件均展示；不能把候选数量视作人工核准的正确实例数量，也不保证看见遮挡内部物体。

最终版本 `0ef4d19bf16d49a0e8b268373e8ca962de299d417e289f6c1868f870f8f08bdb`，存于外部 `assets/semantic-atlas/`。经过原平台数据恢复和完整关闭/启动，语义接口无需扫描即返回250个候选，地图照片列表与JPEG下载、SHA256校验通过。隔离验收环境中在线照片也已实际保存并通过同一路由下载校验；在线照片按会话隔离，不将旧图片当成新观测。

本轮离线检查：宿主36项通过；集成164项通过、4组依赖环境跳过；ROS2批次/任务/IK/语义几何等98项通过；后端语义/任务53项通过；随后新增缓存地图不可变及预览检查，地图专项17项通过（与前述后端重叠，不累加）。前端 vue-tsc 与 Vite 构建通过。无ROS/GPU宿主的标准库CLI帮助、改动Python语法及差异空白检查通过。检查与日志均在外部 `local-tools/semantic-atlas`，不新增仓库内测试。

实际历史目标任务使用独立验收状态，通过公开HTTP检查200、提交201，完成导航到场和实测停稳。现场OpenVox成功选中冻结历史瓶子的当前RGB-D分割，取得新的抓取批次；GraspNet未生成可执行候选，任务停车保留失败锁，未进入执行。过程中修复能力心跳协议遗漏、目标绑定顺序、世界环境查询缺少时间戳以及ROS深度float64/模型float32不一致。过近作业点的另一次导航被规划器拒绝，已通过公开取消接口停下，未修改碰撞门限或导航算法。

**尚未通过**：历史目标完整物理抓取、杯/罐/书等多类别运动验收；默认场景未确认识别出罐子，没有为凑类别往默认场景填充物体。浏览器控制工具反复超时，本轮没有新的真实页面/图片缩放或视频解码证明；HTTP/JPEG与前端构建通过不等于浏览器实测。独立机器部署仍待验证。

验收临时平台和仿真状态完整保存在外部 `acceptance/semantic-atlas/test-state-20261109T045942Z/`。原平台与仿真目录、原 `config.local.json` 已恢复；核对28,898个文件，运行数据无差异，唯一校验差异来自指向本轮已编辑A2 DEVLOG的源码符号链接。原任务锁和历史记录未清除。本轮不提交、不推送；发布锁及稳定标记保持原状。


恢复后的最终只读 `bin/lab verify` 以0退出，`runs/20261109T050332Z/acceptance.json` 中passed=true，8个运行服务均在，OpenVox持续处理。此前刚恢复时首次verify因组件环境尚未就绪返回非零，失败日志保留；确认status的environment_ready=true后重跑通过，未发准备或运动来改变原任务状态。原工作台目前保持运行。


### 语义标记错位排查（2026-11-09）

用户反馈静止与转动视角时语义点偏位、显示高度异常。发现工作台视口内语义状态段落及错误提示参与正常布局，视频100%高度被向下挤出，而绝对定位的实例标记仍以完整视口投影。已将视频/平面画布固定到视口四边，状态和错误提示独立叠加，统一视频与标记坐标原点；不修改任何三维物体坐标或任务目标。前端类型检查/构建后交付，浏览器控制仍超时，尚需真实页面复核修复效果。

另对250候选的照片重新投影：103个候选至少有一个支持视角中代表点落在框外，77个候选不同视角的表面位置差异超过0.3m，最大约0.9m。大物体不同可见面的差异本身不是误检证明，但逐轴中位数不保证是实际表面点或整体中心，当前合并不构成准确定位验收。原始地图与证据保留，未通过整体平移/压低高度、改成场景真值坐标或直接删除候选来掩盖问题。详细报告在外部 local-tools/semantic-atlas/position-audit.json。

转动时HTML标记通过数据通道更新、画面通过视频传输，两者可能存在时序偏差；本次布局修复不能证明这项已消除。


## 2026-11-09 四仓库源码交接发布

按用户授权统一使用 main（保留原稳定基线）和 feature/robot-lab（本轮开发）。三个团队依赖先推送并核验，再锁定其精确 SHA；release_ready 保持 false。新增 GraspBatch、CapturedRGBD、TargetGeometry、SnapshotCommand 与感知、规划、执行配套实现一起发布，网页语义地图、照片与抓取诊断及导航地形/恢复改动同步交接。

本轮重新执行：宿主36项、集成164项（4组环境依赖跳过）、ROS2 95项、ROS1 7项通过；grasp_msgs 独立 colcon 生成通过；导航5个C++专项程序及20项Python检查通过；网页后端修正旧测试配置和家具显示预期后，隔离容器40项通过；前端vue-tsc/Vite构建通过，构建视图与改动源码一致。使用隔离容器是因为原平台容器被其他操作停止，本轮未启动或停止原平台。

语义地图专项对冻结发布快照单独复核，日志保存在本次交接工作目录。发布只包含开始整理时保存的代码快照及本次分支/交接修正；经用户确认，随后另一项任务新增的OpenVox实验修改保留在工作区，不纳入本次提交。不能将工作区之后的改动视作已发布或已检查。

不部署、不发运动、不清任务锁；未重编译UE，未新增浏览器实测。完整物理抓取、多类别抓取及独立机器部署仍待验收。旧分支在归档标签远端核验成功且发布检查通过后删除，任何并发更新或保护拒绝均保留并报告。


## 2026-11-09 独立预览依赖清理

移除独立 Habitat 预览的源码依赖、容器镜像与配置、构建、启动、状态、日志和验收入口及对应文档。默认完整部署只检查现代工作台、A2、导航和 GraspNet 四个锁定源码仓库；保留 MuJoCo 使用的 ReplicaCAD 场景及来源许可，默认物理场景和机器人控制不变。

本次检查：宿主代码通过 Python 3.8 语法检查；全部清单 JSON 可解析；14 个宿主帮助入口成功，已移除的预览参数在解析阶段拒绝；空工作区 bootstrap 替身检查仅请求其余四项依赖。真实只读开发预检 deployment_ready=true、errors=[]；场景全部文件校验值及其余源码锁匹配。没有重启服务、发送运动或重新执行完整容器构建，不将现有环境预检记为全新机器部署通过。

本次提交仅包含预览依赖清理，其他本地 Agent 搬运开发修改保留在工作区，不纳入发布。稳定版标记保持关闭。

## 2026-11-09 feature/Lin：三角色 Agent 与 MuJoCo 跨桌搬运

用户授权将本轮 Agent 并入 robot-lab，并发布到个人分支 `feature/Lin`；团队
`feature/robot-lab` 和 main 保留，release_ready 仍为 false。机械臂配套提交为
`fb7f2b15b826612c3885c782acfc660627f8c1a3`，已在同名远端分支核验并更新源码锁。

### 最终实现

- `bin/lab agent` 在现有 controller 容器启动 `integration.task_agent`，含
  Perceiver、Manager、Planner、共享记忆、ToA Harness、三个技能和交互网页。
  已验证环境没有 PARTNR、Habitat、Habitat-Sim、Magnum，使用已锁定的 HTTPX。
- 默认静态配置 `integration/config/simulation-transfer.json` 固定本例源深色餐桌
  和用户指定的有显示器浅木色高桌；含约0.18553m高差，校验场景清单SHA。
  物体位置使用原生YOLO/深度及采集时TF，物理真值只做独立验收。
- 无可见物体时，模型可以先生成单步来源桌面观察计划；须空手、目的地已绑定、
  路点真实且非目的地，最多两次。未绑定目标不能抓取或放置，观察完成不能结束搬运。
- 原生 `/sim/task/place` 复用已有放置几何与MoveIt规划，实际执行检查过的关节路径。
  保存本轮预抬升位姿，包含跨桌高度变换、测得到位/张爪和沿真实接近轴反向退出22cm。
  MoveIt携物表示仅用于规划；瓶子为自由物体，没有物体焊接、附着或任务中改写位姿。
- 基座改为跟随独立运动参考的自由滑动基座，使接触具有正确空间速度；约束只用于
  基座驱动。每指约16N、上限20N，指垫摩擦`1.5 .01 .005`、condim6；瓶子支撑接触
  `1 .01 .003`、condim6，NoSlip迭代3次。修复持瓶倾转及桌面摇摆。
- 搬运保持同任务新鲜HOLDING、观察连续性和碰撞扫掠检查。原生抓牢至少五秒，
  穿透上限1mm；最终指定桌面支撑、零手部接触、实测张爪和至少两秒双时钟稳定。
  仿真搬运/放置路径速度2倍，直线离桌/停靠上限0.12m/s，验收门槛保留。
- API key仅经隐藏输入和stdin传递，不进入argv、容器配置或清单。服务的PID、
  启动标识、工作区和启动ID用于核对停止对象。完成后关闭监控写盘；再次执行要求
  显式停止、核对空载/解除原生占用、重置场景并重启Agent，不沿用旧世界证据。

### 实际验证

迁移前先完成工具联调：抓取至放置约132.86→110.33秒，放置25.67→13.50秒；
属于一次场景的墙钟观测，不是统计基准。随后真实Qwen三角色任务和用户要求的重跑
均通过，各14次API、2版计划、5次机器人调用，零执行失败。独立证据分别位于外部
`runs/agent-autonomous-transfer-20261109/`、`runs/agent-autonomous-transfer-repeat-20261109-01/`。

迁移后的内置版本通过实际Qwen任务：世界`bfc674337b8d4883a847c5d9868a1fe8`，
Agent任务`58868f1b61224061a58976dba0889d31`，原生任务`d10f141942724a77b4eacd1b49f8df0f`。
5次机器人调用均为模型ToA产生并全部成功，15次API回执有效；存在一次计划依赖检查
要求模型修正，未执行无效动作。瓶子最终直立于另一张高桌，仅collision_960支撑，
实测张爪70mm、零link6/7/8接触、底盘停稳。1750个连续物理样本无失败，最大穿透
约0.49116mm，完成复核时连续稳定约12.95秒；此后监控文件保持不再增长。
报告中的transport_distance_m为离初始采样位置的最大距离，不是累计路径长度。

验证范围为已配置默认场景的单瓶搬运；不代表通用成功率、真实步态或实机效果。
101项迁移回归、27项ROS接口检查、ROS2 11包构建和开发doctor通过；帮助/兼容检查
不读取密钥、不执行运动，服务启动与只停止所属实例已实测。本轮未录制，不改变
用户固定场景机位的录制偏好。独立机器全新部署仍待验证。

模型输入/输出、ToA、事件、记忆、物理证据及agent-acceptance.json保存于外部
`runs/task-agent-20261109T141531Z-607821/`。历史失败、恢复及显式reset均保留。
此前74项临时脚本/录像材料已按用户要求移入回收站，167项旧日志/数据另行归档；
完整路径映射为外部`agent-transfer-checks/cleanup-manifest.json`。详细过程笔记保存为
外部`archives/feature-Lin-publication/validation-working-notes-20261109.md`，不纳入源码。

## 2026-11-12 feature/Lin：HSSD 三卧场景与真实 Go2 实时预览

本轮修改在已有 `feature/Lin` 工作树中完成，并按用户授权发布到同名分支。默认宿主配置改为
`hssd-go2`（场景 `107734479_176000442`），显式 `--profile full` 保留原
apt_3/A2/Piper 平台。没有清除旧工作区、任务锁或停止其他实例。

### 已实现

- 通过唯一入口 `bin/lab preview` / `bin/lab up` 启动本机 Go2 预览；支持预检、
  状态、验证和仅停止本工作区拥有的进程。端口冲突拒绝启动；HTTP 先绑定再创建物理子进程。
- 原始 GLB/KTX2/PBR 外观、房间标签、实时按键与按住按钮、速度调节、去房顶和视角切换。
  没有回放、自动路线或启动后自动运动；页面重载也不重置已有物理世界。
- 真实 Go2 12 关节力矩控制，200Hz MuJoCo 4.0 与50Hz作者MoE-CTS策略。
  指令丢失0.7秒后只清零移动命令，物理姿态稳定控制继续；快照陈旧会反映到健康状态。
- 资产在外部工作区 `assets/hssd-go2`；便携相对路径 MJCF、Go2/碰撞OBJ、策略、
  运行显示资源与第三方许可共14,192个校验文件。运行代码不引用历史聊天输出目录、
  `InteriorAgent_probe` 或 PARTNR 原始绝对路径；本机Python路径仅在config.local.json。
- 同场景2,826条任务标注的搜索/筛选/原始条件和下载入口。
  尚未载入任务物品和操作状态，不把任务展示记为任务执行成功。

### 本轮实际验证

- 资源导入、SHA256全量校验、`doctor --development`、`verify`均通过；重复启动复用同一
  拥有进程；`preview --stop`后status明确停止，随后重新启动通过。
- 将迁移控制器与原来已验证的控制器独立运行1,600个物理步（8秒），最大qpos差为0；
  同条件移动约2.479m，最低upright约0.98524，未跌倒。
- 浏览器实际载入仓库代码，约60FPS，物理约1.00倍，无本轮控制台错误；
  通过页面按住前进1.5秒，后台173个采样中峰值速度约0.790m/s，净位移约1.146m，
  松开后速度回落到约0.008m/s。
- 拒绝直接读取策略私有路径、source/vendor目录穿越；15个宿主帮助入口通过，
  改动Python可按3.8语法解析；差异空白检查通过。检查脚本与证据在外部工作区
  `local-tools/hssd-go2/`，不作为正式源码维护或运行依赖。

### 明确边界

初始旧 full 预检因Docker socket权限、被占用平台端口及旧镜像证据缺失而失败。
本轮使用已有、单独配置的MuJoCo 4.0环境，没有替换旧ROS镜像中的MuJoCo 3.3.7，
也没有更改Docker权限。未重新构建或验收完整UE/ROS导航抓取平台；旧机械臂任务、
语义地图和Piper接口不套用于无机械臂Go2。家具仍固定在初始姿态，未逐房验收通行。
资源包及权重尚未作为公开可获取制品发布；独立机器部署仍待验证，发布锁保持false。

## 2026-11-12 场景资源包与策略权重入 Git LFS

用户明确授权上传当前场景资源和权重。`scenes/hssd-go2/107734479_176000442.tar.gz`
包含完整运行资源、原始显示模型、碰撞、Go2网格、MoE-CTS policy.pt及任务标注。
解包共14,193个文件、141,501,163字节；压缩包40,099,882字节，通过Git LFS管理。
压缩包SHA256和权重SHA256记录在同目录scene-bundle.json，相关许可与来源另行明文保留。
HSSD和任务数据仍适用CC BY-NC 4.0，权重官方模型仓库标记MIT；不更改第三方许可。

`build --component assets`现在默认使用仓库内LFS包，先检查完整包SHA256，再拒绝绝对路径、
目录穿越、符号/硬链接等不安全条目，解包到仓库外工作区，并执行既有逐文件校验。
仍兼容显式传入资源目录或同格式tar.gz；未取得LFS实体时明确提示git lfs pull。

已在全新的外部工作区完成默认包导入，并直接从恢复的MJCF/网格/策略运行400个物理步，
Go2可稳定站立，policy.pt SHA256与原用户文件一致。压缩包不是仅包含链接的占位文件。
本轮不改动当前运行的物理会话或原始资源目录，完整UE/ROS抓取链路验收状态不变。


## 2026-11-13 HSSD / Go2 + Piper 和瓶子

当前配置为 `hssd-go2-piper`：HSSD/PARTNR场景107734479_176000442，
Go2＋Piper模型来自锁定LeggedManip Lab提交4e12109d1eccb1d9da2e4a35f189fabea3f13677。
MoE-CTS权重SHA256为bde5210a8114aa08bb0c611e49def1482faa1b2441673e5681374c30b935e32e。
原45维观测、200Hz物理/50Hz策略及12维腿部动作保留，Piper独立PD和重力补偿保持姿态。
19.11895kg整机保留原质量和惯量，底盘自由刚体，无mocap/基座weld。
默认速度0.8m/s，范围0.1–1.5m/s；80ms显示插值不修改物理状态。

绿色瓶子来自原world.py单瓶抓取场景，高0.24m、质量0.13kg，独立自由关节；
桌面为客厅中央木质茶几object_015_ce9c5810fc2943f1487a73b60e163d1d56f7a5c8。
长边另一端五分之一处、短边居中，初始坐标[0.8974331200,0.4388806856,0.541896508]。
6秒物理检查后[0.8974354201,0.4388848438,0.5388827461]，upright=0.9999999987。
瓶子真实受桌面支撑，重置复用profile位置；浏览器可直接定位瓶子。

此前本机选定路线测试：0.5m/s指令5秒位移1.951m，停止后速度约0.001m/s；
地毯路线位移4.036m、最低upright约0.9568，无跌倒；1.5m/s指令实际约1.35m/s。
这些是选定路线的局部结果，不代表全屋、抓取或实机验收。
诊断与证据保存在外部工作区local-tools/hssd-go2-piper。

本轮整理移除新增A2滑行、A2 final.pt、WBC试验代码及入口，保留原full平台。
试验记录和代码归档在外部工作区archives，未将用户提供的训练包或final.pt纳入提交。
显示模型按资源SHA加载，每次服务重启生成控制实例标识；旧实例控制返回409。
新工作区独立机器完整部署仍待验证，不将旧平台结果用于本配置验收。

整理后通过bin/lab重新装配14,205个资源文件，scene.xml与用户确认的版本逐字节相同；
完整SHA校验、doctor和只读verify通过，浏览器实际载入模型、机械臂和瓶子。
独立物理复查中0.5m/s指令4秒位移1.56383m，最低upright=0.99319，
停止3秒后速度0.00146m/s；瓶子upright=0.9999999979；无mocap/等式约束。
证据cleaned-smoke.json保存在外部工作区。Python3.8语法检查及git diff --check通过。
宿主旧Node不支持页面原有可选链语法，JS以实际浏览器载入和渲染确认。
构建所需trimesh沿用当前已安装5.2.0并写入独立预览依赖，原有版本不变。

## 2026-11-13 网页抓取选择入口

当前Go2仿真页面增加机身/腕部 RGB-D实时观察、RGB像素点选、规划预览、确认执行、
停止和交还控制入口。独立grasp.js不依赖Three.js，浏览器无法创建WebGL时仍可观察真实相机。
RGB与配准深度均来自同一物理会话、同一640x480快照；深度用0.1–3m灰度显示并提供米制读数。
相机只读observe不占用机器人、不发运动；在锁内复制MjData，在锁外渲染，避免GPU渲染阻塞物理循环。
点击显示双图十字标记并冻结选点，刷新/换相机清除选择。任务校验浏览器实例、世界会话、
图像身份及60秒有效期；锁定世界后再次检查相机位置/朝向和所选深度。
网页与CLI共用启动逻辑，不加载旧A2世界。后台输出、源指纹及result.json位于外部运行目录；
面板显示实际退出结果，不把任务入队当作成功。执行需预览通过并由用户再次确认。

已在无活动抓取时重启本工作区预览；/api/grasp/status、/api/health和RGB-D返回200。
浏览器实际显示RGB/深度，并点选到像素(324,392)、约1.2m深度，选点本身未发运动。
旧画面请求明确被拒绝；只读规划请求因画面中无目标返回selected_pixels=0，result记录
execution_requested=false；不以该失败冒充规划/抓取成功。本轮未执行机械臂抓取。
原9项隔离物理测试与新增12项网页任务边界测试通过，源码编译和diff空白检查通过。
浏览器测试环境不能创建WebGL，因此本轮仅验证2D相机和交互，不声称验证了3D显示。
完整可达目标选择、MoveIt规划、接触抓取仍待端到端验证。

## 2026-11-13 页面零位和观察姿态

页面新增机械臂零位/观察姿态、实时模型关节角和页面内二次确认。
零位参考锁定Piper部署URDF，观察姿态读取用户原源码observation.json，当前为
[0,0,0,0,5,0]度。原源码未修改；每次记录配置及参考URDF的SHA256。
不使用另一套agilex_piper展示模型的坐标约定；训练模型J2/J3存在约2度原点差异。
逐关节验证平移、Z轴和相对旋转后映射，当前参考零位对应模型
[0,0.03495000169,-0.03489959525,0,0,0]弧度，而不是模型全0。
观察姿态在此基础上J5加5度；正向运动学对照URDF通过（误差阈值2e-6）。
不修改训练模型网格、惯量、腿部策略、相机安装或现有资产包。
腕部相机继续随真实link6运动；本轮不是新的实机标定，60度FOV仍为近似模拟参数。

姿态动作使用同世界独立占用、低速关节插值（不高于0.12rad/s）、路径采样碰撞检查，
运行中检查碰撞、心跳、站姿和位移。预设姿态允许站立重心调整最多4cm/0.12rad，
另要求upright>=0.97、线速度<=0.15m/s、角速度<=0.3rad/s；原抓取快照仍为1.5cm/0.04rad。
完成须各轴误差<0.005rad、速度<0.02rad/s；夹爪保持不变，不自动执行原配置75mm开爪。
异常停止后保留所有权，检查后交还控制；不会自动重试或回零。旧选点和预览立即失效。

隔离物理测试完成折叠到零位、零位到观察姿态；新增6项姿态测试通过，覆盖FK、
无权限/非法输入、碰撞拒绝不写状态、隔离同世界socket工作进程和成功交还控制。
原9项物理测试和扩展后14项网页边界测试通过；测试与日志位于外部workspace/local-tools。
在线零位测试曾因底盘累计位移超过4cm而安全停止（go2-pose-1794570383267302033），
不能声称在线完整切换通过。确认contact_force=0后仅交还控制，没有复位、张爪或自动重试。
该失败和隔离中从不同静止状态切换触发保护的记录均保留；未放宽限制绕过失败。
浏览器已显示两个按钮、模型角度与真实RGB-D，页面内确认/取消已验证；
WebGL不可用问题仍在，本轮不声称验证3D画面、完整抓取或任意初始站姿都能成功切换。

## 2026-11-13 RGB-D内参ROS类型修复

网页选瓶预览报CameraInfo.k类型断言。原capture.json的K含整型0/1，ROS Humble的
固定9项float64字段拒绝混合int/float列表；报错中的长整数是double范围说明，不是坏内参。
RGB-D生产端改为浮点常量；ROS适配边界对9项有限数、正焦距和齐次行进行验证，
逐项转为Python float后构造CameraInfo，兼容仍在运行的旧世界，不必重启或重置场景。
原内参、相机位置、图像和深度数值不变。原始失败帧回归复现断言，修复后真实ROS消息
序列化/反序列化通过；含浮点/numpy输入和非法几何拒绝，共4项测试通过。
测试位于外部workspace/local-tools/test_go2_camera_info.py，使用Humble的/usr/bin/python3。

只读预览随后发现MoveIt启动缺少moveit_controller_manager而崩溃。
补充已安装SimpleControllerManager及明确的/sim/arm_controller动作端点；
allow_trajectory_execution仍为false，仅由已有仿真桥处理显式执行，不接硬件。
空controller_names曾被Humble拒绝，最终使用明确的六轴仿真控制器列表；失败记录保留。
最新go2-grasp-1794571170637581816中MoveIt启动完成，分割selected_pixels=10171，
GraspNet生成候选并调用/planner/preview。最终仍失败：没有候选通过完整IK/MoveIt规划，
诊断包含IK -31（在当前限位/搜索预算内未找到解），不能据此断言只要靠近就一定可抓。
execution_requested=false；没有臂/爪轨迹、自动导航、张爪或场景重置。
结束后owner=null、fault=null。此次修复不代表抓取成功，不修改用户源码或分支依赖锁。

## 2026-11-13 抓取规划漂移、IK种子与响应超时修复

失败证据go2-grasp-1794571392827530616在选中14229像素目标后因底盘超过快照容差终止；
go2-grasp-1794571326995549261的32个候选均未通过原KDL种子搜索。零速度步态指令不等于
静止保持，长期等待仍会漂移。适配层现以kp80/kd3、有原力矩速度限幅的腿部PD保持站姿，
切入/切出保持初始控制力矩，交还时重置策略历史；不焊接或冻结自由底盘。
等待至少3秒、最近2秒运动小于2mm/0.005rad且速度收敛后建立快照。
稳定前拒绝采集、导出及臂爪命令；此后不再重置参考，原1.5cm/0.04rad保护不变。
首次故障记录实际位移/转角、速度、阈值与阶段，保存在failure_feedback.json。

按导出URDF建立六轴有界数值IK，只作为原MoveIt碰撞检查的种子；不重复施加TCP，
不更改相机外参、机械臂零位、源仓库、依赖锁或关闭碰撞。实际抓取端点和预抓取IK均通过后，
隔离复测发现源节点12秒响应期限早于HSSD轨迹计算结束（约17秒），造成取消/关机异常。
本地适配的MoveIt future等待取原期限与45秒的较大值，保持有限等待及超时取消，
不放宽候选采集时间、碰撞约束或规划成功判据。

外部证据runs/go2-replay-1794572962541296448重建历史瓶前姿态，在独立MuJoCo世界运行
实际感知→GraspNet→MoveIt。候选0通过完整预抓取93点、接近18点、抬升13点，
result.passed=true、motion_executed=false；两次IK均collision-aware返回1。
这是隔离只读规划成功，不是在线抓取、夹持或抬升实测；未移动用户当前世界去复现旧位姿。
此前12秒超时证据go2-replay-1794572578538321394保留。

外部test_go2_planning_repair.py新增6项测试通过：稳定门控、漂移阈值及首次故障锁存、
60秒物理保持和交还、数值IK精度/限位、不可达目标拒绝、同模型状态恢复及模型不符拒绝。
该条件下60秒最大位移0.0006794m，交还后10秒最低upright为0.9903；不能外推到任意站姿。
原物理9项、网页14项、姿态6项、真实ROS内参4项回归通过，共39项。
支持显式开发启动restore-state，校验模型哈希、有限qpos、自由体四元数和关节限位；
仅恢复静止位置，速度清零，不恢复所有权、任务或重放运动。
腕部动态执行期间原感知节点使用静态相机配置的问题尚未验收，不能宣称腕部完整抓取可用。

部署前doctor --development通过。无活动任务、无夹持时导出当前静止场景，依据全部刚体
位姿重建qpos并校验最大位置误差1.26e-6m，通过bin/lab显式restore-state重启，未回到
历史瓶前姿态。重启证据go2-repair-restart-1794573129549178893包含前后状态；
启动日志development-1794573134246555521。重启后verify.passed=true、errors=[]，
健康检查physics_alive=true，owner/fault均为空。页面显示实时640×480 RGB-D和两种姿态按钮。
未执行在线抓取或姿态切换；当前底盘又有运动，未强行发起预览或移动到目标前。
此前测试浏览器WebGL限制未在本次处理，不以2D相机连接成功代替3D视频验收。

## 2026-11-13 执行器缺少Piper消息依赖

用户网页执行任务go2-grasp-1794573373240084172在启动executor时退出，executor.log明确为
ModuleNotFoundError: No module named 'piper_msgs'。result.execution_requested=false，
尚未调用仿真抓取启动服务；此前预览不启动executor，因此未覆盖该依赖。

新增bin/lab grasp prepare，只从工作区锁定Piper源码构建真实piper_msgs，不启动驱动，
不修改用户源码、消息定义或分支依赖锁。源码逐文件记录SHA256，缓存副本须完全一致。
原中文工作区及指向它的ASCII软链接都被构建工具解析回中文路径，rosidl生成失败；
失败日志保留在外部local-tools/grasp-interfaces-build*.log。实际构建使用ASCII缓存
~/.cache/robot-lab/ros-interfaces，工作区builds保留指向它的链接，不升级ROS或替换消息。
编译成功后真实PosCmd序列化检查通过，config.local.json记录interfaces_setup。

适配进程在占用底盘前检查TaskExecutor导入和真实PosCmd序列化；预览与执行均检查。
执行节点通过显式消息overlay启动，预览也加载待命节点并监测存活，但不调用/sim/task/start。
execute=false、simulation_mode=true保持，预览桥继续拒绝轨迹/夹爪动作。
新子进程按任务读取配置，因此此次无需重启物理世界或恢复历史姿态。

外部test_go2_executor_dependencies.py四项通过：真实消息往返序列化、overlay缺失拒绝、
命令参数保留、节点初始化与无规划启动拒绝；隔离ROS域187监测物理/仿真命令话题无输出。
网页14项回归、Python编译、git diff --check通过。
隔离证据runs/go2-replay-1794573606948588015包含executor_preflight.log及executor.log，
待命执行器参与的完整感知/GraspNet/MoveIt预览passed=true、motion_executed=false。
本次未自动重试用户在线执行任务；实际夹持、抬升效果仍不以预览成功代替验收。

## 2026-11-13 茶几新增三个物体（已载入）

保留原绿瓶，新增蓝瓶（高24cm）、橙色圆罐（直径6cm、高12cm）、黄色方块（边长6cm）。
摆放范围依据原茶几可视资产边界x=[0.567233,1.227633]、y=[-0.618319,0.703181]，
物体中心分别为(0.72,0.20)、(1.07,0.18)、(0.89,-0.07)，与原瓶和桌边留有间隔。
profile声明物体参数，已有构建模块生成自由刚体、质量、摩擦和碰撞，manifest记录body_id；
浏览器GLB同步增加方块显示。修正NumPy标量与MuJoCo枚举集合比较导致椭球肩部漏导出，
统一先转Python int；新增瓶肩规划碰撞采用包围球保守覆盖，不移除碰撞检查。

新增build --asset-output只允许工作区assets下的新目录，不覆盖或切换正在运行的资产。
通过bin/lab构建最终候选assets/hssd-go2-piper-tabletop-v2；完整14205文件哈希校验通过。
初版候选和失败诊断保留。外部local-tools/test_tabletop_objects.py完成20秒独立MuJoCo模拟：
四个物体均保持直立、与静态桌面接触、无相互接触，水平偏移<5mm、最终速度<0.01，
GLB含全部物体几何，新增障碍保留机械臂碰撞检查，规划模型导出通过。
证据runs/tabletop-layout/physics.json、tabletop.png，后者为独立物理场景渲染而非在线截图。

当前在线失败go2-grasp-1794574672324218393为RECOVERY_REQUIRED: gripper contact feedback
not confirmed，execution_requested=true；随后状态还记录底盘漂移超限。未据此声称已抓住物体。
任务未运行但仍持有恢复所有权，测得contact_force=0并不自动取消恢复要求。
用户明确批准“允许解除占用并载入新场景”后，先保存静止状态，再通过bin/lab grasp release
解除失败占用并停止预览。旧资产归档至assets/hssd-go2-piper-before-tabletop-1794575481507676224，
最终候选切换为assets/hssd-go2-piper，以映射后的关节状态显式restore-state启动。
证据runs/go2-repair-restart-1794575477582027341包含stationary-state.json、tabletop-state.json
及activation.json；启动日志为runs/development-1794575484660390429/go2-preview.log。
旧模型全部刚体位姿在新模型步进前保持一致，映射误差小于1e-10；未发送张爪、姿态切换、
导航或重试抓取指令，不恢复旧任务所有权。

重启后bin/lab --profile hssd-go2-piper verify通过，errors=[]，owner=null、fault=null。
原抓取回归9项通过；浏览器实时RGB及深度画面可见三个新增物体，在线状态保存于
runs/tabletop-layout/live-state.json。测试浏览器3D视窗仍有既有WebGL加载限制，未作为验收通过项。
原绿瓶在重启前已经落至桌边地面，保存位置约(1.322936,0.684669,0.049839)，此次按授权保留，
未将其重置到桌面；因此在线桌面新增的是三个物体，不能将独立测试的四物体直立结果当作在线状态。
新增其他物体的完整抓取执行尚未验收，当前执行证据仍针对原瓶，其他物体作为规划障碍。

## 2026-11-13 四个物体移至靠机器人一侧桌沿

按用户“物体全放在桌子边上”指令，将原绿瓶也扶正放回桌面，四物体中心排列为
x=0.67/0.82/0.97/1.12、y=0.60，距y正向桌沿约10cm，相邻中心间隔15cm。
profile同步保存布局。独立候选经bin/lab构建，20秒物理稳定、支撑接触、无物体间接触、
GLB与规划障碍检查通过，证据runs/table-edge-layout/physics.json。

当前任务空闲、无故障及夹持时保存状态，仅替换四个物体的自由关节位姿；其余刚体在步进前
位姿保持至1e-10精度，包括用户刚切换的机械臂零位和夹爪状态。未发运动或抓取指令。
快照及切换证据runs/go2-repair-restart-1794576213642876278；旧资产归档至
assets/hssd-go2-piper-before-table-edge-1794576227578477251，启动日志
runs/development-1794576231026219886/go2-preview.log。重启verify通过，在线四物体均在
y约0.60的桌面高度，状态保存在runs/table-edge-layout/live-state.json。
浏览器相机连接正常；不以此声称此前WebGL加载限制或完整多目标抓取已解决。

## 2026-11-13 桌沿物体被固定分割范围过滤

在线预览go2-grasp-1794576390230077739点击橙色罐(231,330)，有效深度约0.488m，
却返回components=0。根因为用户本地perception_node/depth_target.py中固定abs(world_y)<0.5，
把新布局y=0.60的四个物体全部过滤；不是此阶段的IK、机械臂零位或无效深度故障。

本地A2_loco_manipulation感知源码depth_target.py和node.py增加可配置table_bounds_world，
默认保留原范围兼容其他调用，并验证有限有序边界。修改前副本保存在外部
runs/table-edge-segmentation/source-before。Go2适配从profile读取真实桌面边界，预分割和
ROS感知节点使用同一配置，不改相机标定、桌高、碰撞检查或抓取成功判据。
任务子进程按次加载源码和profile，无需重启世界；旧失败记录保留，下一次新预览生效。

外部test_table_edge_segmentation.py七项通过，包含原失败复现、四物体逐个点选、
多目标无点击拒绝、背景/越界/无效深度及非法范围拒绝。原失败帧修复后components=4，
圆罐selected_pixels=4674；真实ROS感知回调在隔离域187输出有效绑定点云4380点，
差异来自原有3×3掩膜腐蚀，真实消息序列化通过。证据为
runs/table-edge-segmentation/segmentation.json及ros-callback.json。
未自动重试在线规划或执行；该结果只验证分割及点云，不代表圆罐已通过完整规划或抓取验收。

## 2026-11-14 蓝瓶抓取与规划耗时修复

在线失败记录 `go2-grasp-1794646920954165675` 表明抓取末端 IK 已通过，但目标蓝瓶同时以
场景刚体和感知目标进入 MoveIt，抬升时与自身碰撞，耗尽规划预算。导出模型现在按点选的
世界质心匹配单个目标刚体，仅从静态障碍中排除该目标，保留桌子和其他物体。物理验证改为
检查实际选中刚体，夹爪以限力、渐进闭合和感知半径计算目标宽度；臂与爪命令串行化。
抓住并完成抬升后及时确认持物状态，避免证据采样阶段的底盘快照误报。

规划模型还过滤距机械臂基座 1.1m 外的静态碰撞体；本场景保留 93/14006 个世界几何体，
URDF 约 42KB。该优化仍保留近处桌面和其他物体的碰撞检测。隔离环境完整执行记录
`runs/run-1794648074903626409` 用时 33.37 秒，`passed=true`，蓝瓶抬升最小值
0.090995m、双侧最小夹持力 4.9976N、`holding=true`、`fault=null`。之前在线完整成功记录
`runs/go2-grasp-1794647684835494034` 约 56 秒，最小抬升 0.0691m、最小双侧力约 4.997N。
两次运行环境不同，33 秒仅作为隔离环境的性能证据，不作为在线服务耗时承诺。

当前在线世界仍夹持蓝瓶，任务 `go2-grasp-1794647684835494034` 保有所有权。为保留现场，
尚未停止或重启在线预览；在线进程尚未加载此次最后的碰撞体过滤和持物状态修复。
独立测试运行使用单独工作区与 socket，没有改变在线物理世界。抓取服务常驻加载仍待实现。
相关外部 `test_go2_grasp` 与 `test_go2_planning_repair` 共 15 项通过；Python 编译与
`git diff --check` 通过。宽范围 `test_go2*` 发现两个 ROS 用例因当前测试环境未加载
`rclpy` 而无法导入，不计为抓取回归通过。

## 2026-11-14 网页恢复初始状态入口

抓取面板新增「恢复初始状态」按钮。网页先确认，服务端再次验证确认标志、会话及任务未运行；
物理进程在同一锁下归档恢复前 qpos/qvel、夹持和故障信息，然后重置机器人与桌面物体、
清除抓取占用及故障。归档保存到外部工作区 `runs/go2-initial-restore-*`。
外部 `test_go2_grasp.GraspTransportTests.test_restore_initial_archives_held_world_and_clears_ownership`
通过，覆盖未确认拒绝、夹持现场归档及恢复后状态。实施此功能时在线进程仍夹持蓝瓶，
未直接执行网页恢复；旧进程不声明支持该功能，重启前网页按钮保持禁用。

用户随后明确同意结束当前任务并重启。重启前只读归档在线 `grasp/status` 和 `state` 到
`runs/go2-held-before-restart-1794650998421294162`，记录原任务
`go2-grasp-1794647684835494034`、夹持力 5.002N。通过 `bin/lab preview --stop` 停止
本工作区旧进程，再以 `preview --development` 从场景初始状态启动；未恢复旧任务。
新服务 `/api/health` 为 ready/physics_alive，`/api/grasp/status` 返回
`supports_restore_initial=true`、`owner=null`、`fault=null`、`holding=false`；网页验证
「恢复初始状态」按钮已启用。启动日志位于
`runs/development-1794651012166961283/go2-preview.log`。用户尚未点击恢复按钮，
此时场景已经因明确授权的重启从模型初始状态启动。

## 2026-11-14 内置浏览器 WebGL 加载降级

用户反馈网页停在「正在打开新的房子」。预览进程及 `/api/health` 正常，`/`、`/app.js`、
`/grasp.js` 均返回 200；内置浏览器日志显示 `THREE.WebGLRenderer: Error creating WebGL
context`，导致脚本在初始化时退出，加载遮罩未移除。前端现捕获渲染器创建失败，移除遮罩，
显示 WebGL 不可用说明并保留独立 RGB-D 抓取面板；禁用无效的三维视图与行走控件。
刷新后截图及可访问性状态确认：加载遮罩消失、相机连接、恢复初始状态按钮可见且启用。
该降级不代表内置浏览器获得三维渲染能力；三维仍需支持 WebGL 的浏览器。

## 2026-11-14 蓝瓶抓取误停与持物回零

在线任务 `runs/go2-grasp-1794651290161930193` 的 GraspNet、MoveIt 完整预抓、接近与
抬升规划均通过，但机械臂动作带动自由底盘偏移 0.015003m，刚超过旧 0.015m 硬停止阈值，
所以机械臂停在瓶旁且夹持力为零。用户明确不需要该底盘位移互锁。移除微小位移/角度硬停，
保留执行心跳丢失与预置姿态明显不稳时的停止。旧失败任务无接触，已交还控制；保存静止
状态到 `runs/go2-repair-restart-1794651609033985704/stationary-state.json` 后重启并恢复
同一机器人/物体位置。新在线任务 `runs/go2-grasp-1794651647116125259` 抓起蓝瓶，
`passed=true`，最小抬升 0.09358m、最小双侧夹持力 4.997N；当时旧进程夹持该瓶，
后续按用户确认结束了该现场。

用户又要求抓后回参考零位并保持夹持。新增持物回零阶段：只在物理抓取确认后启动，
保留目标与其他物体的碰撞检查，仅把手指与被夹目标的预期接触从碰撞拒绝中排除；
限力夹爪在回零前收紧约 12mm、力上限 20N，回零后再连续 3 秒取抬升/双侧接触证据。
默认 5N 和初版 15N 方案在完整回放中会失去物体，失败记录保留。
最终隔离环境完整任务 `/tmp/robotlab-fast-test/runs/run-1794652297346694007` 通过：
先抓取、再持物回零，`returned_to_zero=true`，回零后最小抬升 0.08182m、最小双侧力
18.34N，故障为空。此为部署前隔离仿真验证；后述在线任务已加载该代码。

原本观察配置只改变 J5 +5°，肉眼变化很小。本项目 profile 增加
`observation_pose_deg=[0,30,-45,0,20,0]`；保留外部源码文件不修改，运行报告同时记录
原文件指纹和新 profile 指纹。独立物理试验从零位到新观察姿态成功，无故障，
证据 `runs/observation-probe.log`。网页从服务能力读取实际观察角度；后述重启已加载
新姿态。
外部 `test_go2_grasp`、`test_go2_planning_repair`、`test_go2_pose` 共 22 项通过；
Python 编译和 `git diff --check` 通过。

用户确认结束旧蓝瓶夹持并重启服务。重启前保存旧现场静止状态于
`runs/go2-repair-restart-1794652444069350072/stationary-state.json`，随后停止旧预览，
从无夹持的预抓取静止状态 `runs/go2-repair-restart-1794651609033985704/stationary-state.json`
启动新预览；启动日志为 `runs/development-1794652514266478796/go2-preview.log`。
新服务确认 `owner=null`、`holding=false`、`fault=null`，并报告观察姿态
`[0,30,-45,0,20,0]`。

在线执行 `runs/go2-grasp-1794652522185794240` 成功，`result.json` 为
`passed=true`、`holding=true`、`returned_to_zero=true`。抓起后 30 个采样点的
最低离桌高度 0.09343m、最低夹持力 4.997N；持物回零后 30 个采样点的最低离桌
高度 0.08210m、最低夹持力 18.304N，最后 10 个回零采样点最大关节模型角误差
0.01246rad。在线服务持续报告任务所有权与 `holding=true`、无故障，未主动释放蓝瓶。
观察姿态本轮仅确认新配置已加载；因现场正持物，没有再触发观察姿态动作。

## 2026-11-14 持物回零后行走

用户要求抓取时也能行走。在线旧进程对抓取所有权采取底盘全程锁定：`/api/control`
拒绝方向速度，物理循环也把速度强制归零。现改为抓取与回零阶段保持原有静止规划，
仅在回零完成且连续持物证据通过后，经 `enable_drive` 释放腿部站立保持并启用手动速度；
夹爪、机械臂目标和抓取任务所有权保留。持物时仍拒绝暂停或场景重置，方向指令超时
仍按原 0.7 秒归零。网页显示持物可行走状态。

隔离完整物理任务 `/tmp/robotlab-fast-test/runs/run-1794653441024783171` 通过，
`drive_enabled=true`、`holding=true`，释放站立保持后以 0.3m/s 前进指令运行 600 个
物理步，底盘实际移动 0.21586m，瓶仍离原位 0.08501m，双侧夹持力 19.926N，
故障为空。此前一次完整隔离任务也通过并测得移动 0.21568m。
Python 编译与 `git diff --check` 通过；在线预览仍为旧进程，当前现场有新的持物任务，
尚未为本修改重启服务。

用户确认结束当时夹持并重启。重启前只读保存 `/api/grasp/status` 与 `/api/state` 于
`runs/go2-held-before-drive-restart-1794653558637572213`；停止旧进程，再从无夹持的
预抓取静止状态启动新服务，日志 `runs/development-1794653568340222851/go2-preview.log`。
在线完整任务 `runs/go2-grasp-1794653573834256541` 抓取、持物回零并启用行走成功。
再通过网页同一个 `/api/control` 接口发 0.25m/s 前进指令约 2 秒并显式停步；底盘实际
位移 0.23103m，瓶离原位由 0.07944m 变为 0.08561m，双侧夹持力由 19.973N 变为
19.978N，`holding=true`、`drive_enabled=true`、故障为空。证据见任务目录
`live-drive-evidence.json`。蓝瓶仍由在线任务持有，没有自动松开。
内置浏览器无 WebGL 时原降级样式也隐藏了方向按钮、且不启动控制循环；已在降级页面
恢复独立的方向按钮和键盘控制，保持 0.3m/s 慢速及松开停步。刷新内置网页的可访问性
状态确认四个行走按钮可见，持物提示显示“可用方向键或行走按钮移动”。

## 2026-11-14 RGB-D 自遮挡调查

用户发现机械臂可能挡住相机，但 RGB 图未体现。实测当前下拉框默认机身相机；腕部相机
安装于 `link6` 且朝向末端前方。Go2/Piper 机械臂可视几何为 MuJoCo `group=3`，
默认 `MjvOption().geomgroup=[1,1,1,0,0,0]`，因此现有 RGB-D 渲染隐藏机器人自身。
隔离渲染开启第 3 组后，机身画面清楚显示夹爪遮挡蓝瓶；腕部画面在当前零位朝向地面，
开启第 3 组未产生可见差异。对比图位于 `/tmp/go2-camera-check/`。

本轮将第 3 组加入机身与腕部 RGB-D 彩色和深度渲染；另以仅机器人深度生成自遮挡
掩码，禁止网页/服务端选择机器人像素，并在感知输入中把被机器人遮挡的深度标为
无效。隔离抓帧的机器人掩码为 14,699 像素，源码语法检查与 `git diff --check` 通过。
原先姿态下，蓝瓶可见上半部的点云仍无法让现有 GraspNet 生成候选：隔离任务
`run-1794654705045307590` 在 150 秒候选等待后失败，未发运动指令。换用已提供
的观察姿态后，隔离任务 `run-1794655526055674877` 找到候选且实际抬起蓝瓶约
0.09 米，但持物回零阶段接触力丢失，整次任务未通过。因此真实自遮挡渲染已实现，
完整抓取成功尚未验证；在线预览仍持物，未重启部署新代码。

## 2026-11-14 点选 RGB-D 放置（隔离开发）

旧放置实现位于外部锁定源码 `A2_loco_manipulation/src/manipulation/task_executor/task_executor/placement.py`
和 `node.py`：在持物上下文中使用等高放置参考，先规划到目标上方，再笛卡尔下降、
验证末端到位、张爪、退回，最后由仿真接触反馈核验。Robot Lab 原适配器
`integration/task_agent/robot_lab.py` 会按已配置平面重映射参考点；当前 Go2 网页抓取
流程显式设置 `place_after_grasp=False`，此前并未接入放置。

本轮为 Go2 网页新增点选放置：选点深度和相机标定反投影到世界坐标，周围深度拟合
水平桌面；被机器人遮挡、非桌面、无稳定邻域或旧帧均拒绝。抓取时从被选中物体的
RGB-D 点云保存底部相对夹爪的偏移；放置时只用该偏移、最新 RGB-D 落点及机器人
关节/接触反馈生成上方、下降和退回轨迹。仿真物体真值仅用于放置后的独立物理核验，
不提供规划目标。仅将固定场景的两张茶几碰撞网格纳入相机可见深度，使桌面本身
可选；其他碰撞网格仍隐藏。

隔离检查：Python 语法和 `git diff --check` 通过；恢复历史姿态后 RGB-D 桌面
点 `[350,260]` 测得高度 0.4397 m、法向接近竖直，之前同点仅看到背景/地面；
针对 `[440,422]` 的关节目标与碰撞检查成功，多个不可达或碰撞点按预期拒绝。
隔离端到端运行 `place-grasp-1794657477944352745` 成功夹持蓝瓶，随后从同一服务线程
采集新 RGB-D，选取像素 `[432,439]`，测得桌面点约 `[0.830,0.598,0.439]` m。
放置运行 `place-run-1794657510677349392` 完成到上方、下降、张爪、退回与所有
物理核验，结果 `passed=true`。十次独立接触采样均有支撑接触，最大物体速度
0.00015 m/s，最大夹爪接触力 0 N；最终中心约 `[0.808,0.571,0.539]` m，
与所选桌面点水平偏差约 3.4 cm。此验收限于该蓝瓶和该选点，不代表所有桌面位置
可达。

用户确认结束在线夹持并重启后，先保存状态到工作区
`runs/go2-before-place-restart-1794657857836051778/status.json`，再通过网页
`restore_initial` 归档完整仿真状态到
`runs/go2-initial-restore-1794657857839804265/before.json`；响应确认
`holding=false`、`owner=null`。使用 `bin/lab --profile hssd-go2-piper preview --stop`
及 `bin/lab --profile hssd-go2-piper preview --development` 重启本工作区拥有的预览。
重启后 `api/health` 返回 `ready=true`、`physics_alive=true`；`api/grasp/status`
返回 `supports_rgbd_place=true`、`holding=false`、`fault=null`；机身 RGB-D 接口
返回 RGB、深度及机器人自遮挡掩码。网页源码包含“放置到 RGB-D 选点”按钮。
内置浏览器原标签页报渲染进程崩溃；新建同 URL 标签页后已显示抓取面板、RGB-D
相机连接状态和“放置到 RGB-D 选点”按钮。三维画面仍提示不可用，RGB-D 面板可用。
放置按钮需下一次成功抓取并持物后才会启用。

## 2026-11-14 抓取等待时间定位

在线任务 `go2-grasp-1794658103963481915` 从采集到候选等待超时约 150 秒。
保存的选中目标点云世界高度仅约 0.0504 m，低于瓶体拟合的 0.06 m 下限。
隔离重放相同 20,000 点 GraspNet 配置：模型加载与预热约 1.17 秒、推理约
0.063 秒、候选适配约 0.051 秒；拟合直接报 `Insufficient observed bottle height`。
因此本次长等待由生成错误没有传回主流程造成，不是 CUDA 推理耗时。
已在 RGB-D 分割后检查可见高度，低于阈值立即报出具体原因；主流程同时订阅
`/grasp/diagnostics`，候选生成的其他错误会及时终止等待，并保存诊断文件。
源码语法与 `git diff --check` 通过。该改动由下一次新建的抓取子进程读取；
当前在线服务正在运行时未重启。

历史成功任务 `go2-grasp-1794654080265425794` 总耗时约 52 秒：候选与规划
约 10 秒、抓取运动及夹持验证约 19 秒、持物回零与核验约 24 秒。运动时间受
轨迹与持物稳定性约束，本轮未提高关节速度。随后在线任务
`go2-grasp-1794658306631192058` 在回零后丢失接触而失败，保留占用；其原因
需单独处理，不能把失败误报为快速抓取成功。

## 2026-11-14 在线失败恢复与平台适配修复

用户授权结束失败任务。先保存在线状态到工作区
`runs/go2-failed-grasp-end-1794658430815462214/status-before.json`，再调用
`restore_initial`，完整状态归档于
`runs/go2-initial-restore-1794658430822398638/before.json`。服务确认
`owner=null`、`holding=false`。

在线任务 `go2-grasp-1794658391782436809` 报 `Trajectory exceeds joint limits`。
复查选中 MoveIt 路径：预抓首点 joint3 为 +0.000005099 rad，模型上限为 0；
其他路径点在限位内。`JointPath` 现允许最多 0.0001 rad 的边界数值误差并钳回
物理限位，仍拒绝超过该容差的轨迹。用该任务保存的三段真实轨迹重放校验通过。

在线任务 `go2-grasp-1794658306631192058` 在持物回零后抬升仍有约 0.078 m，
但 30 次接触采样有多次瞬时 0 N；原先取最小接触力导致失败。改为检查回零后的
30 次采样中至少 6 次双侧接触、全程抬升不少于 0.05 m、物体相对末端的稳定
位置散布不超过 0.025 m；单帧接触消失时，启用持物行走最多重试 2 秒。
物体真值仅用于抓后物理核验，不参与 RGB-D 选点或规划。

隔离端到端抓取 `place-grasp-1794658844145765448` 约 60.8 秒完成，
`passed=true`、`holding=true`、`returned_to_zero=true`、`drive_enabled=true`。
回零后的 30 次采样有 7 次双侧接触，最低抬升 0.0744 m，相对末端位置
最大散布 0.02454 m。该结果靠近两项验收阈值，说明当前仿真夹持仍有稳定性余量
不足的问题；不能据此声称所有视角和目标都可靠。一次隔离放置测试的张爪反馈
未达标，放置仍需单独修复与复测。

在线世界在无任务占用时使用 `bin/lab --profile hssd-go2-piper preview --stop`
和 `bin/lab --profile hssd-go2-piper preview --development` 加载本轮修复。
重启后 `/api/health` 返回 `ready=true`、`physics_alive=true`，抓取状态为
`owner=null`、`holding=false`、`fault=null`。内置浏览器旧标签页崩溃，
新建同 URL 标签页后抓取面板出现；三维画面仍不可用，RGB-D 面板正在连接。

## 2026-11-14 放置选点过期的页面修复

在线抓取成功后持物状态正常，但页面仍显示抓取前选中的瓶子像素；随后点击放置，
服务端因该 RGB-D 帧超过 30 秒而拒绝请求，未创建放置任务，也未移动机械臂。
页面现于进入持物状态时清除旧选点并刷新 RGB-D；若用户在 25 秒后选择或提交
放置点，会先刷新画面并要求重新点击空白桌面。服务端 30 秒有效期与提交前新旧
支撑面/相机位姿校验保持不变。在线服务继续保持现有夹持，不重启、不释放。
`git diff --check` 通过；在线静态资源已返回更新后的脚本。当前环境没有 Node
运行时，未完成 JS 解析检查；也未执行在线放置运动。

## 2026-11-14 持物时网页方向键焦点修复

在线状态显示抓取成功、`holding=true`、`drive_enabled=true`、无故障。通过
`/api/control` 发送 0.15 m/s、约 0.45 秒的短指令后立即停车，接口均返回
`ok=true`；底盘位置变化约 5.9 mm，夹持与无故障状态保持。因此底盘控制链路
可用。页面 RGB 画布获得焦点时会消耗方向键用于选点，WebGL 不可用时的行走
回退逻辑也排除了该画布，导致持物时按方向键没有行走指令。现于持物且允许行走
时让画布上的方向键交给底盘控制；非持物时仍可用方向键微调 RGB 选点。
截图里的 `Object may be held; keep ownership and gripper state` 是交还控制的
拒绝提示，不是底盘行走接口的错误。在线服务未重启，现有夹持未释放。

## 2026-11-14 持物后的放置模式入口

用户报告持物后无法切换放置模式。在线接口确认 `operation=run`、持物且无运行中
任务；原网页一直显示“点选抓取目标”，放置按钮又要求先有选点才启用，导致
缺少可见的模式入口。页面现在根据持物状态显示“点选放置位置”，提供可点击的
“选择放置点 · 刷新画面”，然后在新 RGB-D 图中点击空白桌面才能启用“执行放置”。
持物时抓取按钮保持禁用。此改动只涉及静态页面，不重启在线仿真，不释放夹持。

## 2026-11-14 放置失败后的重试与持物按钮状态

在线放置任务 `go2-place-1794668181023203793` 在规划前报
`Clicked placement point is outside collision-free arm reach`，`released=false`；
在线世界仍 `holding=true`、`drive_enabled=true`、`fault=null`，没有张爪。
原后端只在最近一次任务为原抓取任务时接受放置；一次放置失败便把最近任务改成
`place`，使再次选点和重试被拒绝。现从当前世界 `owner` 所指向的原抓取
记录读取成功证据和持物几何，允许失败后重新选点放置，同时保留已有的 RGB-D
帧新鲜度、桌面、视角和相机位姿校验。

页面在当前服务未加载重试功能时禁用重试入口并说明原因；新服务通过
`supports_place_retry` 标识支持。持物且无运行中轨迹时禁用「停止抓取」与
「交还行走控制」，避免误将保持夹持的拒绝当成行走故障。后端 `release`
也改为先验证无夹持力再修改任务状态。不可达且尚未张爪时页面提示靠近桌面、
刷新画面、重新选点。

隔离构造原抓取成功、最近放置失败的状态后，放置预检查成功创建新任务并保留
原抓取 `owner`；捆绑 Node 对 `grasp.js`、`app.js` 的语法检查、`py_compile`
与 `git diff --check` 通过。未执行完整放置轨迹。
当前在线持物进程仍运行旧后端，未重启，现有夹持未释放；该后端修复需在下一次
服务启动后生效。

用户随后明确同意结束当前夹持并重启。先保存状态到外部工作区
`runs/go2-approved-restart-1794668502813672556/status-before.json`，经
`restore_initial` 归档到 `runs/go2-initial-restore-1794668502814853115`，
再由 `bin/lab --profile hssd-go2-piper preview --stop` 和
`preview --development` 重启。新实例为 `ca2582a44372478bbb6c06f628f66ceb`；
`/api/health` 为 `ready=true`、`physics_alive=true`，抓取状态为
`supports_place_retry=true`、`owner=null`、`holding=false`、`fault=null`。
新建内置浏览器标签页可显示抓取面板及右下行走按钮；三维 WebGL 画面在该浏览器
中不可用。未在新实例执行真实抓取或放置，因此完整放置链路仍待验证。

## 2026-11-14 放置起点关节边界

在线放置任务 `go2-place-1794669823955089894` 已生成桌面放置计划，但在第一段
轨迹请求时报 `Trajectory exceeds joint limits`，`released=false`，当前物体仍夹持。
实时 J3 为 +0.00021217 rad，而 Piper 模型 J3 上限为 0；规划的 `above` 和
`bottom` 目标均在模型关节限位内。原执行器只允许 0.0001 rad 的边界数值误差。
放置子进程现仅对距 Piper 硬限位不超过 0.001 rad 的实测起点做边界投影，目标
轨迹及碰撞检查不变；超过该范围仍拒绝。用本次记录的起点和目标调用在线同款
`JointPath`：原起点复现失败，投影后的起点通过，最大修正 0.00021217 rad。
此修改由下一次放置新建的子进程读取，无需重启在线世界；尚未执行下一次在线放置。

## 2026-11-15 发布前流程检查

`bin/lab --profile hssd-go2-piper doctor --development` 通过，MuJoCo 4.0.0、
MoE-CTS 权重与场景资产均就绪。在线世界仍持物、无故障；最近一次放置
`go2-place-1794669954719906156` 在下降后、张爪前报
`Held object lost before placement release`，`released=false`。此前也有一次
在线放置成功记录，但新一轮多次重试并不稳定，不能据单次成功宣布完成验收。

放置子进程现将每段运动中的接触力、夹爪宽度、关节与轨迹状态写入外部工作区的
`place-motion.json`，失败结果增加 `failed_phase` 和失败前的反馈快照，以便区分
规划不可达、下降时丢失夹持、张爪反馈失败与支撑面验证失败。不会改变放置目标、
轨迹或夹爪动作。下降终点若夹爪接触力低于 0.1 N，不再仅凭单帧力值判定
丢失：连续 5 次物理核验中至少 3 次确认物体在 RGB-D 目标附近、受非机械臂
支撑且速度低于 0.1 m/s，才继续张爪；否则保持闭爪并拒绝。预张爪证据写入
`pre-release-support.json`。物理真值仅作安全与结果核验，不提供放置目标或
规划路径。尚未执行采用此诊断与门槛的新放置任务。

抓取配置当前指向 `/home/xubo/桌面/she/A2_loco_manipulation`，该目录没有
Git 元数据；与 `manifests/sources.lock.json` 锁定的工作区仓库相比，抓取生成、
规划和执行中至少 10 个 Python 文件内容不同。只推送本仓库不能复现当前抓取
算法和 ROS 构建结果。发布前须将这份源改动合入可获取的依赖仓库、更新精确
提交锁，并在新的独立工作区重新构建和验收。当前分支为 `feature/Lin`，与
`AGENTS.md` 指定的 `feature/robot-lab` 开发分支不一致；`release_ready=false`
保持不变。仓库根目录的 MuJoCo 运行日志现被 `.gitignore` 排除。

## 2026-11-15 夹爪压力与掉落状态

用户观察到过大夹持力造成明显穿模。当前服务在抓取占用时把夹爪执行器上限设为
±5 N，但持物回零时提高到 ±20 N，且把夹爪目标开度再缩小 12 mm。
这两项会提高仿真接触压力和几何侵入。代码现保持持物回零阶段的 ±5 N 上限，
额外收紧量改为 2 mm；仍按测得的双侧接触和抬升证据验收夹持。

当前在线状态虽仍报告 `holding=true`，但接触力为 0 N；只读物理证据显示目标
瓶子中心高度约 0.04984 m，在地面附近，末端位于约 0.622 m，实际已经掉落。
这是状态保持逻辑的缺口。新世界进程在持物期间若目标持续 0.5 秒低于桌面
下方高度，就清除持物/行走许可并记录 `Held object dropped below tabletop height`
故障，保留任务占用供检查；页面对持续 2 秒夹持力为 0 N 的旧进程也显示
“疑似掉落”并暂停放置入口。该判断是物理核验，不影响 RGB-D 放置选点或规划。

外部工作区的 10 项隔离抓取单元测试通过；额外将独立 MuJoCo 世界里的瓶子
移至 0.05 m 高度，0.6 秒后确认掉落检测清除持物和行走许可并记录故障。
`py_compile`、JS 语法与 `git diff --check` 通过。近期在线抓取原先在
±5 N 夹爪力矩下有约 5 N 双侧接触，持物回零调到 ±20 N 后曾出现最高
15.28 N 与夹爪宽度进一步缩小；这支持先降低回零力矩，但尚未以较低
夹持力完成隔离端到端抓取/放置；
当前在线世界仍运行旧物理进程，现场未重置，调参需重启后生效。
### 2026-11-15 授权恢复与 A2 抓取源码来源

- 用户确认结束当前夹持并重启。恢复前状态存于 `runs/go2-approved-restart-1794741275721080785/status-before.json`；`restore_initial` 的完整仿真归档为 `runs/go2-initial-restore-1794741275723640298`。
- `bin/lab --profile hssd-go2-piper preview --stop` 后以 `preview --development` 启动；新实例 `d642ed36cc76423e833ec2442d1ad31e`。`/api/health` 返回 `ready=true`、`physics_alive=true`，抓取状态 `owner=null`、`fault=null`。此步骤加载此前的夹持力限制和掉落检测改动；尚未以完整实抓验证新参数。
- 在线配置使用无 Git 元数据的 `/home/xubo/桌面/she/A2_loco_manipulation`，而版本锁指向干净的 `fb7f2b15b826612c3885c782acfc660627f8c1a3`。源码差异为 10 个同路径修改和 12 个仅本地存在的 Python、消息及 JSON 文件。
- 可审查差异位于 `runs/a2-source-provenance-1794741315053007813/`（补丁、SHA-256 清单、说明）。`git apply --check` 在版本锁检出上通过。发布前应在 A2 仓库独立分支应用并测试该补丁，发布提交后更新 `sources.lock.json` 与本地配置，重新部署验证；目前版本锁仍不能独立复现在线抓取算法。
### 2026-11-15 故障后继续与仿真夹持力

- 对已停止、未持物且夹爪接触力不大于 0.1 N 的抓取故障，网页增加“处理故障并继续”。服务端核对当前任务归属与会话，保存故障现场到 `runs/go2-fault-recovery-*`，清除占用与故障、张开夹爪，不重置物体和机器人位置。持物、运动中或有接触时拒绝该操作；仍可使用显式“恢复初始状态”。
- 仿真夹爪力范围从 ±5 N 小幅提高到 ±8 N，抓取与持物回零一致；保留 2 mm 的小幅持物收紧。未恢复旧的 ±20 N，避免明显穿模。此参数尚需在线完整抓取与搬运验证。
- 工作区迁移到较长路径后，Unix socket 超过系统路径长度限制；改为带随机会话标识的 `/tmp/go2-grasp-*.sock`，权限 0600。预览所有权检查改为比较 `--workspace` 参数解析后的实际路径，避免符号链接导致误判“没有运行”。
- 隔离抓取测试 11 项通过（含故障恢复与 8 N 断言），Python 编译、JS 语法、`git diff --check` 通过。抓取面板旧测试 14 项中 12 项通过，另外 2 项因夹具缺少既有 `self_mask` 字段失败，与本轮改动无关。预览已重启，健康检查待在线复核。
### 2026-11-15 放置开爪容差与放后回零

- 在线放置 `go2-place-1794745448408716995` 在 `open_gripper` 阶段失败。反馈夹爪宽度 `0.0699999915 m`、接触力 `0 N`，代码严格要求 `>=0.07 m`；物理模型的开爪极限与浮点误差造成假失败。将等待及 `mark_place_released` 的门槛改为 `0.0695 m`，仍要求双指接触力低于 `0.1 N`。
- 放置流程现于退离后采集连续物理支撑证据，验证成功才调用带碰撞路径检查的 `preset` 返回 Piper 零位，确认轨迹完成后交还任务。若物体已稳定放置但回零失败，记录 `placed=true` 与失败阶段，并尝试交还控制；网页明确提示物体已放下、回零未完成，便于人工检查后单独回零。
- 张爪前还用执行后的 link6 位姿与抓取时保存的 RGB-D 底部偏移核对目标点：水平误差不超过 25 mm，底部高于支撑面 2–25 mm；超限则保留夹持并拒绝释放。核对结果写入 `pre-release-pose.json`。
- 重启前现场归档于 `runs/go2-place-fix-before-restart-1794745622204024514/`。物理证据显示瓶子位于地面约 `0.04984 m`、无夹爪接触，不将上一次放置计为成功。预览重启后 `/api/health` 返回 `ready=true`、`physics_alive=true`，抓取服务实例 `20661b2aa9c64aaaaf26e93503f7f199` 无占用与故障。
- 12 项隔离抓取测试通过，含 `0.06999999 m` 开爪边界回归测试；完整在线放置与回零仍待实测。
### 2026-11-15 网页录屏

- 抓取面板加入“开始录屏”和“停止并保存”。浏览器 `getDisplayMedia` 由用户选择当前窗口或屏幕，`MediaRecorder` 录制三维场景和抓取面板，停止后下载带时间戳的 WebM（浏览器若使用 MP4 则保存 MP4）；显示计时、文件名与大小，并在录制中刷新页面时提示。
- Firefox 本地页确认为安全上下文，`getDisplayMedia` 和 `MediaRecorder` 均可用。用测试画面完成开始、停止、下载；生成的 1 秒 WebM 通过 GStreamer 解析为 VP8 视频。测试文件已移除。实际屏幕选择由浏览器授权界面完成。
- 前一轮外部 GStreamer 录屏得到 `runs/grasp-place-recording-20261115/grasp-place-full.mp4`，可解析为 1920×1080 H.264、约 86 秒；其中抓取报告为 `No candidate passed complete IK/MoveIt planning`，因此这份视频**不是**完整成功的抓放证据。

### 2026-11-15 RGB-D placement: moving-base pose correction

A live placement attempt (`go2-place-1794747654952588925`) kept the bottle gripped because the measured bottom was `(0.809, 0.551, 0.359) m` while the selected support was `(0.820, 0.603, 0.439) m`. Releasing there would place the bottle below the tabletop. The plan had been calculated before the arm moved, and the free base can drift during that move. Placement now recalculates collision-checked world-frame targets at the above-table pose; if the measured descent still misses, it performs one bounded correction with the gripper closed and checks again. It never releases on a failed pose check. The place worker is a fresh subprocess per request, so the online held-object owner can remain running and the next placement attempt uses this worker change without restarting the preview. Verification: 12 `test_go2_grasp.py` tests passed and `git diff --check` passed. A successful online placement remains to be verified with a fresh RGB-D selection.

### 2026-11-16 placement drift and release-state regression

Historic online place results contain failures at planning, descent, and open-gripper feedback. One recorded open-gripper failure already had a measured 0.070 m opening and 0 N bilateral finger force; the present 0.0695 m threshold accepts that physics-limit rounding. Feedback now sums only target-bottle/finger contacts, excluding table/finger contact from grasp confirmation, and exposes `grasp_state_consistent`. During descent correction the worker first reverses the checked descent to its retreat pose before replanning; the previous direct replan from a below-table failure could immediately collide. The release path still requires open jaws and loss of target contact, and final `passed` additionally requires stable support at the selected RGB-D position and return to zero.

Verification: 13 isolated `test_go2_grasp.py` tests passed. A worker regression test with a simulated first descent 8 cm below target passed through lift, replan, corrected descent, release, stable support, and return to zero. In a separate MuJoCo kinematic replay using the recorded bottle offset and target, independent base X offsets of -0.02, 0 and +0.02 m produced the same target bottom to printed precision. That replay temporarily disabled collision checking to isolate coordinate math; an additional collision-checked replay from the already below-table failed pose was correctly rejected. These checks do not establish an online end-to-end success rate; the current online world was left untouched because its owner and grasp feedback are inconsistent.
Five additional kinematic targets at the selected RGB-D support point and ±0.03 m in X/Y all solved with printed endpoint error 0.00000 m. Collision checks were bypassed only for this coordinate-math sweep; it does not establish physical placement success at those points.

### 2026-11-16 stopped-task recovery and online reset

The user explicitly requested a reset. Before stopping the service, `restore_initial` archived the abnormal state at `runs/go2-initial-restore-1794816422055397382`; it returned no owner, no held object, and 0 N contact. The preview was then stopped and restarted on port 18769. The restarted `/api/grasp/status` returned `owner=null`, `holding=false`, `contact_force=0`, and `grasp_state_consistent=true`; the served `/grasp.js` contains the new recovery prompt.

Recovery previously required a world `fault`, leaving stopped failed tasks with no physical grasp but no fault unable to clear their owner. The panel and world now permit recovery for any stopped owned task with neither a confirmed held object nor bottle/finger contact. The action archives state, opens the jaw, clears ownership, and retains the scene. The UI enables “处理故障并继续” for this state and explains when contact still requires initial restoration. Isolated verification: 14 grasp-world tests, 15 panel tests, and the placement worker correction regression passed; `git diff --check` passed. No new online grasp/place was executed after reset.

### 2026-11-16 near-contact placement threshold

The live place run `go2-place-1794816797561458877` ended with the held bottom at `(0.667372, 0.557628, 0.439780) m` and RGB-D support at `(0.670234, 0.549077, 0.439315) m`: approximately 9 mm horizontal error and 0.5 mm positive height. The previous requirement of at least 2 mm positive clearance rejected this near-contact pose. `placement_pose_acceptable` now accepts -1.5 to +25 mm vertical with the existing 25 mm horizontal bound; it still rejects 3 mm tabletop penetration and larger horizontal misses. The final physical support check after release remains required. Three placement regression tests passed. The current online bottle remains held; the new worker code is loaded only on the next place attempt, and a successful release has not yet been observed.
## 2026-11-16 RGB-D 放置 MTC 接入

- 用 ROS Humble 的 `moveit_task_constructor_core` 编译 `integration/mtc_place/`，隔离构建通过。
- 从独立 MuJoCo 世界导出 Piper URDF/SRDF，MTC 的 `FixedState`、OMPL 接近、笛卡尔下降、撤离和回零阶段均已实际运行。独立规划试验中，零位直接下降在约 7 cm 处因可达距离不足拒绝；先撤离 10 cm 再反向下降 10 cm 的规划成功，撤离与回零两段轨迹通过 `JointPath` 时序和关节范围校验。未执行运动或张爪。
- `test_go2_grasp.py` 14 项通过，Python 语法检查通过。尚未完成持物状态的端到端放置成功验证；上线前仍需在多个 RGB-D 桌面点重复检验接近、下降、释放、撤离及回零。

### 2026-11-16 在线放置路径和夹持力修正

- 在线放置记录 `go2-place-1794827202421013292` 证明 MTC 已被调用，但最初模块路径多了一层 `integration`，造成 `CMakeLists.txt` 未找到；修正为仓库的 `integration/mtc_place`。之后同次请求进入 MTC，因仿真关节 3 超出零位上界约 0.00021 rad 被 OMPL 拒绝为 `START_STATE_INVALID`。MTC 现对不超过 0.001 rad 的实测边界漂移进行规划状态钳制，超过该限仍报错。使用这次在线请求保留的 URDF、关节状态、RGB-D 目标与参数离线复跑，接近与下降两段规划成功生成；尚未执行在线释放与回零。
- 夹爪仿真单侧执行器力限由 8 调至 10，按 RGB-D 观测半径求闭合目标时再收紧 1 mm，保留物理接触确认与掉落监视。14 项外部抓取单元测试通过，MTC C++ 重新编译通过。当前在线世界仍有持物任务，未重启服务；新夹持力尚未进入在线物理进程，需在任务结束后重启再验证掉落率和穿模程度。

### 2026-11-16 选点画面有效期反馈

- 在线运行 `go2-grasp-1794828113718499077` 在执行前被正确拒绝：选点帧约 24 秒前采集，预检时相机已移动。不能复用旧像素进行抓取。面板现于提交前重新采集相机位姿、选点深度并验证，发现移动会立即返回，不再启动 ROS/MoveIt 工作进程；工作进程稳定底盘后仍再次验证。网页在画面超过 25 秒时刷新并要求重新选点，服务器报过期时也自动刷新。外部面板测试 16 项通过，含相机移动时不启动工作进程。

### 2026-11-16 持物回零掉落

- 在线抓取 `go2-grasp-1794828539408497824` 的抬起验证 30 帧接触力最低 9.895 N，接近当前单侧 10 N 上限；回零后 30 帧接触均为 0，瓶子回到桌面高度，任务报告 `Return to zero lost the held object`。夹爪力限小幅加至单侧 12 N；持物回零轨迹增加 0.5 秒原姿态停留，先让 2 mm 收紧完成，再启动机械臂运动。独立仿真检查轨迹前 0.25 秒关节不变，15 项抓取单元测试通过。
- 原掉落监视只识别落到桌面以下，瓶子掉回桌面时网页仍显示逻辑持物。现在持续 0.5 秒无双侧夹爪接触、且瓶子回到初始桌面高度附近时标记掉落，允许网页按故障恢复路径继续操作。当前在线进程尚未重启，需在现有持物状态结束后加载并实测掉落率与穿模。

### 2026-11-16 网页失败流程整理

- 复核近期在线结果，最频繁的是相机移动后的旧选点、持物回零掉落和掉落后逻辑持物导致的恢复阻塞。网页在选点过期的请求错误后，现先解除提交锁再自动刷新 RGB-D，避免刷新调用被 `submitting` 拦截。
- 后端 `status` 提供可恢复状态：任务停止、夹爪无物体接触时，如果原逻辑仍称持物，会再核对物体已回到抓取前桌面高度附近。满足条件才开放“处理故障并继续”；真实持物或运动轨迹中仍拒绝恢复。恢复会保存现场记录、清占用与张爪，不重置场景。独立世界测试 16 项、面板测试 17 项通过；在线服务仍待当前失败任务清理后重启与实测。
- 用户确认后，已将失败现场归档到 `runs/go2-initial-restore-1794835552032144610` 并重启 18769 预览服务。新实例 `630cb736d939429e885372ea232b3124` 的健康检查为 `ready=true`、`physics_alive=true`；抓取状态 `owner=null`、`holding=false`、`fault=null`，在线 RGB-D 接口返回有效腕部画面与 frame id。新的 `recovery_available` 字段和自动刷新脚本已由网页服务提供。完整在线抓放成功率仍待新任务验证。

### 2026-11-16 空夹爪动作提速

- 只对未夹物的 MoveIt 预抓取轨迹以原时间的 75% 发送执行；抓取接近与抬起轨迹保持原时间。普通零位与观察姿态的预设轨迹从至少 4 秒、每弧度约 8.3 秒调整为至少 3 秒、每弧度约 5.6 秒；持物回零仍按原速度并保留夹爪预收紧停留。
- 放置释放后的空夹爪撤离与回零最低执行时长从 6/8 秒调至 4/5 秒，仍经 `JointPath` 的关节范围和速度上限校验。独立非在线世界预设轨迹验证完成，合成轨迹超出关节速度上限时会被自动放慢到上限；16 项抓取测试通过。修改时在线正有新抓取任务，尚未重启，未做在线速度和物理抓放成功率验证。
- 用户确认归档并重启后，失败抓取现场归档为 `runs/go2-initial-restore-1794836068348707400`；重启后的实例 `28835cc6a61241ef9ac26e4a56fa76ba` 通过 `/api/health` 检查，`owner=null`、`holding=false`、`fault=null`、接触力 `0 N`。提速代码已由新服务加载；尚未执行在线抓放验证。

### 2026-11-16 持物回零掉落后的仿真夹爪调整

- 在线抓取 `go2-grasp-1794836255973625722` 的抬起后瓶子离初始高度约 10.7 cm，双指接触力约 11.7 N；持物回零开始约 2 秒后接触力降至 0，瓶子回到桌面，世界记录 `Held object lost gripper contact at tabletop height`。失败时双指执行器的 12 N 上限接近饱和。
- 仿真任务期间单侧夹爪力上限从 12 N 提到 16 N；指垫滑动摩擦系数从 1.5 提到 2.4，保留原有 2 mm 持物收紧、限速和掉落检测。没有启用刚性吸附或用物理真值生成抓取目标。16 项隔离抓取测试、Python 语法和 `git diff --check` 通过。尚未重启或在线验证新参数，需观察持物回零是否仍滑落及是否出现明显穿模。
- 用户确认后，失败现场归档于 `runs/go2-initial-restore-1794836679981199825`；已重启在线预览，新实例 `d2e530d4abc64cc685be64dca2e3c6fd` 的健康检查为 `ready=true`、`physics_alive=true`，抓取状态 `owner=null`、`holding=false`、`fault=null`、接触力 `0 N`。新夹爪参数已由新世界进程加载；仍需完整在线抓放检验。

### 2026-11-16 预构建模型指垫摩擦修正

- 在线新抓取 `go2-grasp-1794837008601496191` 于闭爪后未得到双指接触反馈，夹爪宽度约 52 mm、最小接触力 0 N；这次是在抬起之前失败，不是持物回零掉落。所选候选中心与 RGB-D 目标质心约有 22 mm 的 Y 差异，仍需检查抓取定位精度。
- 检查实际 `assets/hssd-go2-piper/scene.xml` 加载的 MuJoCo 模型，`link7_collision` 和 `link8_collision` 的摩擦系数均为 0.4。先前在 `build_scene()` 设置的 2.4 没有作用到这个预构建模型；绿色/蓝色瓶体已经有可碰撞的有质量圆柱、肩部和瓶颈，改为“实芯”不会改变 MuJoCo 的接触求解。现于抓取世界初始化时直接把已加载的双指碰撞体设为摩擦 2.4、六维接触和有限软垫参数。
- 用本次失败的规划末端姿态在隔离模型中对 Y 方向 -20/-10/0/+10/+20 mm 五个瓶位做闭爪复现，得到双指最小接触力约 16.2/15.8/8.8/0/0 N，表明位置偏差也可能造成完全无双指接触；该回放直接设定起始姿态，不能代表完整在线成功率。16 项隔离抓取测试通过，包含检查实际加载模型的指垫摩擦值。
- 用户确认归档重启；失败现场保存于 `runs/go2-initial-restore-1794837524486501075`。新在线实例 `811e0a3d91fa4987bab0df37c1acfa62` 的 `/api/health` 为 `ready=true`、`physics_alive=true`，状态为 `owner=null`、`fault=null`、接触力 0 N，在线世界反馈两侧 `finger_friction=[2.4,2.4]`，确认运行时修正实际生效。尚未完成新参数的在线完整抓放。

### 2026-11-17 机械臂关节限速提高

- 仿真轨迹接收器的关节 1–5 限速从 0.5 提到 0.8 rad/s，关节 6 从 0.3 提到 0.5 rad/s；原有路径边界和起点检查保留，超限轨迹仍自动拉长时间。MoveIt 预抓取时间比例从 0.75 调至 0.65，空夹爪接近阶段调至 0.9；持物抬升和持物回零速度未调，以免加重已观察到的掉落。普通零位与观察姿态的最低时长从 3 调至 2.5 秒、关节速度从约 0.18 调至 0.24 rad/s。
- 16 项隔离抓取测试通过，Python 语法和 `git diff --check` 通过；合成双关节 0.8/0.5 rad/s 轨迹请求在 0.5 秒完成时被执行器拉长到 1 秒，确认新限速生效且不会越限。改动时在线实例空闲，无任务占用。
- 重启前用户在网页启动新抓取 `go2-grasp-1794890314951926480`，因 `gripper contact feedback not confirmed` 失败，世界仍由该任务占用。为避免中断在线任务，没有强行重启；当前网页物理进程尚未加载本节新的 `JointPath` 限速。需归档失败现场后重启再验收。
- 用户确认后归档失败现场至 `runs/go2-initial-restore-1794890363103846151` 并重启预览。新实例 `d7e6b7b576eb446f98b454d014b28295` 健康检查为 `ready=true`、`physics_alive=true`，`owner=null`、`holding=false`、`fault=null`，指垫摩擦仍为 `[2.4,2.4]`。新速度代码已加载，尚未完成在线动作计时或抓放成功率验证。

### 2026-11-17 闭爪前末端到位校验

- 最近失败的蓝瓶抓取 `go2-grasp-1794890314951926480` 在抓取接近轨迹后报 `gripper contact feedback not confirmed`；失败记录显示夹爪开度约 59.6 mm，而瓶身直径约 54 mm，双指最小接触力为 0 N。实测关节相对规划接近终点最大误差约 0.016 rad，原抓取轨迹成功门槛为 0.025 rad 且不要求末端速度收敛，闭爪可能在偏位时开始。
- 非预设的 MoveIt 抓取轨迹现需终点误差小于 0.008 rad 且最大关节速度小于 0.02 rad/s 才报告完成；持物回零保留原 0.025 rad 容差。没有放宽双指接触证据，也不把单侧接触当成成功抓取。17 项隔离抓取测试通过，新增用 0.016 rad 误差与 0.03 rad/s 残余速度验证不会提前报完成；Python 语法与 `git diff --check` 通过。尚未做新门槛的在线端到端抓取验证。
- 在线服务在无任务占用时已重启；新实例 `6dd4516a08824f2ebe59bb21787ed05d` 健康检查为 `ready=true`、`physics_alive=true`，`owner=null`、`fault=null`，指垫摩擦 `[2.4,2.4]`。新到位门槛已加载，仍需真实选点、闭爪、抬升、回零的在线验收。

### 2026-11-17 持物抬升跟踪容差修正

- 在线抓取 `go2-grasp-1794890859292234190` 的夹爪实际达到约 14.7 N 双指接触，但抬升结束时关节仍距计划终点约 0.018 rad，触发此前统一施加的 0.008 rad 门槛并报 `Arm tracking tolerance exceeded`；失败后瓶子仍与夹爪接触，故不能直接清理现场。
- 轨迹 RPC 现传入 `pregrasp/approach/lift` 阶段。只有闭爪前的 `approach` 必须满足 0.008 rad 和 0.02 rad/s；持物 `lift` 恢复 0.025 rad 容差，后续仍需双指接触及瓶子抬升证据才能验收。非抓取轨迹默认保持原容差。18 项隔离抓取测试通过，包括 0.016 rad 接近误差拒绝和负载抬升误差接受；Python 语法与 `git diff --check` 通过。在线现场仍被失败任务占用，修复待归档重启后生效。
- 用户确认后归档持物接触现场至 `runs/go2-initial-restore-1794891004919358828` 并重启，新实例 `bc2dc9d48b1844d4beab98aca91bfdfa` 健康检查通过，`owner=null`、`fault=null`；在线完整抓放仍待验证。

### 2026-11-17 SuperMimic 碰撞几何对照

- 用户指定仅比较 `supermimic训练机械臂抓取/bottle_env.py --grasp-style side` 的碰撞几何与接触参数。该参考环境把手指网格设为 `contype=0` 的纯视觉几何，每根手指用两块薄盒碰撞垫，盒厚 5 mm；其指垫摩擦 `[1.2,.01,.001]`、`condim=4`、`solref=[.02,1]`、仿真步长 2 ms。瓶身为半径 18 mm、半高 45 mm 的有质量圆柱，整瓶约 80 g。
- 当前网页平台每根手指直接用整个网格做碰撞，运行时摩擦 `[2.4,.01,.005]`、`condim=6`、`solref=[.004,1]`、步长 5 ms；瓶身半径 27 mm、半高 90 mm，整瓶约 130 g。两套手指局部坐标与指尖模型不同，不能直接复制参考环境的盒坐标。此次仅只读对照，没有改变当前碰撞模型或在线场景。

### 2026-11-17 在线抓取连续接触反馈

- 网页抓取任务 `go2-grasp-1794891244948633337` 的 `request.json` 证实在线服务从 `/home/xubo/桌面/she/A2_loco_manipulation` 加载 ROS2 task executor；工作区 `sources/A2_loco_manipulation` 副本不是此任务使用的代码。该次任务在闭爪后的单次接触判定报 `gripper contact feedback not confirmed`，失败后世界仍被任务占用，在线状态显示双指最小接触力约 15 N，夹持归属尚未确认。
- 在线调用的 `simulation_execution.py` 已改为闭爪后连续至少 0.2 秒的新反馈样本达到双指接触阈值，最长等待 3 秒；抬升中每 50 毫秒持续观察，允许小于 0.25 秒的短暂反馈中断，持续失去接触则取消轨迹；抬升结束后再连续观察 2 秒。未使用夹爪开度或物理真值替代双指力反馈。
- Python 语法检查通过；使用提取后的原方法和模拟反馈，稳定接触、断续接触、短时丢失与持续丢失四种情况均符合预期。当前在线失败现场仍有约 15 N 接触并被任务占用，未重置、未执行新的在线抓取；需安全归档后验证新任务实际表现。
- 用户确认归档重启后，调用网页 `/api/grasp/action` 的 `restore_initial`，先将此前有约 15 N 接触力的失败现场保存到 `runs/go2-initial-restore-1794891686183006192`；恢复返回 `owner=null`、接触力 0 N。旧网页进程未由现存登记文件认领，核对其 18769 端口、启动命令和工作区后仅终止该进程，随后通过 `bin/lab --profile hssd-go2-piper preview --development` 启动新实例 `1ecdc401954f4d5bbd34b6a72330937e`。`/api/health` 返回 `ready=true`、`physics_alive=true`，抓取状态 `owner=null`、`holding=false`、`fault=null`；`preview --status` 显示新进程已登记。连续接触逻辑尚未通过新的在线抓取动作验收。

### 2026-11-17 零接触力误判修正

- 在线任务 `go2-grasp-1794891857713158805` 在抬升中报 `RECOVERY_REQUIRED: gripper contact lost continuously during lift`，失败后在线双指接触力约 14.7 N；用户指出 MuJoCo 穿模会让单次或连续力反馈归零，因此 0 N 不应直接中止抓取。
- 在线加载的 A2 `simulation_execution.py` 已移除闭爪和抬升的接触力门槛：闭爪等待 1 秒后完成抬升。网页桥在轨迹完成后连续采样约 3 秒，以物体抬升至少 5 cm 和物体相对 link6 位置变化不超过 2.5 cm 确认持物；持物回零后同样复核。世界侧持物确认、行走解锁和放置前检查改用物体高度；持物期间持续检查相对夹爪位姿，偏移超过 6 cm 持续 0.5 秒才判掉落。接触力仍记录为诊断值，不作为抓取中止条件。
- Python 语法、`git diff --check`、19 项隔离抓取测试通过，其中新增 0 N 但相对位姿稳定不报掉落、物体相对夹爪移开时报掉落。失败现场归档至 `runs/go2-initial-restore-1794892127635933469`，网页服务通过 `preview --development` 重启；新实例 `7c106e6c836749d8b890e06fb0bd52aa` 健康检查通过，`owner=null`、`fault=null`。新实例 RGB-D 初始画面看不到桌面目标，故尚未进行新的在线实抓验证。

### 2026-11-17 持物回零夹爪目标修正与轻量瓶体

- 在线抓取 `go2-grasp-1794892300154659545` 已把蓝瓶抬起，初始 30 个持物样本的最小抬升约 7.1 cm、相对 link6 最大漂移约 1.6 cm；回零开始时实测夹爪开度约 58.5 mm，旧代码把闭爪目标改成实测开度减 2 mm（约 56.5 mm），相当于主动放松，之后瓶子回到桌面。报错 `Held object moved away from the gripper` 属真实相对位姿掉落，不是 0 N 力反馈误判。
- 修复回零阶段保留原闭爪目标，不再根据受力后的开度重新设置夹爪目标；仿真任务的单侧限力从 16 N 提到 24 N。运行时指垫滑动摩擦系数从 2.4 提到 3.2；绿、蓝瓶的质量及惯量按同一比例从 130 g 降到 90 g。原模型夹爪硬件上限为 100 N，任务仍保持独立的 24 N 限力；没有使用刚性吸附。
- 20 项隔离抓取测试通过，新增回零保持闭爪目标及轻量瓶体参数断言；Python 语法和 `git diff --check` 通过。失败现场归档至 `runs/go2-initial-restore-1794892565090363816`，网页预览已重启，新实例 `bcd5959c09c64057900ef0b254040e55` 健康检查通过，`owner=null`、`fault=null`，在线反馈指垫摩擦 `[3.2,3.2]`。新参数尚未经过在线完整抓放验收。

### 2026-11-17 持物回零轨迹完成门槛

- 在线抓取 `go2-grasp-1794892717860538289` 成功抬起物体，抬升后 30 个采样的最小高度增量约 10.6 cm、相对 link6 位姿最大漂移约 0.064 mm；持物回零时因 `Arm tracking tolerance exceeded` 停止。失败后世界仍 `holding=true`，双指接触约 14.6 N，机器人已接近参考零位，故不是夹爪掉落。
- 回零计划的模型关节目标为 `[0,0.03495,-0.03490,0,0,0]` rad；失败后实测关节与目标的最大误差约 0.025 rad，原预设动作统一要求 `<0.005` rad。现仅对 `held_return_active` 使用 0.03 rad 完成容差和轨迹结束后最多 12 秒稳定等待；普通预设与闭爪前接近仍沿用原严格门槛。超时故障新增阶段、最大关节误差、门槛与速度字段，便于后续诊断。
- 21 项隔离抓取测试通过，包括持物回零 0.025 rad 负载误差完成判定；Python 语法和 `git diff --check` 通过。当前在线进程仍保留已夹持物体的失败现场，尚未归档或重启，因此在线实例尚未加载本次修复。
- 用户随后明确要求取消统一的 0.005 rad 预设完成门槛；现所有预设（包括零位、观察姿态与持物回零）统一采用 0.03 rad 完成容差及结束后最多 12 秒稳定等待。抓取接近阶段仍保持 0.008 rad；碰撞、关节限位及超时保护不变。22 项隔离测试通过，包括普通预设和持物回零的 0.025 rad 误差判定。在线实例仍夹持物体，未重启加载新代码。
- 用户确认结束当前夹持后，现场归档于 `runs/go2-initial-restore-1794892972820298358`，随后通过 `preview --stop` 和 `preview --development` 重启本工作区预览。新实例 `8ec347318eee4ef5973cd014648478d0` 的 `/api/health` 为 `ready=true`、`physics_alive=true`，抓取状态 `owner=null`、`holding=false`、`fault=null`、接触力 0 N。新完成门槛已由世界进程加载；仍需下一次在线抓取回零检验。

### 2026-11-17 场景重置导致瓶子离开视野

- 用户截图可见蓝瓶和绿瓶仍在桌上，机器人却已进入沙发区域；同一在线实例的姿态任务 `go2-pose-1794893188862813573` 在记录基座仍位于桌边 `(0.741,1.072)` m 后，于仿真时间 0.01 s 检测到基座接近原点 `(0,0)` m 并报 `Unsafe base stance during preset motion`。该现象与运行中场景被重置一致，无法仅凭现有日志确定是哪次前端操作触发。
- 网页旧版按键 `R` 可无确认地向 `/api/control` 发送完整场景 reset；Go2＋Piper 现在移除该键绑定、隐藏通用重置按钮，并由服务端拒绝其 `/api/control reset`。需要重置时使用抓取面板的“恢复初始状态”，该路径先归档现场并显式确认。普通 Go2 的通用重置按钮增加确认。
- 失败的姿态任务因无夹持、无接触而通过 `recover_fault` 归档并解除占用，归档目录 `runs/go2-fault-recovery-1794893374873663731`；从其完整 qpos 生成同模型 `restart-state.json`，重启时显式恢复该场景状态，避免本次部署再次主动重置物体。新实例 `42bb51e1c5b240afaa394489d2ab4d8a` 健康检查通过、`owner=null`。在线 POST `/api/control reset` 返回 409，所提供的 `app.js` 已无 `KeyR` 重置绑定并隐藏 Piper 通用重置按钮。此证据证明重置入口受控；具体旧重置来源未从日志确认。

### 2026-11-17 在线抓放闭环复测与物理稳定性

- 旧归档中的绿瓶自由关节含约 65 万米坐标。直接恢复该 qpos 导致 MuJoCo 的瓶体 QPOS/QVEL 发散，机器人跳离桌边，首次在线抓取在抬升时报 `Arm tracking tolerance exceeded`。恢复入口现拒绝超出房间范围的自由刚体坐标；该异常归档的独立加载测试返回 `Snapshot free body is outside the simulation room`。
- 当前预览资产包含过硬的 4 ms 接触与 `impratio=100`。在线抓取在仿真时间约 60.58 s 再次出现瓶体 QACC 不稳定。运行时接触改为瓶体 20 ms、4 维接触，手指 20 ms、6 维接触，`impratio=10`；手指滑动摩擦为 4.0，仿真单侧夹爪限力为 32 N。未来构建资产同步采用较缓和的瓶体参数。此改动不用于真机标定。
- 首次放置在 MTC 接近轨迹完成后，实测 J5 距下一段起点约 0.022 rad，超过轨迹接口 0.020 rad 起点门槛，下降未执行、物体保持夹持。放置工作进程现等待关节收敛至 0.019 rad 内，再发送下一段 MTC 轨迹。一次抓取在执行前遇到空 GraspNet 候选帧并误报目标丢失；仿真执行器现以新鲜 RGB-D 几何复核为目标证据，不再用单帧空候选直接拒绝。
- 在线复测：任务 `go2-grasp-1794894500097537333` 成功抓起蓝瓶、持物回零；随后 `go2-place-1794894549966985106` 从腕部 RGB-D 选得桌面点 `(0.8146, 0.5773, 0.4393)` m，经 MTC 接近、下降、一次落点校正、释放、撤离与回零，结果 `passed=true`。连续物理证据显示蓝瓶在 `(0.8154, 0.5976, 0.5388)` m 稳定支撑，速度低于 0.001 m/s、夹爪接触力 0 N；最终 `owner=null`、`holding=false`、`fault=null`。这证明该场景的一次完整抓放，不代表多目标成功率统计。
- 22 项本地抓取世界测试通过；Python 语法与 `git diff --check` 通过。旧放置工作进程测试仍引用过时的临时目录布局，在创建 `/builds` 时权限失败，未计为通过。成功后的桌边场景归档于 `runs/go2-initial-restore-1794894767023338748`，使用同模型、正常范围 qpos 重启；新实例 `0842f5ea364249f58824f4d7ef9feae4` 的健康检查为 `ready=true`、`physics_alive=true`，`owner=null`、`holding=false`、`fault=null`，腕部 RGB 图仍可见蓝绿瓶位于桌面。

### 2026-11-17 完整抓放录像验收

- 为录像从现有 MuJoCo `overview` 相机读取只读画面，`go2_grasp.py` 的观察接口增加该相机名称；网页的抓取与放置相机选项保持原样。录制器从总览相机和腕部 RGB-D 连续采样，生成 `runs/grasp-place-recording-20261117/overview-grasp-place.mp4`（108.5 秒、1280×520、H.264、2 fps）。抽查约 22、64、104 秒帧，可见桌面、机械臂、蓝瓶抓起和释放；`gst-discoverer-1.0` 确认文件可解码。
- 最终录像对应在线抓取 `go2-grasp-1794895684098349061`：`passed=true`、保持夹持、持物回零；在线放置 `go2-place-1794895732246192072`：RGB-D 选点 `(0.8047, 0.5783, 0.4393)` m，`passed=true`、释放并回零。最终状态 `owner=null`、`holding=false`、`fault=null`，夹爪力 0 N。
- 录制前第一轮从上一次摆放场景抓取时，蓝瓶自由体坐标跳到 100 m 以上，任务失败。归档失败证据 `go2-grasp-1794895123368944464`，恢复已知桌边初始快照后，连续三次完整在线抓放通过；此录像代表其中最后一次，不据此推断长期成功率。
- 另导出兼容播放版 `overview-grasp-place-compatible.mp4`，H.264 High Profile / I420，长度同为 108.5 秒；`gst-discoverer-1.0` 解码检查通过。

### 2026-11-17 低位抓取候选防护与网页重置

- 对此前失败任务 `go2-grasp-1794895123368944464` 追踪物理证据：所选抓取中心 z=0.5648 m，距桌面约 0.126 m；成功抓取候选 z≈0.598 m。失败任务在抬升前后，蓝瓶自由体坐标跳至百米外，接触力归零；这是仿真数值发散，单纯修改掉落判定无法解决。
- 本地 A2 规划器增加可配置的桌面上方最小抓取中心高度，Go2 网页集成传入桌面 z 与 0.135 m 门槛；低位候选会在 IK/轨迹规划前被跳过。原始 A2 其他场景默认门槛为 0。Python 编译和部署仓库 `git diff --check` 通过。
- 将放置后快照用于复测时，低点选取报 `Invalid observed cylinder fit`，未进入低位候选过滤，故尚不能证明原数值发散已根治。使用桌边初始快照对正常点 `(258,390)` 在线验证：`go2-grasp-1794896347235321629` 通过，持物回零。随后按用户要求调用 `restore_initial`，夹持现场归档到 `go2-initial-restore-1794896400612511785`，最终 `owner=null`、`holding=false`、`fault=null`，网页已刷新。


### 2026-11-17：接触反馈小步放置（开发验证）

- MTC 保留接近、完整撤离、回零；移除固定 10 cm 下降阶段。接近目标仍由 RGB-D 支撑点和历史持物偏移估算，小步下降每次按当前末端位姿求解 3 mm，保持朝向并检查关节限位和机械臂碰撞。最多 45 步或累计约 13 cm；无支撑停滞不会触发释放。
- `place_evidence` 在所选支撑点水平 8 cm、高度 1.5 cm 内筛选物体与非机器人表面的接触，用 MuJoCo 接触坐标系转换后的向上力确认支撑。稳定门限为至少 max(0.05 N, 0.3 × 物体重量)、线速度低于 0.03 m/s，连续仿真时间 0.3 s；不使用单次夹持力为零判定下降失败。它使用仿真接触/质量反馈，不属于纯 RGB-D 或真机力传感器方案。
- 检出接触立即停止下压；稳定确认后分四步打开至 75 mm，每步确认支撑；故障时暂停继续张爪。释放后直接执行原有完整撤离、连续支撑验收和回零，不增加 1–2 cm 试探抬升阶段。
- 外部工作区 `local-tools/test_place_support_control.py` 六项通过：稳定支撑释放回零、无接触步数有界、无支撑停滞拒绝释放、瞬态接触拒绝释放、释放中支撑丢失暂停，以及实际 MuJoCo 0.1 kg 方块支撑力约 0.981 N / 错误落点排除。流程测试使用模拟 RPC，不能当作在线抓放成功证据。
- Python 语法检查和 `doctor --development` 通过；MTC `mtc-place-3bca0b89e2bcb59b` 本机编译成功。尚未完成新策略整套在线抓取、放置、回零验收及多落点成功率测试。
- 在线旧失败现场已归档为 `runs/go2-fault-recovery-1794902840321223720`，使用同模型 qpos 快照重启并保留场景布局；新服务 `runs/development-1794902859636229074` 返回 `placement_control_version=2`、健康 ready、无故障/占用。网页刷新后 RGB-D 相机已连接。应用内浏览器仍报告 WebGL 不可用，此项未在本次放置修改中解决。

### 2026-11-17：姿态任务 phase 未定义回归修复

- 修正小步放置改动中错误插入 `run_pose` 的下降接触判断，将其移动到 `run_place.wait_motion`。零位和观察姿态不再访问放置闭包中的 `phase`/`support_sample`；放置运动等待期间可正确检测支撑并中止下压。
- 外部 `test_pose_phase_regression.py` 验证零位和观察姿态从 running 到 succeeded 并释放占用，未调用放置证据接口；原六项支撑控制测试仍通过，新增稳定接触必须触发 abort 的断言。
- 在线旧失败任务通过 recover_fault 归档恢复，随后网页 API 启动零位任务 `go2-pose-1794907528739862907`，结果 passed=true；最终 owner=null、fault=null。此修复在每次新启动的姿态/放置子进程生效，无需重启物理服务。完整在线放置验收仍未完成。


### 2026-11-17：按用户要求恢复摩擦与质量

- 删除抓取服务初始化中的指垫摩擦覆盖 `[4.0, .01, .005]` 和瓶子质量/惯量缩放，恢复资产模型定义：双指垫摩擦 `[0.4, .005, .0001]`，蓝/绿瓶各 0.13 kg，惯量亦取原模型。撤销上一轮为质量修改添加的 `mj_setConst`，因不再运行时改质量。
- 归档无接触、无持物的失败现场 `go2-fault-recovery-1794908824742949044`，使用同模型位置快照重启加载。此项仅恢复摩擦和质量，不代表已验证弹飞问题解决；先前关于减重策略的记录属于历史状态。

### 2026-11-17：网页抓放闭环调试复测

- 物理积分在原 5 ms 控制周期内使用 5 个 1 ms 子步；接触下降、支撑连续确认和释放控制在物理步执行，不依赖网页轮询。数值发散会回退该步并停止场景，保留故障证据，不把无效物体坐标继续显示为正常运动。瓶体质量仍为 0.13 kg，指垫摩擦仍为 `[0.4, 0.005, 0.0001]`。
- RGB-D 持物几何以抬升前实测 link6 位姿绑定，避免把抬升位移算进瓶底偏移。持物回零按连续相对位姿验证，不再要求物体始终高于原桌面；真实掉落会同时清除持物偏移，避免状态不一致。
- 放置首轮 `go2-place-1794914443678632772` 已稳定支撑、接触力 0 N，但张爪完成条件要求 73 mm，超过仿真约 70 mm 关节行程，导致超时。现与已有释放接口统一使用 69.5 mm 实测开度容差，同时仍要求连续支撑及夹爪接触力不超过 0.1 N。外部回归测试覆盖实际饱和开度 69.99998 mm。
- 修复加载后，两轮连续在线 API 流程通过：`go2-grasp-1794914578739470026` → `go2-place-1794914622156041171`，再从放下位置重新抓取 `go2-grasp-1794914657801910248` → `go2-place-1794914700789509151`。每轮均完成观察、RGB 选瓶、抓起、持物回零、RGB-D 选桌面点、接触下降、释放、撤离和回零。两落点约为 `(0.806, 0.619, 0.439)` 与 `(0.833, 0.628, 0.439)` m；最终 `owner=null`、`holding=false`、`fault=null`。第一轮最终支撑力约 1.2753 N，与瓶重一致。两轮通过不代表长期或任意物体成功率。
- 外部回归检查：抓取世界 22 项、网页面板 17 项、物理接触/任务合同 11 项、放置工作流程 6 项、姿态回归 1 项通过；JS 语法、`git diff --check` 和 `doctor --development` 通过。网页运行中不再误提示失败占用；完成后已通过浏览器核对相机连接、空闲状态与重新操作按钮。应用内浏览器仍无法创建 WebGL 三维视图，RGB-D 面板可用；本次未将其计为三维渲染验证通过。

### 2026-11-17：第三落点复测与故障状态合同补充

- 使用同一桌边初始快照重新启动当前版本，在线抓取 `go2-grasp-1794914908901839912` 完成抬升及持物回零；随后通过新采集腕部 RGB-D 在像素 `(300,400)` 选择第三落点，约 `(0.77487,0.61957,0.43927)` m。完整记录位于外部工作区 `runs/task-v3-online-1794914902353637556/`，抓取与放置最终报告均通过；最终为 idle、无占用、无持物、无故障。本次与前两轮使用相同物理参数指纹 `29ec77aa2b27bdb6837620e6bf44712faa1683da8e7798231ff40e5ad4a8ae35`。三落点在线通过不能代替长期成功率或异机部署验收。
- 后端 `task` 合同补充终态错误归并：放置失败但仍有有效持物证据时，报告 `retryable_failure` 和实际错误，同时允许重新选点放置，不把它误显示成新的正常持物任务。网页仅根据后端权限及本地网络／选点／提交状态禁用按钮；不再以零夹持力自行判断掉落。
- 新增实际 70 mm 开度及保留持物重试两项回归，物理接触／任务合同检查共 13 项通过。连同抓取世界 22 项、面板 17 项、放置工作流程 6 项、姿态回归 1 项，本轮共 59 项检查通过（其中工作流程有模拟 RPC；线上成功依据为上述实际任务记录）。

### 2026-11-17：腕部 RGB-D 放置路线 v4（传感器约束收敛）

- 用户明确实际可用传感器为 link6 腕部 RGB-D 和 Go2 机身 RGB。机身相机保留内部 ID `body_rgbd` 以对应既有模型，但网页显示“RGB · 仅观察”，API 返回 `modality=rgb, depth=null`；抓取和放置均拒绝从该相机取得三维点。命令行抓取默认及可选输入均为腕部 RGB-D。
- 本次开始时在线放置 `go2-place-1794915160663760911` 在第一小步报 `Descent tracking stalled`，夹持仍有效。旧控制器以 0.003 rad 全关节误差作为小步完成门槛，且以 MuJoCo 目标接触力决定释放，不是仅凭现有视觉的方案。v4 网页工作进程显式选择 `wrist_rgbd_geometry`；末段位置／直立姿态用已观测桌面、持物几何及关节正运动学闭环校正，不再调用 `place_evidence` 决定下降或松爪。原接触接口只保留为仿真诊断／兼容接口，不能据此宣称整个平台完全不依赖仿真。
- 选点检查约 9 cm 直径的实测支撑区域、桌边、障碍和自身遮挡。先分离主要桌面高度再拟合，避免邻近瓶身混入导致同一桌面间歇报倾斜。新腕部图像通过世界点重投影复核，不复用相机移动前的像素。接近后目标不在腕部视野时，报告沿用最后观测的静态平面，不用仿真真值补造可见性。
- 抓取时不再把分割出的最低可见瓶身当成真实瓶底；下半截被画面裁掉时，使用物体周围实测桌面估计其初始底面，并记录“初始直立刚性瓶”的假设及手腕局部直立轴。接近校正瓶体方向，最后靠近步长不超过 3 mm；到达约 6 mm 几何间隙并连续满足位置与倾角条件后才缓慢张爪。不将机械臂不动或零接触力解释成接触桌面。
- 发现并修正中间版 `go2-place-1794916516308197409` 的释放侧倒：该版固定了接近后的实际倾斜姿态。现在继续按初始观测直立轴校正，松爪前倾角不超过约 2.3°。随后在线 `go2-grasp-1794916825469901371` → `go2-place-1794916870327457956` 完成抓取、持物回零、放置、释放、撤离回零和连续腕部深度复核；独立仿真验收瓶底约 `(0.81982,0.63319,0.43879)` m，竖直方向余弦约 0.9999993。独立真值保存在外部 `runs/task-v3-online-1794916818927851073/`，不输入控制器。
- 多点复测也暴露未成功案例：`task-v3-online-1794916941103644425` 和后续重选在开始执行前被旧在线平面拟合拒绝；没有计为放置成功。`go2-place-1794917341267734621` 在张爪过程中错误地继续套用“瓶底与手腕刚性固定”约束，中途暂停。最终修正为松爪前核对几何间隙，张爪后保持手腕参考位姿并用关节反馈补偿卸载漂移，不再把瓶子独立落稳时的相对位移误判成穿透。失败证据在 `runs/place-v4-release-failure-1794917480077813993/`。一次静止快照因基座未满足静止条件而未生成，该目录是状态证据，不是可直接恢复的 qpos 快照。
- 释放后直接完整撤离、回零，再用新腕部深度连续检查落点附近的直立可见几何及稳定性；不增加试探抬升。该检查不等于力传感器确认支撑，也不保证遮挡／拥挤情况下的物体身份。看不清或发生侧倒会报告“结果未确认”；已释放任务解除占用，避免一直卡死。质量、惯量和摩擦不作本轮调参。
- 外部新增检查：`test_wrist_place_v4.py` 18 项（平面、边缘、障碍、重投影、自遮挡、姿态、租约、张爪后刚性关系解除等），`test_wrist_place_worker_v4.py` 4 项（成功、下降失败、释放暂停、回零后视觉不明解除占用）。这些包含合成相机／模拟 RPC，不能替代在线成功率；最终多点在线复测结果另列于下。
- 最终加载版本在同一在线场景连续完成两轮实际 API 闭环，中间没有恢复场景：`go2-grasp-1794917554345847447` → `go2-place-1794917598817053673`，再从已放下的瓶子重新抓取 `go2-grasp-1794917685583026406` → `go2-place-1794917729466140768`。两轮均完成抓取、持物回零、腕部 RGB-D 选点、几何／姿态校正、保持手腕张爪、撤离、回零及连续深度复核；放置点约 `(0.7764,0.6199,0.4393)` m、`(0.8246,0.6271,0.4393)` m。独立仿真验收两轮瓶底约 `(0.77974,0.62125,0.43879)` m、`(0.83288,0.62532,0.43879)` m，竖直方向余弦均大于 0.999999；此验收数据只在任务完成后读取，不进入控制。
- 最终证据在外部 `runs/task-v3-online-1794917547797289523/` 与 `runs/task-v3-online-1794917681424672270/`，包含逐阶段状态、成功／失败报告、最终公开物理状态和独立验收。后一目录 `final-wrist.png` 经实际图像检查，蓝瓶直立在桌面。最终网页状态 idle、无占用、无持物、无故障，协议 4；机身图像接口实测 `modality=rgb, depth=null`。物理参数指纹仍为 `29ec77aa2b27bdb6837620e6bf44712faa1683da8e7798231ff40e5ad4a8ae35`。
- 本轮完整回归共 75 项通过：新增 22 项、抓取世界 22 项、网页面板 17 项、原物理接触／任务合同兼容检查 13 项、姿态回归 1 项。Python 编译、JavaScript 语法、`git diff --check`、`doctor --development` 均通过。应用内浏览器 CDP 聚焦命令超时，未计为按钮交互／WebGL 验证；网页传感器与任务流程依据为实际 HTTP API、服务端记录和腕部渲染图。两轮连续通过仅覆盖当前直立瓶／固定桌面，不构成长时间、未知物体或实机成功率承诺；本轮未 commit/push。

### 2026-11-17：feature/she 抓放发布整理

- 从实际运行源码同步网页、任务状态、ROS 抓取适配、MTC 放置和积分子步／数值保护到独立 `feature/she` 发布检出；运行目录的现有分支与服务保持不变。此次只整理代码职责与发布材料，不改抓放控制行为或质量／摩擦。
- 新增 `docs/grasp-place.md`，明确网页、任务合同、感知、规划、执行和部署六项职责；README 以选点抓取与放置为主入口。纠正旧文档“由支撑力决定松爪”的现行描述：网页 v4 使用几何小步控制，仿真持物与碰撞依赖仍明确保留。
- 更新 `grasp-source` 中实际使用的规划器、执行器和观察配置，44 个文件逐项 SHA-256 校验通过。算法内容锁在 `grasp-source/manifest.json`，上游基础依赖锁仍在 `manifests/sources.lock.json`；没有将本地修改伪装成上游已发布提交。
- 按源码快照说明在仓库外 ASCII 临时路径创建干净副本，`grasp_msgs` 独立 colcon 构建成功。加载本机 ROS Humble 与既有 Piper 消息 overlay 后，四个算法包实际从干净副本导入，`SelectedGrasp`、`BoundTargetCloud`、`PosCmd` 序列化通过；未实例化节点、未发送运动。此检查使用本机已有依赖，不能替代异机安装验收。
- 对待上传检出执行 75 项外部回归检查全部通过：腕部感知／控制 18、放置工作进程 4、抓取世界 22、面板 17、任务／接触兼容 13、姿态 1。Python 编译、两份网页脚本 Node 语法、快照校验、CLI 帮助与 `git diff --check` 通过。构建有 CMake 兼容性警告，测试有 Torch 弃用及欧拉角表示警告，均未造成检查失败。
- 发布前只读在线健康检查 `ready=true`、`physics_alive=true`；任务 idle、owner=null、holding=false、fault=null，放置协议 4。本轮不额外执行抓放；在线运动证据沿用上节最终 v4 两轮实际记录，不把本次模拟回归计为新增在线成功。
- 本机配置、模型权重、缓存、构建、日志、录像及外部测试不纳入源码提交；已授权固定场景 LFS 包保留。新机器需按 README 和 `grasp-source/README.md` 准备环境与权重，独立机器完整部署仍未验收。


### 2026-11-17：多桌面抓放测试场景

- 检查确认其余桌面已随原 HSSD 场景具有静态碰撞，原抓取仅固定客厅茶几高度／范围。新增 `grasp_surfaces` 配置原茶几及五张桌面；网页和 ROS 工作进程分别由当前腕部 RGB-D 选点匹配桌面分割先验，避免高桌仍按 0.4389 m 分割。放置平面高度范围扩至 0.85 m 以覆盖约 0.8003 m 餐桌；碰撞和可达性仍须检查。
- 另外五张桌各加入一个自由刚体瓶，质量仍为 0.13 kg，不改指垫／瓶体摩擦。新瓶复用瓶子碰撞参数和网页刚体显示导出。先构建隔离候选资产，不覆盖在线模型。最初第六个矮边桌候选在静置试验中出现瓶子滑落，排除该点位后重建；没有通过调摩擦或焊接物体掩盖失败。
- 最终候选在隔离 MuJoCo 世界运行 6 秒，五个新瓶均与静态桌面接触、直立方向余弦大于 0.999999、水平漂移小于 0.12 mm，最终线速度小于 0.0002 m/s。证据 `runs/multitable-validation-1794919949204048812/result.json`。这只是稳定支撑验证，不是逐桌完整抓放成功；只读总览截图受屋顶遮挡，未计为桌面视觉验收。
- 外部新增五项多桌匹配检查通过；原 75 项中面板两项旧合成帧缺相机内参导致失败，补齐真实相机合同与桌面位置后，17 项面板及新增 5 项全通过，其余 58 项通过。Node 脚本语法、Python 编译和 diff 格式检查通过。
- 网页新增桌面列表、测试瓶数和位置、只切换观察视角的按钮；高桌观察时剖面高度随桌高提升。支持无 WebGL 时显示桌面列表和位置说明，未宣称浏览器 WebGL 成功。控制系统不自动导航或传送机器人。
- 部署前在线为空闲、无持物，归档公开任务／刚体姿态到 `runs/multitable-deploy-1794920168999892315`。根据公开刚体姿态重建原有关节位置，并以新模型初始值追加新瓶；正运动学重建位置最大误差小于 10 微米。用匹配新模型哈希的显式快照重启，不恢复任务；旧资产保留于外部 assets 归档。不是直接复用旧模型 qpos 或自动重放历史任务。
- 新服务 `runs/development-1794920195231333646` 已上线：health ready=true、physics_alive=true，任务 idle、无占用／持物／故障。HTTP manifest 实测六个桌面、五个新增瓶子，逐瓶在线位置及直立方向检查通过；网页 HTTP 已提供列表与按钮脚本。`doctor --development` 对 14,205 个资源文件及运行依赖检查通过。在线证据见部署目录 `online-check.json`。浏览器 CDP 聚焦超时，未声称实际按钮交互或 WebGL 观察成功。

### 2026-11-17：修复其他测试桌面不显现

- 用户截图中新增紫瓶与方形桌已在 WebGL 场景出现，但桌面入口置于可隐藏的左侧卡片。将唯一桌面选择器移至右侧抓取面板顶部、独立于可折叠任务区域；增加六处桌面的可点击场景标记。修正方向键监听，焦点在下拉框／输入框时不向底盘发送行走意图。
- 实际发现腕部渲染只显式开放原茶几的两块碰撞网格，其他测试桌面为隐藏组 4，导致瓶子可见而支撑面无对应 RGB-D。现在依据运行资源 manifest 的桌面范围和高度，筛选现有静态桌面碰撞网格作为组 2 的不透明 RGB／深度共同支撑面，不新建重叠碰撞。它是原碰撞网格的简化棕色显示，不是 HSSD 纹理的完整相机渲染；全屋其他隐藏碰撞仍未全部视觉化。
- 隔离六桌渲染逐一检测相应网格的可见分割像素（44,124–300,430 个）及有限深度值，检查质量、惯量、摩擦、碰撞掩码、solref／solimp 均未改变；方形桌渲染图经目视确认桌面与紫瓶共同出现。证据 `runs/surface-render-1794920877223821576/`。五项桌面匹配检查和 22 项原抓取世界回归通过；Python／Node 语法及 diff 格式检查通过。
- 独立无头浏览器验证桌面入口在 1476px 与 600px 视口、模拟 WebGL 不可用时仍可见，六个选项、物体计数和按钮事件正常；这不是在线三维按钮端到端验收。应用内浏览器 CDP 聚焦仍超时，未宣称已替用户刷新或逐个检查三维标记。
- 在线 idle、无持物时归档 `runs/surface-visibility-deploy-1794920914574543820/` 并基于同模型姿态快照重启，保留用户所在桌边场景而非回初始点；重启后基座物理再稳定约漂移 7.1 cm。新实例 `9b0b6fcf186f44e0ad124d6c2718cd8e`，health ready=true、physics_alive=true、任务 idle、无故障。实际在线腕部图像接口可返回 RGB-D，但当前腕部视角朝向室内、未看到桌面与紫瓶；不能将此帧计为在线目标可见验证。桌面与紫瓶共同可见的依据是前述隔离渲染。此次只完成显示、深度与回归修复，没有把新桌面完整抓放计为已成功；未 commit/push。

### 2026-11-17：完整 Lab 与六桌面更新发布

- 按用户要求，`feature/she` 保留完整 Lab 项目：统一部署入口、平台／仿真、网页、MTC、抓取源码快照、配置、环境与锁、场景和文档。本轮同步运行目录的六桌面配置与可见性修复，并在 README 明确目录与部署边界，不仅上传孤立的抓放模块。
- 发布检出验证 80 项外部回归通过，覆盖腕部放置 18、放置工作进程 4、抓取世界 22、网页面板 17、任务／接触 13、姿态 1、多桌匹配 5。解析 108 个 Python 文件及受控 JSON、检查 3 个 Go2 JavaScript 脚本、CLI 帮助、diff 格式通过。44 项算法快照 SHA-256 与当前运行源完全一致；原 HSSD/Go2 LFS 资源包实体 SHA-256 和 LFS fsck 通过。
- 独立发布检出未配置本机工作区时，doctor 如实报告缺少资产、Python 和端口被现有在线服务占用，没有启动第二个服务或停止在线任务。这不是新机器部署成功。新机器仍需 LFS、依赖／GraspNet 权重、消息构建及本机配置；运行缓存、凭据、日志和录像保持仓库外。

## 2026-11-19：11 cm TCP 在线复测及 AnyGrasp 备选

- 在线服务以匹配当前六桌面模型的命名关节快照启动，使用 0.11 m TCP。首轮蓝瓶侧倒，视觉高度不足，在运动前拒绝。改选直立绿瓶，完整抓起、持物回零、腕部选点放置、撤离回零通过，最低抬升约 9.34 cm；放后视觉检查通过，不等于测得支撑力。记录在外部 task-v3-online-1795097228861369990。
- 同轮目标点云 3318 点，GraspNet 预热后运行 10 次，推理中位 47.0 ms，P95 51.2 ms；不含规划或机械臂运动。AnyGrasp SDK 与离线对照脚本已准备，尚缺正式本机授权、检测权重及完整运行依赖，无可报告的 AnyGrasp 性能。默认后端不变。

## GraspGenX 本机实验接入

本机可在 config.local.json 的 go2_grasp.backend 选择 graspgenx 或 graspnet。
GraspGenX 在独立 GPU 进程输出 PiPER 候选，适配夹爪轴和 0.11 m TCP 后沿用既有
候选过滤、MoveIt 与执行流程。已在保存的 3318 点腕部输入上验证 worker 协议和 100 个候选。
首次在线观察受底盘稳定检查阻挡；恢复测试快照后，首轮冷推理超时。加入预热和
GraspGenX 专用 1.5 秒候选有效期后，网页通过一次绿瓶抓起、持物回零、腕部 RGB-D
选点放置和空爪回零。证据：task-v3-online-1795099718069617973。
一次成功仅验证链路，尚无多轮成功率。
## 2026-11-22 双后端候选诊断与 TCP 核对

- `grasp_generation.node` 对 GraspNet／GraspGenX 共用候选链增加逐帧筛选计数，诊断携带实际模型名、输入点数、原始模型候选数、worker 适配剩余数及原因、共用 17 列候选适配拒绝原因、几何／朝向后数量、发布数量、推理与后处理耗时。Lab 运行目录保留 `grasp-screening.jsonl`，任务结束生成 `grasp-screening-summary.json`，包括 MoveIt 分阶段拒绝原因；`result.json` 也标明实际后端。
- `tcp_geometry_audit` 在规划前核对本次导出的 URDF：两指关节均在 link6 z=0.1358 m；算法指尖 z=0.13503 m；11 cm TCP 比算法指垫中心前移 5 mm，距算法指尖 25.03 mm。仿真模型内部核对通过；真机 link6→指垫变换尚未实测，不能据此声称真机 TCP 已标定。
- `/usr/bin/python3 -m py_compile` 对修改的 ROS 桥和候选节点通过；用已有运行目录的 URDF／MoveIt 诊断对 TCP 检查和筛选汇总做隔离测试通过，能统计 `grasp_endpoint:rejected=14`、`lift:selected=1`。
- 尝试在隔离 Lab 对两后端做新一轮预览：恢复场景后观察姿态动作因“底盘尚未稳定”未执行，所选瓶体只有 4.6 cm 可见高度，被既有 6 cm 门槛在模型前拒绝。该轮未生成候选，不能计为两后端在线抓取成功或失败；主网页现场未重置。
- 等待隔离场景稳定、重新做观察姿态并从当前腕部 RGB-D 自动选择可见绿瓶后，两套后端的 MoveIt **预览均通过**，没有执行闭爪或抬升。GraspNet 证据 `/tmp/she-screening-graspnet/runs/go2-grasp-1795360641691439210/`：首帧诊断 8.46 s，首个可发布候选 32.78 s，136 条诊断中 3 帧发布候选；首个有效帧由 1023 个模型候选经共用适配剩 32 个、几何剩 1 个、方向搜索后发布 44 个。GraspGenX 证据 `/tmp/she-screening-graspgenx/runs/go2-grasp-1795360691720923197/`：首帧且首个有效候选 10.21 s，7 条诊断中 6 帧发布候选；首个有效帧由 100 个原始候选经 worker 适配保留 100 个、共用适配保留 46 个、几何剩 8 个、方向搜索后发布 100 个。两边最终 MoveIt 均选出一条完整抓取路径。模型推理中位数分别约 50 ms 与 100 ms；差距主要出现在等待**可用候选**的上游筛选过程，不能只用模型推理时间解释整段等待。这是单场景预览，非抓取成功率或真机速度结论。

## 2026-11-22 抓取内部筛选收敛

- 对上述 GraspNet 同一保存点云重放：1024 个模型行经夹爪适配剩 26 个；26 个方向的世界 X 分量最大 0.552，旧 `refine_candidates` 的 `axis[0] >= 0.7` 固定方向门槛将它们全部丢弃。该门槛与当前桌位绑定，与后面的机器人径向方向搜索重复，已从实际 A2 抓取源移除。保留后续姿态生成、MoveIt 逆解／碰撞和完整轨迹检查。
- 同一隔离恢复场景复测，GraspNet 首个可发布候选从 32.78 s 降至 8.76 s，MoveIt 预览通过，证据 `/tmp/she-screening-graspnet/runs/go2-grasp-1795361629032870248/`。两轮像素有少量差异，约 24 s 改善不能当成大样本速度分布。
- 网页抓取的 TCP 改为从本地 `go2_grasp.tcp_m` 读取（默认 0.11 m）；GraspNet 与 GraspGenX 共享该值，17 列候选转换的指尖差量直接使用本次仿真几何核对的 0.13503 m 指尖值，避免原先硬编码 0.135 m 与几何常数不一致。
- 改动后分别执行完整隔离抓取，均通过物理抬升、持续持物和持物回零：GraspNet 证据 `/tmp/she-screening-graspnet/runs/go2-grasp-1795361680401109029/`，首个可用候选 8.50 s，最低抬升 9.86 cm；GraspGenX 证据 `/tmp/she-screening-graspgenx/runs/go2-grasp-1795361748244870442/`，首个可用候选 10.37 s，最低抬升 9.85 cm。两套模型推理中位分别约 50 ms、107 ms。均为单次隔离仿真成功，不代表真机抓取率或多场景稳健性。本轮不改放置、主网页现场或仿真物理参数。

## 2026-11-22 修改后 GraspNet／GraspGenX 配对计时

使用相同初始 MuJoCo 快照、相同观察动作、同一绿瓶附近 RGB-D 像素，按 GraspNet→GraspGenX→GraspGenX→GraspNet 顺序在四个独立 Lab 实例中各抓一次。每轮从网页提交抓取开始，以单调时钟每 0.2 秒采样后端阶段，检查 `result.json`、`selected_grasp.json`、最低抬升和持物回零结果。四轮均通过；GraspNet／GraspGenX 实际选中候选的 generator 分别为 `graspnet_baseline_preview`／`graspgenx_piper_preview`，TCP 均为 0.11 m。原始记录 `/tmp/she-ab-timing-20261122/results.json`。

- GraspNet 两轮：首个可用候选 8.648／8.721 s；开始接近 10.227／10.307 s；闭爪 15.960／16.043 s；进入持物 18.432／18.532 s；完整持物回零与验证结束 44.607／44.569 s；最低抬升约 9.83 cm。
- GraspGenX 两轮：首个可用候选 10.617／10.469 s；开始接近 12.455／12.229 s；闭爪 18.210／17.974 s；进入持物 20.648／20.440 s；完整持物回零与验证结束 46.563／46.639 s；最低抬升约 9.84–9.86 cm。
- 均值差异：GraspNet 首候选快约 1.86 s，开始移动快约 2.08 s，完整抓取与持物回零快约 2.01 s。单轮内预热后模型推理中位约 48–50 ms／104–117 ms，后处理中位约 46–48 ms／16–17 ms；主要差距仍在启动到可用候选的整段，而非单次模型计算。闭爪到持物及持物回零后半段两方案基本相同。
- 这是 2 次／后端、单场景、隔离仿真计时；RGB-D 帧与像素有微小变化，不能据此推出准确率、真机速度或统计显著性。主网页 `18769` 的故障现场未动。

## 2026-11-23 双方法发布前固定场景抓放门槛

- 从同一 `anygrasp-baseline-start.json` 快照为每轮创建独立工作区，顺序运行 GraspNet 5 轮、GraspGenX 5 轮。每轮都执行观察姿态、腕部 RGB-D 选绿瓶、实际抓起并持稳回零、重新采腕部 RGB-D 选桌面、实际放置、释放、视觉复核、撤离回零；核对 `result.json` 的真实后端和最终 `idle`。完整原始结果在本机外部 `/tmp/she-publish-rate-20261123-v2/results.json`，各轮日志和任务目录在对应工作区。未把预览、候选数量或进入 `holding` 单独算成功。
- 观察动作刚恢复场景时会因未累计 0.5 秒稳定读数而失败。`run_pose` 对这一条明确的暂态错误在 12 秒内等待重试，其余错误照常报出。修复后 10 轮均完成观察；此前启动时序失败的废轮不计入抓放通过率。
- GraspNet 5/5 完整通过；中位首个可用候选 8.64 秒，中位全轮 93.59 秒，2/5 轮首次放置点被新深度帧拒绝并重选。GraspGenX 5/5 完整通过；中位首个可用候选 10.45 秒，中位全轮 99.19 秒，4/5 轮首次放置点被拒绝并重选。所有 10 轮最终放置结果 `released=true`、`returned_to_zero=true`，包含腕部深度视觉结果；`support_confirmed=false`，没有真实支撑力确认。全轮时间包含服务启动、观察、选点、重选、抓放及停止，不能当作纯模型推理时间。
- 独立干净宿主固定在 `1ed97b0`，发布包 `install_overlay.py --apply` 安装 67 个文件，复制后 SHA-256 全部匹配；Python 编译、两个启动脚本 Bash 语法、`git diff --check` 通过。Agent `zero`／`observation` 显式确认字段做了单独接口检查。主网页 18769 的服务与现场未参与这些隔离测试。
- 这 5 轮／方法只覆盖同一仿真瓶和桌面，观测通过率 100% 不能证明总体成功率或真机成功率 ≥80%。当前抓取确认、掉落和放置准入仍读 MuJoCo 瓶子的精确高度／位姿；真机透明瓶与夹爪遮挡下的有效深度可能低于现有 100 点及 6 cm 可见高度门槛。真实部署前必须替换仿真真值判定、处理稀疏深度并做多场景及实机验收。

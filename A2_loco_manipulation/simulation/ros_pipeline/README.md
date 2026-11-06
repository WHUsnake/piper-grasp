# ROS 主链路 → MuJoCo 验证

本目录将主工程维护源作为实际 ROS 节点运行，不调用旧独立仿真程序的 SciPy IK 或抓取动作序列。

实际链路：MuJoCo 虚拟 RGB-D 相机 → `perception_node` 的 YOLO11s-seg 掩膜 → 现有去畸变/侵蚀/深度检查/米制点云 → `grasp_generation` 官方真实权重推理与共享瓶身中段/夹持区适配 → `robot_planner` 的 MoveIt IK、OMPL 预抓取轨迹、完整 Cartesian 进近/抬升轨迹 → `task_executor` 一次抓取状态机 → `/sim/arm_controller/follow_joint_trajectory` → MuJoCo 动力学及实际反馈。

主工程维护源：`src/perception/perception_node`、`src/manipulation/grasp_generation`、`src/manipulation/robot_planner`、`src/manipulation/task_executor`、`src/interfaces/ros2/grasp_msgs`。原树缺少的包从本工程已审计快照恢复，来源及原哈希在 `data/graspnet/maintained_source_provenance.json`；历史快照和安装产物没有被当作修改目标。共享几何策略现在由 ROS 包的 `grasp_generation/refinement.py` 维护，旧独立仿真仅导入它。

## 构建和运行

在 she 根目录：

```bash
A2_loco_manipulation/scripts/graspnet/build_pipeline.sh
A2_loco_manipulation/simulation/ros_pipeline/run.sh \
  --output /tmp/a2_ros_grasp_example \
  --bottle-x .72 --bottle-y 0 --table-z .76 --tcp-m .095
```

运行前需要本机已安装 ROS2 Humble/MoveIt、现有 CUDA GraspNet 环境、YOLO 环境与 MuJoCo 环境。脚本不安装或升级系统依赖。`piper_msgs` 只复用消息定义，不启动 PiPER 驱动。路径配置目前针对这台开发机，不能直接复制到机载端声称部署完成。

隔离域为 182，`ROS_LOCALHOST_ONLY=1`；进程锁防止多个实验同时使用同一 ROS 图。启动只包括 MoveIt、状态发布器、感知/候选/规划/执行器与 MuJoCo bridge，不启动 CAN、PiPER SDK、底盘或实机使能。实机 `/task/start` 保持 `execute=false`；仅显式 `/sim/task/start` 可以调用模拟器控制器，并核对本次会话、起始关节和新鲜目标几何。退出、取消会回收本轮拥有的子进程。

原工程实机的身份绑定链路仍通过 BoundTargetCloud/监督入口接入。新恢复的简单 RGB 分割节点默认只输出 `/perception/unbound_target_cloud`，不会绕过实机身份绑定直接喂候选；模拟单瓶会话必须显式 `simulation_mode=true`。没有伪造实机导航跟踪器身份或解锁实机互锁。

## 模型和规划

`prepare.py` 使用现有 A2/PiPER MJCF 生成与 MuJoCo 完全一致的关节链和碰撞几何，固定站立腿与底座，arm 组仍是六轴 PiPER。URDF 来源网格保留，安装位姿仍是已记录 CAD 估计。它是与仿真一致的 MoveIt 模型，不是宣称已校准的实机模型。

MoveIt 场景包括机器人、自碰撞、桌面/桌腿和由目标点云拟合的圆柱瓶身。进近采用打开到所选宽度的手指状态；抬升规划时将瓶子作为附着碰撞体，仅允许手指触碰。完整路径 `fraction>=0.999`，不会执行部分成功路径。实际执行跟随选中消息中的同一组轨迹，反馈来自 MuJoCo 关节状态和接触力。

`SelectedGrasp.msg` 携带采集 header、会话、候选编号、抓取/预抓取/抬升位姿、开口/距离及三段轨迹，以及规划目标的中心、半径和高度。规划耗时不通过修改采集时间解决；执行前用新的目标几何复核位置/尺寸，并检查起始关节。网络方向类别有波动不等于物体发生运动，因此复核的是物体，不要求网络重复输出同一个角度。

## 已验证和限制

`end_to_end06/` 为真实完整 ROS 链路成功记录：YOLO 掩膜（非 oracle）、约 4822 点、MoveIt 完整三段轨迹、保持阶段最低抬升约 9.88 cm、双指接触 100%、实际指尖余量约 30 mm。之前的失败试验目录保留，不作为成功结果。更新后每次网页实验保存在 `simulation/a2_table_grasp/runs/<id>/`，`metrics.json` 的 `pipeline` 字段必须是 `ROS2_YOLO_GraspNet_MoveIt_TaskExecutor` 才是主链路结果。

证据包括 `perception_provenance.json`、`perception_capture.npz`、`selected_observation.json`、`selected_grasp_diagnostics.json`、`planner_diagnostics.json`、各节点日志、物理帧与接触 trace、指标和视频。目标点云按被选中候选的原采集时间精确匹配保存，不用最新一帧冒充。

目前仍是单个直立圆柱瓶、不透明刚体、虚拟观察相机、固定 A2 的验证。近距离时虚拟相机保持至少约 34 cm 前后观察距离避免裁切；真实相机需单独确定观察姿态和手眼标定。未验证任意物体、透明深度缺失、导航、站立平衡、真机执行或所有网页参数都可达。

当前已安装 MoveIt 2.5.9 在完成/停止后的销毁阶段出现过 SIGSEGV（CallbackGroup 析构），各轮日志保留。它没有被计作规划或抓取成功证据，也没有隐藏退出码；`process_exit_codes.json` 记录子进程状态。物理成功判据在停止前独立采集。这个环境退出问题仍需修复，不能把本机试验当作可无人值守的部署验收。


## IK / MoveIt 多场景回归

`regression.py --output <绝对目录>` 逐项运行默认、低桌、近距离低桌、较远位置、左右偏移及远距离重复实验。每项使用同一套代码和固定 TCP=9.5 cm；汇总 `results.json`，保留每次原始感知、规划、执行和物理证据。不能把规划成功或历史视频当成物理成功。

当前优化包括：点云瓶身约束的网络/径向/中间方位搜索、候选去重、直接抓取优先于等价翻转、32个候选上限；IK多初值；根据重新拟合的瓶身设置适配候选预张开宽度，超出夹爪能力则拒绝。Cartesian采用5 mm插值、0.25 rad绝对关节跳变限制，并额外检查返回路径相邻关节步长；相对平均步长阈值关闭，避碰与fraction>=0.999保留。失败记录区分接近/抬升，并保留仅用于诊断的无避碰查询结果和首个无效状态接触对，绝不执行诊断路径。

仿真规划允许竖直抬升失败后，重新检查水平回收4/8 cm并抬升原高度的备选。每条路径都使用附着目标避碰检查，最终SelectedGrasp携带实际选中的抬升位姿/轨迹。实机默认退让距离列表仍为[0.0]，没有自动开启此仿真参数。

返回轨迹还需逐点FK复核：工具方向相对要求误差<=0.03 rad、距离要求直线<=3 mm，并逐点复核碰撞有效性。此检查可阻止重采样把第6关节跨越正负pi的跳转变成密集但工具方向错误的轨迹。只检查fraction或关节采样间隔不足以保证正确。


## 扩展工作空间优化

当前模拟配置在中段之外搜索观测瓶身40%/60%的夹持高度，保持真实夹持面与TCP的一致关系。先求解实际夹持位姿的IK，再以该解为初值反向求预抓取，避免从零位选到无法连续接近的分支；端点无解的候选不会重复进行相同端点求解。默认10 cm预抓取失败后可搜索6/14 cm，抬升回收选项扩展至12 cm。整个搜索预算90秒，不用过期观测启动执行。

`regression.py --suite wide --output <目录>` 覆盖55 cm高桌、85 cm低桌、大幅左右20 cm偏移和高低桌组合；`--suite edge` 覆盖更低桌面；`--cases default,low,far,far_repeat` 可选择标准用例。所有环境参数保留，不把未找到解误写成物体被抓住。

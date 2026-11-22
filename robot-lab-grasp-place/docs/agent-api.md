# Agent 抓放接口（本地仿真）

唯一对外任务接口为 Lab HTTP 服务；`agent_client.py` 提供零外部依赖的命令行包装。Agent 先读 `/api/grasp/status` 的 `task.state` 和 `task.allowed_actions`。不要直接调用 ROS 节点、推理 worker 或仿真内部 RPC。

## 一次抓取和放置

```bash
CLIENT=/absolute/path/to/piper-grasp/robot-lab-grasp-place/agent_client.py
python3 "$CLIENT" status
python3 "$CLIENT" capture --out /tmp/grasp-frame
# Agent 查看 /tmp/grasp-frame/rgb.ppm、depth-f32le.bin、self-mask-u8.bin，确定目标像素 u,v
python3 "$CLIENT" action run --frame /tmp/grasp-frame/frame.json --u 320 --v 240
python3 "$CLIENT" wait --timeout 180
# 只有 task.state=holding 且 task.allowed_actions.place=true 才继续
python3 "$CLIENT" capture --out /tmp/place-frame
# 选择空白支撑桌面的像素；以下坐标仅为命令格式示例
python3 "$CLIENT" action place --frame /tmp/place-frame/frame.json --u 360 --v 300
python3 "$CLIENT" wait --timeout 180
```

`capture` 保存 640×480 RGB PPM、逐像素 little-endian float32 深度（米）、逐像素 uint8 机械臂遮挡掩码及 `frame_id`。像素从左上角起算。Agent 不应从机身 RGB 猜深度。抓取帧最多保留 60 秒，放置帧最多保留 30 秒；应在选点前尽快采图，过期后重新采集和选点。

`action run` 和 `action place` 只在服务端 `allowed_actions` 允许时发送。HTTP 409、非零退出或 `wait` 超时后先查询 `status`；不能盲目重复运动命令。`result.passed`、`task.state`、`world.holding` 与 `world.hold_valid` 应一起判断。失败可以是 `retryable_failure` 或 `recovery_required`，后者需要检查现场；`restore_initial` 会重置物体和夹持现场，不能自动执行。Agent 选择目标和落点，服务端负责规划、接触控制与恢复边界。

若放置选点被“footprint obstructed / occluded”拒绝且状态仍为 `holding`，保留夹持，重新采集腕部 RGB-D，换一个离桌边和障碍更远的平坦区域；最多按任务策略尝试有限次数，记录每次被拒绝的像素和原因。不能在同一旧帧上反复提交，也不能把重选成功算成首次放置成功。

算法切换在启动服务前设置 `LAB_GRASP_BACKEND=graspnet|graspgenx`（各方法目录的 `launch.sh`）。当前服务一次只加载一套后端，切换需停止当前服务并重新启动；不支持一个任务中途换模型。结果文件的 `backend` 与 `grasp-screening.jsonl` 是审计依据。

## 尚需完善的接口能力

当前 HTTP API 没有幂等请求 ID、任务取消确认或正式 OpenAPI schema；Agent 应将动作串行化，不要并发提交。真正接入真机前必须另做坐标与 TCP 标定、力/接触信号映射、硬件急停和独立验收。当前发布仅验证本地 MuJoCo 仿真。

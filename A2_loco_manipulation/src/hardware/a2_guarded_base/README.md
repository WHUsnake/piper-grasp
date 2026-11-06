# 底盘互锁驱动候选（未启用）

来源：2026-10-28 只读复制实际运行驱动 `/home/unitree/a2_nav/unitree_sdk2/example/a2/a2_move_sub.cpp` 到 upstream_a2_move_sub.cpp。原文件与服务未修改。guarded_a2_move_sub.cpp 为候选版本，保留反馈与 TF 发布，不订阅旧 cmd_vel、mode_switch、upDown、sport_cmd、body_height、speed_level、euler、body_pose、temStatus 命令入口。

所有新运动请求由 /a2_unified/base_command 承载，必须有本次 claim 返回的启动身份、令牌、严格递增序列、有效阶段、时间戳和新鲜底盘反馈。默认 allow_task_control=false；启动不会发运动或 StopMove。存在旧 /unitree_a2_ros_control 时，在初始化 SDK 之前拒绝启动。

租约超时 0.4 秒、命令年龄最多 0.2 秒。初版限速 x=0.10m/s、y=0.05m/s、yaw=0.15rad/s；超限直接拒绝。进入 grasp 后不能回导航。曾经发送导航命令后遇到过期、无反馈或错误，撤销许可并请求 StopMove；SDK 返回成功不是实测停稳证据。故障锁不提供自动重置。

重要边界：这是候选驱动，不是已生效的整机互锁。还未实现/验证上层 authority_socket 到该驱动的连接，未验证遥控器、其他 DDS 客户端和机内控制源的仲裁；不能只因不订阅旧 ROS 话题而声称物理控制权独占。没有自动替换旧服务，也没有发布任何真实控制请求。用户两阶段操作要求不改变这一边界。

构建输出在主工程 workspaces/guarded_base_build。链接现有 SDK 库为只读依赖，未改 SDK 文件。纯 C++ lease_gate 测试 14 个场景通过；候选驱动机载编译链接通过。未执行候选二进制、未打开第二份 SDK 控制实例。

本地无硬件测试：

```bash
cmake -S src/hardware/a2_guarded_base -B data/build-guard
cmake --build data/build-guard
ctest --test-dir data/build-guard --output-on-failure
```

后续须完成适配器、全链路模拟、外部控制源核验，再安排现场切换及低速实测。不要并行启动两份底盘驱动，不要把候选程序加入开机服务。

# PiPER GraspNet / A2 固定站立抓取仿真

## 当前抓取与放置更新（2026-11-17）

新增 [robot-lab-grasp-place/](robot-lab-grasp-place/README.md)：当前 Robot Lab 的腕部 RGB-D
选点抓取、持物回零、MTC 放置、小步下降／释放、撤离回零、网页与故障恢复代码，
包含 44 文件算法源码快照、内容校验及接入说明。此更新需要同版本 Robot Lab 宿主，
不会通过下面的旧 A2 启动脚本自动生效；旧平台与历史验收保持原样。

---

从已验证的 piper graspnet 保存版本整理。包含抓取ROS源码、GraspNet模型与基线源码、MuJoCo仿真、网页服务器、A2/PiPER模型资源和成功回放。大型Python虚拟环境、构建缓存和完整历史运行目录保留在本地保存版本。

主链路：DC1 RGB-D → 可见轮廓/深度连通区域 → 目标点云 → GraspNet与几何适配 → IK/MoveIt → ROS执行器 → MuJoCo接触抓取。网页预览支持直接点击水瓶、苹果、小纸盒、易拉罐和水杯；候选生成只使用运行时深度点云，不依赖 YOLO 类别。TCP默认9.5 cm，固定站立底盘，不启用实机执行。

网页源码：A2_loco_manipulation/simulation/a2_table_grasp；ROS/MuJoCo桥接：A2_loco_manipulation/simulation/ros_pipeline。部署说明和已知限制见各目录README及docs/抓取代码审计与MuJoCo接入_20261106.md。示例静态资源保留于site/dist/assets。

需要Ubuntu/ROS2 Humble、MoveIt、Python3.10和兼容CUDA环境。启动真实 GraspNet worker 前设置 `GRASPNET_PYTHON`、`GRASPNET_BASELINE`、`GRASPNET_CHECKPOINT`；如需额外的 PiPER 消息工作空间，再设置 `PIPER_MSGS_SETUP`。配置不再写入保存机器的绝对路径。requirements文件记录保存机器的环境，不代表跨机器免配置安装包。各第三方组件保留原许可证。

本轮迭代增加 DC1 多物体点击预览、目标深度区域提取、五物体验收脚本和记录。验收记录位于 `A2_loco_manipulation/data/graspnet/multi_object_validation_20261112.json`，固定仿真场景最低保持抬升为 8.3–10.8 cm。运行网页预览：

```bash
cd A2_loco_manipulation/simulation/a2_table_grasp
./start_web.sh
```

然后打开终端输出的本机地址，点击“生成 DC1 多物体预览”，直接点击目标并运行。

这是仿真验证代码；直立圆柱瓶身适配和已测试场景成功不等于任意物体、全滑条范围或实机均已验收。

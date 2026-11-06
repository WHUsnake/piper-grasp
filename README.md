# PiPER GraspNet / A2 固定站立抓取仿真

从已验证的 piper graspnet 保存版本整理。包含抓取ROS源码、GraspNet模型与基线源码、MuJoCo仿真、网页服务器、A2/PiPER模型资源和成功回放。大型Python虚拟环境、构建缓存和完整历史运行目录保留在本地保存版本。

主链路：RGB-D → YOLO掩膜 → 目标点云 → GraspNet与瓶身几何适配 → IK/MoveIt → ROS执行器 → MuJoCo接触抓取。TCP默认9.5 cm，固定站立底盘，不启用实机执行。

网页源码：A2_loco_manipulation/simulation/a2_table_grasp；ROS/MuJoCo桥接：A2_loco_manipulation/simulation/ros_pipeline。部署说明和已知限制见各目录README及docs/抓取代码审计与MuJoCo接入_20261106.md。示例静态资源保留于site/dist/assets。

需要Ubuntu/ROS2 Humble、MoveIt、Python3.10和兼容CUDA环境。模型配置中的本机路径在其他机器部署时需要设置；requirements文件记录保存机器的环境，不代表跨机器免配置安装包。各第三方组件保留原许可证。

这是仿真验证代码；直立圆柱瓶身适配和已测试场景成功不等于任意物体、全滑条范围或实机均已验收。

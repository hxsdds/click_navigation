# 第三方来源与许可

本仓库从本机 `uav_dog_plume` 的点击导航工作版本提取，保留已有改动，未重新引入未经验证的最新控制器。来源快照哈希记录见 `docs/extraction_manifest.json`；该文件记录提取时的源内容，后续为独立打包而作的修改以 Git 记录为准。

| 内容 | 来源及版本 | 许可记录 |
| --- | --- | --- |
| 必要机器狗控制/模型包 | 原项目中的 quadruped_ros2_control 工作副本，含本项目步态调整 | `src/quadruped_ros2_control/LICENSE`、`LICENSES/`、各包 package.xml 与源码声明 |
| Go2 模型与网格 | 上述工作副本中的 go2_description | 保留上游 package.xml 的 BSD 声明及文件声明 |
| Unitree Guide 与控制公共代码 | 上述 unitree_guide_controller/controller_common | 保留源码声明，附上游 LICENSES/unitree_guide 和 legged_control |
| PX4 ROS 消息 | https://github.com/PX4/px4_msgs ，提交 `86d8239e962f6939e05c3737784f60c02fa884db` | 保留 `src/px4_msgs/LICENSE` |
| PX4 SITL | https://github.com/PX4/PX4-Autopilot ，`v1.17.0` | 外部源码由安装脚本获取，保留其上游许可，不加入运行二进制到 Git |
| Micro XRCE-DDS Agent | https://github.com/eProsima/Micro-XRCE-DDS-Agent ，`v2.4.3` | 外部源码由安装脚本获取，保留其上游许可 |

`uav_dog_nav2` 与 `uav_dog_sim` 沿用原包 Apache-2.0 声明；第三方资源按各自声明分发，不因提取而改写所有权或许可。

`px4_msgs/CMakeLists.txt` 增加可选的点击导航最小接口构建开关，所有上游消息文件原样保留，未改消息格式。

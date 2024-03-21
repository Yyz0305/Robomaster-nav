# Robomaster-nav
# Robomaster 机甲大师 - 鸿烈战队导航开源项目

## 项目简介
Robomaster 机甲大师是一项由大疆创新推出的教育机器人竞赛项目，旨在培养青少年对科学、技术、工程和数学（STEM）的兴趣。
鸿烈战队是一支热衷于Robomaster比赛的团队，致力于开发导航系统，提高机器人自主导航能力。

## 项目目标
- 开发高效、精准的导航系统，使机器人能够在复杂的比赛场地中自主导航。
- 提高机器人的运动控制和路径规划能力，以应对各种挑战。
- 促进团队成员在机器人技术和团队合作方面的学习和成长。

## 技术特点
- 使用激光雷达传感器获取环境信息，使用IMU（惯导模块）获得当前位姿及速度信息。
- 基于SLAM技术实现实时地图构建和定位。
- 结合amcl算法，提取环境特征。
- 采用socket传输数据
- 将现实数据通过变换映射到虚拟地图中，实现无场地导航

## 功能包对应功能
- **fdilink_ahrs**：IMU传感器数据采集和预处理，得到加速度和位姿等数据。
- **rplidar—_ros**：激光雷达传感器数据采集和处理，得到周围障碍距离。

- **mbot_gazebo**：gazebo中物体组成相关参数文件。
- **mbot_description**：机器人相关组成部分相对位置参数文件。

- **mbot_navigation**：1.amcl(蒙特卡洛):实时地图构建和定位，生成机器人所在环境和位姿的定位。
- **mbot_navigation**：2.nav_cloister_demo:路径规划和导航控制，根据地图信息进行路径规划.
- **mbot_navigation**：3.move_base 运动框架：机器人运动控制，实现机器人的各种运动功能。
- **mbot_navigation**(-config)：rviz中的描述性文件参数调整，包括刷新率、发布频率等。

- **YJM_MID**(-pub_odom_test)：1发布/cmd_vel消息,使真实惯导数据可以映射到gazebo中
- **YJM_MID**(-yjm_mid)：2读取转换真实的雷达数据发布在/scan中

- **odom_scan**：1.进行socket通信部分 2.计算部分仅需要获得机器人的当前位姿的yaw与plan的yaw差值
：

## 开源计划
鸿烈战队致力于将导航系统开源，以促进机器人领域的技术交流和共享。具体开源计划包括：
1. 发布导航系统的源代码和文档，供其他团队学习和使用。
2. 提供技术支持和社区论坛，与开发者共同探讨问题和解决方案。
3. 定期更新项目，引入新的功能和优化，提升系统性能和稳定性。

## 贡献方式
- 欢迎开发者参与项目贡献，包括但不限于提交代码、报告问题、提出建议等。
- 参与社区讨论，分享经验和技术，共同推动机器人导航技术的发展。
- 如果您对项目感兴趣，欢迎联系我们加入鸿烈战队，一起探索机器人的奥秘！

## 联系方式
- 项目Gitee仓库：[Robomaster Navigation](https://github.com/your_repository)
- 项目GitHub仓库：[Robomaster Navigation](https://github.com/your_repository)
- 鸿烈战队官方网站：[鸿烈战队](https://www.example.com)
- 联系邮箱：team@example.com

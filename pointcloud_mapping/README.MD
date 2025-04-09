# pointcloud_mapping

用于演示ICP里程计及地图构建的简易程序，该程序接收激光点云数据，利用PCL库中的ICP 算法实现相邻帧点云匹配，计算运动增量。在此基础上累积运动增量，获取激光雷达运动的轨迹，并对点云进行拼接。

### 使用方法

为VIO位姿估计节点，基于openvins修改，输入双目图像和IMU测量数据，输出以IMU为中心的位姿态

**step1** 启动roscore节点

`roscore`

**step2** 新建一个终端，启动建图节点

`rosrun pointcloud_mapping pointcloud_mapping`

**step3** 新建一个终端，播放数据集

rosbag play -r 0.1 zed_rslidar_xsens_rtk_2023-05-31-17-33-49.bag

 **step4** 新建一个终端，开启RVIZ可视化数据 (rviz中fix fram 设置为 rslidar)

`rviz`
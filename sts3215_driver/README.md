
# sts3215_driver

飞特 STS3215-C001串型舵机ROS节点，

连接方式：电脑通过串口与舵机控制板连接，实现对舵机控制

波特率：1M

<img src="reference/sts3215.png" alt="sts3215" style="zoom:60%;" />

 ## 1.1 下载与配置

 1. 安装依赖项

    sudo apt-get install ros-xxx-serial
    
 2. cd catkin_ws/src

 3. git clone  **sts3215_driver**

 4. catkin_make

 ## 1.2 启动节点

     roslaunch sts3215_driver sts3215.launch


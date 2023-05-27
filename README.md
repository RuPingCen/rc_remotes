# rc_remotes
该ROS节点是利用单片机获取航模遥控器输出的SBUS信号，通过STM32F103串口2发送到ROS中。数据协议与
# 下载安装
 1.安装依赖项

    sudo apt-get install ros-xxxx-serial

 2. cd catkin_ws/src

 3.  git clone  https://github.com/RuPingCen/rc_remotes.git

 4. catkin_make

# 参数说明

cmdvel_topic (发布的话题名称)

dev（串口设备名称）

baud（串口波特率）

hz（数据发布频率）

show_message（是否打印接收机数据）

RC_K（速度比例因子，默认为1，此时遥控器输出的速度为1、2、3m/s）

RC_MIN（遥控器摇杆通道的最小值）

RC_MAX（遥控器摇杆通道的最大值，最大值和最小用于判断遥控器输出的数据是否在这个边界范围内）

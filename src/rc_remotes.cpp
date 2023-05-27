/**
 * 接收SBUS的數據發布到topic
 *
 * maker:crp
 * 2017-5-13
 */

#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

#include <serial/serial.h>
#include <std_msgs/String.h>

#include <boost/asio.hpp> //包含boost库函数
#include <boost/bind.hpp>
#include <sys/time.h>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;
using namespace boost::asio; //定义一个命名空间，用于后面的读写操作

union INT32Data // union的作用为实现char数组和float之间的转换
{
	uint32_t uint32_data;
	unsigned char byte_data[4];
} rc_remote_upload_counter;
typedef struct
{
	uint16_t ch1, ch2, ch3, ch4;
	uint16_t ch5, ch6, ch7, ch8, ch9, ch10, ch11, ch12, ch13, ch14, ch15, ch16, ch17, ch18;
	uint8_t sw1, sw2;
	uint16_t ch1_offset, ch2_offset, ch3_offset, ch4_offset;

	uint32_t cnt;
	uint8_t available;
	uint8_t update;
	uint8_t type; // 1 DJI-DBUS   2 SBUS
	uint8_t state;

} rc_info_t;

rc_info_t rc;
float RC_MIN = 0, RC_MAX = 2500, RC_K = 1; //遥控器摇杆通道输出的最小值、最大值、比例系数
unsigned int init_times = 0;
int sum_offset[4] = {0};
int show_message =0;

struct timeval time_val; // time varible
struct timezone tz;
double time_stamp;
serial::Serial ros_ser;
ros::Publisher cmdvel_pub;

bool analy_uart_recive_data(std_msgs::String serial_data);
bool publish_cmd_vel(void);
bool calculate_rc_offset(void);

int main(int argc, char **argv)
{
	string out_result;
	bool uart_recive_flag;

	//     unsigned char buf[200];                      //定义字符串长度
	//     boost::asio::io_service iosev;
	//     serial_port sp(iosev, "/dev/ttyUSB0");         //定义传输的串口
	//     sp.set_option(serial_port::baud_rate(115200));
	//     sp.set_option(serial_port::flow_control());
	//     sp.set_option(serial_port::parity());
	//     sp.set_option(serial_port::stop_bits());
	//     sp.set_option(serial_port::character_size(8));

	string cmdvel_topic, dev;
	int baud, time_out, hz;
	ros::init(argc, argv, "mick robot");
	ros::NodeHandle n("~");

	n.param<std::string>("cmdvel_topic", cmdvel_topic, "/cmd_vel");
	n.param<std::string>("dev", dev, "/dev/rc_remote");
	n.param<int>("baud", baud, 115200);
	n.param<int>("time_out", time_out, 1000);
	n.param<int>("hz", hz, 50);
	n.param<int>("show_message", show_message, 0);
	n.param<float>("RC_K", RC_K, 1);
	n.param<float>("RC_MIN", RC_MIN, 0);
	n.param<float>("RC_MAX", RC_MAX, 2500);

	ROS_INFO_STREAM("dev:   " << dev);
	ROS_INFO_STREAM("baud:   " << baud);
	ROS_INFO_STREAM("time_out:   " << time_out);
	ROS_INFO_STREAM("hz:   " << hz);
	ROS_INFO_STREAM("show_message:   " << show_message);
	ROS_INFO_STREAM("RC_K:   " << RC_K);
	ROS_INFO_STREAM("RC_MIN:   " << RC_MIN);
	ROS_INFO_STREAM("RC_MAX:   " << RC_MAX);
	
	//发布主题sensor
	cmdvel_pub = n.advertise<geometry_msgs::Twist>(cmdvel_topic, 20); //定义要发布cmd主题

	// 开启串口模块
	try
	{
		ros_ser.setPort(dev);
		ros_ser.setBaudrate(baud);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		// serial::Timeout to = serial::Timeout(1,time_out,0,time_out,0);
		to.inter_byte_timeout = 1;
		to.read_timeout_constant = 5;
		to.read_timeout_multiplier = 0;
		ros_ser.setTimeout(to);
		ros_ser.open();
		ros_ser.flushInput(); //清空缓冲区数据
	}
	catch (serial::IOException &e)
	{
		ROS_ERROR_STREAM("Unable to open port ");
		return -1;
	}
	if (ros_ser.isOpen())
	{
		ros_ser.flushInput(); //清空缓冲区数据
		ROS_INFO_STREAM("Serial Port opened");
	}
	else
	{
		return -1;
	}

	ros::Rate loop_rate(hz);

	while (ros::ok())
	{
		if (ros_ser.available())
		{
			// ROS_INFO_STREAM("Reading from serial port");
			std_msgs::String serial_data;
			//获取串口数据
			serial_data.data = ros_ser.read(ros_ser.available());
			uart_recive_flag = analy_uart_recive_data(serial_data);
			if (uart_recive_flag)
			{
				uart_recive_flag = 0;
				publish_cmd_vel();
			}
			else
			{
				ROS_WARN_STREAM(" analy uart recive data error ...");
				// serial_data.data = ros_ser.read(ros_ser.available());
				ros_ser.flushInput(); //清空缓冲区数据
				sleep(0.2);			  //延时0.1秒,确保有数据进入
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	std::cout << " EXIT ..." << std::endl;
	ros::waitForShutdown();
	ros::shutdown();
	return 1;
}

/**
 * @function 解析串口发送过来的数据帧
 * 成功则返回true　否则返回false
 */
bool analy_uart_recive_data(std_msgs::String serial_data)
{
	unsigned char reviced_tem[500];
	uint16_t len = 0, i = 0, j = 0;
	// unsigned char check=0;
	unsigned char tem_last = 0, tem_curr = 0, rec_flag = 0; //定义接收标志位
	uint16_t header_count = 0, step = 0;					//计数这个数据序列中有多少个帧头
	len = serial_data.data.size();
	if (len < 1 || len > 500)
	{
		ROS_WARN_STREAM("serial data is too short ,  len: " << serial_data.data.size());
		std_msgs::String serial_data;
		string str_tem;

		serial_data.data = ros_ser.read(ros_ser.available());
		str_tem = serial_data.data;
		return false; //数据长度太短　
	}
	//ROS_INFO_STREAM("Read: " << serial_data.data.size());

	// 有可能帧头不在第一个数组位置
	for (i = 0; i < len; i++)
	{
		tem_last = tem_curr;
		tem_curr = serial_data.data.at(i);
		if (tem_last == 0xAE && tem_curr == 0xEA && rec_flag == 0) //在接受的数据串中找到帧头　
		{
			rec_flag = 1;
			reviced_tem[j++] = tem_last;
			reviced_tem[j++] = tem_curr;
			// ROS_INFO_STREAM("found frame head" );
		}
		else if (rec_flag == 1)
		{
			reviced_tem[j++] = serial_data.data.at(i);
			if (tem_last == 0xEF && tem_curr == 0xFE)
			{
				header_count++;
				rec_flag = 0;
			}
		}
		else
			rec_flag = 0;
	}
	if(header_count == 0)
	{
		ROS_WARN_STREAM("can not found frame head");
		return 0;
	}
	// 检验数据长度和校验码是否正确
	//   if(reviced_tem[len-3] ==check || reviced_tem[len-3]==0xff)
	//     ;
	//   else
	//     return;
	// 检验接受数据的长度
	step = 0;
	for (int k = 0; k < header_count; k++)
	{
		len = (reviced_tem[2 + step] + 4); //第一个帧头的长度
										   // cout<<"read head :" <<i<< "      len:   "<<len;
		if (reviced_tem[0 + step] == 0xAE && reviced_tem[1 + step] == 0xEA && reviced_tem[len - 2 + step] == 0xEF && reviced_tem[len - 1 + step] == 0xFE)
		{ //检查帧头帧尾是否完整
			if (reviced_tem[3 + step] == 0x03)
			{
				i = 4 + step;
				rc_remote_upload_counter.byte_data[3] = reviced_tem[i++];
				rc_remote_upload_counter.byte_data[2] = reviced_tem[i++];
				rc_remote_upload_counter.byte_data[1] = reviced_tem[i++];
				rc_remote_upload_counter.byte_data[0] = reviced_tem[i++];

				rc.cnt = rc_remote_upload_counter.uint32_data;
				rc.ch1 = reviced_tem[i++];
				rc.ch1 = (rc.ch1 << 8) + reviced_tem[i++];
				rc.ch2 = reviced_tem[i++];
				rc.ch2 = (rc.ch2 << 8) + reviced_tem[i++];
				rc.ch3 = reviced_tem[i++];
				rc.ch3 = (rc.ch3 << 8) + reviced_tem[i++];
				rc.ch4 = reviced_tem[i++];
				rc.ch4 = (rc.ch4 << 8) + reviced_tem[i++];
				rc.sw1 = reviced_tem[i++];
				rc.sw2 = reviced_tem[i++];
				rc.update = 0x01;
				if (rc.ch1 >= (RC_MIN-200) && rc.ch1 <=(RC_MAX+200))
				{
					rc.available = 0x01;
				}
				else
				{
					ROS_WARN_STREAM("rc.chx < RC_MIN || rc.chx > RC_MAX");
				}
				if(show_message)
				{
					ROS_INFO_STREAM("RC_Remotes date  ch1: " << rc.ch1 << " ch2: " << rc.ch2
														 << " ch3: " << rc.ch3 << " ch4: " << rc.ch4 << " sw1: " << rc.sw1 << " sw2: " << rc.sw2);
				}
				return true;
			}
			else
			{
				ROS_WARN_STREAM("unrecognize frame");
			}
			// return  true;
		}
		else
		{
			ROS_WARN_STREAM("frame head is wrong");
			return false;
		}
		step += len;
	}
	return true;
}

bool publish_cmd_vel(void)
{
	if (rc.update == 0x01)
	{
		if (rc.state != 0x02) // 0 未标定   1 标定中   2标定完成
		{
			int flag = calculate_rc_offset();
			if (flag == 0)
			{
				ROS_WARN_STREAM("calculate_rc_offset failed .... ");
			}
			else if (flag == 1)
			{
				;//ROS_WARN_STREAM("initial .... ");
			}
		}
		else
		{
			if (rc.available == 1 && rc.sw1 != 1) //左侧开关拨到遥控器控制
			{
				geometry_msgs::Twist msg;
				float speed_x, speed_y, speed_w;
				float rc_k = 1;

				if (rc.sw2 == 1)
					rc_k = 1;
				else if (rc.sw2 == 3)
					rc_k = 2;
				else if (rc.sw2 == 2)
					rc_k = 3; // 3m/s
				else
					rc_k = 0;

				speed_x = (rc.ch1 - rc.ch1_offset) / (RC_MAX - rc.ch1_offset);
				if(abs(speed_x)<0.2) speed_x=0;
				speed_y =0;
				speed_w =(rc.ch2 - rc.ch2_offset) / (RC_MAX - rc.ch2_offset);
				if(abs(speed_w)<0.2) speed_w=0;

				speed_x = RC_K *rc_k*speed_x ;
				speed_y = RC_K *rc_k* speed_y;
				speed_w = RC_K * rc_k*speed_w;
				msg.linear.x = speed_x;
				msg.linear.y = speed_y;
				msg.angular.z = speed_w;
				cmdvel_pub.publish(msg);
				//ROS_INFO_STREAM("speed_x: " << speed_x << " speed_y: " << speed_y << " speed_w " << speed_w);
				return 1;
			}
		}
		rc.update = 0;
	}
	return 0;
}

bool calculate_rc_offset(void)
{
	if ((rc.ch1 > 900 && rc.ch1 < 1100) && (rc.ch2 > 900 && rc.ch2 < 1100)
			 && (rc.ch3 > 900 && rc.ch3 < 1100) && (rc.ch4 > 900 && rc.ch4 < 1100))
	{
		sum_offset[0] += rc.ch1;
		sum_offset[1] += rc.ch2;
		sum_offset[2] += rc.ch3;
		sum_offset[3] += rc.ch4;
		rc.state = 0x01; // 0 未标定   1 标定中   2标定完成
		init_times++;
	}
	if (init_times > 20)
	{
		rc.ch1_offset = sum_offset[0] / init_times;
		rc.ch2_offset = sum_offset[1] / init_times;
		rc.ch3_offset = sum_offset[2] / init_times;
		rc.ch4_offset = sum_offset[3] / init_times;
		//ROS_INFO_STREAM("ch1_offset: " <<rc. ch1_offset << " ch2_offset: " << rc.ch2_offset
		//		 << "ch3_offset: " <<rc. ch3_offset << " ch4_offset: " << rc.ch4_offset);
		if (rc.ch1_offset == 0 || rc.ch2_offset == 0 || rc.ch3_offset == 0 || rc.ch4_offset == 0)
		{
			rc.available = 0;
			rc.state = 0x00;
			init_times = 0;
			sum_offset[0] = 0;
			sum_offset[1] = 0;
			sum_offset[2] = 0;
			sum_offset[3] = 0;
			return 0;
		}
		else
		{
			rc.available = 1;
			rc.state = 0x02;
			return 2; //标定成功
		}
	}
	return 1; //标定中
}
/**
 * @function 通过串口与舵机建立通讯 波特率1M
 *  读取舵机状态 控制舵机转动角度 （sts3215）
 * 
 * 备注： 
 * crp
 * 2024-3-19
 ****************************************************/

#include <deque>
#include <ros/ros.h>
#include <chrono>
#include <locale>
#include <tuple>
#include <algorithm>
#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <boost/assert.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

extern "C" {
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <time.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <unistd.h> //  close
#include <string.h> //  strerror
}

#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <cmath>
  
using namespace std;
  

boost::asio::serial_port* serial_port = 0;
serial::Serial ros_ser;
ros::Publisher motor_position_pub;

typedef struct{
 		uint16_t  position,speed,duty;
		uint8_t voltage,temperature;
}motor_measure_t;
motor_measure_t motor_sts3215;

 // 同步写指令 控制舵机转动角度, ID 舵机编号 1-254, angle 表示舵机转动角度 0-360度
void send_angle_to_steeringmotor(uint8_t ID,float angle);
// 查询舵机 位置 速度 负载 电压 温度
void read_angle_to_steeringmotor(uint8_t ID);
bool  analy_uart_recive_data( std_msgs::String& serial_data);

int main(int argc,char** argv)
{
	string out_result;
	string sub_cmdvel_topic,pub_odom_topic,dev;

	int buad,time_out,hz;
	ros::init(argc, argv, "sts3215_driver");
	ros::NodeHandle n("~");

	n.param<std::string>("dev", dev, "/dev/ttyUSB0");
	n.param<int>("buad", buad, 1000000);
	n.param<int>("time_out", time_out, 1000);
	n.param<int>("hz", hz, 5);

	ROS_INFO_STREAM("dev:   "<<dev);
	ROS_INFO_STREAM("buad:   "<<buad);
	ROS_INFO_STREAM("time_out:   "<<time_out);
	ROS_INFO_STREAM("hz:   "<<hz);
	  
	ros::Rate loop_rate(hz);
	motor_position_pub =  n.advertise<std_msgs::Float32>("motor/position",20, true);
	// 开启串口模块
	 try
	 {
	    ros_ser.setPort(dev);
	    ros_ser.setBaudrate(buad);
	    //ros_serial::Timeout to = serial::Timeout::simpleTimeout(1000);
	    serial::Timeout to = serial::Timeout::simpleTimeout(time_out);
	    ros_ser.setTimeout(to);
	    ros_ser.open();
	    ros_ser.flushInput(); //清空缓冲区数据
	 }
	 catch (serial::IOException& e)
	 {
		ROS_ERROR_STREAM("Unable to open port ");
		return -1;
	}
	if(ros_ser.isOpen())
	{
		ros_ser.flushInput(); //清空缓冲区数据
		ROS_INFO_STREAM("Serial Port opened");
	}
	else
	{
	    return -1;
	}
	float ang=0;
	int dir = 1;
    while(ros::ok())
    { 
		send_angle_to_steeringmotor(1,ang);
		ang +=dir;
		if(ang>=350)
		{
			dir = -3;
		}
		if(ang<=5)
		{
			dir = 3;
		}

		read_angle_to_steeringmotor(1);
		if(ros_ser.available())
		{
			std_msgs::String serial_data;
			serial_data.data = ros_ser.read(ros_ser.available());//获取串口数据
			int data_length=serial_data.data.size();
			if(data_length<1 || data_length>500)
			{
				ros_ser.flushInput(); //清空缓冲区数据	
				ROS_INFO_STREAM("serial data is too short ,  len: " << serial_data.data.size() );
			}
			else
			{
				if(!analy_uart_recive_data(serial_data))		
				{
					ROS_INFO_STREAM("error: analy uart recived  data  " );
				}
			}

		}
 
        ros::spinOnce();
        loop_rate.sleep();
			
    }
   
    std::cout<<" EXIT ..."<<std::endl;
    ros::waitForShutdown();
    ros::shutdown();
    return 1;

}




// 同步写指令 控制舵机转动角度
// ID 舵机编号 1-254
// angle 表示舵机转动角度 0-360度
void send_angle_to_steeringmotor(uint8_t ID,float angle)
{
  uint8_t data_tem[50];
  uint8_t  counter=0,check=0,len  = 0x00;
  uint16_t motor_position =( angle/360.0f*4095+0.5)/1;
  uint16_t motor_time =0x0000;
  uint16_t motor_speed =1000;
  data_tem[counter++] =0xFF;
  data_tem[counter++] =0xFF;
  data_tem[counter++] =0xfe;
  data_tem[counter++] =len;
  data_tem[counter++] =0x83; // Instruction
  data_tem[counter++] =0x2A; // 首地址是2A  
  data_tem[counter++] =0x06; // 每个舵机数据长度
  data_tem[counter++] = ID; //  舵机ID
  data_tem[counter++] =(motor_position)%256;  //低位在前
  data_tem[counter++] =(motor_position)/256; 
  data_tem[counter++] =(motor_time)%256; 
  data_tem[counter++] =(motor_time)/256; 
  data_tem[counter++] =(motor_speed)%256; 
  data_tem[counter++] =(motor_speed)/256; 
  data_tem[3] =counter-3;
  for(int i=2;i<counter;i++)
  {
    check+=data_tem[i];
  }
  data_tem[counter++] =~check;
//    for(int i=0;i<counter;i++)
//   {
//       printf(" 0x%x",data_tem[i]);
//   }
//   printf("\n");
  ros_ser.write(data_tem,counter);
}
// 查询舵机 位置 速度 负载 电压 温度
void read_angle_to_steeringmotor(uint8_t ID)
{
  uint8_t data_tem[50];
  uint8_t  counter=0,check=0,len  = 0x00;
  data_tem[counter++] =0xFF;
  data_tem[counter++] =0xFF;
  data_tem[counter++] =0xfe;
  data_tem[counter++] =len;
  data_tem[counter++] =0x82; // Instruction
  data_tem[counter++] =0x38; // 首地址是0x38 
  data_tem[counter++] =0x08; // 每个舵机数据长度
  data_tem[counter++] = ID; //  舵机ID
  data_tem[3] =counter-3;
  for(int i=2;i<counter;i++)
  {
    check+=data_tem[i];
  }
  data_tem[counter++] =~check;
//    for(int i=0;i<counter;i++)
//   {
//       printf(" 0x%x",data_tem[i]);
//   }
//   printf("\n");
  ros_ser.write(data_tem,counter);
}

// 返回数据包 FF FF 01 0A 00 00 08 00 00 00 00 79 1E 55
// 帧头(FF FF ) + ID(01) +长度(0A) + 工作状态(00) + 参数 ()
// 工作状态    为 0,则舵机无报错

/**
 * @function 解析串口发送过来的数据帧
 * 成功则返回true　否则返回false
 */
bool  analy_uart_recive_data( std_msgs::String& serial_data)
{
  unsigned char reviced_tem[500];
  uint16_t len=0,i=0,j=0;
  unsigned char tem_last=0,tem_curr=0,rec_flag=0;//定义接收标志位
  uint16_t header_count=0,step=0; //计数这个数据序列中有多少个帧头
  len=serial_data.data.size();
  if(len<1 || len>500)
  {
	ROS_INFO_STREAM("serial data is too short ,  len: " << serial_data.data.size() );
	std_msgs::String serial_data;
	string str_tem;
	serial_data.data = ros_ser.read(ros_ser.available());
	str_tem =  serial_data.data;
     return false; //数据长度太短　
   }
   //ROS_INFO_STREAM("Read: " << serial_data.data.size() );
   // 有可能帧头不在第一个数组位置
  for( i=0;i<len;i++) 
  {
	 tem_last=  tem_curr;
	 tem_curr = serial_data.data.at(i);
	 if(tem_last == 0xFF && tem_curr==0xFF&&rec_flag==0) //在接受的数据串中找到帧头　
	 {
		 rec_flag=1;
		 header_count++;
		 reviced_tem[j++]=tem_last;
		 reviced_tem[j++]=tem_curr;
		 //ROS_INFO_STREAM("found frame head" ); 
	}
	else if (rec_flag==1)
	{
		reviced_tem[j++]=serial_data.data.at(i);
	}
	else
		rec_flag=0;
  }
  // 检验接受数据的长度
  step=0;
  for(int k=0;k<header_count;k++) 
  {
	len = (reviced_tem[2+step] +4 ) ; //第一个帧头的长度
	//cout<<"read head :" <<i<< "      len:   "<<len;
	if(reviced_tem[0+step] ==0xFF && reviced_tem[1+step] == 0xFF ) 
	{//检查帧头帧尾是否完整
		if (reviced_tem[2+step] == 0x01 )
		{
			//ROS_INFO_STREAM("recived motor  ID : 01 data" ); 
			motor_sts3215.position =( reviced_tem[5+step]+reviced_tem[6+step]*256);
			motor_sts3215.speed =( reviced_tem[7+step]+reviced_tem[8+step]*256); // 单位时间（每秒）内运动的步数         ,单位  步/s
			motor_sts3215.duty =( reviced_tem[9+step]+reviced_tem[10+step]*256); // 当前控制输出驱动电机的电压占空比  ,单位 0.1%
			motor_sts3215.voltage =reviced_tem[11+step]; // 单位 0.1V 
			motor_sts3215.temperature =reviced_tem[12+step];// 单位 °C 
			// printf("motor  ID : 01 position : %d \n" ,  motor_sts3215.position);
			// printf("speed (step/s): %d\n" ,  motor_sts3215.speed);
			// printf("duty(0.1%) : %d\n" ,  motor_sts3215.duty);
			// printf("voltage(0.1V) : %d\n" ,  motor_sts3215.voltage);
			// printf("temperature( °C) : %d\n" ,  motor_sts3215.temperature);

			std_msgs::Float32 tem_position;
			tem_position.data = motor_sts3215.position/4096.0f*360.f;
			motor_position_pub.publish(tem_position);
		}
		else
		{
			ROS_WARN_STREAM("unrecognize frame" ); 
		}
	}
	else
	{
	ROS_WARN_STREAM("frame head is wrong" ); 
		return  false;	
	}
	step+=len; 
  }
 return  true;	         
}

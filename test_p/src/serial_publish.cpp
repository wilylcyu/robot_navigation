//订阅/car_vel主题,并进行串口通讯
#include "ros/ros.h"  //ros需要的头文件
#include "test_p/car_msg.h"   //调用消息所用到的头文件
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <string>        //以下为串口通讯需要的头文件
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include "serial/serial.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

unsigned char temp0=0x01 ;   //右轮正符号
unsigned char temp1=0x02 ;   //右轮负符号
unsigned char temp2=0x03 ;   //左轮正符号
unsigned char temp3=0x04;    //左轮负符号
unsigned char temp4=0x05;    //左右轮速度均为0时
unsigned char temp5=0x06;
unsigned char data_terminal0=0x0d;
unsigned char data_terminal1=0x0a;

unsigned char publish_data[10]={0};

string rec_buffer;

//发送给下位机的左右轮速度，里程计的坐标和方向
union floatData
{
     float d;
     unsigned char data[4];
 }rightdata,leftdata,position_x,position_y,oriention;

int count = 0;

 float odom_vx = 0;      //里程计要发布的速度信息
 float odom_vy = 0;
 float odom_vth = 0;

float lastPosition_x= 0 ;
float lastPosition_y= 0 ;
float lastOriention= 0 ;

void enumerate_ports()
{
	vector<serial::PortInfo> devices_found = serial::list_ports();

	vector<serial::PortInfo>::iterator iter = devices_found.begin();

	while( iter != devices_found.end() )
	{
		serial::PortInfo device = *iter++;
	}
}

void carCallback(const test_p::car_msg& msg)
{
	string port("/dev/ttyUSB1");
	unsigned long baud = 115200;
	serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

	rightdata.d=msg.rightspeed;
	leftdata.d=msg.leftspeed;

	for(int i=0;i<4;i++)
	{
		publish_data[i]=rightdata.data[i];
		publish_data[i+4]=leftdata.data[i];
	}
	publish_data[8]=data_terminal0;
	publish_data[9]=data_terminal1;

	my_serial.write(publish_data,10);
}

int main(int argc, char **argv)
{
	string port("/dev/ttyUSB1");
	unsigned long baud = 115200;
	serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

	ros::init(argc, argv, "serial_publish");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("car_vel", 20, carCallback);
	ros::Publisher  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 20);

	static tf::TransformBroadcaster odom_broadcaster;

  	ros::Time current_time, last_time;
  	current_time = ros::Time::now();
  	last_time = ros::Time::now();

	while(ros::ok())
	{
		rec_buffer =my_serial.readline(15,"\n");
		const char *receive_data=rec_buffer.data();
		if(rec_buffer.length()==13)
		{
			for(int i=0;i<4;i++)
			{
				position_x.data[i]=receive_data[i];
				position_y.data[i]=receive_data[i+4];
				oriention.data[i]=receive_data[i+8];
			}
			current_time = ros::Time::now();
			position_x.d/=1000;
			position_y.d/=1000;
			oriention.d/=1000;
			float dt = (current_time - last_time).toSec();
			float delta_x=position_x.d-lastPosition_x;
			float delta_y=position_y.d-lastPosition_y;
			float delta_th=oriention.d-lastOriention;

			odom_vx=(delta_x*cos(oriention.d)+delta_y*sin(oriention.d))/dt;
			odom_vy=(delta_y*cos(oriention.d)-delta_x*sin(oriention.d))/dt;
			odom_vth=delta_th/dt;
			//cout<<odom_vx<<"	"<<odom_vy<<"	"<<odom_vth<<endl;

			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(oriention.d);

			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = current_time;
			odom_trans.header.frame_id = "odom";			//发布坐标变换的父子坐标系
			odom_trans.child_frame_id = "base_link";

			odom_trans.transform.translation.x = position_x.d;
			odom_trans.transform.translation.y = position_y.d;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			odom_broadcaster.sendTransform(odom_trans);
			nav_msgs::Odometry odom;

			odom.header.stamp = current_time;			
			odom.header.frame_id = "odom";
			odom.child_frame_id = "base_link";

			odom.pose.pose.position.x = position_x.d;        //里程计位置数据
			odom.pose.pose.position.y = position_y.d;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;

   			odom.twist.twist.linear.x = odom_vx;
    			odom.twist.twist.linear.y = odom_vy;
    			odom.twist.twist.angular.z = odom_vth;

			odom_pub.publish(odom);

			last_time = current_time;
			lastPosition_x=position_x.d;
			lastPosition_y=position_y.d;
			lastOriention=oriention.d;
		}
		else
		{
			if(receive_data[0]==0x01)
			{
				cout<<"command right ！"<<endl;
			}
			if(receive_data[0]==0x00)
			{
				my_serial.write(publish_data,10);
				cout<<"command send again !"<<endl;
			}

		}
		//cout<<rec_buffer.length()<<endl;
		ros::spinOnce();  //callback函数必须处理所有问题时，才可以用到
	}
	return 0;
}

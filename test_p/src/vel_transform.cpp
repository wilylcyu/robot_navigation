#include <ros/ros.h>  
#include <test_p/car_msg.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

char ratio = 10 ;   //转速转换比例，执行速度调整比例
float D = 272.5 ;    //两轮间距，单位是mm
char const_num=2;

class vel_transform
{  
public:  
  vel_transform()
  {  
    //Topic you want to publish  
    pub_ = n_.advertise<test_p::car_msg>("car_vel", 20); //缓存大一些，防止处理速度不够导致数据丢失
  
    //Topic you want to subscribe  
    sub_ = n_.subscribe("cmd_vel", 20, &vel_transform::callback, this);
  }  
  
  void callback(const geometry_msgs::Twist & cmd_input)
  {
    test_p::car_msg output;
    if ( ( (int)cmd_input.linear.x) != 0 )     //转换前进速度到两轮速度,单位是mm/s
    {
    	output.leftspeed = cmd_input.linear.x *ratio ;
    	output.rightspeed = cmd_input.linear.x * ratio ;
    }
    else if(((int)cmd_input.angular.z)!=0)      //转换转动速度到两轮速度，单位是mm/s
    {
    	output.leftspeed= - cmd_input.angular.z * D / (ratio*const_num) ;
    	output.rightspeed= cmd_input.angular.z * D / (ratio*const_num)  ;
    }
    else                                                             //停止时两轮速度均为0
    {
    	output.leftspeed = 0 ;
    	output.rightspeed = 0;
    }

    //写入/cmd_vel发出的消息转换成左右轮速度的过程
    pub_.publish(output);
  }
  
private:

  ros::NodeHandle n_;   
  ros::Publisher pub_;  
  ros::Subscriber sub_;  
  
};//End of class vel_transform
  
int main(int argc, char **argv)  
{

  //Initiate ROS  
  ros::init(argc, argv, "vel_transfrom");  
  
  //Create an object of class SubscribeAndPublish that will take care of everything  
  vel_transform go_on;  //main 函数中
  
  ros::spin();  
  
  return 0;  
}
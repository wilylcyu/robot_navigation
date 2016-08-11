#include <ros/ros.h>  
#include <test_p/car_msg.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <test_p/odometry_data.h>
#include <nav_msgs/Odometry.h>

class odometry_pub
{  
public:  
  odometry_pub()
  {  
    odom_pub = n_.advertise<nav_msgs::Odometry>("odom", 20); 
    sub_ = n_.subscribe("odom_data", 20, &odometry_pub::callback, this);
  }  
  
  void callback(const test_p::odometry_data& odometry_input)
  {
     static tf::TransformBroadcaster odom_broadcaster;
      geometry_msgs::TransformStamped odom_trans;
      nav_msgs::Odometry odom; 

      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odometry_input.th);

      float covariance[36] =  {0.01, 0, 0, 0, 0, 0,  // covariance on gps_x
            0, 0.01, 0, 0, 0, 0,  // covariance on gps_y
            0, 0, 99999, 0, 0, 0,  // covariance on gps_z
            0, 0, 0, 99999, 0, 0,  // large covariance on rot x
            0, 0, 0, 0, 99999, 0,  // large covariance on rot y
            0, 0, 0, 0, 0, 0.01};  // large covariance on rot z 
      for(int i = 0; i < 36; i++)
      {
        odom.pose.covariance[i] = covariance[i];
        odom.twist.covariance[i] = covariance[i];
      } 

      odom_trans.header.stamp = ros::Time::now();
      odom_trans.header.frame_id = "odom";      //发布坐标变换的父子坐标系
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x =odometry_input.x;// position_x.d;
      odom_trans.transform.translation.y = odometry_input.y;//;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      odom_broadcaster.sendTransform(odom_trans);
      
      odom.header.stamp = ros::Time::now();     
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";

      odom.pose.pose.position.x = odometry_input.x;//;        //里程计位置数据
      odom.pose.pose.position.y = odometry_input.y;//;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      odom.twist.twist.linear.x = odometry_input.vel_x;
      //odom.twist.twist.linear.y = odom_vy;
      odom.twist.twist.angular.z = odometry_input.vel_th;

      odom_pub.publish(odom);
  
  }
  
private:

  ros::NodeHandle n_;   
  ros::Publisher odom_pub;  
  ros::Subscriber sub_;  
  
};

  
int main(int argc, char **argv)  
{
      ros::init(argc, argv, "odometry_node");  

      odometry_pub go_on;
      ros::spin();  
  
      return 0;  
}

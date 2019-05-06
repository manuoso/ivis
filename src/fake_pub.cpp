
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"

#include <sstream>


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "fake_pub");

  ros::NodeHandle n;


  ros::Publisher point_pub = n.advertise<geometry_msgs::Point>("pointcloud_pub", 1000);
  ros::Publisher _pub = n.advertise<geometry_msgs::PoseStamped>("uavpose_pub", 1000);
  ros::Rate loop_rate(1);


  int count = 0;
  float count2= 0.5;
  while (ros::ok())
  {

    geometry_msgs::Point msg;
    geometry_msgs::PoseStamped _msg;

    msg.x=count;
    msg.y=0;
    msg.z=0;

    _msg.pose.position.x=0;
    _msg.pose.position.y=count2;
    _msg.pose.position.z=0;
    _msg.pose.orientation.x=0.1;
    _msg.pose.orientation.y=0.001;
    _msg.pose.orientation.z=0.001;
    _msg.pose.orientation.w=0;


    point_pub.publish(msg);
    _pub.publish(_msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
    count2=count2+0.5;
  }


  return 0;
}
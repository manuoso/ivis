#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZ PointT1;
typedef pcl::PointCloud<PointT1> PointCloudT1;

int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/luis/ws_ivis/src/ivis/tools/read_pointcloud/Bridge_Algodonales.pcd", *cloud) == -1) 
  {
    PCL_ERROR("Couldn't read file .pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from .pcd with the following fields: "
            << std::endl;

  /*for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z << std::endl;*/

  ros::init(argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloudT1>("/bridge_pub", 1);

  PointCloudT1::Ptr msg(new PointCloudT1);
  msg->header.frame_id = "some_tf_frame";
  msg->height = cloud->height;
  msg->width = cloud->width;
  for (size_t i = 0; i < cloud->points.size (); ++i){
  //msg->points.push_back(pcl::PointXYZ(0,0,0));
  msg->points.push_back(pcl::PointXYZ(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
  }
  

  ros::Rate loop_rate(1);
  while (nh.ok())
  {
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }


return (0);
}

//---------------------------------------------------------------------------------------------------------------------
//  IVIS
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 Manuel Pérez Jiménez (a.k.a. manuoso) manuperezj@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#ifndef PCLVIEWER_GUI_H
#define PCLVIEWER_GUI_H

#include <QMainWindow>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <mutex>
#include <thread>
#include <chrono>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <vtkRenderWindow.h>
#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>


typedef pcl::PointXYZ PointT1;
typedef pcl::PointCloud<PointT1> PointCloudT1;

typedef pcl::PointXYZRGB PointT2;
typedef pcl::PointCloud<PointT2> PointCloudT2;


namespace Ui{
    class PCLViewer_gui;
}

class PCLViewer_gui : public QMainWindow {
    Q_OBJECT

public:
    /// Constructor
    explicit PCLViewer_gui(QWidget *parent = 0);

    /// Destructor
    ~PCLViewer_gui ();

signals:
    /// Signal that warns that there is a change in PCL GUI
    void updateGUIChanged();

    /// Signal that warns that there is a change in qvtk widget and update it
    void qvtkChanged();

private:
    /// Method for visualize a change of pose from a topic of ROS
    /// \param _msg: data receive to update pose
    void CallbackPose(const geometry_msgs::PoseStamped::ConstPtr& _msg);

    /// Method that receive new points for the point cloud
    void CallbackPoint(const geometry_msgs::Point::ConstPtr& _msg);

    /// Method that receive a pointcloud from a bridge
    void CallbackPointcloud(const PointCloudT2::ConstPtr& _msg);

    /// Method that update PCL GUI visualizer
    void updateGUI();   

    /// Method that update QVTK widget in PCL GUI visualizer
    void updateQVTK();   

private:
    Ui::PCLViewer_gui *ui;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    PointCloudT1::Ptr cloudT1_, cloudT1Filtered_;
    PointCloudT2::Ptr cloudT2_, cloudT2Filtered_;
    pcl::PolygonMesh untransformedUav_;

    std::chrono::time_point<std::chrono::high_resolution_clock> lastTimePose_;
    
    ros::Subscriber poseSub_, pointcloudSub_, pointSub_;

    std::string typePoint_ = "";
    std::string nameCallbackPose_ = "";
    std::string nameCallbackPointcloud_ = "";
    std::string nameCallbackPoint_ = "";
    std::string typeModelPose_ = "";
    std::string pathModelPose_ = "";

    PointT2 point_, uavPoint_, line_vector_[3];
    double radSphere_ = 0.1;
    std::string idUavSphere_ = "", idSphere_ = "", idPointcloud_ = "", uavIdLine_ = "";
    int cont_ = 0, contPointcloud_ = 0;
    bool firstTime_ = true;
 
    float poseX_ = 0.0, poseY_ = 0.0, poseZ_ = 0.0, poseOX_ = 0.0, poseOY_ = 0.0, poseOZ_ = 0.0, poseOW_ = 1.0;

    std::thread *updateThread_;
    std::mutex objectLockPose_, objectLockPointcloud_, objectLockPoint_;
    bool stopAll_ = false;

};

#endif // PCLVIEWER_GUI_H
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

#include <ivis/pclviewer_gui.h>
#include <ivis/ui_pclviewer_gui.h>

#include <pcl/features/normal_3d.h>
#include <random>

//---------------------------------------------------------------------------------------------------------------------
// PUBLIC 
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
PCLViewer_gui::PCLViewer_gui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PCLViewer_gui)
    {

    ui->setupUi(this);
    this->setWindowTitle("PCL viewer");

    connect(this, &PCLViewer_gui::poseUAVchanged , this, &PCLViewer_gui::updateObjectUAV);
    connect(this, &PCLViewer_gui::qvtkChanged , this, &PCLViewer_gui::updateQVTK);

    // Config
    nameCallbackPose_ = "";
    pathModelPose_ = "";
    typeModelPose_ = "OBJ";
    typePoint_ = "PointXYZ";

    // Set up the QVTK window
    viewer_.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    viewer_->setBackgroundColor(0.6, 0.6, 0.6);
    ui->qvtkWidget->SetRenderWindow(viewer_->getRenderWindow());
    viewer_->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    emit qvtkChanged();

    viewer_->resetCamera();

    if(pathModelPose_ != ""){
        if(typeModelPose_ == "OBJ"){
            pcl::io::loadPolygonFileOBJ(pathModelPose_, untransformedUav_);
        }else if(typeModelPose_ == "STL" ){
            pcl::io::loadPolygonFileSTL(pathModelPose_, untransformedUav_);
        }else{
            std::cout << "Type of model pose unrecognised!" << std::endl;
        }

        if(typePoint_ == "PointXYZ"){
            PointCloudT1::Ptr cloudUAV;
            cloudUAV.reset (new PointCloudT1);
            pcl::fromPCLPointCloud2(untransformedUav_.cloud, *cloudUAV);
            viewer_->addPointCloud(cloudUAV, "uav_pose");
        }else if(typePoint_ == "PointXYZRGB"){
            PointCloudT2::Ptr cloudUAV;
            cloudUAV.reset (new PointCloudT2);
            pcl::fromPCLPointCloud2(untransformedUav_.cloud, *cloudUAV);
            for(size_t i = 0; i < cloudUAV->points.size(); i++){
                cloudUAV->points[i].r = 255;
                cloudUAV->points[i].g = 255;
                cloudUAV->points[i].b = 255;
            }
            viewer_->addPointCloud(cloudUAV, "uav_pose");
        }
        emit qvtkChanged();
    }

    ros::NodeHandle nh;
    poseSub_ = nh.subscribe(nameCallbackPose_, 1, &PCLViewer_gui::CallbackPose, this);

    lastTimePose_ = std::chrono::high_resolution_clock::now();

    }

//---------------------------------------------------------------------------------------------------------------------
PCLViewer_gui::~PCLViewer_gui()
{
    delete ui;
}

//---------------------------------------------------------------------------------------------------------------------
// PRIVATE
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::CallbackPose(const geometry_msgs::PoseStamped::ConstPtr& _msg){
    
    objectLock_.lock();
    poseX_ = _msg->pose.position.x;
    poseY_ = _msg->pose.position.y;
    poseZ_ = _msg->pose.position.z;
    poseOX_ = _msg->pose.orientation.x;
    poseOY_ = _msg->pose.orientation.y;
    poseOZ_ = _msg->pose.orientation.z;
    poseOW_ = _msg->pose.orientation.w;
    objectLock_.unlock();

    if(pathModelPose_ != ""){
        auto t1 = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::milliseconds>(t1 - lastTimePose_).count() > 100){
            lastTimePose_ = t1;
            emit poseUAVchanged(); 
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::updateObjectUAV(){
    
    objectLock_.lock();
    Eigen::Quaternionf q;
    q.x() = poseOX_;
    q.y() = poseOY_;
    q.z() = poseOZ_;
    q.w() = poseOW_;    
    Eigen::Matrix3f R = q.normalized().toRotationMatrix();

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3,3>(0,0) = R;
    pose(0,3) = poseX_;
    pose(1,3) = poseY_;
    pose(2,3) = poseZ_;
    objectLock_.unlock();

    Eigen::Affine3f transform(pose);
    viewer_->updatePointCloudPose("uav_pose", transform);

    ui->qvtkWidget->update();
}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::updateQVTK(){
    ui->qvtkWidget->update();
}

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

        connect(this, &PCLViewer_gui::updateGUIChanged , this, &PCLViewer_gui::updateGUI);
        connect(this, &PCLViewer_gui::qvtkChanged , this, &PCLViewer_gui::updateQVTK);

        // Config
        nameCallbackPose_ = "/dji_telem/local_position";
        nameCallbackPointcloud_ = "/pointcloud_pub"; 
        nameCallbackPoint_ = "/point_pub";  

        pathModelPose_ = "/home/apollo/programming/catkin_ivis/src/ivis/uav_models/aquiles.stl";
        typeModelPose_ = "STL";
        typePoint_ = "PointXYZRGB";

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
                    cloudUAV->points[i].r = 0;
                    cloudUAV->points[i].g = 0;
                    cloudUAV->points[i].b = 255;
                }
                viewer_->addPointCloud(cloudUAV, "uav_pose");
            }
            emit qvtkChanged();
        }

        ros::NodeHandle nh;
        poseSub_ = nh.subscribe(nameCallbackPose_, 1, &PCLViewer_gui::CallbackPose, this);
        pointSub_ = nh.subscribe(nameCallbackPoint_, 1, &PCLViewer_gui::CallbackPoint, this);
        pointcloudSub_ = nh.subscribe<PointCloudT2>(nameCallbackPointcloud_, 1, &PCLViewer_gui::CallbackPointcloud, this);

        lastTimePose_ = std::chrono::high_resolution_clock::now();

        updateThread_ = new std::thread([&]{
                while(!stopAll_ && ros::ok()){
                    if(pathModelPose_ != ""){
                        auto t1 = std::chrono::high_resolution_clock::now();
                        if(std::chrono::duration_cast<std::chrono::milliseconds>(t1 - lastTimePose_).count() > 100){
                            lastTimePose_ = t1;
                            emit updateGUIChanged(); 
                        }
                    }
                }
            });

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
    
    objectLockPose_.lock();
    poseX_ = _msg->pose.position.x;
    poseY_ = _msg->pose.position.y;
    poseZ_ = _msg->pose.position.z;
    poseOX_ = _msg->pose.orientation.x;
    poseOY_ = _msg->pose.orientation.y;
    poseOZ_ = _msg->pose.orientation.z;
    poseOW_ = _msg->pose.orientation.w;
    objectLockPose_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::CallbackPoint(const geometry_msgs::Point::ConstPtr& _msg){

    objectLockPoint_.lock();
    point_.x = _msg->x;
    point_.y = _msg->y;
    point_.z = _msg->z;
    point_.r = 0;
    point_.g = 1;
    point_.b = 0;
    objectLockPoint_.unlock();

}


//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::CallbackPointcloud(const PointCloudT2::ConstPtr& _msg){

    idPointcloud_ = "point_cloud" + std::to_string(contPointcloud_);
    viewer_->addPointCloud(_msg, idPointcloud_);

    contPointcloud_++;
    emit qvtkChanged();

}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::updateGUI(){
    
    objectLockPose_.lock();
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

    uavPoint_.x = poseX_;
    uavPoint_.y = poseY_;
    uavPoint_.z = poseZ_;
    uavPoint_.r = 1;
    uavPoint_.g = 0;
    uavPoint_.b = 0;
    objectLockPose_.unlock();

    Eigen::Affine3f transform(pose);
    viewer_->updatePointCloudPose("uav_pose", transform);
    
    idUavSphere_ = "uav_pose_sphere" + std::to_string(cont_);
    viewer_->addSphere(uavPoint_, radSphere_, uavPoint_.r, uavPoint_.g, uavPoint_.b, idUavSphere_);

    if(firstTime_){
        firstTime_ = false;
        line_vector_[0] = uavPoint_;
    }else{
        line_vector_[1] = uavPoint_;
      
        uavIdLine_ = "uav_pose_line" + std::to_string(cont_);
        viewer_->addLine<pcl::PointXYZRGB>(line_vector_[0], line_vector_[1], uavIdLine_);

        line_vector_[0] = line_vector_[1];
    }

    objectLockPoint_.lock();
    PointT2 newPoint = point_;
    objectLockPoint_.unlock();
    
    idSphere_ = "sphere" + std::to_string(cont_);
    viewer_->addSphere(newPoint, radSphere_, newPoint.r, newPoint.g, newPoint.b, idSphere_);

    ui->lineEdit_lp1->setText(QString::number(uavPoint_.x));
    ui->lineEdit_lp2->setText(QString::number(uavPoint_.y));
    ui->lineEdit_lp3->setText(QString::number(uavPoint_.z));

    ui->lineEdit_tp1->setText(QString::number(newPoint.x));
    ui->lineEdit_tp2->setText(QString::number(newPoint.y));
    ui->lineEdit_tp3->setText(QString::number(newPoint.z));

    cont_++;
    emit qvtkChanged();
}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::updateQVTK(){
    ui->qvtkWidget->update();
}

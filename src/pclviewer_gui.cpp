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

#ifdef IVIS_USE_PCL    

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
        connect(ui->reset, SIGNAL(clicked()), this, SLOT(resetGUI()));

        std::ifstream rawFile("src/ivis/config/config_pcl.json");
        if (!rawFile.is_open()) {
            std::cout << "Error opening config file" << std::endl;
        }

        std::stringstream strStream;
        strStream << rawFile.rdbuf(); //read the file
        std::string json = strStream.str(); //str holds the content of the file

        if(configFile_.Parse(json.c_str()).HasParseError()){
            std::cout << "Error parsing json" << std::endl;
        }

        nameCallbackUAV_        = configFile_["callback_uav"].GetString();
        nameCallbackPoseVO_     = configFile_["callback_pose"].GetString();
        nameCallbackPointcloud_ = configFile_["callback_pointcloud"].GetString();

        pathModelPose_ = configFile_["path_model"].GetString();
        typeModelPose_ = configFile_["type_model"].GetString();
        typePoint_ = configFile_["type_point"].GetString();

        // Set up the QVTK window
        viewer_.reset(new pcl::visualization::PCLVisualizer("viewer", false));
        viewer_->setBackgroundColor(0.6, 0.6, 0.6);
        ui->qvtkWidget->SetRenderWindow(viewer_->getRenderWindow());
        viewer_->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
        emit qvtkChanged();
        
        viewer_->resetCamera();
        viewer_->registerPointPickingCallback(&PCLViewer_gui::pointPickingOccurred, *this);

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
        poseVOSub_       = nh.subscribe(nameCallbackPoseVO_, 1, &PCLViewer_gui::CallbackPoseVO, this);
        uavSub_          = nh.subscribe(nameCallbackUAV_, 1, &PCLViewer_gui::CallbackUAV, this);
        pointcloudSub_   = nh.subscribe(nameCallbackPointcloud_, 1, &PCLViewer_gui::CallbackPointcloud, this);

        vizClickedPoint_ = nh.advertise<geometry_msgs::PointStamped>("/ivis/ClickedPoint", 1);

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
// PRIVATE SLOTS
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::resetGUI(){

    viewer_.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    viewer_->setBackgroundColor(0.6, 0.6, 0.6);
    ui->qvtkWidget->SetRenderWindow(viewer_->getRenderWindow());
    viewer_->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    emit qvtkChanged();

    if(pathModelPose_ != ""){
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

    cont_ = 0; 
    contPointcloud_ = 0;

}

//---------------------------------------------------------------------------------------------------------------------
// PRIVATE
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
// Obtained from SLAM
void PCLViewer_gui::CallbackPoseVO(const geometry_msgs::PoseStamped::ConstPtr& _msg){

    objectLockPose_.lock();
    poseVO_.x = _msg->pose.position.x;
    poseVO_.y = _msg->pose.position.y;
    poseVO_.z = _msg->pose.position.z;
    poseVO_.r = 0;
    poseVO_.g = 1;
    poseVO_.b = 0;
    objectLockPose_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
// Obtained from DJI topics
void PCLViewer_gui::CallbackUAV(const geometry_msgs::PoseStamped::ConstPtr& _msg){
    
    objectLockUAV_.lock();
    poseX_ = _msg->pose.position.x;
    poseY_ = _msg->pose.position.y;
    poseZ_ = _msg->pose.position.z;
    poseOX_ = _msg->pose.orientation.x;
    poseOY_ = _msg->pose.orientation.y;
    poseOZ_ = _msg->pose.orientation.z;
    poseOW_ = _msg->pose.orientation.w;
    objectLockUAV_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::CallbackPointcloud(const sensor_msgs::PointCloud2::ConstPtr& _msg){

    idPointcloud_ = "point_cloud" + std::to_string(contPointcloud_);

    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    cloudMap_.reset (new PointCloudT2);

    pcl_conversions::toPCL(*_msg, *cloud);
    pcl::fromPCLPointCloud2(*cloud, *cloudMap_);

    for(size_t i = 0; i < cloudMap_->points.size(); i++){
        cloudMap_->points[i].r = 87; // pointcloud color
        cloudMap_->points[i].g = 35;
        cloudMap_->points[i].b = 100;
    }
    viewer_->addPointCloud(cloudMap_, idPointcloud_);

    contPointcloud_++;
    emit qvtkChanged();

}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::updateGUI(){
    
    objectLockUAV_.lock();
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
    objectLockUAV_.unlock();

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

    objectLockPose_.lock();
    PointT2 newPose = poseVO_; // visual odometry pose
    objectLockPose_.unlock();
    
    idPoseSphere_ = "sphere" + std::to_string(cont_);
    viewer_->addSphere(newPose, radSphere_, newPose.r, newPose.g, newPose.b, idPoseSphere_);

    ui->lineEdit_lp1->setText(QString::number(uavPoint_.x));
    ui->lineEdit_lp2->setText(QString::number(uavPoint_.y));
    ui->lineEdit_lp3->setText(QString::number(uavPoint_.z));

    ui->lineEdit_tp1->setText(QString::number(newPose.x));
    ui->lineEdit_tp2->setText(QString::number(newPose.y));
    ui->lineEdit_tp3->setText(QString::number(newPose.z));

    cont_++;
    emit qvtkChanged();
}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::updateQVTK(){
    ui->qvtkWidget->update();
}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::pointPickingOccurred(const pcl::visualization::PointPickingEvent &_event, void* _args){
    int idx = _event.getPointIndex();
    if(idx == -1){
        return;
    }

    float x, y, z;
    _event.getPoint(x, y, z);

    geometry_msgs::PointStamped clickedPoint;
    clickedPoint.header.stamp = ros::Time::now();
    clickedPoint.header.frame_id = "map";
    clickedPoint.point.x = x;
    clickedPoint.point.y = y;
    clickedPoint.point.z = z;
    vizClickedPoint_.publish(clickedPoint);

    // std::cout << "Position (" << x << ", " << y << ", " << z << ")" << std::endl;

    std::string sSphere = "goalPoint_" + std::to_string(contSpheres_);
    //std::cout << "sSphere: " << sSphere << std::endl;
    if(typePoint_ == "PointXYZ"){
        viewer_->addSphere(cloudMap_->points[idx], 1.0, 1, 0, 0, sSphere);
        ui->qvtkWidget->update();
    }else if(typePoint_ == "PointXYZRGB"){ //cloudMap_
        viewer_->addSphere(cloudMap_->points[idx], 1.0, 1, 0, 0, sSphere);
        ui->qvtkWidget->update();
    }
    contSpheres_++;
}

#endif
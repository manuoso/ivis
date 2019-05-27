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

#include <ivis/uav_control.h>
#include <ui_uav_control.h>

//---------------------------------------------------------------------------------------------------------------------
// PUBLIC 
//---------------------------------------------------------------------------------------------------------------------

UAV_control::UAV_control(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::UAV_control)
    {

        ui->setupUi(this);

        ui->stopPos->setVisible(0);
        ui->stopVel->setVisible(0);

        connect(ui->takeoff, SIGNAL(clicked()), this, SLOT(takeoffUAV()));
        connect(ui->land, SIGNAL(clicked()), this, SLOT(landUAV()));
        connect(ui->emergStop, SIGNAL(clicked()), this, SLOT(emergStopUAV()));
        connect(ui->startPos, SIGNAL(clicked()), this, SLOT(startPositionUAV()));
        connect(ui->startVel, SIGNAL(clicked()), this, SLOT(startVelocityUAV()));
        connect(ui->stopPos, SIGNAL(clicked()), this, SLOT(stopPositionUAV()));
        connect(ui->stopVel, SIGNAL(clicked()), this, SLOT(stopVelocityUAV()));

        connect(this, &UAV_control::telemChanged , this, &UAV_control::updateTelem);

        ros::NodeHandle nh;
        landReq_ = nh.serviceClient<std_srvs::SetBool>("/dji_control/land");
        takeoffReq_ = nh.serviceClient<std_srvs::SetBool>("/dji_control/takeoff");
        emergencyBrakeReq_ = nh.serviceClient<std_srvs::SetBool>("/dji_control/emergency_brake");

        velocityPub_ = nh.advertise<geometry_msgs::TwistStamped>("/dji_control/go_velocity", 1);
        positionPub_ = nh.advertise<geometry_msgs::PoseStamped>("/dji_control/go_position", 1); 

        poseGPSSub_ = nh.subscribe("/dji_telem/pos_gps", 1, &UAV_control::CallbackPoseGPS, this);
        poseLocalSub_ = nh.subscribe("/dji_telem/local_position", 1, &UAV_control::CallbackPoseLocal, this);
        poseVelSub_ = nh.subscribe("/dji_telem/velocity", 1, &UAV_control::CallbackVel, this);
        poseRCSub_ = nh.subscribe("/dji_telem/rc_commands", 1, &UAV_control::CallbackRC, this);
        nBatSub_ = nh.subscribe("/dji_telem/batery_state", 1, &UAV_control::CallbackBat, this);
        modeSub_ = nh.subscribe("/dji_telem/mode", 1, &UAV_control::CallbackMode, this);
        flyStatusSub_ = nh.subscribe("/dji_telem/fly_status", 1, &UAV_control::CallbackFS, this);
        djiConStaSub_ = nh.subscribe("/dji_control/status", 1, &UAV_control::CallbackDJICS, this);
        
        lastTimePose_ = std::chrono::high_resolution_clock::now();

        telemThread_ = new std::thread([&]{
            while(!stopAll_ && ros::ok()){
                auto t1 = std::chrono::high_resolution_clock::now();
                if(std::chrono::duration_cast<std::chrono::milliseconds>(t1 - lastTimePose_).count() > 50){
                    lastTimePose_ = t1;
                    emit telemChanged(); 
                }
            }
        });

    }

//---------------------------------------------------------------------------------------------------------------------
UAV_control::~UAV_control(){
    delete ui;
}

//---------------------------------------------------------------------------------------------------------------------
// PRIVATE SLOTS
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::takeoffUAV(){

    std_srvs::SetBool srv;
    srv.request.data = true;
    
    if(takeoffReq_.call(srv)){
        if(srv.response.success){
            std::cout << "Service of TAKE OFF success" << std::endl;
        }else{
            std::cout << "Service of TAKE OFF failed" << std::endl;
        }
    }else{
        std::cout << "Failed to call service of TAKE OFF" << std::endl;
    }

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::landUAV(){

    std_srvs::SetBool srv;
    srv.request.data = true;
    
    if(landReq_.call(srv)){
        if(srv.response.success){
            std::cout << "Service of LAND success" << std::endl;
        }else{
            std::cout << "Service of LAND failed" << std::endl;
        }
    }else{
        std::cout << "Failed to call service of LAND" << std::endl;
    }

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::emergStopUAV(){

    std_srvs::SetBool srv;
    srv.request.data = true;

    if(emergencyBrakeReq_.call(srv)){
        if(srv.response.success){
            std::cout << "Service of EMERGENCY BRAKE success" << std::endl;
        }else{
            std::cout << "Service of EMERGENCY BRAKE failed" << std::endl;
        }
    }else{
        std::cout << "Failed to call service of EMERGENCY BRAKE" << std::endl;
    }

}


//---------------------------------------------------------------------------------------------------------------------
void UAV_control::startPositionUAV(){

    QString qX = ui->lineEdit_p1->text();
    double x = qX.toDouble(); 

    QString qY = ui->lineEdit_p2->text();
    double y = qY.toDouble(); 

    QString qZ = ui->lineEdit_p3->text();
    double z = qZ.toDouble(); 

    msgPosition_.header.stamp = ros::Time::now();
    msgPosition_.pose.position.x = x;
    msgPosition_.pose.position.y = y;
    msgPosition_.pose.position.z = z;

    type_ = "position";
    sendThread_ = new std::thread(&UAV_control::sendThread, this);

    ui->startPos->setVisible(0);
    ui->stopPos->setVisible(1);

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::stopPositionUAV(){

    stopSend_ = true;
    sendThread_->join();

    std_srvs::SetBool srv;
    srv.request.data = true;

    if(emergencyBrakeReq_.call(srv)){
        if(srv.response.success){
            std::cout << "Service of EMERGENCY BRAKE success" << std::endl;
        }else{
            std::cout << "Service of EMERGENCY BRAKE failed" << std::endl;
        }
    }else{
        std::cout << "Failed to call service of EMERGENCY BRAKE" << std::endl;
    }

    ui->startPos->setVisible(1);
    ui->stopPos->setVisible(0);

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::startVelocityUAV(){

    QString qX = ui->lineEdit_v1->text();
    double x = qX.toDouble(); 

    QString qY = ui->lineEdit_v2->text();
    double y = qY.toDouble(); 

    QString qZ = ui->lineEdit_v3->text();
    double z = qZ.toDouble(); 

    msgVelocity_.header.stamp = ros::Time::now();
    msgVelocity_.twist.linear.x = x;
    msgVelocity_.twist.linear.y = y;
    msgVelocity_.twist.linear.z = z;

    type_ = "velocity";
    sendThread_ = new std::thread(&UAV_control::sendThread, this);

    ui->startVel->setVisible(0);
    ui->stopVel->setVisible(1);

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::stopVelocityUAV(){

    stopSend_ = true;
    sendThread_->join();

    std_srvs::SetBool srv;
    srv.request.data = true;

    if(emergencyBrakeReq_.call(srv)){
        if(srv.response.success){
            std::cout << "Service of EMERGENCY BRAKE success" << std::endl;
        }else{
            std::cout << "Service of EMERGENCY BRAKE failed" << std::endl;
        }
    }else{
        std::cout << "Failed to call service of EMERGENCY BRAKE" << std::endl;
    }

    ui->startVel->setVisible(1);
    ui->stopVel->setVisible(0);

}

//---------------------------------------------------------------------------------------------------------------------
// PRIVATE
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::sendThread(){

    ros::Rate rate(20);

    while(!stopSend_ && ros::ok()){
        if(type_ == "position"){
            positionPub_.publish(msgPosition_);
        }else if(type_ == "velocity"){
            velocityPub_.publish(msgVelocity_);
        }else{
            std::cout << "Send thread in unrecognized State" << std::endl;
        }
        
        rate.sleep();
    }

    stopSend_ = false;

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::updateTelem(){

    objectLockGPS_.lock();
    ui->lineEdit_rg1->setText(QString::number(poseGPSLat_));
    ui->lineEdit_rg2->setText(QString::number(poseGPSLon_));
    ui->lineEdit_rg3->setText(QString::number(poseGPSAlt_));
    ui->lineEdit_ngps->setText(QString::number(nGPS_));
    objectLockGPS_.unlock();

    objectLockLocal_.lock();
    ui->lineEdit_l1->setText(QString::number(position_.x));
    ui->lineEdit_l2->setText(QString::number(position_.y));
    ui->lineEdit_l3->setText(QString::number(position_.z));
    objectLockLocal_.unlock();

    objectLockVel_.lock();
    ui->lineEdit_rv1->setText(QString::number(velocity_.x));
    ui->lineEdit_rv2->setText(QString::number(velocity_.y));
    ui->lineEdit_rv3->setText(QString::number(velocity_.z));
    objectLockVel_.unlock();

    objectLockRC_.lock();
    ui->lineEdit_rc1->setText(QString::number(rc1_));
    ui->lineEdit_rc2->setText(QString::number(rc2_));
    ui->lineEdit_rc3->setText(QString::number(rc3_));
    ui->lineEdit_rc4->setText(QString::number(rc4_));
    objectLockRC_.unlock();

    objectLockBat_.lock();
    ui->lineEdit_bat->setText(QString::number(batLevel));
    objectLockBat_.unlock();

    objectLockMode_.lock();
    ui->lineEdit_mode->setText(QString::fromStdString(mode_));
    objectLockMode_.unlock();

    objectLockFS_.lock();
    ui->lineEdit_fs->setText(QString::fromStdString(flyStatus_));
    objectLockFS_.unlock();

    objectLockDJICS_.lock();
    ui->lineEdit_djiStatus->setText(QString::fromStdString(djiConSta_));
    objectLockDJICS_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::CallbackPoseGPS(const sensor_msgs::NavSatFix::ConstPtr& _msg){

    objectLockGPS_.lock();
    nGPS_ = _msg->status.status;
    poseGPSLat_ = _msg->latitude;
    poseGPSLon_ = _msg->longitude;
    poseGPSAlt_ = _msg->altitude;
    objectLockGPS_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::CallbackPoseLocal(const geometry_msgs::PoseStamped::ConstPtr& _msg){

    objectLockLocal_.lock();
    position_.x = _msg->pose.position.x;
    position_.y = _msg->pose.position.y;
    position_.z = _msg->pose.position.z;
    objectLockLocal_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::CallbackVel(const geometry_msgs::TwistStamped::ConstPtr& _msg){

    objectLockVel_.lock();
    velocity_.x = _msg->twist.linear.x;
    velocity_.y = _msg->twist.linear.y;
    velocity_.z = _msg->twist.linear.z;
    objectLockVel_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::CallbackRC(const std_msgs::Float64MultiArray::ConstPtr& _msg){

    objectLockRC_.lock();
    rc1_ = _msg->data[0];
    rc2_ = _msg->data[1];
    rc3_ = _msg->data[2];
    rc4_ = _msg->data[3];
    objectLockRC_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::CallbackBat(const sensor_msgs::BatteryState::ConstPtr& _msg){

    objectLockBat_.lock();
    batLevel = _msg->voltage/1000.0;
    objectLockBat_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::CallbackMode(const std_msgs::String::ConstPtr& _msg){

    objectLockMode_.lock();
    mode_ = _msg->data;
    objectLockMode_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::CallbackFS(const std_msgs::String::ConstPtr& _msg){

    objectLockFS_.lock();
    flyStatus_ = _msg->data;
    objectLockFS_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::CallbackDJICS(const std_msgs::String::ConstPtr& _msg){

    objectLockDJICS_.lock();
    djiConSta_ = _msg->data;
    objectLockDJICS_.unlock();

}
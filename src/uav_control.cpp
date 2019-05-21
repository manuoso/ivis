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

        connect(ui->takeoff, SIGNAL(clicked()), this, SLOT(takeoffUAV()));

        connect(this, &UAV_control::positionGPSChanged , this, &UAV_control::updatePoseGPS);

        ros::NodeHandle nh;
        landReq_
        takeoffReq_
        emergencyBrakeReq_

        velocityPub_
        positionPub_

        poseGPSSub_ = nh.subscribe("/dji_telem/pos_gps", 1, &UAV_control::CallbackPoseGPS, this);
        poseLocalSub_
        poseVelSub_
        poseRCSub_
        modeSub_
        flyStatusSub_
        nBatSub_
        djiConStaSub_
        
        poseGPSThread_ = new std::thread([&]{
            while(!stopAll_){
                auto t1 = std::chrono::high_resolution_clock::now();
                if(std::chrono::duration_cast<std::chrono::milliseconds>(t1 - lastTimePose_).count() > 50){
                    lastTimePose_ = t1;
                    emit positionGPSChanged(); 
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


}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::landUAV(){


}


//---------------------------------------------------------------------------------------------------------------------
void UAV_control::emergStopUAV(){


}


//---------------------------------------------------------------------------------------------------------------------
void UAV_control::positionUAV(){


}


//---------------------------------------------------------------------------------------------------------------------
void UAV_control::velocityUAV(){


}

//---------------------------------------------------------------------------------------------------------------------
// PRIVATE
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::updatePoseGPS(){

    objectLockGPS_.lock();
    ui->lineEdit_rg1->setText(QString::number(poseGPSLat_));
    ui->lineEdit_rg2->setText(QString::number(poseGPSLon_));
    ui->lineEdit_rg3->setText(QString::number(poseGPSAlt_));
    ui->lineEdit_ngps->setText(QString::number(nGPS_));
    objectLockGPS_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::updatePoseLocal(){

    objectLockLocal_.lock();
    ui->lineEdit_l1->setText(QString::number(poseLocalX_));
    ui->lineEdit_l2->setText(QString::number(poseLocalY_));
    ui->lineEdit_l3->setText(QString::number(poseLocalZ_));
    objectLockLocal_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::updateVel(){

    objectLockVel_.lock();
    ui->lineEdit_rv1->setText(QString::number(velX_));
    ui->lineEdit_rv2->setText(QString::number(velY_));
    ui->lineEdit_rv3->setText(QString::number(velZ_));
    objectLockVel_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::updateRC(){

    objectLockRC_.lock();
    ui->lineEdit_rc1->setText(QString::number(rc1_));
    ui->lineEdit_rc2->setText(QString::number(rc2_));
    ui->lineEdit_rc3->setText(QString::number(rc3_));
    ui->lineEdit_rc4->setText(QString::number(rc4_));
    objectLockRC_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::updateBat(){

    objectLockBat_.lock();
    ui->lineEdit_bat->setText(QString::number(batLevel));
    objectLockBat_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::updateMode(){

    objectLockMode_.lock();
    ui->lineEdit_mode->setText(QString::fromStdString(mode_));
    objectLockMode_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::updateFS(){

    objectLockFS_.lock();
    ui->lineEdit_fs->setText(QString::fromStdString(flyStatus_));
    objectLockFS_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::updateDJICS(){

    objectLockDJICS_.lock();
    ui->lineEdit_djiStatus->setText(QString::fromStdString(djiConSta_));
    objectLockDJICS_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::CallbackPoseGPS(const sensor_msgs::NavSatFix::ConstPtr& _msg){

    objectLockGPS_.lock();
    nGPS_ = _msg->status.status;
    latUAV_ = _msg->latitude;
    lonUAV_ = _msg->longitude;
    altUAV_ = _msg->altitude;
    objectLockGPS_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::CallbackPoseLocal(const geometry_msgs::PoseStamped::ConstPtr& _msg){

    objectLockLocal_.lock();
    poseLocalX_ = _msg->pose.position.x;
    poseLocalY_ = _msg->pose.position.y;
    poseLocalZ_ = _msg->pose.position.z;
    objectLockLocal_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::CallbackVel(const geometry_msgs::TwistStamped::ConstPtr& _msg){

    objectLockVel_.lock();
    velX_ = _msg->twist.linear.x;
    velY_ = _msg->twist.linear.y;
    velZ_ = _msg->twist.linear.z;
    objectLockVel_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::CallbackRC(const std_msgs::Float64MultiArray::ConstPtr& _msg){

    objectLockRC_.lock();
    rc1_ = _msg->data(0);
    rc2_ = _msg->data(1);
    rc3_ = _msg->data(2);
    rc4_ = _msg->data(3);
    objectLockRC_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_control::CallbackBat(const sensor_msgs::BatteryState::ConstPtr& _msg){

    objectLockBat_.lock();
    batLevel = _msg->data;
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
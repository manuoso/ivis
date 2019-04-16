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

#include <ivis/marble_vis.h>
#include <ui_marble_vis.h>

//---------------------------------------------------------------------------------------------------------------------
// PUBLIC 
//---------------------------------------------------------------------------------------------------------------------

MARBLE_vis::MARBLE_vis(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MARBLE_vis)
    {

        ui->setupUi(this);

        connect(this, &MARBLE_vis::positionChanged , this, &MARBLE_vis::updatePose);

        mapWidget_= new Marble::MarbleWidget();
        mapWidget_->setProjection(Marble::Mercator);
        mapWidget_->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
        mapWidget_->show();

        ui->horizontalLayout->addWidget(mapWidget_);

        ros::NodeHandle nh;
        std::string nameCallbackPose = "";
        poseSub_ = nh.subscribe(nameCallbackPose, 1, &MARBLE_vis::CallbackPose, this);

        lastTimePose_ = std::chrono::high_resolution_clock::now();

        poseThread_ = new std::thread([&]{
            while(!stopAll_){
                auto t1 = std::chrono::high_resolution_clock::now();
                if(std::chrono::duration_cast<std::chrono::milliseconds>(t1 - lastTimePose_).count() > 50){
                    lastTimePose_ = t1;
                    emit positionChanged(); 
                }
            }
        });


    }

//---------------------------------------------------------------------------------------------------------------------
MARBLE_vis::~MARBLE_vis(){
    delete ui;
}

//---------------------------------------------------------------------------------------------------------------------
// PRIVATE
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::updatePose(){
    objectLockPose_.lock();
    ui->lineEdit_p1->setText(QString::number(poseUAVx_));
    ui->lineEdit_p2->setText(QString::number(poseUAVy_));
    ui->lineEdit_p3->setText(QString::number(poseUAVz_));

    ui->lineEdit_p4->setText(QString::number(poseUAVox_));
    ui->lineEdit_p5->setText(QString::number(poseUAVoy_));
    ui->lineEdit_p6->setText(QString::number(poseUAVoz_));
    ui->lineEdit_p7->setText(QString::number(poseUAVow_));
    objectLockPose_.unlock();
}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::CallbackPose(const geometry_msgs::PoseStamped::ConstPtr& _msg){
    objectLockPose_.lock();
    poseUAVx_ = _msg->pose.position.x;
    poseUAVy_ = _msg->pose.position.y;
    poseUAVz_ = _msg->pose.position.z;

    poseUAVox_ = _msg->pose.orientation.x;
    poseUAVoy_ = _msg->pose.orientation.y;
    poseUAVoz_ = _msg->pose.orientation.z;
    poseUAVow_ = _msg->pose.orientation.w;
    objectLockPose_.unlock();
}

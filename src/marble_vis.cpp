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
        std::string nameCallbackPose = "/dji/gps_pos";
        poseSub_ = nh.subscribe(nameCallbackPose, 1, &MARBLE_vis::CallbackPose, this);

        place_ = new Marble::GeoDataPlacemark("Pose");
        place_->setCoordinate(lonGPS_, latGPS_, altGPS_, Marble::GeoDataCoordinates::Degree);

        document_ = new Marble::GeoDataDocument;
        document_->append(place_);

        // Add the document to MarbleWidget's tree model
        mapWidget_->model()->treeModel()->addDocument(document_);

	    // mapWidget_->zoomView(4000);

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
void MARBLE_vis::updatePose(){
    objectLockPose_.lock();
    ui->lineEdit_p1->setText(QString::number(latGPS_));
    ui->lineEdit_p2->setText(QString::number(lonGPS_));
    ui->lineEdit_p3->setText(QString::number(altGPS_));

    place_->setCoordinate(lonGPS_, latGPS_, altGPS_, Marble::GeoDataCoordinates::Degree);

    // Add the document to MarbleWidget's tree model
    mapWidget_->model()->treeModel()->updateFeature(place_);

    objectLockPose_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void MARBLE_vis::CallbackPose(const sensor_msgs::NavSatFix::ConstPtr& _msg){
    objectLockPose_.lock();
    latGPS_ = _msg->latitude;
    lonGPS_ = _msg->longitude;
    altGPS_ = _msg->altitude;

    objectLockPose_.unlock();
}

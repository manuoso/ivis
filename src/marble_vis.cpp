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

using namespace Marble;

//---------------------------------------------------------------------------------------------------------------------
// PUBLIC 
//---------------------------------------------------------------------------------------------------------------------

MARBLE_vis::MARBLE_vis(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MARBLE_vis)
    {

        ui->setupUi(this);

        connect(ui->center, SIGNAL(clicked()), this, SLOT(centerUAV()));
        connect(ui->addPoint, SIGNAL(clicked()), this, SLOT(addPointList()));
        connect(ui->deleteWP, SIGNAL(clicked()), this, SLOT(deleteWaypointList()));
        connect(ui->sendWP, SIGNAL(clicked()), this, SLOT(sendWaypointList()));
        connect(this, &MARBLE_vis::positionChanged , this, &MARBLE_vis::updatePose);

        mapWidget_= new Marble::MarbleWidget();
        mapWidget_->setProjection(Marble::Mercator);
        mapWidget_->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
        mapWidget_->show();

        ui->horizontalLayout->addWidget(mapWidget_);

        // Connect the map widget to the position label
        connect(mapWidget_, SIGNAL(mouseClickGeoPosition(qreal,qreal,GeoDataCoordinates::Unit)), this, SLOT(clickMouse(qreal,qreal,GeoDataCoordinates::Unit)));

        ros::NodeHandle nh;
<<<<<<< HEAD
        poseSub_ = nh.subscribe("/dji_telem/pos_gps", 1, &MARBLE_vis::CallbackPose, this);
        configMissionReq_ = nh.serviceClient<ivis::configMission>("/gui_marble/waypoints");

        place_ = new Marble::GeoDataPlacemark("Pose");
        // place_->setCoordinate(lon_, lat_, alt_, Marble::GeoDataCoordinates::Degree);
        // place_->setCoordinate(-6.003450, 37.412269, 0, Marble::GeoDataCoordinates::Degree);
        place_->setCoordinate(0, 0, 0, Marble::GeoDataCoordinates::Degree);
=======
        std::string nameCallbackPose = "/dji/gps_pos";
        poseSub_ = nh.subscribe(nameCallbackPose, 1, &MARBLE_vis::CallbackPose, this);

        place_ = new Marble::GeoDataPlacemark("Pose");
        place_->setCoordinate(lonGPS_, latGPS_, altGPS_, Marble::GeoDataCoordinates::Degree);
>>>>>>> master

        document_ = new Marble::GeoDataDocument;
        document_->append(place_);

        // Add the document to MarbleWidget's tree model
        mapWidget_->model()->treeModel()->addDocument(document_);

<<<<<<< HEAD
=======
	    // mapWidget_->zoomView(4000);

>>>>>>> master
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
// PRIVATE SLOTS
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
void MARBLE_vis::clickMouse(qreal _lon, qreal _lat, GeoDataCoordinates::Unit _unit){

    lastLonClicked = RAD_DEG(_lon);
    lastLatClicked = RAD_DEG(_lat);

    ui->lineEdit_c1->setText(QString::number(lastLatClicked));
    ui->lineEdit_c2->setText(QString::number(lastLonClicked));

}

//---------------------------------------------------------------------------------------------------------------------
void MARBLE_vis::centerUAV(){

    objectLockPose_.lock();
<<<<<<< HEAD
    mapWidget_->centerOn(Marble::GeoDataCoordinates(lonUAV_, latUAV_, altUAV_, Marble::GeoDataCoordinates::Degree));
    mapWidget_->zoomView(4000);
    objectLockPose_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void MARBLE_vis::addPointList(){
    
    std::vector<double> point = {latUAV_, lonUAV_, altUAV_};
    waypoints_.push_back(std::make_pair(idWP_, point));

    std::string swaypoint = "ID: " + std::to_string(idWP_);
    ui->listWidget_WayPoints->addItem(QString::fromStdString(swaypoint));

    idWP_++;

}

//---------------------------------------------------------------------------------------------------------------------
void MARBLE_vis::deleteWaypointList(){
    
    QList<QListWidgetItem*> items = ui->listWidget_WayPoints->selectedItems();
    foreach(QListWidgetItem * item, items){
        int index = ui->listWidget_WayPoints->row(item);
        waypoints_.erase(waypoints_.begin() + index); 
        delete ui->listWidget_WayPoints->takeItem(ui->listWidget_WayPoints->row(item));
    }

}

//---------------------------------------------------------------------------------------------------------------------
void MARBLE_vis::sendWaypointList(){
    
    QString qTypeMission;
    qTypeMission = ui->lineEdit_type->text();
    std::string typeMission = qTypeMission.toStdString();;
    
    ivis::configMission srvConfig;
    srvConfig.request.type = typeMission;

    for(unsigned i = 0; i < waypoints_.size(); i++){
        geometry_msgs::PoseStamped wp;
        wp.pose.position.x = waypoints_[i].second[0];
        wp.pose.position.y = waypoints_[i].second[1];
        wp.pose.position.z = waypoints_[i].second[2];
        srvConfig.request.poseWP.push_back(wp);
    }
    
    if(configMissionReq_.call(srvConfig)){
        if(srvConfig.response.success){
            std::cout << "Service of CONFIG MISSION success" << std::endl;
        }else{
            std::cout << "Service of CONFIG MISSION failed" << std::endl;
        }
    }else{
        std::cout << "Failed to call service of CONFIG MISSION" << std::endl;
    }

}

//---------------------------------------------------------------------------------------------------------------------
// PRIVATE
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
void MARBLE_vis::updatePose(){

    objectLockPose_.lock();
    ui->lineEdit_p1->setText(QString::number(latUAV_));
    ui->lineEdit_p2->setText(QString::number(lonUAV_));
    ui->lineEdit_p3->setText(QString::number(altUAV_));
    ui->lineEdit_ngps->setText(QString::number(nGPS_));
    objectLockPose_.unlock();

    place_->setCoordinate(lonUAV_, latUAV_, altUAV_, Marble::GeoDataCoordinates::Degree);

    // Update the document MarbleWidget's tree model
    mapWidget_->model()->treeModel()->updateFeature(place_);
=======
    ui->lineEdit_p1->setText(QString::number(latGPS_));
    ui->lineEdit_p2->setText(QString::number(lonGPS_));
    ui->lineEdit_p3->setText(QString::number(altGPS_));

    place_->setCoordinate(lonGPS_, latGPS_, altGPS_, Marble::GeoDataCoordinates::Degree);

    // Add the document to MarbleWidget's tree model
    mapWidget_->model()->treeModel()->updateFeature(place_);

    objectLockPose_.unlock();
>>>>>>> master

}

//---------------------------------------------------------------------------------------------------------------------
void MARBLE_vis::CallbackPose(const sensor_msgs::NavSatFix::ConstPtr& _msg){
    objectLockPose_.lock();
<<<<<<< HEAD
    nGPS_ = _msg->status.status;
    latUAV_ = _msg->latitude;
    lonUAV_ = _msg->longitude;
    altUAV_ = _msg->altitude;
=======
    latGPS_ = _msg->latitude;
    lonGPS_ = _msg->longitude;
    altGPS_ = _msg->altitude;

>>>>>>> master
    objectLockPose_.unlock();
}

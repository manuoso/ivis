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

        ui->checkBox_alt->setChecked(false);
        ui->checkBox_clicked->setChecked(false);
        ui->sendWP->setVisible(0);
        ui->startWP->setVisible(0);
        ui->stopWP->setVisible(0);
        ui->pauseWP->setVisible(0);
        ui->resumeWP->setVisible(0);

        connect(ui->center, SIGNAL(clicked()), this, SLOT(centerUAV()));
        connect(ui->deleteWP, SIGNAL(clicked()), this, SLOT(deleteWaypointList()));
        connect(ui->cleanWP, SIGNAL(clicked()), this, SLOT(cleanWaypointList()));
        connect(ui->visualizeMission, SIGNAL(clicked()), this, SLOT(visualizeMissionList()));
        connect(ui->sendWP, SIGNAL(clicked()), this, SLOT(sendWaypointList()));
        connect(ui->startWP, SIGNAL(clicked()), this, SLOT(startWaypointList()));
        connect(ui->stopWP, SIGNAL(clicked()), this, SLOT(stopWaypointList()));
        connect(ui->pauseWP, SIGNAL(clicked()), this, SLOT(pauseWaypointList()));
        connect(ui->resumeWP, SIGNAL(clicked()), this, SLOT(resumeWaypointList()));
        connect(this, &MARBLE_vis::positionChanged , this, &MARBLE_vis::updatePose);
        connect(this, &MARBLE_vis::addPoint , this, &MARBLE_vis::addPointList);

        mapWidget_= new Marble::MarbleWidget();
        mapWidget_->setProjection(Marble::Mercator);
        mapWidget_->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
        mapWidget_->show();

        ui->horizontalLayout->addWidget(mapWidget_);

        mapWidget_->addLayer(this);

        // Connect the map widget to the position label
        connect(mapWidget_, SIGNAL(mouseClickGeoPosition(qreal,qreal,GeoDataCoordinates::Unit)), this, SLOT(clickMouse(qreal,qreal,GeoDataCoordinates::Unit)));

        ros::NodeHandle nh;
        poseSub_ = nh.subscribe("/dji_telem/pos_gps", 1, &MARBLE_vis::CallbackPose, this);
        configMissionReq_ = nh.serviceClient<ivis::configMission>("/dji_control/configure_mission");
        // configMissionReq_ = nh.serviceClient<ivis::configMission>("/gui_marble/waypoints");
        startMissionReq_ = nh.serviceClient<std_srvs::SetBool>("/dji_control/start_mission");
        stopMissionReq_ = nh.serviceClient<std_srvs::SetBool>("/dji_control/stop_mission");
        pauseMissionReq_ = nh.serviceClient<std_srvs::SetBool>("/dji_control/pause_mission");
        resumeMissionReq_ = nh.serviceClient<std_srvs::SetBool>("/dji_control/resume_mission");

        currentClicked_ = new Marble::GeoDataPlacemark("Clicked");
        place_ = new Marble::GeoDataPlacemark("Pose");

        // place_->setCoordinate(lon_, lat_, alt_, Marble::GeoDataCoordinates::Degree);
        // place_->setCoordinate(-6.003450, 37.412269, 0, Marble::GeoDataCoordinates::Degree);
        currentClicked_->setCoordinate(0, 0, 0, Marble::GeoDataCoordinates::Degree);
        place_->setCoordinate(0, 0, 0, Marble::GeoDataCoordinates::Degree);

        document_ = new Marble::GeoDataDocument;
        document_->append(currentClicked_);
        document_->append(place_);

        // Add the document to MarbleWidget's tree model
        mapWidget_->model()->treeModel()->addDocument(document_);

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
QStringList MARBLE_vis::renderPosition() const{
    return QStringList() << "SURFACE";
}

//---------------------------------------------------------------------------------------------------------------------
bool MARBLE_vis::render(GeoPainter *painter, ViewportParams *viewport, const QString& renderPos, GeoSceneLayer * layer){
    
    if(visualizeMission_ && typeMission_ != ""){

        if(typeMission_ == "waypoint"){
            
            painter->setRenderHint(QPainter::Antialiasing, true);
            painter->setBrush(QBrush(Qt::yellow));
            painter->setPen(QPen(QBrush(Qt::green), 2.0, Qt::SolidLine, Qt::RoundCap));

            GeoDataCoordinates *point, *nextPoint;

            for(unsigned i = 0; i < waypoints_.size(); i++){
                
                if(i < (waypoints_.size()-1) ){
                    
                    point = new GeoDataCoordinates(waypoints_[i].second[1], waypoints_[i].second[0], waypoints_[i].second[2], GeoDataCoordinates::Degree);
                    nextPoint = new GeoDataCoordinates(waypoints_[i+1].second[1], waypoints_[i+1].second[0], waypoints_[i+1].second[2], GeoDataCoordinates::Degree);

                    GeoDataLineString lineNoTess(NoTessellation);
                    lineNoTess << *point << *nextPoint;

                    painter->drawPolyline(lineNoTess);
                }
            }

            GeoDataCoordinates initPoint(waypoints_[0].second[1], waypoints_[0].second[0], waypoints_[0].second[2], GeoDataCoordinates::Degree);
            GeoDataCoordinates finalPoint(waypoints_[waypoints_.size()-1].second[1], waypoints_[waypoints_.size()-1].second[0], waypoints_[waypoints_.size()-1].second[2], GeoDataCoordinates::Degree);
            
            GeoDataLineString lineNoTess(NoTessellation);
            lineNoTess << finalPoint << initPoint;

            // painter->setPen(oxygenForestGreen4);
            painter->drawPolyline(lineNoTess);

        }else if(typeMission_ == "hotpoint"){

            GeoDataCoordinates hotpoint(waypoints_[0].second[1], waypoints_[0].second[0], waypoints_[0].second[2], GeoDataCoordinates::Degree);
            painter->setRenderHint(QPainter::Antialiasing, true);
            painter->setBrush(QBrush(Qt::yellow));
            painter->setPen(QPen(QBrush(Qt::green), 2.0, Qt::SolidLine, Qt::RoundCap));
            // 666 TODO: RADIO REAL??
            painter->drawEllipse(hotpoint, 100, 100);

        }else{
            std::cout << "Unrecognized TYPE of MISSION" << std::endl;
        }
        
    }

    // visualizeMission_ = false;

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
// PRIVATE SLOTS
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
void MARBLE_vis::clickMouse(qreal _lon, qreal _lat, GeoDataCoordinates::Unit _unit){

    lastLonClicked_ = RAD_DEG(_lon);
    lastLatClicked_ = RAD_DEG(_lat);

    ui->lineEdit_c1->setText(QString::number(lastLatClicked_));
    ui->lineEdit_c2->setText(QString::number(lastLonClicked_));

    
    if(ui->checkBox_clicked->isChecked()){
        currentClicked_->setCoordinate(lastLonClicked_, lastLatClicked_, 0, Marble::GeoDataCoordinates::Degree);

        // Update the document MarbleWidget's tree model
        mapWidget_->model()->treeModel()->updateFeature(currentClicked_);
    }

}

//---------------------------------------------------------------------------------------------------------------------
void MARBLE_vis::centerUAV(){

    objectLockPose_.lock();
    mapWidget_->centerOn(Marble::GeoDataCoordinates(lonUAV_, latUAV_, altUAV_, Marble::GeoDataCoordinates::Degree));
    mapWidget_->zoomView(4000);
    objectLockPose_.unlock();

}

//---------------------------------------------------------------------------------------------------------------------
void MARBLE_vis::addPointList(){
    
    QString qTypeMission;
    qTypeMission = ui->lineEdit_type->text();
    std::string typeMission = qTypeMission.toStdString();;

    typeMission_ = typeMission;

    if(typeMission == "waypoint"){
        std::string sPlace = "WP" + std::to_string(idWP_);
        QString qPlace = QString::fromStdString(sPlace);
        Marble::GeoDataPlacemark *place = new Marble::GeoDataPlacemark(qPlace);

        if(ui->checkBox_alt->isChecked()){
            place->setCoordinate(lastLonClicked_, lastLatClicked_, altUAV_, Marble::GeoDataCoordinates::Degree);

            std::vector<double> point = {lastLatClicked_, lastLonClicked_, altUAV_};
            waypoints_.push_back(std::make_pair(idWP_, point));
        }else{
            QString qAlt = ui->lineEdit_alt->text();
            double alt = qAlt.toDouble(); 

            place->setCoordinate(lastLonClicked_, lastLatClicked_, alt, Marble::GeoDataCoordinates::Degree);

            std::vector<double> point = {lastLatClicked_, lastLonClicked_, alt};
            waypoints_.push_back(std::make_pair(idWP_, point));
        }
        

        mapWidget_->model()->treeModel()->addFeature(document_, place);

        std::string swaypoint = "ID: " + std::to_string(idWP_);
        ui->listWidget_WayPoints->addItem(QString::fromStdString(swaypoint));

        idWP_++;
    }else if(typeMission == "hotpoint"){
        std::string sPlace = "HP" + std::to_string(idWP_);
        QString qPlace = QString::fromStdString(sPlace);
        Marble::GeoDataPlacemark *place = new Marble::GeoDataPlacemark(qPlace);

        QString qRad = ui->lineEdit_radius->text();
        double rad = qRad.toDouble(); 

        radiusHP_ = rad;

        QString qAlt = ui->lineEdit_alt->text();
        double alt = qAlt.toDouble();  

        place->setCoordinate(lastLonClicked_, lastLatClicked_, alt, Marble::GeoDataCoordinates::Degree);

        std::vector<double> point = {lastLatClicked_, lastLonClicked_, alt};
        waypoints_.push_back(std::make_pair(idWP_, point));

        mapWidget_->model()->treeModel()->addFeature(document_, place);

        std::string swaypoint = "ID: " + std::to_string(idWP_);
        ui->listWidget_WayPoints->addItem(QString::fromStdString(swaypoint));

        idWP_++;
    }else{
        std::cout << "Unrecognized TYPE of MISSION to add" << std::endl;
    }
    

}

//---------------------------------------------------------------------------------------------------------------------
void MARBLE_vis::deleteWaypointList(){
    
    QList<QListWidgetItem*> items = ui->listWidget_WayPoints->selectedItems();
    foreach(QListWidgetItem * item, items){
        int index = ui->listWidget_WayPoints->row(item);
        waypoints_.erase(waypoints_.begin() + index);
        mapWidget_->model()->treeModel()->removeFeature(document_, index+2);    // +2 because we have pose and clicked too in features
        delete ui->listWidget_WayPoints->takeItem(ui->listWidget_WayPoints->row(item));
    }

}

//---------------------------------------------------------------------------------------------------------------------
void MARBLE_vis::cleanWaypointList(){
    
    visualizeMission_ = false;
    ui->sendWP->setVisible(0);

    // Clear vector
    waypoints_.clear();
    
    QList<QListWidgetItem*> items = ui->listWidget_WayPoints->findItems("*", Qt::MatchWildcard);
    foreach(QListWidgetItem *item, items){
        int index = ui->listWidget_WayPoints->row(item);
        delete ui->listWidget_WayPoints->takeItem(ui->listWidget_WayPoints->row(item));
        mapWidget_->model()->treeModel()->removeFeature(document_, index+2);
    }

    idWP_ = 0;
}

//---------------------------------------------------------------------------------------------------------------------
void MARBLE_vis::visualizeMissionList(){
    
    visualizeMission_ = true;

    ui->sendWP->setVisible(1);

}

//---------------------------------------------------------------------------------------------------------------------
void MARBLE_vis::sendWaypointList(){
    
    QString qTypeMission = ui->lineEdit_type->text();
    std::string typeMission = qTypeMission.toStdString();;
    
    ivis::configMission srvConfig;
    srvConfig.request.type = typeMission;

    for(unsigned i = 0; i < waypoints_.size(); i++){
        sensor_msgs::NavSatFix wp;
        wp.latitude = waypoints_[i].second[0];
        wp.longitude = waypoints_[i].second[1];
        wp.altitude = waypoints_[i].second[2];
        wp.position_covariance[0] = radiusHP_;
        srvConfig.request.waypoint.push_back(wp);
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

    ui->startWP->setVisible(1);
    ui->stopWP->setVisible(1);

}

//---------------------------------------------------------------------------------------------------------------------
void MARBLE_vis::startWaypointList(){
    
    std_srvs::SetBool srv;
    srv.request.data = true;

    if(startMissionReq_.call(srv)){
        if(srv.response.success){
            std::cout << "Service of START MISSION success" << std::endl;
        }else{
            std::cout << "Service of START MISSION failed" << std::endl;
        }
    }else{
        std::cout << "Failed to call service of START MISSION" << std::endl;
    }

}

//---------------------------------------------------------------------------------------------------------------------
void MARBLE_vis::stopWaypointList(){
    
    std_srvs::SetBool srv;
    srv.request.data = true;

    if(stopMissionReq_.call(srv)){
        if(srv.response.success){
            std::cout << "Service of STOP MISSION success" << std::endl;
        }else{
            std::cout << "Service of STOP MISSION failed" << std::endl;
        }
    }else{
        std::cout << "Failed to call service of STOP MISSION" << std::endl;   
    }

}

//---------------------------------------------------------------------------------------------------------------------
void MARBLE_vis::pauseWaypointList(){
    
    std_srvs::SetBool srv;
    srv.request.data = true;

    if(pauseMissionReq_.call(srv)){
        if(srv.response.success){
            std::cout << "Service of PAUSE MISSION success" << std::endl;
        }else{
            std::cout << "Service of PAUSE MISSION failed" << std::endl;
        }
    }else{
        std::cout << "Failed to call service of PAUSE MISSION" << std::endl;   
    }

}

//---------------------------------------------------------------------------------------------------------------------
void MARBLE_vis::resumeWaypointList(){
    
    std_srvs::SetBool srv;
    srv.request.data = true;

    if(resumeMissionReq_.call(srv)){
        if(srv.response.success){
            std::cout << "Service of RESUME MISSION success" << std::endl;
        }else{
            std::cout << "Service of RESUME MISSION failed" << std::endl;
        }
    }else{
        std::cout << "Failed to call service of RESUME MISSION" << std::endl;   
    }

}

//---------------------------------------------------------------------------------------------------------------------
// PROTECTED
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
void MARBLE_vis::keyPressEvent(QKeyEvent *_event){
    if(_event->key() == Qt::Key_A){
        if(ui->checkBox_addPoint->isChecked()){
            emit addPoint(); 
        }
    }else if(_event->key() == Qt::Key_E){
        ui->checkBox_addPoint->setChecked(true);
    }else if(_event->key() == Qt::Key_D){
        ui->checkBox_addPoint->setChecked(false);
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

}

//---------------------------------------------------------------------------------------------------------------------
void MARBLE_vis::CallbackPose(const sensor_msgs::NavSatFix::ConstPtr& _msg){
    objectLockPose_.lock();
    nGPS_ = _msg->status.status;
    latUAV_ = _msg->latitude;
    lonUAV_ = _msg->longitude;
    altUAV_ = _msg->altitude;
    objectLockPose_.unlock();
}

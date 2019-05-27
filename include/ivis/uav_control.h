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

#ifndef UAV_CONTROL_H
#define UAV_CONTROL_H

#include <QMainWindow>
#include <QTextStream>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <chrono>
#include <stdlib.h>
#include <mutex>
#include <thread>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h> 
#include <sensor_msgs/BatteryState.h> 
#include <geometry_msgs/PoseStamped.h>

#include <ivis/configMission.h>
#include <std_srvs/SetBool.h>

#define CONST_PI (double)3.141592653589793
#define DEG_RAD(DEG) ((DEG) * ((CONST_PI) / (180.0)))
#define RAD_DEG(RAD) ((RAD) * (180.0) / (CONST_PI))

namespace Ui{
    class UAV_control;
}

class UAV_control : public QMainWindow{
    Q_OBJECT

    public:
        /// Constructor
        explicit UAV_control(QWidget *parent = 0);

        /// Destructor
        ~UAV_control();

        /// Struct of data
        struct xyz_data
        {   
            float x = 0.0;
            float y = 0.0;
            float z = 0.0;
        };

    signals:
        /// Signal that warns that there is a change in the Telemetry of the uav
        void telemChanged();

    private slots:
        /// Method that send take off to the UAV
        void takeoffUAV();

        /// Method that send land to the UAV
        void landUAV();

        /// Method that send emergency stop to the UAV
        void emergStopUAV();

        /// Method that send position to the UAV
        void startPositionUAV();

        /// Method that finish send position to the UAV
        void stopPositionUAV();

        /// Method that send velocity to the UAV
        void startVelocityUAV();

        /// Method that finish send velocity to the UAV
        void stopVelocityUAV();

        /// Method for recover control from manual of the UAV
        void recoverControlUAV();

    private:
        /// Send it by ROS Publishers
        void sendThread();

        /// Method that notify that the Telemetry of the UAV have changed
        void updateTelem();

        /// Callback method for visualize a change of pose GPS from a topic of ROS
        /// \param _msg: data receive to update pose GPS
        void CallbackPoseGPS(const sensor_msgs::NavSatFix::ConstPtr& _msg);

        /// Callback method for visualize a change of local pose from a topic of ROS
        /// \param _msg: data receive to update local pose
        void CallbackPoseLocal(const geometry_msgs::PoseStamped::ConstPtr& _msg);

        /// Callback method for visualize a change of velocity from a topic of ROS
        /// \param _msg: data receive to update velocity
        void CallbackVel(const geometry_msgs::TwistStamped::ConstPtr& _msg);

        /// Callback method for visualize a change of RC from a topic of ROS
        /// \param _msg: data receive to update RC
        void CallbackRC(const std_msgs::Float64MultiArray::ConstPtr& _msg);

        /// Callback method for visualize a change of Battery level from a topic of ROS
        /// \param _msg: data receive to update battery level
        void CallbackBat(const sensor_msgs::BatteryState::ConstPtr& _msg);

        /// Callback method for visualize a change of mode from a topic of ROS
        /// \param _msg: data receive to update mode
        void CallbackMode(const std_msgs::String::ConstPtr& _msg);

        /// Callback method for visualize a change of fly status from a topic of ROS
        /// \param _msg: data receive to update fly status
        void CallbackFS(const std_msgs::String::ConstPtr& _msg);

        /// Callback method for visualize a change of dji control status from a topic of ROS
        /// \param _msg: data receive to update dji control status
        void CallbackDJICS(const std_msgs::String::ConstPtr& _msg);

    private:
        Ui::UAV_control *ui;

        ros::Publisher velocityPub_, positionPub_;
        ros::Subscriber poseGPSSub_, poseLocalSub_, poseVelSub_, poseRCSub_, modeSub_, flyStatusSub_, nBatSub_, djiConStaSub_;
        ros::ServiceClient landReq_, takeoffReq_, emergencyBrakeReq_, recoverControlReq_;

        std::thread *sendThread_, *telemThread_;
        std::mutex objectLockGPS_, objectLockLocal_, objectLockVel_, objectLockRC_, objectLockBat_, objectLockMode_, objectLockFS_, objectLockDJICS_;
        
        std::chrono::time_point<std::chrono::high_resolution_clock> lastTimePose_;	

        bool stopAll_ = false;
        bool stopSend_ = false;

        xyz_data velocity_, position_;
        geometry_msgs::TwistStamped msgVelocity_;
        geometry_msgs::PoseStamped msgPosition_;

        std::string type_ = "";

        std::string mode_ = "", flyStatus_ = "", djiConSta_ = ""; 

        float rc1_ = 0.0, rc2_ = 0.0, rc3_ = 0.0, rc4_ = 0.0;
        double poseGPSLat_ = 0.0, poseGPSLon_ = 0.0, poseGPSAlt_ = 0.0;
        
        float batLevel = 0.0;
        int nGPS_ = 0;
        
        
};

#endif // UAV_CONTROL_H

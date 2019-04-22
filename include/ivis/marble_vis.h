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

#ifndef MARBLE_VIS_H
#define MARBLE_VIS_H

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

#include <marble/MarbleWidget.h>
#include <marble/GeoDataDocument.h>
#include <marble/GeoDataPlacemark.h>
#include <marble/GeoDataLineString.h>
#include <marble/GeoDataTreeModel.h>
#include <marble/MarbleModel.h>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

namespace Ui{
    class MARBLE_vis;
}

class MARBLE_vis : public QMainWindow {
    Q_OBJECT

    public:
        /// Constructor
        explicit MARBLE_vis(QWidget *parent = 0);

        /// Destructor
        ~MARBLE_vis();

    signals:
        /// Signal that warns that there is a change in the pose of the uav
        void positionChanged();

    private:
        /// Method that update local position of the UAV
        void updatePose();
    
        /// Method for visualize a change of pose from a topic of ROS
        /// \param _msg: data receive to update pose
        void CallbackPose(const sensor_msgs::NavSatFix::ConstPtr& _msg);

    private:
        Ui::MARBLE_vis *ui;
        Marble::MarbleWidget *mapWidget_;

        Marble::GeoDataPlacemark *place_;
        Marble::GeoDataDocument *document_;

        ros::Subscriber poseSub_;

        std::thread *poseThread_;
        std::mutex objectLockPose_;
        
        std::chrono::time_point<std::chrono::high_resolution_clock> lastTimePose_;	

        int cont_ = 0;
        bool stopAll_ = false;
        float latGPS_ = 0, lonGPS_ = 0, altGPS_ = 0;
        
};

#endif // MARBLE_VIS_H

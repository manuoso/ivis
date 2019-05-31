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

#ifdef IVIS_USE_TABS

#ifndef TABS_GUI_H
#define TABS_GUI_H

#include <QMainWindow>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <chrono>
#include <stdlib.h>
#include <mutex>
#include <thread>
#include <cmath>

#ifdef IVIS_USE_PCL    
    #include <ivis/pclviewer_gui.h>
#endif

#ifdef IVIS_USE_MARBLE    
    #include <ivis/marble_vis.h>
#endif

#ifdef IVIS_USE_CONTROL    
    #include <ivis/uav_control.h>
#endif

class TABS_gui : public QMainWindow{
    Q_OBJECT

    public:
        /// Constructor
        explicit TABS_gui(QWidget *parent = 0);

        /// Destructor
        ~TABS_gui();

    private:
        QTabWidget *tabWidget_;
        
        #ifdef IVIS_USE_PCL    
            PCLViewer_gui *pclVis_;
        #endif

        #ifdef IVIS_USE_MARBLE    
            MARBLE_vis *marbleVis_;
        #endif

        #ifdef IVIS_USE_CONTROL    
            UAV_control *uavVis_;
        #endif

};

#endif // TABS_GUI_H

#endif

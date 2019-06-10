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

#include <ivis/tabs_gui.h>

//---------------------------------------------------------------------------------------------------------------------
// PUBLIC 
//---------------------------------------------------------------------------------------------------------------------

TABS_gui::TABS_gui(QWidget *parent) :
    QMainWindow(parent)
    {

        tabWidget_ = new QTabWidget(this);
        tabWidget_->setFixedSize(1150, 800);

        #ifdef IVIS_USE_CONTROL    
            uavVis_ = new UAV_control;
            tabWidget_->addTab(uavVis_, "UAV CONTROL GUI");
        #endif   

        #ifdef IVIS_USE_MARBLE    
            marbleVis_ = new MARBLE_vis;
            tabWidget_->addTab(marbleVis_, "MISSIONS GUI");
        #endif     

        #ifdef IVIS_USE_PCL    
            pclVis_ = new PCLViewer_gui;
            tabWidget_->addTab(pclVis_, "PCL GUI");
        #endif

        resize(1200, 900);

        setWindowTitle(tr("Tabs GUI"));

    }

//---------------------------------------------------------------------------------------------------------------------
TABS_gui::~TABS_gui(){

}

#endif

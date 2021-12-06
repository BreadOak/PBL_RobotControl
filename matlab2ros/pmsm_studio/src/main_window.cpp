/*
 * main_window.cpp
 *      Author: junyoung kim / lgkimjy
 */

#include <QtGui>
#include <QMessageBox>
#include <QPixmap>
#include <iostream>
#include "main_window.hpp"
#include <stdio.h>
#include <algorithm>
#include <string>
#include <sstream>

namespace pmsm_studio
{
    using namespace Qt;

    MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent), qnode(argc,argv)
    {
        ui.setupUi(this);                                                                    // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
        QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
        qnode.init();
        
        Plot_MakeUI();
    }
    MainWindow::~MainWindow() {}
}

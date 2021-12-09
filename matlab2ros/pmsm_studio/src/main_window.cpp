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
        ui.setupUi(this);
        plot_window = new QMainWindow;
        ui_plot.setupUi(plot_window);
        
        QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
        qnode.init();
        
        QObject::connect(ui.horizontalSlider_2, SIGNAL(valueChanged(int)),this, SLOT(readvalue(int)));
        Plot_MakeUI();
    }
    MainWindow::~MainWindow() {}
    
    void MainWindow::on_none_module_button_clicked()
    {
        // RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "Initialize the PMSM nodes");
        qnode.sendData();
    }
        
    void MainWindow::readvalue(int value)
    {
        RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "data : %d", value);
    }

    void MainWindow::on_popup_button_clicked()
    {
        plot_window->show();

        RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "callback occured");
    }
}

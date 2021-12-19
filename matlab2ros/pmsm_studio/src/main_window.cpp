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
        plot_window_1 = new QMainWindow;
        plot_window_2 = new QMainWindow;
        plot_window_3 = new QMainWindow;
        plot_window_4 = new QMainWindow;
        plot_window_5 = new QMainWindow;
        plot_window_6 = new QMainWindow;
        ui_plot_1.setupUi(plot_window_1);
        ui_plot_2.setupUi(plot_window_2);
        ui_plot_3.setupUi(plot_window_3);
        ui_plot_4.setupUi(plot_window_4);
        ui_plot_5.setupUi(plot_window_5);
        ui_plot_6.setupUi(plot_window_6);

        // QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
        // QObject::connect(ui.horizontalSlider_2, SIGNAL(valueChanged(int)),this, SLOT(readvalue(int)));
       
        /* ROS INITIALIZATION */
        qnode.init();
        /* ABOUT UI */
        connect(ui.current_mode, SIGNAL(clicked()), this, SLOT(current_mode()));
        connect(ui.velocity_mode, SIGNAL(clicked()), this, SLOT(velocity_mode()));
        connect(ui.position_mode, SIGNAL(clicked()), this, SLOT(position_mode()));
        mode_count = 0;
        mode_selected_ = false;
        Plot_MakeUI();
    }
    MainWindow::~MainWindow() {}
    
    void MainWindow::on_none_module_button_clicked()
    {
        qnode.sendData();
    }
        
    void MainWindow::readvalue(int value)
    {
        RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "data : %d", value);
    }

    void MainWindow::current_mode()
    {
        RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "Mode Change to Current Mode");
        mode_selected_ = true;
        qnode.pubMode(1);
        dataTimer->stop();
    }
    void MainWindow::velocity_mode()
    {
        RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "Mode Change to Velocity Mode");
        mode_selected_ = true;
        qnode.pubMode(2);
        dataTimer->stop();
    }
    void MainWindow::position_mode()
    {
        RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "Mode Change to Position Mode");
        mode_selected_ = true;
        qnode.pubMode(3);
        dataTimer->stop();
    }

    void MainWindow::on_mode_selection_clicked()
    {
        vector<int> mode;

        /* Current Control Mode */
        if(ui.current_comboBox->currentText() == "Not use"){
            RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "Not use Current Mode");
            mode.push_back(0);
            mode_count += 1;
        }
        else if(ui.current_comboBox->currentText() == "PI"){
            RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "PI Current Mode");
            mode.push_back(1);
            mode_count += 1;
        }
        else if(ui.current_comboBox->currentText() == "MPC"){
            RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "MPC Current Mode");
            mode.push_back(2);
            mode_count += 1;
        }
        /* Velocity Control Mode */
        if(ui.velocity_comboBox->currentText() == "Not use"){
            RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "Not use Velocity Mode");
            mode.push_back(0);
            mode_count += 1;
        }
        else if(ui.velocity_comboBox->currentText() == "PI"){
            RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "PI Velocity Mode");
            mode.push_back(1);
            mode_count += 1;
        }
        else if(ui.velocity_comboBox->currentText() == "MPC"){
            RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "MPC Velocity Mode");
            mode.push_back(2);
            mode_count += 1;
        }
        else if(ui.velocity_comboBox->currentText() == "H-inf"){
            RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "H-inf Velocity Mode");
            mode.push_back(3);
            mode_count += 1;
        }
        /* Position Control Mode */
        if(ui.position_comboBox->currentText() == "Not use"){
            RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "Not use Position Mode");
            mode_count += 1;
            mode.push_back(0);
        }
        else if(ui.position_comboBox->currentText() == "PI"){
            RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "PI Position Mode");
            mode.push_back(1);
            mode_count += 1;
        }

        /* DoB OPtion Mode */
        if(ui.dob_comboBox->currentText() == "Not use"){
            RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "Not use DOB Option");
            mode.push_back(0);
            mode_count += 1;
        }
        else if(ui.dob_comboBox->currentText() == "PI+DOB"){
            RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "PI+DOB Option");
            mode.push_back(1);
            mode_count += 1;
        }

        if(mode_count == 4){
            mode_selected_ = true;
            qnode.pubMode(mode);
            mode_count = 0;
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("PMSM Node"), "PLEASE SELECT MODE COREECTLY");
            mode_count = 0;
        }
    }

    void MainWindow::on_ref_data_send_clicked()
    {
        qnode.ref_input = ui.ref_data->text().toDouble();
        RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "REF INPUT SUBMITTED : %f", qnode.ref_input);
        if(mode_selected_ == false)
            RCLCPP_WARN(rclcpp::get_logger("PMSM Node"), "PLEASE SELECT CONTROL MODE");
        else
            qnode.pubData(qnode.ref_input);
    }

    void MainWindow::on_popup_button_1_clicked()
    {
        plot_window_1->show();
        RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "callback occured");
    }
    void MainWindow::on_popup_button_2_clicked()
    {
        plot_window_2->show();
        RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "callback occured");
    }
    void MainWindow::on_popup_button_3_clicked()
    {
        plot_window_3->show();
        RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "callback occured");
    }
    void MainWindow::on_popup_button_4_clicked()
    {
        plot_window_4->show();
        RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "callback occured");
    }
    void MainWindow::on_popup_button_5_clicked()
    {
        plot_window_5->show();
        RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "callback occured");
    }
    void MainWindow::on_popup_button_6_clicked()
    {
        plot_window_6->show();
        RCLCPP_INFO(rclcpp::get_logger("PMSM Node"), "callback occured");
    }
}

/*
 * main_window.hpp
 *      Author: junyoung kim / lgkimjy
 */

#include <QtWidgets/QMainWindow>
#include <QMouseEvent>
#include <utility>
#include "ui_main_window.h"
#include "ui_plot.h"
#include "qnode.hpp"
#include <iostream>

namespace pmsm_studio
{
    class MainWindow : public QMainWindow
    {
        Q_OBJECT

    public:
        MainWindow(int argc, char **argv, QWidget *parent = 0);
        ~MainWindow();

        bool mode_selected_;
        int mode_count;
        vector<string> legend_id={"Idse", "M_Idse"};
        vector<string> legend_iq={"Iqse", "M_Iqse"};
        vector<string> legend_M_I={"M_Ias", "M_Ibs", "M_Ics"};
        vector<string> legend_V={"Van_Ref", "Vbn_Ref", "Vcn_Ref"};
        vector<string> legend_Torq={"Torque_Ref", "Torque_Real", "Torque_Load"};
        vector<string> legend_rpm={"Ref_rpm", "Cur_rpm"};
        vector<string> legend_angle={"Ang_Ref", "Ang_Real"};
        vector<QPen> color={QPen(QColor(255, 0, 0)), QPen(QColor(0, 255, 0)), QPen(QColor(0, 0, 255))};

    public Q_SLOTS:

        void current_mode();
        void velocity_mode();
        void position_mode();

        void on_mode_selection_clicked();

        void on_none_module_button_clicked();
        void on_plot_stop_button_clicked();
        void on_plot_start_button_clicked();
        void on_popup_button_1_clicked();
        void on_popup_button_2_clicked();
        void on_popup_button_3_clicked();
        void on_popup_button_4_clicked();
        void on_popup_button_5_clicked();
        void on_popup_button_6_clicked();
        void on_popup_button_7_clicked();
        void on_ref_data_send_clicked();
        void realtimeDataSlot();
        // void updatePoses();
        void readvalue(int value);

    private:
        Ui::MainWindowDesign ui;
        Ui::PlotDesign ui_plot_1;
        Ui::PlotDesign ui_plot_2;
        Ui::PlotDesign ui_plot_3;
        Ui::PlotDesign ui_plot_4;
        Ui::PlotDesign ui_plot_5;
        Ui::PlotDesign ui_plot_6;
        Ui::PlotDesign ui_plot_7;
        QNode qnode;
        bool initalization_ = false;
        QTimer *dataTimer;
        double key;
        QMainWindow *plot_window_1;
        QMainWindow *plot_window_2;
        QMainWindow *plot_window_3;
        QMainWindow *plot_window_4;
        QMainWindow *plot_window_5;
        QMainWindow *plot_window_6;
        QMainWindow *plot_window_7;

        void Plot_MakeUI();
        void Plot_Current(QCustomPlot *ui_graph);
        
        void Plot_clear(QCustomPlot *ui_graph);
        
        void Plot_Current(QCustomPlot *ui_graph, float val_1, float val_2, float val_3);
        void Plot_Current(QCustomPlot *ui_graph, float val_1, float val_2);
        void Plot_Current(QCustomPlot *ui_graph, float val_1);
        void Plot_replot(QCustomPlot *ui_graph);
        void Plot_Init(QCustomPlot *ui_graph, int min_value, int max_value, int tick_count, vector<string> legend);
    
        void Plot_example(QCustomPlot *ui_graph);
    };

}
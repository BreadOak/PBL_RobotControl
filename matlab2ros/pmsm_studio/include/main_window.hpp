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

    public Q_SLOTS:
        void on_none_module_button_clicked();
        void on_plot_stop_button_clicked();
        void on_plot_start_button_clicked();
        void on_popup_button_clicked();
        void realtimeDataSlot();
        // void updatePoses();
        void readvalue(int value);

    private:
        Ui::MainWindowDesign ui;
        Ui::PlotDesign ui_plot;
        QNode qnode;
        bool initalization_ = false;
        QTimer *dataTimer;
        double key;
        QMainWindow *plot_window;

        void Plot_MakeUI();
        void Plot_Current(QCustomPlot *ui_graph);

        void Plot_Current(QCustomPlot *ui_graph, float val_1, float val_2, float val_3);
        void Plot_Current(QCustomPlot *ui_graph, float val_1, float val_2);
        void Plot_Current(QCustomPlot *ui_graph, float val_1);
        void Plot_replot(QCustomPlot *ui_graph);
        void Plot_Init(QCustomPlot *ui_graph, int min_value, int max_value, int tick_count);
    
        void Plot_example(QCustomPlot *ui_graph);
    };

}
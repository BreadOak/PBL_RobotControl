/*
 * main_window.hpp
 *      Author: junyoung kim / lgkimjy
 */

#include <QtWidgets/QMainWindow>
#include <QMouseEvent>
#include <utility>
#include "ui_main_window.h"
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
        void realtimeDataSlot();
        void on_none_module_button_clicked();
        // void updatePoses();

    private:
        Ui::MainWindowDesign ui;
        QNode qnode;
        bool initalization_ = false;
        QTimer *dataTimer;
        double key;

        void Plot_MakeUI();
        void Plot_Current();
        void Plot_replot(QCustomPlot *ui_graph);
        void Plot_Init(QCustomPlot *ui_graph, int min_value, int max_value, int tick_count);
    
        void Plot_example(QCustomPlot *ui_graph);
    };

}
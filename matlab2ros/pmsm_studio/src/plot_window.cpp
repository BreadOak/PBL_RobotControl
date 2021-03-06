/*
 * plot_window.cpp
 *      Author: junyoung kim / lgkimjy
 */

#include "main_window.hpp"

namespace pmsm_studio
{
	using namespace Qt;

	// Plot
	void MainWindow::Plot_MakeUI()
	{
		dataTimer = new QTimer(this);
		connect(dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
		dataTimer->start(0.02); // Interval 0 means to refresh as fast as possible
		dataTimer->stop();

		Plot_Init(ui.plot_pmsm, -5, 5, 10, legend_id);
        // Plot_example(ui.plot_pmsm);
        Plot_example(ui.plot_pmsm_2);
        // Plot_example(ui.plot_pmsm_3);
        Plot_Init(ui.plot_pmsm_4, -5, 5, 10, legend_iq);
        Plot_Init(ui.plot_pmsm_5, -5, 5, 10, legend_M_I);
        Plot_Init(ui.plot_pmsm_6, -5, 5, 10, legend_V);
        Plot_Init(ui.plot_pmsm_7, -5, 5, 10, legend_Torq);
        Plot_Init(ui.plot_pmsm_8, -5, 5, 10, legend_rpm);
        Plot_Init(ui.plot_pmsm_9, -5, 5, 10, legend_angle);

		Plot_Init(ui_plot_1.plot_enlarged, -5, 5, 10, legend_id);
		Plot_Init(ui_plot_2.plot_enlarged, -5, 5, 10, legend_iq);
		Plot_Init(ui_plot_3.plot_enlarged, -5, 5, 10, legend_M_I);
		Plot_Init(ui_plot_4.plot_enlarged, -5, 5, 10, legend_V);
		Plot_Init(ui_plot_5.plot_enlarged, -5, 5, 10, legend_Torq);
		Plot_Init(ui_plot_6.plot_enlarged, -5, 5, 10, legend_rpm);
		Plot_Init(ui_plot_7.plot_enlarged, -5, 5, 10, legend_angle);
	}

    void MainWindow::realtimeDataSlot()
    {   
 		static QTime time(QTime::currentTime());
		key = time.elapsed() / 100000.0;
		static double lastPointKey = 0;

		if (key - lastPointKey > 0.0002)
		{
			Plot_Current(ui.plot_pmsm, qnode.Idse, qnode.M_Idse);
            Plot_Current(ui.plot_pmsm_4, qnode.Iqse, qnode.M_Iqse);
            Plot_Current(ui.plot_pmsm_5, qnode.M_Ias, qnode.M_Ibs, qnode.M_Ics);
            Plot_Current(ui.plot_pmsm_6, qnode.Van_Ref, qnode.Vbn_Ref, qnode.Vcn_Ref);
            Plot_Current(ui.plot_pmsm_7, qnode.Torque_Ref, qnode.Torque_Real, qnode.Torque_Load);
            Plot_Current(ui.plot_pmsm_8, qnode.Ref_rpm, qnode.Cur_rpm);
            Plot_Current(ui.plot_pmsm_9, qnode.Ang_Ref, qnode.Ang_Real);

			Plot_Current(ui_plot_1.plot_enlarged, qnode.Idse, qnode.M_Idse);
            Plot_Current(ui_plot_2.plot_enlarged, qnode.Iqse, qnode.M_Iqse);
            Plot_Current(ui_plot_3.plot_enlarged, qnode.M_Ias, qnode.M_Ibs, qnode.M_Ics);
            Plot_Current(ui_plot_4.plot_enlarged, qnode.Van_Ref, qnode.Vbn_Ref, qnode.Vcn_Ref);
            Plot_Current(ui_plot_5.plot_enlarged, qnode.Torque_Ref, qnode.Torque_Real, qnode.Torque_Load);
            Plot_Current(ui_plot_6.plot_enlarged, qnode.Ref_rpm, qnode.Cur_rpm);
            Plot_Current(ui_plot_7.plot_enlarged, qnode.Ang_Ref, qnode.Ang_Real);
			lastPointKey = key;
		}
		Plot_replot(ui.plot_pmsm);
        Plot_replot(ui.plot_pmsm_4);
        Plot_replot(ui.plot_pmsm_5);
        Plot_replot(ui.plot_pmsm_6);
        Plot_replot(ui.plot_pmsm_7);
        Plot_replot(ui.plot_pmsm_8);
        Plot_replot(ui.plot_pmsm_9);
        Plot_replot(ui_plot_1.plot_enlarged);
        Plot_replot(ui_plot_2.plot_enlarged);
        Plot_replot(ui_plot_3.plot_enlarged);
        Plot_replot(ui_plot_4.plot_enlarged);
        Plot_replot(ui_plot_5.plot_enlarged);
        Plot_replot(ui_plot_6.plot_enlarged);
        Plot_replot(ui_plot_7.plot_enlarged);
    }

	void MainWindow::Plot_Init(QCustomPlot *ui_graph, int min_value, int max_value, int tick_count, vector<string> legend)
	{
        ui_graph->legend->setVisible(true);
        QFont legendFont = font();  // start out with MainWindow's font..
        legendFont.setPointSize(9); // and make a bit smaller for legend
        ui_graph->legend->setFont(legendFont);
        ui_graph->legend->setBrush(QBrush(QColor(255,255,255,230)));

        // ui_graph->setLocale(QLocale(QLocale::English, QLocale::UnitedKingdom));
		// ui_graph->legend->setVisible(true);
		// ui_graph->legend->setBrush(QBrush(QColor(255, 255, 255, 230)));
		// ui_graph->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom | Qt::AlignRight);

        for(int i=0; i<legend.size(); i++){
		    ui_graph->addGraph();
            // ui_graph->graph(i)->setPen(QPen(QColor(0, 255, 0)));
            ui_graph->graph(i)->setPen(color[i]);
		    ui_graph->graph(i)->setName(legend[i].c_str());
        }

		// ui_graph->addGraph();
		// ui_graph->graph(0)->setPen(QPen(QColor(0, 255, 0)));
		// ui_graph->graph(0)->setName("Y");
		// ui_graph->addGraph();
		// ui_graph->graph(1)->setPen(QPen(QColor(255, 0, 0)));
		// ui_graph->graph(1)->setName("X");
		// ui_graph->addGraph();
		// ui_graph->graph(2)->setPen(QPen(QColor(0, 0, 255)));
		// ui_graph->graph(2)->setName("Z");

		ui_graph->legend->setVisible(false);
		ui_graph->axisRect()->setAutoMargins(QCP::msNone);
		ui_graph->xAxis->setVisible(false);
		ui_graph->yAxis->setVisible(false);
		ui_graph->axisRect()->setMargins(QMargins(35, 10, 10, 20));

		QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
		timeTicker->setTimeFormat("%s");
		timeTicker->setFieldWidth(timeTicker->tuSeconds, 0.01);
		timeTicker->setTickCount(tick_count);
		ui_graph->xAxis->setTicker(timeTicker);

		ui_graph->axisRect()->setupFullAxesBox();
		ui_graph->yAxis->setRange(min_value, max_value);
        ui_graph->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
	}
    void MainWindow::Plot_example(QCustomPlot *ui_graph)
    {
        /* BASIC EXMAPLES */
        ui_graph->addGraph();
        ui_graph->graph(0)->setPen(QPen(Qt::blue)); // line color blue for first graph
        ui_graph->graph(0)->setBrush(QBrush(QColor(0, 0, 255, 20))); // first graph will be filled with translucent blue
        ui_graph->addGraph();
        ui_graph->graph(1)->setPen(QPen(Qt::red)); // line color red for second graph
        // generate some points of data (y0 for first, y1 for second graph):
        QVector<double> x(251), y0(251), y1(251);
        for (int i=0; i<251; ++i)
        {
            x[i] = i;
            y0[i] = qExp(-i/150.0)*qCos(i/10.0); // exponentially decaying cosine
            y1[i] = qExp(-i/150.0);              // exponential envelope
        }
        // configure right and top axis to show ticks but no labels:
        // (see QCPAxisRect::setupFullAxesBox for a quicker method to do this)
        ui_graph->xAxis2->setVisible(true);
        ui_graph->xAxis2->setTickLabels(false);
        ui_graph->yAxis2->setVisible(true);
        ui_graph->yAxis2->setTickLabels(false);
        // make left and bottom axes always transfer their ranges to right and top axes:
        connect(ui_graph->xAxis, SIGNAL(rangeChanged(QCPRange)), ui_graph->xAxis2, SLOT(setRange(QCPRange)));
        connect(ui_graph->yAxis, SIGNAL(rangeChanged(QCPRange)), ui_graph->yAxis2, SLOT(setRange(QCPRange)));
        // pass data points to graphs:
        ui_graph->graph(0)->setData(x, y0);
        ui_graph->graph(1)->setData(x, y1);
        // let the ranges scale themselves so graph 0 fits perfectly in the visible area:
        ui_graph->graph(0)->rescaleAxes();
        // same thing for graph 1, but only enlarge ranges (in case graph 1 is smaller than graph 0):
        ui_graph->graph(1)->rescaleAxes(true);
        // Note: we could have also just called customPlot->rescaleAxes(); instead
        // Allow user to drag axis ranges with mouse, zoom with mouse wheel and select graphs by clicking:
        ui_graph->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    }

    void MainWindow::Plot_Current(QCustomPlot *ui_graph, float val_1, float val_2, float val_3)
    {
        // ui_.plot_ft_l->yAxis->setRange(-3, 3);
        ui_graph->graph(0)->addData(qnode.Sim_time, val_1);
		ui_graph->graph(1)->addData(qnode.Sim_time, val_2);
		ui_graph->graph(2)->addData(qnode.Sim_time, val_3);
		ui_graph->graph(0)->rescaleValueAxis(true);
		ui_graph->graph(1)->rescaleValueAxis(true);
		ui_graph->graph(2)->rescaleValueAxis(true);
    }
    void MainWindow::Plot_Current(QCustomPlot *ui_graph, float val_1, float val_2)
    {
        // ui_.plot_ft_l->yAxis->setRange(-3, 3);
        ui_graph->graph(0)->addData(qnode.Sim_time, val_1);
		ui_graph->graph(1)->addData(qnode.Sim_time, val_2);
		ui_graph->graph(0)->rescaleValueAxis(true);
		ui_graph->graph(1)->rescaleValueAxis(true);
    }
    void MainWindow::Plot_Current(QCustomPlot *ui_graph, float val_1)
    {
        // ui_.plot_ft_l->yAxis->setRange(-3, 3);
        ui_graph->graph(0)->addData(qnode.Sim_time, val_1);
		ui_graph->graph(0)->rescaleValueAxis(true);
    }

	void MainWindow::Plot_replot(QCustomPlot *ui_graph)
	{
        ui_graph->legend->setVisible(true);
        QFont legendFont = font();  // start out with MainWindow's font..
        legendFont.setPointSize(9); // and make a bit smaller for legend
        ui_graph->legend->setFont(legendFont);
        ui_graph->legend->setBrush(QBrush(QColor(255,255,255,230)));

		ui_graph->xAxis->setRange(qnode.Sim_time, 1, Qt::AlignRight);
		ui_graph->replot();
	}

    void MainWindow::Plot_clear(QCustomPlot *ui_graph)
    {
        ui_graph->graph(0)->data().clear();
        ui_graph->graph(1)->data().clear();
        ui_graph->graph(2)->data().clear();
    }

	void MainWindow::on_plot_stop_button_clicked()
	{
		dataTimer->stop();
	}
	void MainWindow::on_plot_start_button_clicked()
	{
        if(mode_selected_ == false)
            RCLCPP_WARN(rclcpp::get_logger("PMSM Node"), "PLEASE SELECT CONTROL MODE");
        else
    		dataTimer->start(0.02);
	}
}
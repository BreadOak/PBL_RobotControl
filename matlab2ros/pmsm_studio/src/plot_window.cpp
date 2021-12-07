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
		// dataTimer->stop();

		// Plot_Init(ui.plot_pmsm, -50, 50, 10);
        Plot_example(ui.plot_pmsm);
        Plot_example(ui.plot_pmsm_2);
        Plot_example(ui.plot_pmsm_3);
	}

    void MainWindow::realtimeDataSlot()
    {
    }

	void MainWindow::Plot_Init(QCustomPlot *ui_graph, int min_value, int max_value, int tick_count)
	{
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

    void MainWindow::Plot_Current()
    {
    }

	void MainWindow::Plot_replot(QCustomPlot *ui_graph)
	{
		ui_graph->xAxis->setRange(key, 8, Qt::AlignRight);
		ui_graph->replot();
	}

}
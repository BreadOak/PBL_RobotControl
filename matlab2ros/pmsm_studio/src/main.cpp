/*
 * main.cpp
 *      Author: junyoung kim / lgkimjy
 */

#include <QtGui>
#include <QApplication>
#include "main_window.hpp"


int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    pmsm_studio::MainWindow w(argc, argv);
    w.show();
    // w.showFullScreen();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

    return result;
}

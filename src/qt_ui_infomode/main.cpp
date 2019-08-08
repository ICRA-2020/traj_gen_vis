#include "mainwindow.h"
#include <QApplication>
#include <QtGui>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QNode qnode(argc,argv,"planner_client");
    MainWindow w(&qnode);

    w.show();
    a.connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()));

    return a.exec();
}

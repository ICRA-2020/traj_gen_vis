#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPixmap>
#include <QSettings>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);


    // intial configuration

    // logos
    QPixmap pix_larr("/home/jbs/catkin_ws/src/traj_gen/qt_ui/resources/LARR.jpg");
    int w = ui->label_larr->width();
    int h = ui->label_larr->height();
    ui->label_larr->setPixmap(pix_larr.scaled(w,h,Qt::KeepAspectRatio));

    QPixmap pix_larr2("/home/jbs/catkin_ws/src/traj_gen/qt_ui/resources/maxresdefault.jpg");
    int w2 = ui->label_larr2->width();
    int h2 = ui->label_larr2->height();
    ui->label_larr2->setPixmap(pix_larr2.scaled(w2,h2,Qt::KeepAspectRatio));

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_ros_clicked()
{

}

void MainWindow::on_pushButton_waypoint_clicked()
{

}

void MainWindow::on_pushButton_trajectory_clicked()
{

}

void MainWindow::on_pushButton_simulation_clicked()
{

}

void MainWindow::on_pushButton_save_clicked()
{

}

void MainWindow::on_pushButton_load_clicked()
{

}

void MainWindow::on_pushButton_clear_clicked()
{

}

void MainWindow::on_pushButton_undo_clicked()
{

}

void MainWindow::on_pushButton_chaser_clicked()
{

}

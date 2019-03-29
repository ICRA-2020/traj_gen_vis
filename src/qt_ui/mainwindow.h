#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qnode.h"
#include <unistd.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QNode* qnode,QWidget *parent = 0);
    ~MainWindow();
    void closeEvent(QCloseEvent * event);
    void ReadSettings(); 
    void WriteSettings();     

private Q_SLOTS:
    void on_pushButton_chaser_clicked();

    void on_pushButton_ros_clicked();

    void on_pushButton_waypoint_clicked();

    void on_pushButton_trajectory_clicked();

    void on_pushButton_simulation_clicked();

    void on_pushButton_save_clicked();

    void on_pushButton_load_clicked();

    void on_pushButton_clear_clicked();

    void on_pushButton_undo_clicked();

    void on_pushButton_one_shot_clicked();


    void textEdit_write(QString);

private:
    Ui::MainWindow *ui;
    QNode* qnode;



};

#endif // MAINWINDOW_H

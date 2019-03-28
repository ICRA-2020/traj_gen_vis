#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_pushButton_chaser_clicked();

private Q_SLOT:
    void on_pushButton_ros_clicked();

    void on_pushButton_waypoint_clicked();

    void on_pushButton_trajectory_clicked();

    void on_pushButton_simulation_clicked();

    void on_pushButton_save_clicked();

    void on_pushButton_load_clicked();

    void on_pushButton_clear_clicked();

    void on_pushButton_undo_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H

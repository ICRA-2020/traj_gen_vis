/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QTextEdit>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *label_larr;
    QLabel *label_larr2;
    QLabel *label;
    QPushButton *pushButton_ros;
    QPushButton *pushButton_waypoint;
    QPushButton *pushButton_trajectory;
    QPushButton *pushButton_simulation;
    QTextEdit *textEdit_board;
    QPushButton *pushButton_save;
    QLineEdit *lineEdit_logging_dir;
    QLabel *label_2;
    QLabel *label_3;
    QLineEdit *lineEdit_target_trajectory;
    QPushButton *pushButton_load;
    QPushButton *pushButton_clear;
    QPushButton *pushButton_undo;
    QLabel *label_4;
    QLineEdit *lineEdit_tf;
    QPushButton *pushButton_chaser;
    QMenuBar *menuBar;
    QMenu *menuAuto_chaser;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(340, 675);
        MainWindow->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255)"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        label_larr = new QLabel(centralWidget);
        label_larr->setObjectName(QString::fromUtf8("label_larr"));
        label_larr->setGeometry(QRect(30, 470, 131, 61));
        label_larr2 = new QLabel(centralWidget);
        label_larr2->setObjectName(QString::fromUtf8("label_larr2"));
        label_larr2->setGeometry(QRect(200, 470, 131, 61));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 540, 281, 81));
        pushButton_ros = new QPushButton(centralWidget);
        pushButton_ros->setObjectName(QString::fromUtf8("pushButton_ros"));
        pushButton_ros->setGeometry(QRect(40, 20, 131, 71));
        pushButton_ros->setStyleSheet(QString::fromUtf8("background-color:rgba(170, 255, 255, 80)"));
        pushButton_ros->setCheckable(true);
        pushButton_waypoint = new QPushButton(centralWidget);
        pushButton_waypoint->setObjectName(QString::fromUtf8("pushButton_waypoint"));
        pushButton_waypoint->setGeometry(QRect(180, 60, 131, 31));
        pushButton_waypoint->setStyleSheet(QString::fromUtf8("background-color: rgba(255, 170, 255, 100)"));
        pushButton_waypoint->setCheckable(true);
        pushButton_trajectory = new QPushButton(centralWidget);
        pushButton_trajectory->setObjectName(QString::fromUtf8("pushButton_trajectory"));
        pushButton_trajectory->setGeometry(QRect(180, 100, 131, 71));
        pushButton_simulation = new QPushButton(centralWidget);
        pushButton_simulation->setObjectName(QString::fromUtf8("pushButton_simulation"));
        pushButton_simulation->setGeometry(QRect(40, 100, 131, 71));
        pushButton_simulation->setStyleSheet(QString::fromUtf8("background-color:rgba(170, 255, 255, 80)"));
        textEdit_board = new QTextEdit(centralWidget);
        textEdit_board->setObjectName(QString::fromUtf8("textEdit_board"));
        textEdit_board->setGeometry(QRect(50, 330, 241, 131));
        pushButton_save = new QPushButton(centralWidget);
        pushButton_save->setObjectName(QString::fromUtf8("pushButton_save"));
        pushButton_save->setGeometry(QRect(50, 260, 61, 27));
        lineEdit_logging_dir = new QLineEdit(centralWidget);
        lineEdit_logging_dir->setObjectName(QString::fromUtf8("lineEdit_logging_dir"));
        lineEdit_logging_dir->setGeometry(QRect(40, 200, 261, 21));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(40, 180, 211, 17));
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(40, 220, 191, 17));
        lineEdit_target_trajectory = new QLineEdit(centralWidget);
        lineEdit_target_trajectory->setObjectName(QString::fromUtf8("lineEdit_target_trajectory"));
        lineEdit_target_trajectory->setGeometry(QRect(40, 240, 261, 21));
        pushButton_load = new QPushButton(centralWidget);
        pushButton_load->setObjectName(QString::fromUtf8("pushButton_load"));
        pushButton_load->setGeometry(QRect(110, 260, 61, 27));
        pushButton_clear = new QPushButton(centralWidget);
        pushButton_clear->setObjectName(QString::fromUtf8("pushButton_clear"));
        pushButton_clear->setGeometry(QRect(170, 260, 61, 27));
        pushButton_undo = new QPushButton(centralWidget);
        pushButton_undo->setObjectName(QString::fromUtf8("pushButton_undo"));
        pushButton_undo->setGeometry(QRect(230, 260, 61, 27));
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(40, 300, 191, 17));
        lineEdit_tf = new QLineEdit(centralWidget);
        lineEdit_tf->setObjectName(QString::fromUtf8("lineEdit_tf"));
        lineEdit_tf->setGeometry(QRect(230, 300, 61, 21));
        pushButton_chaser = new QPushButton(centralWidget);
        pushButton_chaser->setObjectName(QString::fromUtf8("pushButton_chaser"));
        pushButton_chaser->setGeometry(QRect(180, 20, 131, 31));
        pushButton_chaser->setStyleSheet(QString::fromUtf8("background-color:rgba(85, 170, 255, 100)"));
        pushButton_chaser->setCheckable(true);
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 340, 25));
        menuAuto_chaser = new QMenu(menuBar);
        menuAuto_chaser->setObjectName(QString::fromUtf8("menuAuto_chaser"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuAuto_chaser->menuAction());

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "AutoChaser", 0, QApplication::UnicodeUTF8));
        label_larr->setText(QString());
        label_larr2->setText(QString());
        label->setText(QApplication::translate("MainWindow", "Maintainer: Boseong Felipe Jeon \n"
" junbs95@gmail.com \n"
" https://github.com/icsl-Jeon", 0, QApplication::UnicodeUTF8));
        pushButton_ros->setText(QApplication::translate("MainWindow", "ROS connect", 0, QApplication::UnicodeUTF8));
        pushButton_waypoint->setText(QApplication::translate("MainWindow", "select waypoints", 0, QApplication::UnicodeUTF8));
        pushButton_trajectory->setText(QApplication::translate("MainWindow", "target trajectory\n"
" generation", 0, QApplication::UnicodeUTF8));
        pushButton_simulation->setText(QApplication::translate("MainWindow", "simulation\n"
" start", 0, QApplication::UnicodeUTF8));
        pushButton_save->setText(QApplication::translate("MainWindow", "save", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "simulation logging directory", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "target global trajectory", 0, QApplication::UnicodeUTF8));
        pushButton_load->setText(QApplication::translate("MainWindow", "load", 0, QApplication::UnicodeUTF8));
        pushButton_clear->setText(QApplication::translate("MainWindow", "clear", 0, QApplication::UnicodeUTF8));
        pushButton_undo->setText(QApplication::translate("MainWindow", "undo", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "simluation total time [sec]", 0, QApplication::UnicodeUTF8));
        pushButton_chaser->setText(QApplication::translate("MainWindow", "select chaser init", 0, QApplication::UnicodeUTF8));
        menuAuto_chaser->setTitle(QApplication::translate("MainWindow", "auto_chaser", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H

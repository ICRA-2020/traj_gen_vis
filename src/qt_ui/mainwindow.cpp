#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPixmap>
#include <QSettings>
MainWindow::MainWindow(QNode* qnode,QWidget *parent) :
    QMainWindow(parent),qnode(qnode),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);


    // intial configuration

    // logos
    string cd = __FILE__;
    cd.erase(cd.end()-14,cd.end());
    cout<<"current directory: "<<cd<<endl;

    QPixmap pix_larr((cd + "/resources/LARR.jpg").c_str());
    int w = ui->label_larr->width();
    int h = ui->label_larr->height();
    ui->label_larr->setPixmap(pix_larr.scaled(w,h,Qt::KeepAspectRatio));

    QPixmap pix_larr2((cd +"/resources/maxresdefault.jpg").c_str());
    int w2 = ui->label_larr2->width();
    int h2 = ui->label_larr2->height();
    ui->label_larr2->setPixmap(pix_larr2.scaled(w2,h2,Qt::KeepAspectRatio));

    // checkable 
    ui->pushButton_simulation->setStyleSheet("QPushButton:checked{background-color: rgba(100, 20, 20,50); }");
    
    QObject::connect(qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(qnode,SIGNAL(writeOnBoard(QString)),this,SLOT(textEdit_write(QString)));

    // load settings 
    ReadSettings();

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_ros_clicked()
{

    
    if(qnode->on_init()){
        ui->textEdit_board->append("ros connected.");
        qnode->is_connected = true;

    }else{
        ui->textEdit_board->append("failed. retry");
    }
}

void MainWindow::on_pushButton_waypoint_clicked()
{
    if(ui->pushButton_waypoint->isChecked()){        
        ui->textEdit_board->append("please select waypoints : /target_waypoints ");
        qnode->target_manager.is_insert_permit = true;

    }else{

        ui->textEdit_board->append("finishing waypoints selection");
        qnode->target_manager.is_insert_permit = false;
    }

}

void MainWindow::on_pushButton_chaser_clicked(){

    if(ui->pushButton_chaser->isChecked()){      
        qnode->chaser_wrapper.objects_handler.is_insert_permit = true;  
        ui->textEdit_board->append("please select chaser init pose: /chaser_init_pose");
    }else{
        ui->textEdit_board->append("finishing chaser spawning selection");
        qnode->chaser_wrapper.objects_handler.is_insert_permit = false;  

    }

}

void MainWindow::on_pushButton_trajectory_clicked()
{
    
    double tf = atof(ui->lineEdit_tf->text().toStdString().c_str());
    if(qnode->target_manager.global_path_generate(tf))
        textEdit_write("target trajectory obtainted");
    else
        textEdit_write("target trajectory failed");    
}

void MainWindow::on_pushButton_simulation_clicked()
{
    if(ui->pushButton_simulation->isChecked()){ 

        if(qnode->chaser_wrapper.objects_handler.is_chaser_spawned and qnode->target_manager.is_path){
            ui->textEdit_board->append("move target..");
            // simulation end time 
            qnode->simulation_end_time = atof(ui->lineEdit_tf->text().toStdString().c_str());
            qnode->is_in_session = true;
            qnode->button_click_time = ros::Time::now();        
        }
        else{
            ui->textEdit_board->append("target path not obtained or no chaser spawned.");
        }
    }else{
        ui->textEdit_board->append("stop target.");
        qnode->is_in_session = false;
        qnode->previous_elapsed = (ros::Time::now() - qnode->button_click_time).toSec() + qnode->previous_elapsed; // total elasped time
    }
}

void MainWindow::on_pushButton_save_clicked()
{

    // file write
    std::ofstream wnpt_file;
    string filename = ui->lineEdit_target_trajectory->text().toStdString();
    wnpt_file.open(filename);

    if(wnpt_file.is_open()){
        for(auto it = qnode->target_manager.queue.begin();it<qnode->target_manager.queue.end();it++){
            wnpt_file<<std::to_string(it->pose.position.x)<<","<<std::to_string(it->pose.position.y)<<","<<std::to_string(it->pose.position.z)<<"\n";
        }
        wnpt_file.close();

        
        ui->textEdit_board->append(QString((string("to ") + GetCurrentWorkingDir()+"/" + filename + string(", written")).data()));

    }else
        ui->textEdit_board->append(QString("file not written."));

}

void MainWindow::on_pushButton_load_clicked()
{
    // file read : queue wil be filled with these

    string filename = ui->lineEdit_target_trajectory->text().toStdString();
    std::ifstream infile;
    infile.open(filename);
    if(infile.is_open())
        ui->textEdit_board->append(QString("pnts reading.."));
    else
    {
        ui->textEdit_board->append(QString("could not open file."));
        return;
    }

    std::vector<geometry_msgs::PoseStamped> queue_replace;

    while (! infile.eof()){
        std::string line;
        getline(infile, line); // if no delimiter given, new line is that
        // std::cout<<line<<std::endl;
        std::stringstream stream(line);
        std::string val;
        int xyz_idx = 0;
        geometry_msgs::PoseStamped wpnt;

        while(! stream.eof()) {
            getline(stream, val, ',');
            if(xyz_idx == 0)
                wpnt.pose.position.x = atof(val.c_str());
            else if(xyz_idx == 1)
                wpnt.pose.position.y = atof(val.c_str());
            else
                wpnt.pose.position.z = atof(val.c_str());
            xyz_idx ++;
        }
        queue_replace.push_back(wpnt);
        // std::cout<< wpnt.pose.position.x <<" , "<< wpnt.pose.position.y <<" , "<<wpnt.pose.position.z<<std::endl;
    }

    queue_replace.pop_back();
    qnode->target_manager.queue_file_load(queue_replace);
}

void MainWindow::on_pushButton_clear_clicked()
{
    qnode->target_manager.clear_waypoint();
}

void MainWindow::on_pushButton_undo_clicked()
{
    qnode->target_manager.pop_waypoint();
};

void MainWindow::on_pushButton_one_shot_clicked(){
    ui->textEdit_board->append("one shot simulatoin requested.");
    double tf = atoi(ui->lineEdit_tf->text().toStdString().c_str());    
    if (qnode->trigger_one_shot(tf))
        ui->textEdit_board->append("chasing path obtained");
    else 
        ui->textEdit_board->append("chasing failed");    
};

void MainWindow::textEdit_write(QString line){    
    ui->textEdit_board->append(line);
};


void MainWindow::ReadSettings(){
    QSettings settings("auto_chaser", qnode->nodeName().c_str());

    // setting names    
    QString filename_logging = settings.value("filename_logging",QString("path_saved.txt")).toString();
    QString filename_waypoints = settings.value("filename_waypoints",QString("path_saved.txt")).toString();
    QString simulation_tf = settings.value("tf", QString("20")).toString();

    
    // fill with previous settings 
    ui->lineEdit_logging_dir->setText(filename_logging);
    ui->lineEdit_tf->setText(simulation_tf);
    ui->lineEdit_target_trajectory->setText(filename_waypoints);
    
}

void MainWindow::WriteSettings(){

    QSettings settings("auto_chaser", qnode->nodeName().c_str());
    
    settings.setValue("geometry", geometry());
    settings.setValue("windowState", saveState());

    settings.setValue("filename_logging",ui->lineEdit_logging_dir->text());
    settings.setValue("tf",ui->lineEdit_tf->text());
    settings.setValue("filename_waypoints",ui->lineEdit_target_trajectory->text());


}

void MainWindow::closeEvent(QCloseEvent *event){

        qnode->shutdown();
        WriteSettings();
        QMainWindow::closeEvent(event);
}

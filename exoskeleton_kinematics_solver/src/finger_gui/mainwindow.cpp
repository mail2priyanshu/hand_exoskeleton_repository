#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QSocketNotifier> // for communication with other apps

#include <QDebug> // to be erased

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Set table header
    TableModel = new QStandardItemModel();
    TableModel->setHorizontalHeaderItem(0, new QStandardItem(QString(" Angle ")));
    TableModel->setHorizontalHeaderItem(1, new QStandardItem(QString(" Uncertainty ")));

    TableModel->setVerticalHeaderItem(0, new QStandardItem(" MCP Adduction " ) );
    TableModel->setVerticalHeaderItem(1, new QStandardItem(" MCP Flexion " ) );
    TableModel->setVerticalHeaderItem(2, new QStandardItem(" PIP Flexion " ) );
    TableModel->setVerticalHeaderItem(3, new QStandardItem(" DIP Flexion " ) );

    ui->tableView->setModel(TableModel);

    // Set Visualization Widget
//    TableModel = new QStandardItemModel();

}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_start_button_clicked()
{

    // prepare to receive the data
    data_receiver.initialization();

    // try to open the capture program.
    process = new QProcess();
    process->start("./pose_estimation_motion_capture/estimate_pose_real_time");

    // if fail to connect show the message
    connect(process,SIGNAL(error(QProcess::ProcessError)),this,SLOT(motion_capture_failed()));

    // timer to GUI update
    connect(&update_Timer,SIGNAL(timeout()),this,SLOT(update_GUI()));
    update_Timer.start(40);

    // load model parameter data
    ui->widget->IndxFinger.Read_Finger_Param_From_File();
}

void MainWindow::update_GUI()
{
    // update table
//    qDebug()<<data_receiver.pose_data[0];
//    ui->tableView->model()->setData(ui->tableView->model()->index(0,0),data_receiver.pose_data[0]);
//    ui->tableView->model()->setData(ui->tableView->model()->index(1,0),data_receiver.pose_data[1]);
//    ui->tableView->model()->setData(ui->tableView->model()->index(2,0),data_receiver.pose_data[2]);
//    ui->tableView->model()->setData(ui->tableView->model()->index(3,0),data_receiver.pose_data[3]);
//    ui->tableView->model()->setData(ui->tableView->model()->index(0,1),data_receiver.pose_data[4]);
//    ui->tableView->model()->setData(ui->tableView->model()->index(1,1),data_receiver.pose_data[5]);
//    ui->tableView->model()->setData(ui->tableView->model()->index(2,1),data_receiver.pose_data[6]);
//    ui->tableView->model()->setData(ui->tableView->model()->index(3,1),data_receiver.pose_data[7]);

    ui->tableView->model()->setData(ui->tableView->model()->index(1,0),data_receiver.pose_data[0]);
    ui->tableView->model()->setData(ui->tableView->model()->index(2,0),data_receiver.pose_data[1]);
    ui->tableView->model()->setData(ui->tableView->model()->index(3,0),data_receiver.pose_data[2]);



    // update OpenGL
//    ui->widget->IndxFinger.Ang_MCP_Add = data_receiver.pose_data[1];
//    ui->widget->IndxFinger.Ang_MCP_Flex = data_receiver.pose_data[0];
//    ui->widget->IndxFinger.Ang_PIP_Flex = data_receiver.pose_data[2];
//    ui->widget->IndxFinger.Ang_DIP_Flex= data_receiver.pose_data[3];
//    ui->widget->IndxFinger.Cov_MCP_Flex= data_receiver.pose_data[5];
//    ui->widget->IndxFinger.Cov_MCP_Add= data_receiver.pose_data[4];
//    ui->widget->IndxFinger.Cov_PIP_Flex= data_receiver.pose_data[6];
//    ui->widget->IndxFinger.Cov_DIP_Flex= data_receiver.pose_data[7];

    ui->widget->IndxFinger.Ang_MCP_Add = 0;
    ui->widget->IndxFinger.Ang_MCP_Flex = data_receiver.pose_data[0];
    ui->widget->IndxFinger.Ang_PIP_Flex = data_receiver.pose_data[1];
    ui->widget->IndxFinger.Ang_DIP_Flex= data_receiver.pose_data[2];

    ui->widget->IndxFinger.Update_Joint_Points();
    ui->widget->updateGL();

    update_status("Now finger data updating...");
}

void MainWindow::motion_capture_failed()
{
    update_status("Failed to initiate motion capture process!");
}


void MainWindow::update_status(QString string)
{
    ui->status_text_editor->setPlainText(string);
}

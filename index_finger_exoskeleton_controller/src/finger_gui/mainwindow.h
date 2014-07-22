#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QStandardItem> // for table view

#include <QProcess> // to execute realtime ang-estimation program
#include "data_recv.h" // to receive data from the other program
#include <QTimer>       // for update table and OpenGL


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

private:

    void update_status(QString string);

    QStandardItemModel *TableModel;
    QProcess *process;          // to execute realtime ang-estimation program
    Data_Recv data_receiver;    // to receive data from the other program
    QTimer update_Timer;


public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    Ui::MainWindow *ui;


private slots:
    void on_start_button_clicked();
    void update_GUI();
    void motion_capture_failed();


};

#endif // MAINWINDOW_H

#ifndef DATA_RECV_H
#define DATA_RECV_H

#include <QMainWindow>
#include <QObject>
#include <QSocketNotifier>
#include <cfinger.h>

class Data_Recv : public QObject
{
    Q_OBJECT
public:
    Data_Recv();
    ~Data_Recv();
    int K2UDataFifo;
    int Get_MarkerData();

//    CFinger received_finger;
    float pose_data[3];

private:
    QSocketNotifier *Data;


public slots:
    void Data_ReadHandler();
    void initialization();

signals:
//    void data_changed();
};

#endif // DATA_RECV_H

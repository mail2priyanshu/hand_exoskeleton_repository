#include "data_recv.h"
#include <iostream>
#include <string.h>
#include <QDebug>
#include <fcntl.h>


using namespace std;

Data_Recv::Data_Recv()
{
//    int i;
//    for(i=0;i<MARKER_COUNT;i++)
//    {

//        marker_data[i].x = 0;
//   marker_data[i].y = 0;
//      marker_data[i].z = 0;

//    }

}

void Data_Recv::Data_ReadHandler()
{
    read(K2UDataFifo, (char*)&pose_data, sizeof(pose_data));
//    qDebug()<<sizeof(pose_data);
//    qDebug()<<pose_data[0];
//    qDebug()<<sizeof(marker_data);
//    emit data_changed();
}

void Data_Recv::initialization()
{
    K2UDataFifo = open("/dev/rtf1", O_RDONLY | O_NONBLOCK);
    Data = new QSocketNotifier(K2UDataFifo,QSocketNotifier::Read);
    QObject::connect(Data, SIGNAL(activated(int)),this, SLOT(Data_ReadHandler()));
}

Data_Recv::~Data_Recv()
{
    close(K2UDataFifo);
}

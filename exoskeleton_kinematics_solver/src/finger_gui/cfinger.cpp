#define FILE_NAME_MODEL_PARAM "m.txt"


#include "cfinger.h"
#include <fstream>
#include <Eigen/Dense>
#include <GL/glut.h>
#include <qdebug.h>

using namespace std;
using namespace Eigen;

CFinger::CFinger()
{
    Len_Metacarpal = 60;
    Len_Dis_Phal = 23;

    Len_Prox_Phal = 40.0;
    Len_Mid_Phal = 30;

    Ang_MCP_Flex = 20*2*3.14/360;
    Ang_MCP_Add =  25*2*3.14/360;
    Ang_PIP_Flex = 10*2*3.14/360;;
    Ang_DIP_Flex = 0.4;

    Cov_MCP_Flex = 1e-4;
    Cov_PIP_Flex = 1e-4;
    Cov_DIP_Flex - 1e-4;

}


void CFinger::Read_Finger_Param_From_File(){

    // File reading
    ifstream File_Model_Param;
    double Data_from_file[29];

    File_Model_Param.open(FILE_NAME_MODEL_PARAM);
    for(int i=0;i<29;i++){
        File_Model_Param >> Data_from_file[i];
//        qDebug()<<Data_from_file[i];
    }


    File_Model_Param.close();

    // Assigning values
    Len_Prox_Phal = Data_from_file[27];
    Len_Mid_Phal = Data_from_file[28];

}

void CFinger::Update_Joint_Points(){

//    Vector3f U_Vector_MCP2PIP;
//    Vector3f U_Vector_PIP2DIP;
//    Vector3f U_Vector_DIP2Tip;
    Vector3f U_X_Dir;
    Vector3f temp1;
    Vector3f temp2;
    Vector3f temp3;
    float t;

    U_X_Dir << 1,0,0;

    Matrix3f M_MCP_Add;
    t = Ang_MCP_Add;
    M_MCP_Add <<    cos(t) , -sin(t) , 0,
                    sin(t) , cos(t) , 0,
                    0 , 0 , 1;


    Matrix3f M_MCP_Flex;
    t = Ang_MCP_Flex;
    M_MCP_Flex <<   cos(t) , 0 , sin(t),
                    0 , 1 , 0,
                    -sin(t), 0,  cos(t);

    Matrix3f M_PIP_Flex;
    t = Ang_MCP_Flex + Ang_PIP_Flex;
    M_PIP_Flex <<   cos(t) , 0 , sin(t),
                    0 , 1 , 0,
                    -sin(t), 0,  cos(t);

    Matrix3f M_DIP_Flex;
    t = Ang_MCP_Flex + Ang_PIP_Flex + Ang_DIP_Flex;
    M_DIP_Flex <<   cos(t) , 0 , sin(t),
                    0 , 1 , 0,
                    -sin(t), 0,  cos(t);



    MCP_Point[0]= 0;
    MCP_Point[1]= 0;
    MCP_Point[2]= 0;

    Metacarpal_Point[0] = -Len_Metacarpal;
    Metacarpal_Point[1] = 0;
    Metacarpal_Point[2] = 0;



    temp1= M_MCP_Flex*M_MCP_Add * Len_Prox_Phal *U_X_Dir;

    PIP_Point[0] = temp1[0];
    PIP_Point[1] = temp1[1];
    PIP_Point[2] = temp1[2];

    temp2= temp1 + M_PIP_Flex*M_MCP_Add * Len_Mid_Phal *U_X_Dir ;

    DIP_Point[0] = temp2[0];
    DIP_Point[1] = temp2[1];
    DIP_Point[2] = temp2[2];

    temp3= temp2 + M_DIP_Flex*M_MCP_Add * Len_Dis_Phal *U_X_Dir ;

    Tip_Point[0] = temp3[0];
    Tip_Point[1] = temp3[1];
    Tip_Point[2] = temp3[2];
}

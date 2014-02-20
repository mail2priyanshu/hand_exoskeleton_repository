#ifndef CFINGER_H
#define CFINGER_H

#include <GL/glut.h>

class CFinger
{
public:
    CFinger();

    // The below two variables are just for visualization, and they are pre-given values
    float Len_Metacarpal;
    float Len_Dis_Phal;

    // read from file
    float Len_Prox_Phal;
    float Len_Mid_Phal;

    // estimation value
    float Ang_MCP_Flex;
    float Ang_MCP_Add;
    float Ang_PIP_Flex;
    float Ang_DIP_Flex;

    float Cov_MCP_Flex;
    float Cov_MCP_Add;
    float Cov_PIP_Flex;
    float Cov_DIP_Flex;

    // for visualization, will be calculated
    float Metacarpal_Point[3];
    float DIP_Point[3];
    float MCP_Point[3];
    float PIP_Point[3];
    float Tip_Point[3];


    // important
    void Read_Finger_Param_From_File();
    void Update_Joint_Points();
    

};

#endif // CFINGER_H

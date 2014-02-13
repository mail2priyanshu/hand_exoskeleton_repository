// Exoskeleton Kinematics/Dynamics header files
#ifndef EXO_FINGER
#define EXO_FINGER
#include "exo_finger.h"
#endif

// Constructor initializing finger and exoskeleton kinematic/dynamic parameters
exo_finger::exo_finger()
{

    // Geometric Parameters Lengths
    //Mok
    double pixeltomm = 58.65/(130.32*1000);
    x_A = pixeltomm*6.15;
    y_A = -pixeltomm*73.96;
    l_AB = sqrt(x_A*x_A+y_A*y_A);

    l_BC = pixeltomm*110.67;

    l_CD = pixeltomm*50.89;
    l_AH = pixeltomm*95.32;

    l_CE = pixeltomm*90.6;
    l_EF = pixeltomm*86.33;
    l_FG = pixeltomm*47.05;
    l_GH = pixeltomm*24.7;
    l_GK = pixeltomm*22.64;
    l_FI = pixeltomm*60.07;
    l_IJ = pixeltomm*35.5;

    // Evaluating model parameters
    // PIP Chain
    l_HF = sqrt(l_GH*l_GH+l_FG*l_FG);
    t_HFG = atan(l_GH/l_FG);

    // DIP Chain
    t_GFK = atan(l_GK/l_FG);
    l_KF = sqrt(l_GK*l_GK+l_FG*l_FG);

}

// Destructor cleaning the exo_finger class members
exo_finger::~exo_finger()
{

}

bool solve_quad_eqn(double A, double B, double C, double *x)
{
    if(C!=B && A*A+B*B-C*C>0)
    {
        x[0] = 2*atan2(-A+sqrt(A*A+B*B-C*C),C-B);
        x[1] = 2*atan2(-A-sqrt(A*A+B*B-C*C),C-B); 
        return true;
    }
    else
        if(C==B)
        {
        x[0] = -(B+C)/(2*A);
        x[1] = x[0];
        return true;
    }
    else
        return false;
}


// Exoskeleton kinematics equations for MCP chain
bool exo_finger::exo_kin_eqns_mcp(double *x)
{

    if(solve_quad_eqn(x_A-l_BC*cos(exo_t1), l_BC*sin(exo_t1)-y_A, -l_CD, x))
        return true;
    else
        return false;
}

// Exoskeleton kinematics equations for PIP chain
bool exo_finger::exo_kin_eqns_pip(double *x)
{    

    if(solve_quad_eqn(2*l_CH*l_EF*sin(exo_t6_rel),2*l_CH*(l_EF*cos(exo_t6_rel)-l_CE),pow(l_CH,2)+pow(l_CE,2)+pow(l_EF,2)-pow(l_HF,2)-2*l_CE*l_EF*cos(exo_t6_rel), x))
      return true;
    else
        return false;

}

// Exoskeleton kinematics equations for PIP chain
bool exo_finger::exo_kin_eqns_dip(double *x)
{

    exo_t9 = PI + exo_t6 - exo_t9_rel;

    if(solve_quad_eqn(l_FI*cos(exo_t9)-l_KF*cos(exo_t7+t_GFK),
                      -l_FI*sin(exo_t9)+l_KF*sin(exo_t7+t_GFK),
                      l_IJ,x))
        return true;
    else
        return false;
}

// Function solving for exoskeleton kinematics
bool exo_finger::exo_kinematics(double *exo_t, double *estimates)
{
    bool fail_flag=false;
    exo_t1_rel = exo_t[0];
    exo_t5_rel = exo_t[1];
    exo_t6_rel = exo_t[2];
    exo_t9_rel = exo_t[3];
    exo_abad = exo_t[4];
    exo_t1 = 2*PI-exo_t1_rel;

//    cout<<"exo_t1_rel="<<exo_t1_rel*180/PI<<" exo_t6_rel="<<exo_t6_rel*180/PI<<" exo_t9_rel="<<exo_t9_rel*180/PI<<endl;

    double x[2], y[2];

    // MCP Chain
    if(exo_kin_eqns_mcp(x))
    {
        t_mcp = x[1];
        exo_t2 = t_mcp - PI/2;
        exo_x3 = sqrt(pow(l_BC*cos(exo_t1)+l_CD*sin(t_mcp)-x_A,2)+pow(l_BC*sin(exo_t1)-l_CD*cos(t_mcp)-y_A,2));        
//        cout<<"t1_r= "<<exo_t1_rel*180/PI<<", x3 ="<<exo_x3<<", mcp = "<<t_mcp*180/PI<<", t2 = "<<exo_t2*180/PI<<endl;
    }
    else
    {
        cout<<"Failed to solve MCP kinematics!"<<endl;
        fail_flag=true;
    }


    // PIP Chain
    l_DH = l_AH-exo_x3;
    l_CH = sqrt(l_CD*l_CD+l_DH*l_DH);
    t_HFG = atan(l_GH/l_FG);
    t_DCH = atan(l_DH/l_CD);
    if(exo_kin_eqns_pip(x))
    {

        exo_t5 = exo_t2+t_DCH-x[1];
        exo_t7 = atan2(l_CH*sin(exo_t2+t_DCH)-l_CE*sin(exo_t5)+l_EF*sin(exo_t5+exo_t6_rel),
                       l_CH*cos(exo_t2+t_DCH)-l_CE*cos(exo_t5)+l_EF*cos(exo_t5+exo_t6_rel))+t_HFG;
        exo_t6 = exo_t5+PI+exo_t6_rel;
        t_pip = exo_t7+PI/2;
        if(t_pip<0)
            t_pip = t_pip+2*PI;
//        cout<<"t5= "<<exo_t5*180/PI<<", t6= "<<exo_t6*180/PI<<", t7 ="<<exo_t7*180/PI<<", pip = "<<t_pip*180/PI<<endl;
    }
    else
    {
        cout<<"Failed to solve PIP kinematics!"<<endl;
        fail_flag=true;
    }

    // DIP Chain
    if(exo_kin_eqns_dip(x))
    {
        t_dip = x[0];
        if(t_dip<0)
            t_dip = t_dip+2*PI;
        exo_t10 = t_dip-PI/2;
        exo_x11 = sqrt(pow(l_FI*cos(exo_t9)+l_IJ*cos(exo_t10)-l_KF*cos(exo_t7+t_GFK),2)+pow(l_FI*sin(exo_t9)+l_IJ*sin(exo_t10)-l_KF*sin(exo_t7+t_GFK),2));
//        cout<<"t9= "<<exo_t9*180/PI<<", t10= "<<exo_t10*180/PI<<", x11 ="<<exo_x11<<", dip = "<<t_dip*180/PI<<endl;
    }
    else
    {
        cout<<"Failed to solve DIP kinematics!"<<endl;
        fail_flag=true;
    }

    if(!fail_flag)
    {
        estimates[0] = exo_t1;
        estimates[1] = exo_t2;
        estimates[2] = exo_x3;
        estimates[3] = t_mcp;
        estimates[4] = exo_t5;
        estimates[5] = exo_t6;
        estimates[6] = exo_t7;
        estimates[7] = t_pip;
        estimates[8] = exo_t9;
        estimates[9] = exo_t10;
        estimates[10] = exo_x11;
        estimates[11] = t_dip;
        return true;
    }
    else
        return false;
}

bool exo_finger::exo_calibration()
{

}

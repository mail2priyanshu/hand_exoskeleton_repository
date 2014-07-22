//PURPOSE : Class function definitions implementing the kinematics solution for the hand exoskeleton being developed in the ReNeu Robotics Lab at The University of Texas at Austin
//AUTHORS  : Priyanshu Agarwal
//CONTACT : mail2priyanshu@utexas.edu
//AFFILIATION : The University of Texas at Austin
//To DOs
//1. Add auto calibration functionality to automatically estimate the kinematic model parameters
//2.

/////////////////////////////////////////////////////////////////////
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
//    double pixeltomm = 58.65/(130.32*1000);
//    x_A = pixeltomm*6.15;
//    y_A = -pixeltomm*73.96;
//    l_AB = sqrt(x_A*x_A+y_A*y_A);
//
//    l_BC = pixeltomm*110.67;
//
//    l_CD = pixeltomm*50.89;
//    l_AH = pixeltomm*95.32;
//
//    l_CE = pixeltomm*90.6;
//    l_EF = pixeltomm*86.33;
//    l_FG = pixeltomm*47.05;
//    l_GH = pixeltomm*24.7;
//    l_GK = pixeltomm*22.64;
//    l_FI = pixeltomm*60.07;
//    l_IJ = pixeltomm*35.5;

//    // Priyanshu
    x_A = 0.007;
    y_A = -0.026;

    l_BC = 0.045;

    l_CD = 0.023;
    l_AH = 0.04473;

    l_CE = 0.03772;
    l_EF = 0.03912;
    l_FG = 0.02026;
    l_GH = 0.011;
    l_GK = 0.01;//0.007
    l_FI = 0.02617;
    l_IJ = 0.01968;


//    x_A-0.005,y_A-0.005,l_BC-0.005,l_CD-0.005,l_FG-0.005,l_GH-0.005,l_AH-0.005,
//    0.00894651,-0.0235032,0.0411893,0.0274564,0.0184318,0.0158719,0.0425704
//    0.011723,-0.0239534,0.0400118,0.0279916,0.0192485,0.0145635,0.0397619
    // Priyanshu Auto-calibrated
//    x_A = 0.00986423;
//    y_A = -0.0240355;

//    l_BC = 0.0414119;

//    l_CD = 0.028;
//    l_AH = 0.0421161;

//    l_CE = 0.03772;
//    l_EF = 0.03912;
//    l_FG = 0.0172878;
//    l_GH = 0.016;
//    l_GK = 0.007;
//    l_FI = 0.02617;
//    l_IJ = 0.01968;

    // Evaluating model parameters
    // MCP Chain
    l_AB = sqrt(x_A*x_A+y_A*y_A);

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

double exo_finger_func(unsigned n, const double *x, double *grad, void *finger_data)
{
    VectorNd Dx_mcp, Dy_mcp, Dx_pip, Dy_pip, exo_t1, exo_t2, exo_x3, t_mcp, exo_t5, exo_t6, exo_t7;
    VectorNd l_DH, l_CH, t_DCH;
    double x_A=x[0], y_A=x[1], l_BC=x[2], l_CD=x[3], l_FG=x[4], l_GH=x[5], l_AH=x[6];
    double E, l_HF, t_HFG;

    data_exo_finger index_finger_data = *(data_exo_finger *)(finger_data);

    // Evaluating model parameters
    // PIP Chain
    l_HF = sqrt(l_GH*l_GH+l_FG*l_FG);
    l_DH = l_AH*VectorNd::Ones()-exo_x3;
    l_CH = sqrt((l_CD*l_CD)*VectorNd::Ones().array()+l_DH.array()*l_DH.array());
    t_HFG = atan(l_GH/l_FG);
    for(unsigned i=0;i<SAMPLE_SIZE;i++)
        t_DCH[i] = atan(l_DH[i]/l_CD);

    //    l_DH = l_AH-exo_x3;
    //    t_DCH = atan(l_DH/l_CD);

    for(unsigned i=PARAMETER_SIZE;i<SAMPLE_SIZE+PARAMETER_SIZE;i++)
        exo_t2[i-PARAMETER_SIZE] = x[i];

    for(unsigned i=SAMPLE_SIZE+PARAMETER_SIZE;i<2*SAMPLE_SIZE+PARAMETER_SIZE;i++)
        exo_x3[i-(SAMPLE_SIZE+PARAMETER_SIZE)] = x[i];

    for(unsigned i=2*SAMPLE_SIZE+PARAMETER_SIZE;i<3*SAMPLE_SIZE+PARAMETER_SIZE;i++)
        t_mcp[i-(2*SAMPLE_SIZE+PARAMETER_SIZE)] = x[i];

    for(unsigned i=3*SAMPLE_SIZE+PARAMETER_SIZE;i<4*SAMPLE_SIZE+PARAMETER_SIZE;i++)
        exo_t7[i-(3*SAMPLE_SIZE+PARAMETER_SIZE)] = x[i];

    exo_t1 = 2*PI*VectorNd::Ones()-index_finger_data.exo_t1_rel;

    Dx_mcp = l_BC*cos(exo_t1.array())+l_CD*cos(exo_t2.array())-exo_x3.array()*cos(t_mcp.array())-x_A*VectorNd::Ones().array();
    Dy_mcp = l_BC*sin(exo_t1.array())+l_CD*sin(exo_t2.array())-exo_x3.array()*sin(t_mcp.array())-y_A*VectorNd::Ones().array();

    exo_t5 = PI*VectorNd::Ones() - (index_finger_data.exo_t1_rel+index_finger_data.exo_t5_rel);
    exo_t6 = exo_t5 + PI*VectorNd::Ones() + index_finger_data.exo_t6_rel;
    Dx_pip = index_finger_data.l_CE*cos(exo_t5.array())+index_finger_data.l_EF*cos(exo_t6.array())+l_HF*cos(exo_t7.array()
                                                                                                            -t_HFG*VectorNd::Ones().array())-l_CH.array()*cos(exo_t2.array()+t_DCH.array());
    Dy_pip = index_finger_data.l_CE*sin(exo_t5.array())+index_finger_data.l_EF*sin(exo_t6.array())+l_HF*sin(exo_t7.array()
                                                                                                            -t_HFG*VectorNd::Ones().array())-l_CH.array()*sin(exo_t2.array()+t_DCH.array());

//    cout<<"("<<Dx_mcp.dot(Dx_mcp)<<","<<Dy_mcp.dot(Dy_mcp)<<","<<Dx_pip.dot(Dx_pip)<<","<<Dy_pip.dot(Dy_pip)<<")"<<endl;
    E = Dx_mcp.dot(Dx_mcp)+Dy_mcp.dot(Dy_mcp)+Dx_pip.dot(Dx_pip)+Dy_pip.dot(Dy_pip);
    cout<<E<<endl;
    return E;
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
    {
        cout<<"A="<<A<<" B="<<B<<" C="<<C<<" A^2+B^2-C^2="<<A*A+B*B-C*C<<endl;
        return false;
    }
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

    double x[2], y[2];

    // MCP Chain
    if(exo_kin_eqns_mcp(x))
    {
        t_mcp = x[1];
        while(t_mcp<PI)
            t_mcp = t_mcp+2*PI;
        exo_t2 = t_mcp - PI/2;
        exo_x3 = sqrt(pow(l_BC*cos(exo_t1)+l_CD*sin(t_mcp)-x_A,2)+pow(l_BC*sin(exo_t1)-l_CD*cos(t_mcp)-y_A,2));        
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
        while(t_pip<PI)
            t_pip = t_pip+2*PI;
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
        while(t_dip<PI)
            t_dip = t_dip+2*PI;
        exo_t10 = t_dip-PI/2;
        exo_x11 = sqrt(pow(l_FI*cos(exo_t9)+l_IJ*cos(exo_t10)-l_KF*cos(exo_t7+t_GFK),2)+pow(l_FI*sin(exo_t9)+l_IJ*sin(exo_t10)-l_KF*sin(exo_t7+t_GFK),2));
    }
    else
    {        
        cout<<"Failed to solve DIP kinematics!"<<endl;
//        fail_flag=true;
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

 // Function to calibrate the index finger
bool exo_finger::exo_calibration()
{
    nlopt_opt opt; // optimization problem
    nlopt_result result; // optimization problem result
    double minf; // the minimum objective value, upon return
    unsigned i,j;
    Vector4Nd temp;
    data_exo_finger index_finger_data;

    ifstream recorded_calibration_data_file;
    ofstream calibrated_parameters_file, calibration_data_file;

//    double exo_t_rel[5], t, t_rel;
    double t;

    cout<<"Running optimization for calibration! Please be patient!"<<endl;
    // Evaluating model parameters
    // PIP Chain
    l_HF = sqrt(l_GH*l_GH+l_FG*l_FG);
    l_CH = sqrt(l_CD*l_CD+l_DH*l_DH);
    //    l_DH = l_AH-exo_x3;
    l_DH = l_AH-0.025;
    t_HFG = atan(l_GH/l_FG);
    t_DCH = atan(l_DH/l_CD);

    //    // DIP Chain
    //    t_GFK = atan(l_GK/l_FG);
    //    l_KF = sqrt(l_GK*l_GK+l_FG*l_FG);

    // Chain Optimization
    VectorNd exo_t2 = (VectorNd::LinSpaced(SAMPLE_SIZE,270,270-60))*PI/180;
    VectorNd exo_x3 = (VectorNd::LinSpaced(SAMPLE_SIZE,0.040,0.010));
    VectorNd t_mcp = (VectorNd::LinSpaced(SAMPLE_SIZE,360,360-60))*PI/180;
    VectorNd exo_t7 = (VectorNd::LinSpaced(SAMPLE_SIZE,270,270-60))*PI/180;

    // initializing lower bound for optimization
    temp << x_A-0.005,y_A-0.005,l_BC-0.005,l_CD-0.005,l_FG-0.005,l_GH-0.005,l_AH-0.005,
            exo_t2-VectorNd::Ones()*PI/6,exo_x3-VectorNd::Ones()*0.015,t_mcp-VectorNd::Ones()*PI/6,exo_t7-VectorNd::Ones()*PI/6;
    double lb[PARAMETER_SIZE+4*SAMPLE_SIZE];
    for(i=0;i<PARAMETER_SIZE+4*SAMPLE_SIZE;i++)
        lb[i] = temp[i];

    // initializing upper bound for optimization
    temp << x_A+0.005,y_A+0.005,l_BC+0.005,l_CD+0.005,l_FG+0.005,l_GH+0.005,l_AH+0.005,
            exo_t2+VectorNd::Ones()*PI/6,exo_x3+VectorNd::Ones()*0.015,t_mcp+VectorNd::Ones()*PI/6,exo_t7+VectorNd::Ones()*PI/6;
    double ub[PARAMETER_SIZE+4*SAMPLE_SIZE];
    for(i=0;i<PARAMETER_SIZE+4*SAMPLE_SIZE;i++)
        ub[i] = temp[i];

    // initializing absolute tolerance on optimization variables
    temp << 1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,VectorNd::Ones()*PI/180,VectorNd::Ones()*1e-4,VectorNd::Ones()*PI/180,VectorNd::Ones()*PI/180;
    double xtol[PARAMETER_SIZE+4*SAMPLE_SIZE];
    for(i=0;i<PARAMETER_SIZE+4*SAMPLE_SIZE;i++)
        xtol[i] = temp[i];

    recorded_calibration_data_file.open("encoder_data_4_sensors");
    calibration_data_file.open("calibration_data_file");

    for(i=0;i<SAMPLE_SIZE;i++)
    {
        for(j=0;j<100;j++)
            recorded_calibration_data_file>>t>>exo_t1_rel>>exo_t5_rel>>exo_t6_rel>>exo_t9_rel;

        index_finger_data.exo_t1_rel[i] = exo_t1_rel;
        index_finger_data.exo_t5_rel[i] = exo_t5_rel;
        index_finger_data.exo_t6_rel[i] = exo_t6_rel;
        calibration_data_file<<t<<"\t"<<exo_t1_rel<<"\t"<<exo_t5_rel<<"\t"<<exo_t6_rel<<"\t"<<exo_t9_rel<<endl;
    }

    recorded_calibration_data_file.close();
    calibration_data_file.close();

    index_finger_data.l_CE = l_CE;
    index_finger_data.l_EF = l_EF;

    //    opt = nlopt_create(NLOPT_LD_MMA, 2); /* algorithm and dimensionality */
    opt = nlopt_create(NLOPT_LN_COBYLA, PARAMETER_SIZE+4*SAMPLE_SIZE); /* algorithm and dimensionality */
    nlopt_set_lower_bounds(opt, lb);
    nlopt_set_upper_bounds(opt, ub);
    nlopt_set_min_objective(opt, exo_finger_func, (void *) &index_finger_data);
    nlopt_set_xtol_abs(opt, xtol);

    temp << x_A,y_A,l_BC,l_CD,l_FG,l_GH,l_AH,exo_t2,exo_x3,t_mcp,exo_t7;
    double x[PARAMETER_SIZE+4*SAMPLE_SIZE];
    for(i=0;i<PARAMETER_SIZE+4*SAMPLE_SIZE;i++)
        x[i] = temp[i];

    result = nlopt_optimize(opt, x, &minf);
    nlopt_destroy(opt);

    if (result < 0)
    {
        printf("nlopt failed due to %d!\n",result);
        return 0;
    }
    else
    {
        printf("Terminated due to %d! \n Found minimum at f(%g,%g,%g,%g,%g,%g,%g) = %0.10g\n",result,x[0],x[1],x[2],x[3],x[4],x[5],x[6],minf);
        x_A = x[0];
        y_A = x[1];
        l_BC = x[2];
        l_CD = x[3];
        l_FG = x[4];
        l_GH = x[5];
        l_AH = x[6];

        calibrated_parameters_file.open("calibrated_parameters_file");
        for(i=0;i<4*SAMPLE_SIZE+PARAMETER_SIZE;i++)
            calibrated_parameters_file<<x[i]<<"\t";
        cout<<"Done Calibrating!"<<endl;
        return 1;
    }

}

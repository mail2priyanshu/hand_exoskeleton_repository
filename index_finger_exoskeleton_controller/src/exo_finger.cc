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

// Constructor initializing finger and exoskeleton kinematic/dynamic parameters and the exoskeleton initial angles
exo_finger::exo_finger(Vector2d &Theta_r, double l_AH0)
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

//    //    // Priyanshu
//    x_A = 0.008;//0.007;
//    y_A = -0.03;//-0.026;

//    l_BC = 0.047;

//    l_CD = 0.025;//0.023;

////    l_AH = 0.048;//0.04473 // actual
//    l_AH = l_AH0;//0.057;//0.048;//0.04473 // assumed for the Jacobian to be non-singular

//    l_CE = 0.04;//0.03772;
//    l_EF = 0.035;//0.03912;
//    l_FG = 0.018;//0.02026;
//    l_GH = 0.015;//0.011;
//    l_GK = 0.007;
//    l_FI = 0.02617;
//    l_IJ = 0.01968;


    x_A = 0.0000;
    y_A = -0.0357;

    l_BC = 0.0424;

    l_CD = 0.0350;
    l_AH = 0.0400;

    l_CE = 0.04;
    l_EF = 0.035;
    l_FG = 0.0290;
    l_GH = 0.0290;
    l_GK = 0.007;
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


    // Finger Joint Torque Model Parameters
    // MCP Joint
    A_mcp = 1.01;
    B_mcp = 0.05;
    C_mcp = 3.39;
    D_mcp = 0.05;
    E_mcp = 70.96;
    F_mcp = 13.68;

    // PIP Joint
    A_pip = 0.6992;
    B_pip = 0.05;
    C_pip = 2.3469;
    D_pip = 0.05;
    E_pip = 36.2987;
    F_pip = 23.3078;

    // Limits on the finger joint torques
    Tau_finger_min(0) = -0.1; // MCP min
    Tau_finger_min(1) = -0.1; // PIP min

    Tau_finger_max(0) = 0.1; // MCP max
    Tau_finger_max(1) = 0.1; // PIP max

    // Error handling flags
    fail_flag = false;

    // Initial Exoskeleton Joint angles (zero torque)
//    T_rel0(0) = -27*PI/180;
////    T_rel0(1) = 78*PI/180;
//    T_rel0(1) = 60*PI/180;
    T_rel0(0) = -Theta_r(0); // exo_t1_rel = -t1
    T_rel0(1) = Theta_r(1);


    // Initializing SEA Parameters
    rj = 0.4635*0.0254; // m
    rm = 1.09375*0.0254; // m
    k(0) = 5632.25*0.5; // N/m (Mc master: 9435K84, 9435K81, 9657K304)
    k(1) = 1488.57*0.5;//1576.68*0.5;//1488.57*0.5; // N/m (Mc master: 9435K84, 9435K81, 9657K304)
    //(only half of the spring stiffness is effective)

    // Initializing controller parameters
    Tau_exo_I(0) = 0;
    Tau_exo_I(1) = 0;

//    kp_exo << 0, 10;//2;//11; (best w/o unstable)
//    kd_exo << 0.0, 0.001;
//    ki_exo << 0.0, 0.001;

    kp_fin << 0.0, 0.0;//8;//11; (best w/o unstable)
    kd_fin << 0.0, 0.0;
    ki_fin << 0.0, 0.0;

    kp_exo << 0.0, 0.0;
    kd_exo << 0.0, 0;
    ki_exo << 0.0, 0.0;

    kp_fbl << 0.0,0.0;
    kd_fbl << 0.0,0.0;
    ki_fbl << 0.0,0.0;

//    // Parameter Jacobian initialization
//    Jp<<0, 0,
//        0, 0;

    // State Covariance matrix initialization
    Sigma_p<<0.1, 0, 0,
             0, 0.1, 0,
             0, 0, 0.1;
    // Kalman Gain
    Kp<<0, 0, 0;

    // Measurement noise Covariance matrix initialization
    Rp=2;

    dl_AH = 0.01*l_AH;
    dl_HF = 0.01*l_HF;
    dl_CD = 0.01*l_CD;
    exo_t5_rel_hat = 0;
}

// Destructor cleaning the exo_finger class members
exo_finger::~exo_finger()
{

    // All variables destroyed automatically when they go out of scope!
}

// Function solving for the exact or the least squares solution of a quadratic/linear equation
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
            x[0] = 2*atan2(-A,(C-B));
            x[1] = x[0];
            //            cout<<"LS: "<<"A="<<A<<" B="<<B<<" C="<<C<<" A^2+B^2-C^2="<<A*A+B*B-C*C<<endl;
            //return false;
            return true;
        }
}

// Index Finger Exoskeleton kinematics equations for MCP chain
bool exo_finger::exo_kin_eqns_mcp(double *x)
{

    if(solve_quad_eqn(x_A-l_BC*cos(exo_t1), l_BC*sin(exo_t1)-y_A, -l_CD, x))
        return true;
    else
        return false;
}

// Index Finger Exoskeleton kinematics equations for PIP chain
bool exo_finger::exo_kin_eqns_pip(double *x)
{    

    if(solve_quad_eqn(2*l_CH*l_EF*sin(exo_t6_rel),2*l_CH*(l_EF*cos(exo_t6_rel)-l_CE),pow(l_CH,2)+pow(l_CE,2)+pow(l_EF,2)-pow(l_HF,2)-2*l_CE*l_EF*cos(exo_t6_rel), x))
        return true;
    else
        return false;

}

// Index Finger Exoskeleton kinematics equations for PIP chain
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

// Function solving for index finger exoskeleton kinematics
bool exo_finger::exo_kinematics(double *exo_t, double *estimates, Vector2d Theta_r_dot)
{
    bool fail_flag=false;
    exo_t1_rel = exo_t[0];
    exo_t5_rel = exo_t[1];
    exo_t6_rel = exo_t[2];
    exo_t9_rel = exo_t[3];
    exo_abad = exo_t[4];
    exo_t1 = 2*PI-exo_t1_rel;

    exo_t1_rel_dot = Theta_r_dot(0);
    exo_t6_rel_dot = Theta_r_dot(1);
    exo_t1_dot = -exo_t1_rel_dot;

    T_rel(0) = -exo_t1_rel;
    T_rel(1) = exo_t6_rel;
    T_rel_dot(0) = exo_t1_dot; // this is the way Jacobian is formulated
    T_rel_dot(1) = exo_t6_rel_dot; // this is the way Jacobian is formulated

    double x[2];



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
        exo_t5 = exo_t2+t_DCH-x[1]-2*PI;
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
//        estimates[12] = t_mcp_rel;
//        estimates[13] = t_pip_rel;
//        estimates[14] = t_dip_rel;

        // wrapping MCP joint angle between [0,2PI]
        while (t_mcp>2*PI)
            t_mcp -= 2*PI;
        while (t_mcp < 0)
            t_mcp += 2*PI;

        // wrapping PIP joint angle between [0,2PI]
        while (t_pip>2*PI)
            t_pip -= 2*PI;
        while (t_pip < 0)
            t_pip += 2*PI;

        // evaluating relaive MCP angle
        if(t_mcp>PI)
            t_mcp_rel = (2*PI- t_mcp)*180/PI;
        else
        {
            if(t_mcp<PI/2)
                t_mcp_rel = -t_mcp*180/PI;
            else
            {
                cout<<"t_mcp exceeding limits!";
                return false;
            }
        }

        // evaluating relaive PIP angle
        if(t_pip>PI)
        {
            t_pip_rel = (2*PI-t_pip)*180/PI-t_mcp_rel;
        }
        else
        {
            if(t_pip<PI/2)
            {
                t_pip_rel = -t_pip*180/PI-t_mcp_rel;
            }
            else
            {
//                cout<<"t_pip exceeding limits!";
                return false;
            }
        }

        // evaluating relaive DIP angle
        if(t_dip>PI)
        {
            t_dip_rel = (2*PI-t_dip)*180/PI-t_mcp_rel-t_pip_rel;
        }
        else
        {
            if(t_dip<PI/2)
            {
                t_dip_rel = -t_dip*180/PI-t_mcp_rel-t_pip_rel;
            }
            else
            {
                cout<<"t_dip exceeding limits!";
                return false;
            }
        }

        exo_jacobian();

        X_dot = J* T_rel_dot;
        exo_x3_dot = X_dot(0);
        t_mcp_dot = X_dot(1);
        exo_t5_dot = X_dot(2);
        t_pip_dot = X_dot(3);

        return true;
    }
    else
        return false;
}

// Function to calibrate the index finger
void exo_finger::exo_calibration()
{

}

// Evaluate the Jacobian of the index finger exoskeleton
void exo_finger::exo_jacobian()
{

    t193 = cos(exo_t1-t_mcp);
    t196 = t_pip-t_HFG;
    t197 = cos(t196);
    t198 = sin(t196);
    t200 = exo_t5+exo_t6_rel;
    t201 = cos(exo_t1);
    t202 = cos(t_mcp);
    t203 = 1.0/l_CD;
    t204 = l_AH-exo_x3;
    t205 = t203*t204;
    t206 = atan(t205);
    t207 = t_mcp+t206;
    t208 = exo_x3*exo_x3;
    t209 = cos(t207);
    t210 = sin(t_mcp);
    t211 = sin(t207);
    t212 = sin(exo_t1);
    t213 = l_AH*l_AH;
    t214 = -exo_t5+t_pip-exo_t6_rel-t_HFG;
    t215 = cos(t214);
    t216 = cos(exo_t5);
    t217 = sin(exo_t5);
    t218 = cos(t200);
    t219 = l_EF*t197*t218;
    t220 = sin(t200);
    t221 = l_EF*t198*t220;
    t222 = t219+t221-l_CE*t197*t216-l_CE*t198*t217;
    t223 = 1.0/t222;
    t224 = exo_t5-t_pip+t_HFG;
    t225 = cos(t224);
    J(0,0) = -l_BC*sin(exo_t1-t_mcp)+l_BC*l_CD*1.0/exo_x3*t193;
    J(1,0) = l_BC*1.0/exo_x3*t193;
    J(2,0) = l_BC*1.0/exo_x3*t223*(t198*t201*t202*t208*t209-t197*t201*t202*t208*t211+t198*t201*t202*t209*
                                   t213+t197*t201*t208*t209*t210*2.0-t197*t202*t208*t209*t212*2.0-
                                   t197*t201*t202*t211*t213+t198*t201*t208*t210*t211*2.0-t198*t202*
                                   t208*t211*t212*2.0+t198*t208*t209*t210*t212-t197*t208*t210*t211*
                                   t212+t198*t209*t210*t212*t213-t197*t210*t211*t212*t213-l_AH*l_CD*
                                   t197*t201*t202*t209*2.0-l_AH*l_CD*t198*t201*t202*t211*2.0-l_AH*l_CD*
                                   t197*t209*t210*t212*2.0-l_AH*l_CD*t198*t210*t211*t212*2.0-
                                   l_AH*exo_x3*t198*t201*t202*t209*2.0+l_AH*exo_x3*t197*t201*t202*t211*2.0-
                                   l_AH*exo_x3*t197*t201*t209*t210*2.0+l_AH*exo_x3*t197*t202*t209*t212*2.0-
                                   l_AH*exo_x3*t198*t201*t210*t211*2.0+l_AH*exo_x3*t198*t202*t211*t212*2.0+
                                   l_CD*exo_x3*t197*t201*t202*t209*2.0-l_AH*exo_x3*t198*t209*t210*t212*2.0+
                                   l_CD*exo_x3*t198*t201*t202*t211*2.0+l_AH*exo_x3*t197*t210*t211*t212*2.0-
                                   l_CD*exo_x3*t198*t201*t209*t210+l_CD*exo_x3*t198*t202*t209*t212+
                                   l_CD*exo_x3*t197*t201*t210*t211-l_CD*exo_x3*t197*t202*t211*t212+
                                   l_CD*exo_x3*t197*t209*t210*t212*2.0+l_CD*exo_x3*t198*t210*t211*t212*2.0);
    J(3,0) = -(l_BC*1.0/exo_x3*t223*(l_CE*t201*t202*t208*t209*t216+l_CE*t201*t202*t208*t211*t217+
                                     l_CE*t201*t202*t209*t213*t216-l_CE*t201*t208*t209*t210*t217*2.0+
                                     l_CE*t202*t208*t209*t212*t217*2.0+l_CE*t201*t202*t211*t213*t217+
                                     l_CE*t201*t208*t210*t211*t216*2.0-l_CE*t202*t208*t211*t212*t216*2.0+
                                     l_CE*t208*t209*t210*t212*t216-l_EF*t201*t202*t208*t209*t218+
                                     l_CE*t208*t210*t211*t212*t217+l_CE*t209*t210*t212*t213*t216-
                                     l_EF*t201*t202*t209*t213*t218+l_CE*t210*t211*t212*t213*t217-
                                     l_EF*t201*t202*t208*t211*t220+l_EF*t201*t208*t209*t210*t220*2.0-
                                     l_EF*t201*t208*t210*t211*t218*2.0-l_EF*t202*t208*t209*t212*t220*2.0+
                                     l_EF*t202*t208*t211*t212*t218*2.0-l_EF*t208*t209*t210*t212*t218-
                                     l_EF*t201*t202*t211*t213*t220-l_EF*t209*t210*t212*t213*t218-
                                     l_EF*t208*t210*t211*t212*t220-l_EF*t210*t211*t212*t213*t220+
                                     l_AH*l_CD*l_CE*t201*t202*t209*t217*2.0-l_AH*l_CD*l_CE*t201*t202*t211*t216*2.0+
                                     l_AH*l_CD*l_CE*t209*t210*t212*t217*2.0-l_AH*l_CD*l_CE*t210*t211*t212*t216*2.0-
                                     l_AH*l_CD*l_EF*t201*t202*t209*t220*2.0+l_AH*l_CD*l_EF*t201*t202*t211*t218*2.0-
                                     l_AH*l_CD*l_EF*t209*t210*t212*t220*2.0+l_AH*l_CD*l_EF*t210*t211*t212*t218*2.0-
                                     l_AH*l_CE*exo_x3*t201*t202*t209*t216*2.0-l_AH*l_CE*exo_x3*t201*t202*t211*t217*2.0+
                                     l_AH*l_CE*exo_x3*t201*t209*t210*t217*2.0-l_AH*l_CE*exo_x3*t202*t209*t212*t217*2.0-
                                     l_AH*l_CE*exo_x3*t201*t210*t211*t216*2.0+l_AH*l_CE*exo_x3*t202*t211*t212*t216*2.0-
                                     l_CD*l_CE*exo_x3*t201*t202*t209*t217*2.0-l_AH*l_CE*exo_x3*t209*t210*t212*t216*2.0+
                                     l_AH*l_EF*exo_x3*t201*t202*t209*t218*2.0+l_CD*l_CE*exo_x3*t201*t202*t211*t216*2.0-
                                     l_AH*l_CE*exo_x3*t210*t211*t212*t217*2.0-l_CD*l_CE*exo_x3*t201*t209*t210*t216+
                                     l_CD*l_CE*exo_x3*t202*t209*t212*t216+l_AH*l_EF*exo_x3*t201*t202*t211*t220*2.0-
                                     l_CD*l_CE*exo_x3*t201*t210*t211*t217+l_CD*l_CE*exo_x3*t202*t211*t212*t217-
                                     l_AH*l_EF*exo_x3*t201*t209*t210*t220*2.0+l_AH*l_EF*exo_x3*t201*t210*t211*t218*2.0+
                                     l_AH*l_EF*exo_x3*t202*t209*t212*t220*2.0-l_AH*l_EF*exo_x3*t202*t211*t212*t218*2.0-
                                     l_CD*l_CE*exo_x3*t209*t210*t212*t217*2.0+l_AH*l_EF*exo_x3*t209*t210*t212*t218*2.0+
                                     l_CD*l_CE*exo_x3*t210*t211*t212*t216*2.0+l_CD*l_EF*exo_x3*t201*t202*t209*t220*2.0-
                                     l_CD*l_EF*exo_x3*t201*t202*t211*t218*2.0+l_CD*l_EF*exo_x3*t201*t209*t210*t218-
                                     l_CD*l_EF*exo_x3*t202*t209*t212*t218+l_AH*l_EF*exo_x3*t210*t211*t212*t220*2.0+
                                     l_CD*l_EF*exo_x3*t201*t210*t211*t220-l_CD*l_EF*exo_x3*t202*t211*t212*t220+
                                     l_CD*l_EF*exo_x3*t209*t210*t212*t220*2.0-l_CD*l_EF*exo_x3*t210*t211*t212*t218*2.0))/l_HF;
    J(0,1) = 0;
    J(1,1) = 0;
    J(2,1) = (l_EF*t215)/(l_CE*t225-l_EF*t215);
    J(3,1) = -(l_CE*l_EF*sin(exo_t6_rel))/(l_CE*l_HF*t225-l_EF*l_HF*t215);

    // updating the reduced space Jacobian
    Jn(0,0) = J(1,0);
    Jn(1,0) = J(3,0);
    Jn(0,1) = J(1,1);
    Jn(1,1) = J(3,1);

    cout<<"Jn= "<<Jn<<endl;
}

// Evaluates the derivative of the Jacobian of the index finger exoskeleton
void exo_finger::exo_jacobian_dot()
{
    t36 = exo_t1-t_mcp;
    t37 = cos(t36);
    t38 = exo_t1_dot-t_mcp_dot;
    t39 = 1.0/exo_x3;
    t40 = sin(t36);
    t41 = 1.0/(exo_x3*exo_x3);
    t42 = -t_HFG+t_pip;
    t43 = exo_t5+exo_t6_rel;
    t44 = cos(t42);
    t45 = sin(t42);
    t46 = 1.0/l_CD;
    t47 = exo_x3-l_AH;
    t48 = (exo_x3*exo_x3);
    t49 = t46*t47;
    t50 = atan(t49);
    t51 = -t50+t_mcp;
    t52 = sin(exo_t1);
    t53 = sin(t_mcp);
    t54 = sin(t51);
    t55 = cos(t51);
    t56 = cos(exo_t1);
    t57 = cos(t_mcp);
    t58 = 1.0/(l_CD*l_CD);
    t59 = (t47*t47);
    t60 = t58*t59;
    t61 = t60+1.0;
    t62 = 1.0/t61;
    t65 = exo_x3_dot*t46*t62;
    t63 = -t65+t_mcp_dot;
    t64 = (l_AH*l_AH);
    t66 = cos(t43);
    t67 = sin(t43);
    t68 = cos(exo_t5);
    t69 = l_CE*t44*t68;
    t70 = sin(exo_t5);
    t71 = l_CE*t45*t70;
    t74 = l_EF*t44*t66;
    t75 = l_EF*t45*t67;
    t72 = t69+t71-t74-t75;
    t73 = 1.0/t72;
    t76 = exo_t5_dot+exo_t6_dot;
    t77 = t44*t48*t52*t55*t57*2.0;
    t78 = t44*t48*t54*t56*t57;
    t79 = t44*t54*t56*t57*t64;
    t80 = t44*t48*t52*t53*t54;
    t81 = t45*t48*t52*t54*t57*2.0;
    t82 = t44*t52*t53*t54*t64;
    t83 = l_AH*l_CD*t44*t55*t56*t57*2.0;
    t84 = exo_x3*l_AH*t44*t53*t55*t56*2.0;
    t85 = exo_x3*l_AH*t45*t55*t56*t57*2.0;
    t86 = exo_x3*l_CD*t44*t52*t54*t57;
    t87 = exo_x3*l_CD*t45*t53*t55*t56;
    t88 = l_AH*l_CD*t44*t52*t53*t55*2.0;
    t89 = l_AH*l_CD*t45*t54*t56*t57*2.0;
    t90 = exo_x3*l_AH*t45*t52*t53*t55*2.0;
    t91 = exo_x3*l_AH*t45*t53*t54*t56*2.0;
    t92 = l_AH*l_CD*t45*t52*t53*t54*2.0;
    t93 = t77+t78+t79+t80+t81+t82+t83+t84+t85+t86+t87+t88+t89+t90+t91+t92-t45*t48*t52*t53*t55-t44*t48*t53*t55*t56*2.0-t45*t48*t53*t54*t56*2.0-t45*t48*t55*t56*t57-t45*t52*t53*t55*t64-t45*t55*t56*t57*t64-exo_x3*l_AH*t44*t52*t53*t54*2.0-exo_x3*l_CD*t44*t52*t53*t55*2.0-exo_x3*l_CD*t45*t52*t53*t54*2.0-exo_x3*l_AH*t44*t52*t55*t57*2.0-exo_x3*l_AH*t45*t52*t54*t57*2.0-exo_x3*l_CD*t44*t53*t54*t56-exo_x3*l_CD*t45*t52*t55*t57-exo_x3*l_AH*t44*t54*t56*t57*2.0-exo_x3*l_CD*t44*t55*t56*t57*2.0-exo_x3*l_CD*t45*t54*t56*t57*2.0;
    t94 = -exo_t5-exo_t6_rel-t_HFG+t_pip;
    t95 = exo_t5+t_HFG-t_pip;
    t96 = cos(t94);
    t97 = sin(t94);
    t98 = cos(t95);
    t99 = l_CE*t98;
    t100 = t99-l_EF*t96;
    t101 = exo_t5_dot+exo_t6_dot-t_pip_dot;
    t102 = 1.0/l_HF;
    t103 = exo_t5_dot*l_CE*t45*t68;
    t104 = l_CE*t44*t70*t_pip_dot;
    t105 = l_EF*t44*t67*t76;
    t106 = l_EF*t45*t66*t_pip_dot;
    t107 = 1.0/(t72*t72);
    t108 = t103+t104+t105+t106-exo_t5_dot*l_CE*t44*t70-l_EF*t45*t66*t76-
            l_CE*t45*t68*t_pip_dot-l_EF*t44*t67*t_pip_dot;
    t109 = l_EF*t48*t52*t54*t57*t66*2.0;
    t110 = l_EF*t48*t53*t55*t56*t67*2.0;
    t111 = l_CE*t48*t55*t56*t57*t68;
    t112 = l_CE*t55*t56*t57*t64*t68;
    t113 = l_CE*t48*t52*t53*t55*t68;
    t114 = l_CE*t48*t52*t55*t57*t70*2.0;
    t115 = l_CE*t48*t53*t54*t56*t68*2.0;
    t116 = l_CE*t48*t54*t56*t57*t70;
    t117 = l_CE*t52*t53*t55*t64*t68;
    t118 = l_CE*t54*t56*t57*t64*t70;
    t119 = l_CE*t48*t52*t53*t54*t70;
    t120 = l_CE*t52*t53*t54*t64*t70;
    t121 = exo_x3*l_AH*l_EF*t55*t56*t57*t66*2.0;
    t122 = exo_x3*l_CD*l_EF*t53*t55*t56*t66;
    t123 = exo_x3*l_CD*l_EF*t55*t56*t57*t67*2.0;
    t124 = l_AH*l_CD*l_EF*t54*t56*t57*t66*2.0;
    t125 = exo_x3*l_AH*l_EF*t52*t53*t55*t66*2.0;
    t126 = exo_x3*l_AH*l_EF*t53*t54*t56*t66*2.0;
    t127 = exo_x3*l_AH*l_EF*t52*t55*t57*t67*2.0;
    t128 = exo_x3*l_AH*l_EF*t54*t56*t57*t67*2.0;
    t129 = exo_x3*l_CD*l_EF*t52*t53*t55*t67*2.0;
    t130 = exo_x3*l_CD*l_EF*t53*t54*t56*t67;
    t131 = l_AH*l_CD*l_EF*t52*t53*t54*t66*2.0;
    t132 = exo_x3*l_AH*l_EF*t52*t53*t54*t67*2.0;
    t133 = exo_x3*l_CD*l_CE*t52*t55*t57*t68;
    t134 = exo_x3*l_CD*l_CE*t54*t56*t57*t68*2.0;
    t135 = l_AH*l_CD*l_CE*t55*t56*t57*t70*2.0;
    t136 = exo_x3*l_AH*l_CE*t53*t55*t56*t70*2.0;
    t137 = exo_x3*l_AH*l_CE*t52*t54*t57*t68*2.0;
    t138 = exo_x3*l_CD*l_CE*t52*t53*t54*t68*2.0;
    t139 = exo_x3*l_CD*l_CE*t52*t54*t57*t70;
    t140 = l_AH*l_CD*l_CE*t52*t53*t55*t70*2.0;
    t141 = t109+t110+t111+t112+t113+t114+t115+t116+t117+t118+t119+t120+t121+t122+t123+
            t124+t125+t126+t127+t128+t129+t130+t131+t132+t133+t134+t135+t136+t137+t138+
            t139+t140-l_EF*t48*t52*t53*t54*t67-l_EF*t48*t52*t53*t55*t66-
            l_CE*t48*t52*t54*t57*t68*2.0-l_EF*t48*t53*t54*t56*t66*2.0-
            l_EF*t48*t52*t55*t57*t67*2.0-l_EF*t48*t54*t56*t57*t67-
            l_EF*t48*t55*t56*t57*t66-l_CE*t48*t53*t55*t56*t70*2.0-
            l_EF*t52*t53*t54*t64*t67-l_EF*t52*t53*t55*t64*t66-
            l_EF*t54*t56*t57*t64*t67-l_EF*t55*t56*t57*t64*t66-
            exo_x3*l_AH*l_CE*t52*t53*t55*t68*2.0-exo_x3*l_CD*l_EF*t52*t53*t54*t66*2.0-
            exo_x3*l_AH*l_CE*t53*t54*t56*t68*2.0-exo_x3*l_AH*l_EF*t52*t54*t57*t66*2.0-
            exo_x3*l_AH*l_EF*t53*t55*t56*t67*2.0-exo_x3*l_CD*l_CE*t53*t55*t56*t68-
            exo_x3*l_CD*l_EF*t52*t54*t57*t67-exo_x3*l_CD*l_EF*t52*t55*t57*t66-
            exo_x3*l_AH*l_CE*t55*t56*t57*t68*2.0-exo_x3*l_CD*l_EF*t54*t56*t57*t66*2.0-
            exo_x3*l_AH*l_CE*t52*t53*t54*t70*2.0-exo_x3*l_CD*l_CE*t52*t53*t55*t70*2.0-
            exo_x3*l_AH*l_CE*t52*t55*t57*t70*2.0-exo_x3*l_CD*l_CE*t53*t54*t56*t70-
            exo_x3*l_AH*l_CE*t54*t56*t57*t70*2.0-exo_x3*l_CD*l_CE*t55*t56*t57*t70*2.0-
            l_AH*l_CD*l_CE*t52*t53*t54*t68*2.0-l_AH*l_CD*l_EF*t52*t53*t55*t67*2.0-
            l_AH*l_CD*l_CE*t54*t56*t57*t68*2.0-l_AH*l_CD*l_EF*t55*t56*t57*t67*2.0;
    t142 = sin(t95);
    t143 = exo_t5_dot-t_pip_dot;
    t144 = l_CE*l_HF*t98;
    t145 = t144-l_EF*l_HF*t96;
    J_dot(0,0) = -l_BC*t37*t38-exo_x3_dot*l_BC*l_CD*t37*t41-l_BC*l_CD*t38*t39*t40;
    J_dot(1,0) =  -exo_x3_dot*l_BC*t37*t41-l_BC*t38*t39*t40;
    J_dot(2,0) = -l_BC*t39*t73*(exo_x3_dot*l_AH*t44*t52*t53*t54*2.0-
                                exo_x3_dot*l_AH*t45*t52*t53*t55*2.0+
                                exo_x3_dot*l_CD*t44*t52*t53*t55*2.0+
                                exo_x3_dot*l_CD*t45*t52*t53*t54*2.0+
                                exo_x3_dot*l_AH*t44*t52*t55*t57*2.0-
                                exo_x3_dot*l_AH*t44*t53*t55*t56*2.0+
                                exo_x3_dot*l_AH*t45*t52*t54*t57*2.0-
                                exo_x3_dot*l_AH*t45*t53*t54*t56*2.0-
                                exo_x3_dot*l_CD*t44*t52*t54*t57+
                                exo_x3_dot*l_CD*t44*t53*t54*t56+
                                exo_x3_dot*l_CD*t45*t52*t55*t57-
                                exo_x3_dot*l_CD*t45*t53*t55*t56+
                                exo_x3_dot*l_AH*t44*t54*t56*t57*2.0-
                                exo_x3_dot*l_AH*t45*t55*t56*t57*2.0+
                                exo_x3_dot*l_CD*t44*t55*t56*t57*2.0+
                                exo_x3_dot*l_CD*t45*t54*t56*t57*2.0-
                                exo_t1_dot*t44*t48*t52*t53*t55*2.0-
                                exo_t1_dot*t45*t48*t52*t53*t54*2.0+
                                exo_t1_dot*t44*t48*t52*t54*t57-
                                exo_t1_dot*t44*t48*t53*t54*t56-
                                exo_t1_dot*t45*t48*t52*t55*t57+
                                exo_t1_dot*t45*t48*t53*t55*t56-
                                exo_t1_dot*t44*t48*t55*t56*t57*2.0-
                                exo_t1_dot*t45*t48*t54*t56*t57*2.0+
                                exo_t1_dot*t44*t52*t54*t57*t64-
                                exo_t1_dot*t44*t53*t54*t56*t64-
                                exo_t1_dot*t45*t52*t55*t57*t64+
                                exo_t1_dot*t45*t53*t55*t56*t64-
                                t44*t48*t52*t53*t55*t63-t45*t48*t52*t53*t54*t63+
                                t44*t48*t52*t54*t57*t63*2.0-t44*t48*t53*t54*t56*t63*2.0-
                                t45*t48*t52*t55*t57*t63*2.0+t45*t48*t53*t55*t56*t63*2.0-
                                t44*t48*t55*t56*t57*t63-t45*t48*t54*t56*t57*t63-
                                t44*t52*t53*t55*t63*t64-t45*t52*t53*t54*t63*t64-
                                t44*t55*t56*t57*t63*t64-t45*t54*t56*t57*t63*t64+
                                t44*t48*t52*t53*t55*t_mcp_dot*2.0+
                                t45*t48*t52*t53*t54*t_mcp_dot*2.0-
                                t44*t48*t52*t54*t57*t_mcp_dot+
                                t44*t48*t53*t54*t56*t_mcp_dot+
                                t44*t48*t52*t53*t55*t_pip_dot+
                                t45*t48*t52*t53*t54*t_pip_dot+
                                t45*t48*t52*t55*t57*t_mcp_dot-
                                t45*t48*t53*t55*t56*t_mcp_dot-
                                t44*t48*t52*t54*t57*t_pip_dot*2.0+
                                t44*t48*t53*t54*t56*t_pip_dot*2.0+
                                t44*t48*t55*t56*t57*t_mcp_dot*2.0+
                                t45*t48*t54*t56*t57*t_mcp_dot*2.0+
                                t45*t48*t52*t55*t57*t_pip_dot*2.0-
                                t45*t48*t53*t55*t56*t_pip_dot*2.0+
                                t44*t48*t55*t56*t57*t_pip_dot+
                                t45*t48*t54*t56*t57*t_pip_dot-
                                t44*t52*t54*t57*t64*t_mcp_dot+
                                t44*t53*t54*t56*t64*t_mcp_dot+
                                t44*t52*t53*t55*t64*t_pip_dot+
                                t45*t52*t53*t54*t64*t_pip_dot+
                                t45*t52*t55*t57*t64*t_mcp_dot-
                                t45*t53*t55*t56*t64*t_mcp_dot+
                                t44*t55*t56*t57*t64*t_pip_dot+
                                t45*t54*t56*t57*t64*t_pip_dot-
                                exo_x3*exo_x3_dot*t44*t52*t53*t54*2.0+
                                exo_x3*exo_x3_dot*t45*t52*t53*t55*2.0-
                                exo_x3*exo_x3_dot*t44*t52*t55*t57*4.0+
                                exo_x3*exo_x3_dot*t44*t53*t55*t56*4.0-
                                exo_x3*exo_x3_dot*t45*t52*t54*t57*4.0+
                                exo_x3*exo_x3_dot*t45*t53*t54*t56*4.0-
                                exo_x3*exo_x3_dot*t44*t54*t56*t57*2.0+
                                exo_x3*exo_x3_dot*t45*t55*t56*t57*2.0+
                                exo_x3*exo_t1_dot*l_AH*t44*t52*t53*t55*2.0+
                                exo_x3*exo_t1_dot*l_AH*t45*t52*t53*t54*2.0-
                                exo_x3*exo_t1_dot*l_CD*t44*t52*t53*t54+
                                exo_x3*exo_t1_dot*l_CD*t45*t52*t53*t55-
                                exo_x3*exo_t1_dot*l_AH*t44*t52*t54*t57*2.0+
                                exo_x3*exo_t1_dot*l_AH*t44*t53*t54*t56*2.0+
                                exo_x3*exo_t1_dot*l_AH*t45*t52*t55*t57*2.0-
                                exo_x3*exo_t1_dot*l_AH*t45*t53*t55*t56*2.0-
                                exo_x3*exo_t1_dot*l_CD*t44*t52*t55*t57*2.0+
                                exo_x3*exo_t1_dot*l_CD*t44*t53*t55*t56*2.0-
                                exo_x3*exo_t1_dot*l_CD*t45*t52*t54*t57*2.0+
                                exo_x3*exo_t1_dot*l_CD*t45*t53*t54*t56*2.0+
                                exo_x3*exo_t1_dot*l_AH*t44*t55*t56*t57*2.0+
                                exo_x3*exo_t1_dot*l_AH*t45*t54*t56*t57*2.0-
                                exo_x3*exo_t1_dot*l_CD*t44*t54*t56*t57+
                                exo_x3*exo_t1_dot*l_CD*t45*t55*t56*t57+
                                exo_t1_dot*l_AH*l_CD*t44*t52*t55*t57*2.0-
                                exo_t1_dot*l_AH*l_CD*t44*t53*t55*t56*2.0+
                                exo_t1_dot*l_AH*l_CD*t45*t52*t54*t57*2.0-
                                exo_t1_dot*l_AH*l_CD*t45*t53*t54*t56*2.0+
                                exo_x3*l_AH*t44*t52*t53*t55*t63*2.0+
                                exo_x3*l_AH*t45*t52*t53*t54*t63*2.0-
                                exo_x3*l_CD*t44*t52*t53*t54*t63*2.0+
                                exo_x3*l_CD*t45*t52*t53*t55*t63*2.0-
                                exo_x3*l_AH*t44*t52*t54*t57*t63*2.0+
                                exo_x3*l_AH*t44*t53*t54*t56*t63*2.0+
                                exo_x3*l_AH*t45*t52*t55*t57*t63*2.0-
                                exo_x3*l_AH*t45*t53*t55*t56*t63*2.0-
                                exo_x3*l_CD*t44*t52*t55*t57*t63+
                                exo_x3*l_CD*t44*t53*t55*t56*t63-
                                exo_x3*l_CD*t45*t52*t54*t57*t63+
                                exo_x3*l_CD*t45*t53*t54*t56*t63+
                                exo_x3*l_AH*t44*t55*t56*t57*t63*2.0+
                                exo_x3*l_AH*t45*t54*t56*t57*t63*2.0-
                                exo_x3*l_CD*t44*t54*t56*t57*t63*2.0+
                                exo_x3*l_CD*t45*t55*t56*t57*t63*2.0-
                                exo_x3*l_AH*t44*t52*t53*t55*t_mcp_dot*2.0-
                                exo_x3*l_AH*t45*t52*t53*t54*t_mcp_dot*2.0+
                                exo_x3*l_CD*t44*t52*t53*t54*t_mcp_dot-
                                exo_x3*l_CD*t45*t52*t53*t55*t_mcp_dot+
                                exo_x3*l_AH*t44*t52*t54*t57*t_mcp_dot*2.0-
                                exo_x3*l_AH*t44*t53*t54*t56*t_mcp_dot*2.0-
                                exo_x3*l_AH*t44*t52*t53*t55*t_pip_dot*2.0-
                                exo_x3*l_AH*t45*t52*t53*t54*t_pip_dot*2.0+
                                exo_x3*l_CD*t44*t52*t53*t54*t_pip_dot*2.0-
                                exo_x3*l_AH*t45*t52*t55*t57*t_mcp_dot*2.0+
                                exo_x3*l_AH*t45*t53*t55*t56*t_mcp_dot*2.0+
                                exo_x3*l_CD*t44*t52*t55*t57*t_mcp_dot*2.0-
                                exo_x3*l_CD*t44*t53*t55*t56*t_mcp_dot*2.0+
                                exo_x3*l_CD*t45*t52*t54*t57*t_mcp_dot*2.0-
                                exo_x3*l_CD*t45*t53*t54*t56*t_mcp_dot*2.0-
                                exo_x3*l_CD*t45*t52*t53*t55*t_pip_dot*2.0+
                                exo_x3*l_AH*t44*t52*t54*t57*t_pip_dot*2.0-
                                exo_x3*l_AH*t44*t53*t54*t56*t_pip_dot*2.0-
                                exo_x3*l_AH*t44*t55*t56*t57*t_mcp_dot*2.0-
                                exo_x3*l_AH*t45*t54*t56*t57*t_mcp_dot*2.0-
                                exo_x3*l_AH*t45*t52*t55*t57*t_pip_dot*2.0+
                                exo_x3*l_AH*t45*t53*t55*t56*t_pip_dot*2.0+
                                exo_x3*l_CD*t44*t54*t56*t57*t_mcp_dot+
                                exo_x3*l_CD*t44*t52*t55*t57*t_pip_dot-
                                exo_x3*l_CD*t44*t53*t55*t56*t_pip_dot+
                                exo_x3*l_CD*t45*t52*t54*t57*t_pip_dot-
                                exo_x3*l_CD*t45*t53*t54*t56*t_pip_dot-
                                exo_x3*l_CD*t45*t55*t56*t57*t_mcp_dot-
                                exo_x3*l_AH*t44*t55*t56*t57*t_pip_dot*2.0-
                                exo_x3*l_AH*t45*t54*t56*t57*t_pip_dot*2.0+
                                exo_x3*l_CD*t44*t54*t56*t57*t_pip_dot*2.0-
                                exo_x3*l_CD*t45*t55*t56*t57*t_pip_dot*2.0+
                                l_AH*l_CD*t44*t52*t53*t54*t63*2.0-
                                l_AH*l_CD*t45*t52*t53*t55*t63*2.0+
                                l_AH*l_CD*t44*t54*t56*t57*t63*2.0-
                                l_AH*l_CD*t45*t55*t56*t57*t63*2.0-
                                l_AH*l_CD*t44*t52*t53*t54*t_pip_dot*2.0-
                                l_AH*l_CD*t44*t52*t55*t57*t_mcp_dot*2.0+
                                l_AH*l_CD*t44*t53*t55*t56*t_mcp_dot*2.0-
                                l_AH*l_CD*t45*t52*t54*t57*t_mcp_dot*2.0+
                                l_AH*l_CD*t45*t53*t54*t56*t_mcp_dot*2.0+
                                l_AH*l_CD*t45*t52*t53*t55*t_pip_dot*2.0-
                                l_AH*l_CD*t44*t54*t56*t57*t_pip_dot*2.0+
                                l_AH*l_CD*t45*t55*t56*t57*t_pip_dot*2.0)-
                                exo_x3_dot*l_BC*t41*t73*t93-l_BC*t107*t108*t39*t93;
    J_dot(3,0) = l_BC*t102*t39*t73*(exo_x3*exo_x3_dot*l_CE*t52*t53*t55*t68*2.0-
                                    exo_x3*exo_x3_dot*l_EF*t52*t53*t54*t67*2.0-
                                    exo_x3*exo_x3_dot*l_EF*t52*t53*t55*t66*2.0-
                                    exo_x3*exo_x3_dot*l_CE*t52*t54*t57*t68*4.0+
                                    exo_x3*exo_x3_dot*l_CE*t53*t54*t56*t68*4.0+
                                    exo_x3*exo_x3_dot*l_EF*t52*t54*t57*t66*4.0-
                                    exo_x3*exo_x3_dot*l_EF*t53*t54*t56*t66*4.0-
                                    exo_x3*exo_x3_dot*l_EF*t52*t55*t57*t67*4.0+
                                    exo_x3*exo_x3_dot*l_EF*t53*t55*t56*t67*4.0+
                                    exo_x3*exo_x3_dot*l_CE*t55*t56*t57*t68*2.0-
                                    exo_x3*exo_x3_dot*l_EF*t54*t56*t57*t67*2.0-
                                    exo_x3*exo_x3_dot*l_EF*t55*t56*t57*t66*2.0+
                                    exo_x3*exo_x3_dot*l_CE*t52*t53*t54*t70*2.0+
                                    exo_x3*exo_x3_dot*l_CE*t52*t55*t57*t70*4.0-
                                    exo_x3*exo_x3_dot*l_CE*t53*t55*t56*t70*4.0+
                                    exo_x3*exo_x3_dot*l_CE*t54*t56*t57*t70*2.0-
                                    exo_x3_dot*l_AH*l_CE*t52*t53*t55*t68*2.0+
                                    exo_x3_dot*l_AH*l_EF*t52*t53*t54*t67*2.0+
                                    exo_x3_dot*l_AH*l_EF*t52*t53*t55*t66*2.0+
                                    exo_x3_dot*l_CD*l_CE*t52*t53*t54*t68*2.0-
                                    exo_x3_dot*l_CD*l_EF*t52*t53*t54*t66*2.0+
                                    exo_x3_dot*l_CD*l_EF*t52*t53*t55*t67*2.0+
                                    exo_x3_dot*l_AH*l_CE*t52*t54*t57*t68*2.0-
                                    exo_x3_dot*l_AH*l_CE*t53*t54*t56*t68*2.0-
                                    exo_x3_dot*l_AH*l_EF*t52*t54*t57*t66*2.0+
                                    exo_x3_dot*l_AH*l_EF*t53*t54*t56*t66*2.0+
                                    exo_x3_dot*l_AH*l_EF*t52*t55*t57*t67*2.0-
                                    exo_x3_dot*l_AH*l_EF*t53*t55*t56*t67*2.0+
                                    exo_x3_dot*l_CD*l_CE*t52*t55*t57*t68-
                                    exo_x3_dot*l_CD*l_CE*t53*t55*t56*t68-
                                    exo_x3_dot*l_CD*l_EF*t52*t54*t57*t67-
                                    exo_x3_dot*l_CD*l_EF*t52*t55*t57*t66+
                                    exo_x3_dot*l_CD*l_EF*t53*t54*t56*t67+
                                    exo_x3_dot*l_CD*l_EF*t53*t55*t56*t66-
                                    exo_x3_dot*l_AH*l_CE*t55*t56*t57*t68*2.0+
                                    exo_x3_dot*l_AH*l_EF*t54*t56*t57*t67*2.0+
                                    exo_x3_dot*l_AH*l_EF*t55*t56*t57*t66*2.0+
                                    exo_x3_dot*l_CD*l_CE*t54*t56*t57*t68*2.0-
                                    exo_x3_dot*l_CD*l_EF*t54*t56*t57*t66*2.0+
                                    exo_x3_dot*l_CD*l_EF*t55*t56*t57*t67*2.0-
                                    exo_x3_dot*l_AH*l_CE*t52*t53*t54*t70*2.0-
                                    exo_x3_dot*l_CD*l_CE*t52*t53*t55*t70*2.0-
                                    exo_x3_dot*l_AH*l_CE*t52*t55*t57*t70*2.0+
                                    exo_x3_dot*l_AH*l_CE*t53*t55*t56*t70*2.0+
                                    exo_x3_dot*l_CD*l_CE*t52*t54*t57*t70-
                                    exo_x3_dot*l_CD*l_CE*t53*t54*t56*t70-
                                    exo_x3_dot*l_AH*l_CE*t54*t56*t57*t70*2.0-
                                    exo_x3_dot*l_CD*l_CE*t55*t56*t57*t70*2.0-
                                    exo_t1_dot*l_CE*t48*t52*t53*t54*t68*2.0+
                                    exo_t1_dot*l_EF*t48*t52*t53*t54*t66*2.0+
                                    exo_t5_dot*l_CE*t48*t52*t53*t54*t68-
                                    exo_t1_dot*l_EF*t48*t52*t53*t55*t67*2.0-
                                    exo_t1_dot*l_CE*t48*t52*t55*t57*t68+
                                    exo_t1_dot*l_CE*t48*t53*t55*t56*t68+
                                    exo_t1_dot*l_EF*t48*t52*t54*t57*t67+
                                    exo_t1_dot*l_EF*t48*t52*t55*t57*t66-
                                    exo_t1_dot*l_EF*t48*t53*t54*t56*t67-
                                    exo_t1_dot*l_EF*t48*t53*t55*t56*t66+
                                    exo_t5_dot*l_CE*t48*t52*t55*t57*t68*2.0-
                                    exo_t5_dot*l_CE*t48*t53*t55*t56*t68*2.0-
                                    exo_t1_dot*l_CE*t48*t54*t56*t57*t68*2.0+
                                    exo_t1_dot*l_EF*t48*t54*t56*t57*t66*2.0+
                                    exo_t5_dot*l_CE*t48*t54*t56*t57*t68-
                                    exo_t1_dot*l_EF*t48*t55*t56*t57*t67*2.0+
                                    exo_t1_dot*l_CE*t48*t52*t53*t55*t70*2.0-
                                    exo_t5_dot*l_CE*t48*t52*t53*t55*t70-
                                    exo_t1_dot*l_CE*t48*t52*t54*t57*t70+
                                    exo_t1_dot*l_CE*t48*t53*t54*t56*t70+
                                    exo_t5_dot*l_CE*t48*t52*t54*t57*t70*2.0-
                                    exo_t5_dot*l_CE*t48*t53*t54*t56*t70*2.0+
                                    exo_t1_dot*l_CE*t48*t55*t56*t57*t70*2.0-
                                    exo_t5_dot*l_CE*t48*t55*t56*t57*t70+
                                    exo_t5_dot*l_CE*t52*t53*t54*t64*t68-
                                    exo_t1_dot*l_CE*t52*t55*t57*t64*t68+
                                    exo_t1_dot*l_CE*t53*t55*t56*t64*t68+
                                    exo_t1_dot*l_EF*t52*t54*t57*t64*t67+
                                    exo_t1_dot*l_EF*t52*t55*t57*t64*t66-
                                    exo_t1_dot*l_EF*t53*t54*t56*t64*t67-
                                    exo_t1_dot*l_EF*t53*t55*t56*t64*t66+
                                    exo_t5_dot*l_CE*t54*t56*t57*t64*t68-
                                    exo_t5_dot*l_CE*t52*t53*t55*t64*t70-
                                    exo_t1_dot*l_CE*t52*t54*t57*t64*t70+
                                    exo_t1_dot*l_CE*t53*t54*t56*t64*t70-
                                    exo_t5_dot*l_CE*t55*t56*t57*t64*t70-
                                    l_CE*t48*t52*t53*t54*t63*t68+
                                    l_EF*t48*t52*t53*t54*t63*t66-
                                    l_EF*t48*t52*t53*t55*t63*t67-
                                    l_CE*t48*t52*t55*t57*t63*t68*2.0+
                                    l_CE*t48*t53*t55*t56*t63*t68*2.0+
                                    l_EF*t48*t52*t54*t57*t63*t67*2.0+
                                    l_EF*t48*t52*t55*t57*t63*t66*2.0-
                                    l_EF*t48*t53*t54*t56*t63*t67*2.0-
                                    l_EF*t48*t53*t55*t56*t63*t66*2.0-
                                    l_CE*t48*t54*t56*t57*t63*t68+
                                    l_EF*t48*t54*t56*t57*t63*t66-
                                    l_EF*t48*t55*t56*t57*t63*t67+
                                    l_CE*t48*t52*t53*t55*t63*t70-
                                    l_CE*t48*t52*t54*t57*t63*t70*2.0+
                                    l_CE*t48*t53*t54*t56*t63*t70*2.0+
                                    l_CE*t48*t55*t56*t57*t63*t70-
                                    l_EF*t48*t52*t53*t54*t66*t76+
                                    l_EF*t48*t52*t53*t55*t67*t76-
                                    l_EF*t48*t52*t54*t57*t67*t76*2.0-
                                    l_EF*t48*t52*t55*t57*t66*t76*2.0+
                                    l_EF*t48*t53*t54*t56*t67*t76*2.0+
                                    l_EF*t48*t53*t55*t56*t66*t76*2.0-
                                    l_EF*t48*t54*t56*t57*t66*t76+
                                    l_EF*t48*t55*t56*t57*t67*t76-
                                    l_CE*t52*t53*t54*t63*t64*t68+
                                    l_EF*t52*t53*t54*t63*t64*t66-
                                    l_EF*t52*t53*t55*t63*t64*t67-
                                    l_CE*t54*t56*t57*t63*t64*t68+
                                    l_EF*t54*t56*t57*t63*t64*t66-
                                    l_EF*t55*t56*t57*t63*t64*t67+
                                    l_CE*t52*t53*t55*t63*t64*t70+
                                    l_CE*t55*t56*t57*t63*t64*t70-
                                    l_EF*t52*t53*t54*t64*t66*t76+
                                    l_EF*t52*t53*t55*t64*t67*t76-
                                    l_EF*t54*t56*t57*t64*t66*t76+
                                    l_EF*t55*t56*t57*t64*t67*t76+
                                    l_CE*t48*t52*t53*t54*t68*t_mcp_dot*2.0-
                                    l_EF*t48*t52*t53*t54*t66*t_mcp_dot*2.0+
                                    l_EF*t48*t52*t53*t55*t67*t_mcp_dot*2.0+
                                    l_CE*t48*t52*t55*t57*t68*t_mcp_dot-
                                    l_CE*t48*t53*t55*t56*t68*t_mcp_dot-
                                    l_EF*t48*t52*t54*t57*t67*t_mcp_dot-
                                    l_EF*t48*t52*t55*t57*t66*t_mcp_dot+
                                    l_EF*t48*t53*t54*t56*t67*t_mcp_dot+
                                    l_EF*t48*t53*t55*t56*t66*t_mcp_dot+
                                    l_CE*t48*t54*t56*t57*t68*t_mcp_dot*2.0-
                                    l_EF*t48*t54*t56*t57*t66*t_mcp_dot*2.0+
                                    l_EF*t48*t55*t56*t57*t67*t_mcp_dot*2.0-
                                    l_CE*t48*t52*t53*t55*t70*t_mcp_dot*2.0+
                                    l_CE*t48*t52*t54*t57*t70*t_mcp_dot-
                                    l_CE*t48*t53*t54*t56*t70*t_mcp_dot-
                                    l_CE*t48*t55*t56*t57*t70*t_mcp_dot*2.0+
                                    l_CE*t52*t55*t57*t64*t68*t_mcp_dot-
                                    l_CE*t53*t55*t56*t64*t68*t_mcp_dot-
                                    l_EF*t52*t54*t57*t64*t67*t_mcp_dot-
                                    l_EF*t52*t55*t57*t64*t66*t_mcp_dot+
                                    l_EF*t53*t54*t56*t64*t67*t_mcp_dot+
                                    l_EF*t53*t55*t56*t64*t66*t_mcp_dot+
                                    l_CE*t52*t54*t57*t64*t70*t_mcp_dot-
                                    l_CE*t53*t54*t56*t64*t70*t_mcp_dot+
                                    exo_x3*exo_t1_dot*l_AH*l_CE*t52*t53*t54*t68*2.0-
                                    exo_x3*exo_t1_dot*l_AH*l_EF*t52*t53*t54*t66*2.0-
                                    exo_x3*exo_t5_dot*l_AH*l_CE*t52*t53*t54*t68*2.0+
                                    exo_x3*exo_t1_dot*l_AH*l_EF*t52*t53*t55*t67*2.0+
                                    exo_x3*exo_t1_dot*l_CD*l_CE*t52*t53*t55*t68-
                                    exo_x3*exo_t1_dot*l_CD*l_EF*t52*t53*t54*t67-
                                    exo_x3*exo_t1_dot*l_CD*l_EF*t52*t53*t55*t66-
                                    exo_x3*exo_t5_dot*l_CD*l_CE*t52*t53*t55*t68*2.0+
                                    exo_x3*exo_t1_dot*l_AH*l_CE*t52*t55*t57*t68*2.0-
                                    exo_x3*exo_t1_dot*l_AH*l_CE*t53*t55*t56*t68*2.0-
                                    exo_x3*exo_t1_dot*l_AH*l_EF*t52*t54*t57*t67*2.0-
                                    exo_x3*exo_t1_dot*l_AH*l_EF*t52*t55*t57*t66*2.0+
                                    exo_x3*exo_t1_dot*l_AH*l_EF*t53*t54*t56*t67*2.0+
                                    exo_x3*exo_t1_dot*l_AH*l_EF*t53*t55*t56*t66*2.0-
                                    exo_x3*exo_t5_dot*l_AH*l_CE*t52*t55*t57*t68*2.0+
                                    exo_x3*exo_t5_dot*l_AH*l_CE*t53*t55*t56*t68*2.0-
                                    exo_x3*exo_t1_dot*l_CD*l_CE*t52*t54*t57*t68*2.0+
                                    exo_x3*exo_t1_dot*l_CD*l_CE*t53*t54*t56*t68*2.0+
                                    exo_x3*exo_t1_dot*l_CD*l_EF*t52*t54*t57*t66*2.0-
                                    exo_x3*exo_t1_dot*l_CD*l_EF*t53*t54*t56*t66*2.0+
                                    exo_x3*exo_t5_dot*l_CD*l_CE*t52*t54*t57*t68-
                                    exo_x3*exo_t5_dot*l_CD*l_CE*t53*t54*t56*t68-
                                    exo_x3*exo_t1_dot*l_CD*l_EF*t52*t55*t57*t67*2.0+
                                    exo_x3*exo_t1_dot*l_CD*l_EF*t53*t55*t56*t67*2.0+
                                    exo_x3*exo_t1_dot*l_AH*l_CE*t54*t56*t57*t68*2.0-
                                    exo_x3*exo_t1_dot*l_AH*l_EF*t54*t56*t57*t66*2.0-
                                    exo_x3*exo_t5_dot*l_AH*l_CE*t54*t56*t57*t68*2.0+
                                    exo_x3*exo_t1_dot*l_AH*l_EF*t55*t56*t57*t67*2.0+
                                    exo_x3*exo_t1_dot*l_CD*l_CE*t55*t56*t57*t68-
                                    exo_x3*exo_t1_dot*l_CD*l_EF*t54*t56*t57*t67-
                                    exo_x3*exo_t1_dot*l_CD*l_EF*t55*t56*t57*t66-
                                    exo_x3*exo_t5_dot*l_CD*l_CE*t55*t56*t57*t68*2.0-
                                    exo_x3*exo_t1_dot*l_AH*l_CE*t52*t53*t55*t70*2.0+
                                    exo_x3*exo_t5_dot*l_AH*l_CE*t52*t53*t55*t70*2.0+
                                    exo_x3*exo_t1_dot*l_CD*l_CE*t52*t53*t54*t70-
                                    exo_x3*exo_t5_dot*l_CD*l_CE*t52*t53*t54*t70*2.0+
                                    exo_x3*exo_t1_dot*l_AH*l_CE*t52*t54*t57*t70*2.0-
                                    exo_x3*exo_t1_dot*l_AH*l_CE*t53*t54*t56*t70*2.0-
                                    exo_x3*exo_t5_dot*l_AH*l_CE*t52*t54*t57*t70*2.0+
                                    exo_x3*exo_t5_dot*l_AH*l_CE*t53*t54*t56*t70*2.0+
                                    exo_x3*exo_t1_dot*l_CD*l_CE*t52*t55*t57*t70*2.0-
                                    exo_x3*exo_t1_dot*l_CD*l_CE*t53*t55*t56*t70*2.0-
                                    exo_x3*exo_t5_dot*l_CD*l_CE*t52*t55*t57*t70+
                                    exo_x3*exo_t5_dot*l_CD*l_CE*t53*t55*t56*t70-
                                    exo_x3*exo_t1_dot*l_AH*l_CE*t55*t56*t57*t70*2.0+
                                    exo_x3*exo_t5_dot*l_AH*l_CE*t55*t56*t57*t70*2.0+
                                    exo_x3*exo_t1_dot*l_CD*l_CE*t54*t56*t57*t70-
                                    exo_x3*exo_t5_dot*l_CD*l_CE*t54*t56*t57*t70*2.0+
                                    exo_t5_dot*l_AH*l_CD*l_CE*t52*t53*t55*t68*2.0+
                                    exo_t1_dot*l_AH*l_CD*l_CE*t52*t54*t57*t68*2.0-
                                    exo_t1_dot*l_AH*l_CD*l_CE*t53*t54*t56*t68*2.0-
                                    exo_t1_dot*l_AH*l_CD*l_EF*t52*t54*t57*t66*2.0+
                                    exo_t1_dot*l_AH*l_CD*l_EF*t53*t54*t56*t66*2.0+
                                    exo_t1_dot*l_AH*l_CD*l_EF*t52*t55*t57*t67*2.0-
                                    exo_t1_dot*l_AH*l_CD*l_EF*t53*t55*t56*t67*2.0+
                                    exo_t5_dot*l_AH*l_CD*l_CE*t55*t56*t57*t68*2.0+
                                    exo_t5_dot*l_AH*l_CD*l_CE*t52*t53*t54*t70*2.0-
                                    exo_t1_dot*l_AH*l_CD*l_CE*t52*t55*t57*t70*2.0+
                                    exo_t1_dot*l_AH*l_CD*l_CE*t53*t55*t56*t70*2.0+
                                    exo_t5_dot*l_AH*l_CD*l_CE*t54*t56*t57*t70*2.0+
                                    exo_x3*l_AH*l_CE*t52*t53*t54*t63*t68*2.0-
                                    exo_x3*l_AH*l_EF*t52*t53*t54*t63*t66*2.0+
                                    exo_x3*l_AH*l_EF*t52*t53*t55*t63*t67*2.0+
                                    exo_x3*l_CD*l_CE*t52*t53*t55*t63*t68*2.0-
                                    exo_x3*l_CD*l_EF*t52*t53*t54*t63*t67*2.0-
                                    exo_x3*l_CD*l_EF*t52*t53*t55*t63*t66*2.0+
                                    exo_x3*l_AH*l_CE*t52*t55*t57*t63*t68*2.0-
                                    exo_x3*l_AH*l_CE*t53*t55*t56*t63*t68*2.0-
                                    exo_x3*l_AH*l_EF*t52*t54*t57*t63*t67*2.0-
                                    exo_x3*l_AH*l_EF*t52*t55*t57*t63*t66*2.0+
                                    exo_x3*l_AH*l_EF*t53*t54*t56*t63*t67*2.0+
                                    exo_x3*l_AH*l_EF*t53*t55*t56*t63*t66*2.0-
                                    exo_x3*l_CD*l_CE*t52*t54*t57*t63*t68+
                                    exo_x3*l_CD*l_CE*t53*t54*t56*t63*t68+
                                    exo_x3*l_CD*l_EF*t52*t54*t57*t63*t66-
                                    exo_x3*l_CD*l_EF*t53*t54*t56*t63*t66-
                                    exo_x3*l_CD*l_EF*t52*t55*t57*t63*t67+
                                    exo_x3*l_CD*l_EF*t53*t55*t56*t63*t67+
                                    exo_x3*l_AH*l_CE*t54*t56*t57*t63*t68*2.0-
                                    exo_x3*l_AH*l_EF*t54*t56*t57*t63*t66*2.0+
                                    exo_x3*l_AH*l_EF*t55*t56*t57*t63*t67*2.0+exo_x3*l_CD*l_CE*t55*t56*t57*t63*t68*2.0-exo_x3*l_CD*l_EF*t54*t56*t57*t63*t67*2.0-exo_x3*l_CD*l_EF*t55*t56*t57*t63*t66*2.0-exo_x3*l_AH*l_CE*t52*t53*t55*t63*t70*2.0+exo_x3*l_CD*l_CE*t52*t53*t54*t63*t70*2.0+exo_x3*l_AH*l_CE*t52*t54*t57*t63*t70*2.0-exo_x3*l_AH*l_CE*t53*t54*t56*t63*t70*2.0+exo_x3*l_CD*l_CE*t52*t55*t57*t63*t70-exo_x3*l_CD*l_CE*t53*t55*t56*t63*t70-exo_x3*l_AH*l_CE*t55*t56*t57*t63*t70*2.0+exo_x3*l_CD*l_CE*t54*t56*t57*t63*t70*2.0+exo_x3*l_AH*l_EF*t52*t53*t54*t66*t76*2.0-exo_x3*l_AH*l_EF*t52*t53*t55*t67*t76*2.0+exo_x3*l_CD*l_EF*t52*t53*t54*t67*t76*2.0+exo_x3*l_CD*l_EF*t52*t53*t55*t66*t76*2.0+exo_x3*l_AH*l_EF*t52*t54*t57*t67*t76*2.0+exo_x3*l_AH*l_EF*t52*t55*t57*t66*t76*2.0-exo_x3*l_AH*l_EF*t53*t54*t56*t67*t76*2.0-exo_x3*l_AH*l_EF*t53*t55*t56*t66*t76*2.0-exo_x3*l_CD*l_EF*t52*t54*t57*t66*t76+exo_x3*l_CD*l_EF*t53*t54*t56*t66*t76+exo_x3*l_CD*l_EF*t52*t55*t57*t67*t76-exo_x3*l_CD*l_EF*t53*t55*t56*t67*t76+exo_x3*l_AH*l_EF*t54*t56*t57*t66*t76*2.0-exo_x3*l_AH*l_EF*t55*t56*t57*t67*t76*2.0+exo_x3*l_CD*l_EF*t54*t56*t57*t67*t76*2.0+exo_x3*l_CD*l_EF*t55*t56*t57*t66*t76*2.0-exo_x3*l_AH*l_CE*t52*t53*t54*t68*t_mcp_dot*2.0+exo_x3*l_AH*l_EF*t52*t53*t54*t66*t_mcp_dot*2.0-exo_x3*l_AH*l_EF*t52*t53*t55*t67*t_mcp_dot*2.0-exo_x3*l_CD*l_CE*t52*t53*t55*t68*t_mcp_dot+exo_x3*l_CD*l_EF*t52*t53*t54*t67*t_mcp_dot+exo_x3*l_CD*l_EF*t52*t53*t55*t66*t_mcp_dot-exo_x3*l_AH*l_CE*t52*t55*t57*t68*t_mcp_dot*2.0+exo_x3*l_AH*l_CE*t53*t55*t56*t68*t_mcp_dot*2.0+exo_x3*l_AH*l_EF*t52*t54*t57*t67*t_mcp_dot*2.0+exo_x3*l_AH*l_EF*t52*t55*t57*t66*t_mcp_dot*2.0-exo_x3*l_AH*l_EF*t53*t54*t56*t67*t_mcp_dot*2.0-exo_x3*l_AH*l_EF*t53*t55*t56*t66*t_mcp_dot*2.0+exo_x3*l_CD*l_CE*t52*t54*t57*t68*t_mcp_dot*2.0-exo_x3*l_CD*l_CE*t53*t54*t56*t68*t_mcp_dot*2.0-exo_x3*l_CD*l_EF*t52*t54*t57*t66*t_mcp_dot*2.0+exo_x3*l_CD*l_EF*t53*t54*t56*t66*t_mcp_dot*2.0+exo_x3*l_CD*l_EF*t52*t55*t57*t67*t_mcp_dot*2.0-exo_x3*l_CD*l_EF*t53*t55*t56*t67*t_mcp_dot*2.0-exo_x3*l_AH*l_CE*t54*t56*t57*t68*t_mcp_dot*2.0+exo_x3*l_AH*l_EF*t54*t56*t57*t66*t_mcp_dot*2.0-exo_x3*l_AH*l_EF*t55*t56*t57*t67*t_mcp_dot*2.0-exo_x3*l_CD*l_CE*t55*t56*t57*t68*t_mcp_dot+exo_x3*l_CD*l_EF*t54*t56*t57*t67*t_mcp_dot+exo_x3*l_CD*l_EF*t55*t56*t57*t66*t_mcp_dot+exo_x3*l_AH*l_CE*t52*t53*t55*t70*t_mcp_dot*2.0-exo_x3*l_CD*l_CE*t52*t53*t54*t70*t_mcp_dot-exo_x3*l_AH*l_CE*t52*t54*t57*t70*t_mcp_dot*2.0+exo_x3*l_AH*l_CE*t53*t54*t56*t70*t_mcp_dot*2.0-exo_x3*l_CD*l_CE*t52*t55*t57*t70*t_mcp_dot*2.0+exo_x3*l_CD*l_CE*t53*t55*t56*t70*t_mcp_dot*2.0+exo_x3*l_AH*l_CE*t55*t56*t57*t70*t_mcp_dot*2.0-exo_x3*l_CD*l_CE*t54*t56*t57*t70*t_mcp_dot-l_AH*l_CD*l_CE*t52*t53*t55*t63*t68*2.0+l_AH*l_CD*l_EF*t52*t53*t54*t63*t67*2.0+l_AH*l_CD*l_EF*t52*t53*t55*t63*t66*2.0-l_AH*l_CD*l_CE*t55*t56*t57*t63*t68*2.0+l_AH*l_CD*l_EF*t54*t56*t57*t63*t67*2.0+l_AH*l_CD*l_EF*t55*t56*t57*t63*t66*2.0-l_AH*l_CD*l_CE*t52*t53*t54*t63*t70*2.0-l_AH*l_CD*l_CE*t54*t56*t57*t63*t70*2.0-l_AH*l_CD*l_EF*t52*t53*t54*t67*t76*2.0-l_AH*l_CD*l_EF*t52*t53*t55*t66*t76*2.0-l_AH*l_CD*l_EF*t54*t56*t57*t67*t76*2.0-l_AH*l_CD*l_EF*t55*t56*t57*t66*t76*2.0-l_AH*l_CD*l_CE*t52*t54*t57*t68*t_mcp_dot*2.0+l_AH*l_CD*l_CE*t53*t54*t56*t68*t_mcp_dot*2.0+l_AH*l_CD*l_EF*t52*t54*t57*t66*t_mcp_dot*2.0-l_AH*l_CD*l_EF*t53*t54*t56*t66*t_mcp_dot*2.0-l_AH*l_CD*l_EF*t52*t55*t57*t67*t_mcp_dot*2.0+l_AH*l_CD*l_EF*t53*t55*t56*t67*t_mcp_dot*2.0+l_AH*l_CD*l_CE*t52*t55*t57*t70*t_mcp_dot*2.0-l_AH*l_CD*l_CE*t53*t55*t56*t70*t_mcp_dot*2.0)-exo_x3_dot*l_BC*t102*t141*t41*t73-l_BC*t102*t107*t108*t141*t39;
    J_dot(0,1) = 0.0;
    J_dot(1,1) = 0.0;
    J_dot(2,1) = (l_EF*t101*t97)/t100+l_EF*1.0/(t100*t100)*t96*(l_CE*t142*t143+l_EF*t101*t97);
    J_dot(3,1) = -(exo_t6_dot*l_CE*l_EF*cos(exo_t6_rel))/t145-l_CE*l_EF*1.0/(t145*t145)*sin(exo_t6_rel)*(l_CE*l_HF*t142*t143+l_EF*l_HF*t101*t97);

    // updating the reduced space Jacobian derivative
    Jn_dot(0,0) = J_dot(1,0);
    Jn_dot(1,0) = J_dot(3,0);
    Jn_dot(0,1) = J_dot(1,1);
    Jn_dot(1,1) = J_dot(3,1);

//    cout<<"Jn_dot= "<<Jn_dot<<endl;
}

// Evaluates finger joint torque using the double exponential model
bool exo_finger::evaluate_joint_torque(Vector2d &Tau_finger, Vector2d &Tau_finger_dot)
{

    // MCP Joint torque evaluation (with adapted torque sign convention (couterclockwise positive))
    Tau_finger(0) = -((A_mcp*(exp(-B_mcp*(t_mcp_rel-E_mcp))-1) -
                       C_mcp*(exp(D_mcp*(t_mcp_rel-F_mcp))-1))*0.001); // Nm

    Tau_finger_dot(0) = -(A_mcp*B_mcp*exp(-B_mcp*(t_mcp_rel-E_mcp)) +
                          C_mcp*D_mcp*exp(D_mcp*(t_mcp_rel-F_mcp)))*0.001*t_mcp_dot; // t_mcp_rel_dot = -t_mcp_dot


    // PIP Joint torque evaluation (adapting for torque sign convention (couterclockwise positive))
    Tau_finger(1) = -(A_pip*(exp(-B_pip*(t_pip-E_pip))-1) -
                      C_pip*(exp(D_pip*(t_pip-F_pip))-1))*0.001; // (Nm)
    Tau_finger_dot(1) = -(A_pip*B_pip*exp(-B_pip*(t_pip-E_pip)) +
                          C_pip*D_pip*exp(D_pip*(t_pip-F_pip)))*0.001*t_pip_dot; // t_pip_rel_dot = -t_pip_dot


    //// Expressions for second derivative of finger torque at MCP joint
    //K_1m = (-A_mcp*B_mcp^2*exp(-B_mcp*(t_mcp_rel-E_mcp)) +...
    //    C_mcp*D_mcp^2*exp(D_mcp*(t_mcp_rel-F_mcp)))*10^(-3);
    //K_2m = (A_mcp*B_mcp*exp(-B_mcp*(t_mcp_rel-E_mcp)) +...
    //    C_mcp*D_mcp*exp(D_mcp*(t_mcp_rel-F_mcp)))*10^(-3);

    //// Expressions for second derivative of finger torque a PIP joint
    //K_1p = (-A_mcp*B_mcp^2*exp(-B_mcp*(t_mcp-E_mcp)) +...
    //    C_mcp*D_mcp^2*exp(D_mcp*(t_mcp-F_mcp)))*10^(-3);
    //K_2p = (A_mcp*B_mcp*exp(-B_mcp*(t_mcp-E_mcp)) +...
    //    C_mcp*D_mcp*exp(D_mcp*(t_mcp-F_mcp)))*10^(-3);

    //tau_finger = [tau_mcp; tau_pip];
    //tau_finger_dot = [tau_mcp_dot; tau_pip_dot];
    //K1 = [K_1m, 0;
    //    0, K_1p];
    //K2 = [K_2m, 0;
    //     0, K_2p];

    if(Tau_finger(0)>Tau_finger_max(0) || Tau_finger(0)<Tau_finger_min(0))
    {
        cout <<"MCP Joint Torque exceeding limits!";
        return false;
    }

    if(Tau_finger(1)>Tau_finger_max(1) || Tau_finger(1)<Tau_finger_min(1))
    {
        cout <<"PIP Joint Torque exceeding limits!";
        return false;
    }

    return true;
}

// Evaluates exoskeleton torque and torque derivative given the finger joint torque and torque derivative
bool exo_finger::exo_inverse_statics(Vector2d &Tau_finger, Vector2d &Tau_finger_dot, Vector2d &Tau_exo, Vector2d &Tau_exo_dot)
{

    //[tau_finger,tau_finger_dot,K1,K2] = evaluate_joint_torque(t_mcp,t_pip,...
    //    t_mcp_dot,t_pip_dot);
    //f3 = 0;
    //tau_5 = 0;

    Tau_exo = Jn.transpose()*Tau_finger; // reaction force on the exoskeleton due to torque at the finger
    exo_jacobian_dot();
    Tau_exo_dot = (Jn_dot.transpose()*Tau_finger+Jn.transpose()*Tau_finger_dot);

    cout<<"Tau_exo = "<<Tau_exo.transpose()<<" Tau_exo_dot = "<<Tau_exo_dot.transpose()<<endl;

    //K_1 = K1*Jn+K2*Jn_dot;
    //K_2 = K2*Jn;
    return true;
}

// Evaluates finger torque and torque derivative given the exoskeleton joint torque and torque derivative
bool exo_finger::exo_forward_statics(Vector2d &Tau_finger, Vector2d &Tau_finger_dot, Vector2d &Tau_exo, Vector2d &Tau_exo_dot)
{


    //[tau_finger,tau_finger_dot,K1,K2] = evaluate_joint_torque(t_mcp,t_pip,...
    //    t_mcp_dot,t_pip_dot);
    //f3 = 0;
    //tau_5 = 0;

    Tau_finger = Jn.transpose().inverse()*Tau_exo; // reaction force on the exoskeleton due to torque at the finger
    exo_jacobian_dot();

    Tau_finger_dot = (Jn_dot.transpose().inverse()*Tau_exo+Jn.transpose().inverse()*Tau_exo_dot);

    if(isnan(Tau_finger_dot(0)) || isnan(Tau_finger_dot(1)) || isinf(Tau_finger_dot(0)) || isinf(Tau_finger_dot(1)))
    {
//        cout<<"Tau_finger_dot could not be evaluated!"<<endl;
        Tau_finger_dot(0) = 0;
        Tau_finger_dot(1) = 0;
    }

//    cout<<"Tau_finger = "<<Tau_finger.transpose()<<" Tau_finger_dot = "<<Tau_finger_dot.transpose()<<endl;

    //K_1 = K1*Jn+K2*Jn_dot;
    //K_2 = K2*Jn;
    return true;
}

// Implements a feed-forward control on the exoskeleton joint torque
bool exo_finger::exo_control_FF_exo(Vector2d &Tau_finger_d, Vector2d &Tau_exo_d, Vector2d &Theta_m, Vector2d &Tau_finger, Vector2d &Tau_exo, Vector2d &Theta_m_u)
{

    Vector2d Tau_finger_dot(0,0), Tau_exo_dot(0,0);
    Tau_exo_d = Tau_finger_d;

    Theta_m_u = (1/rm)*((Tau_exo_d.array()/k.array())*(1/(2*rj))+(rj*(T_rel-T_rel0)).array());

    Tau_exo = 2*k.array()*rj*(Theta_m.array()*rm-((T_rel-T_rel0)*rj).array());

    exo_forward_statics(Tau_finger, Tau_finger_dot, Tau_exo, Tau_exo_dot);

    cout<<"Theta_m_u ="<<Theta_m_u.transpose()*180/PI<<endl;

    return true;
}

// Implements a feed-forward+PID control on the exoskeleton joint torque
bool exo_finger::exo_control_FF_PID_exo(Vector2d &Tau_finger_d, Vector2d &Tau_finger_dot_d, Vector2d &Tau_exo_d, Vector2d &Tau_exo_dot_d, Vector2d &Theta_m, Vector2d &Theta_m_dot, Vector2d &Tau_finger, Vector2d &Tau_finger_dot, Vector2d &Tau_exo, Vector2d &Tau_exo_dot, Vector2d &Theta_m_u, Matrix22d &Jn_val, Matrix22d &Jn_dot_val)
{
    Vector2d Tau_exo_err(0,0), Tau_exo_err_dot(0,0);
    Tau_exo_d = Tau_finger_d;
    Tau_exo_dot_d = Tau_finger_dot_d;
    Tau_exo = 2*k.array()*rj*(Theta_m*rm-((T_rel-T_rel0)*rj)).array();
    Tau_exo_dot = 2*k.array()*rj*((Theta_m_dot*rm-T_rel_dot*rj)).array();

    // PID control
    Tau_exo_err = Tau_exo_d-Tau_exo;
    Tau_exo_err_dot = Tau_exo_dot_d-Tau_exo_dot;
    Tau_exo_I = Tau_exo_I+Tau_exo_err; //Ki should take care of dt, so not multiplying dt
    Theta_m_u = (1/rm)*((Tau_exo_d).array()/(2*k.array()*rj)+rj*(T_rel-T_rel0).array())+
            kp_exo.array()*(Tau_exo_d-Tau_exo).array();//+kd_exo.array()*(Tau_exo_dot_d-Tau_exo_dot).array()+
            //ki_exo.array()*Tau_exo_I.array();

    exo_forward_statics(Tau_finger, Tau_finger_dot, Tau_exo, Tau_exo_dot);

//    cout<<T_rel.transpose()<<"\t"<<T_rel0.transpose()<<endl;
    cout<<((1/rm)*((Tau_exo_d.array())/(2*k.array()*rj))).transpose()<<"\t"
        <<((1/rm)*(rj*(T_rel-T_rel0))).transpose()<<"\t"
        <<(kp_exo.array()*(Tau_exo_d-Tau_exo).array()).transpose()<<"\t"
        <<(kd_exo.array()*(Tau_exo_dot_d-Tau_exo_dot).array()).transpose()<<"\t"
        <<(ki_exo.array()*Tau_exo_I.array()).transpose()<<endl;
    cout<<"Theta_m_u ="<<Theta_m_u.transpose()*180/PI<<endl<<endl;

    Jn_val = Jn;
    Jn_dot_val = Jn_dot;
    return true;
}

// Implements a feed-forward control on the finger joint torque
bool exo_finger::exo_control_FF_finger(Vector2d &Tau_finger_d, Vector2d &Tau_finger_dot_d, Vector2d &Tau_exo_d, Vector2d &Tau_exo_dot_d, Vector2d &Theta_m, Vector2d &Theta_m_dot, Vector2d &Tau_finger, Vector2d &Tau_finger_dot, Vector2d &Tau_exo, Vector2d &Tau_exo_dot, Vector2d &Theta_m_u)
{
    Vector2d Tau_finger_err(0,0), Tau_finger_err_dot(0,0), Tau_exo_err(0,0), Tau_exo_err_dot(0,0);

    Tau_exo = 2*k.array()*rj*(Theta_m*rm-(T_rel-T_rel0)*rj).array();
    Tau_exo_dot = 2*k.array()*rj*(Theta_m_dot*rm-T_rel_dot*rj).array();

    Tau_exo_d = Jn.transpose()*Tau_finger_d;
    Theta_m_u = (1/rm)*((Tau_exo_d).array()/(2*k.array()*rj)+rj*(T_rel-T_rel0).array())+
            kp_exo.array()*Tau_exo_err.array()+kd_exo.array()*Tau_exo_err_dot.array()+
            ki_exo.array()*Tau_exo_I.array();

//    fe = [0; yd(1)-y(1); 0; yd(2)-y(2)];
//                fe_dot = [0; yd_dot(1)-y_dot(1); 0; yd_dot(2)-y_dot(2)];
//                err = -J'*fe;
//                err_dot = -(J_dot'*fe+J'*fe_dot);
//                int_err = int_err + err*dt;
//                tau_jd = -J'*[0; yd(1); 0; yd(2)];
//                theta_m_ff = (1/rm)*((tau_jd)/(2*k*rj)+rj*(exo_t-exo_t0));
//                theta_m = theta_m_ff+kp.*err+kd.*err_dot+ki.*int_err; % FF with PID
    return true;
}

// Implements a feed-forward + PID control on the finger joint torque
bool exo_finger::exo_control_FF_PID_finger(Vector2d &Tau_finger_d, Vector2d &Tau_finger_dot_d, Vector2d &Tau_exo_d, Vector2d &Tau_exo_dot_d, Vector2d &Theta_m, Vector2d &Theta_m_dot, Vector2d &Tau_finger, Vector2d &Tau_finger_dot, Vector2d &Tau_exo, Vector2d &Tau_exo_dot, Vector2d &Theta_m_u, Matrix22d &Jn_val, Matrix22d &Jn_dot_val)
{        
        Vector2d Tau_finger_err(0,0), Tau_finger_err_dot(0,0), Tau_exo_err(0,0), Tau_exo_err_dot(0,0);

        Tau_exo = 2*k.array()*rj*(Theta_m*rm-(T_rel-T_rel0)*rj).array();
        Tau_exo_dot = 2*k.array()*rj*(Theta_m_dot*rm-T_rel_dot*rj).array();

        exo_forward_statics(Tau_finger, Tau_finger_dot, Tau_exo, Tau_exo_dot);
//        Tau_finger = -Tau_finger;// torque developed in the finger
//        Tau_finger_dot = -Tau_finger_dot;
        Tau_finger_err = Tau_finger_d-Tau_finger;
        Tau_finger_err_dot = Tau_finger_dot_d-Tau_finger_dot;

        Tau_exo_err = Jn.transpose()*Tau_finger_err;
        exo_jacobian_dot();
        Tau_exo_err_dot = (Jn_dot.transpose()*Tau_finger_err+Jn.transpose()*Tau_finger_err_dot);
        Tau_exo_I = Tau_exo_I+Tau_exo_err; //Ki should take care of dt, so not multiplying dt

        Tau_exo_d = Jn.transpose()*Tau_finger_d;
        Theta_m_u = (1/rm)*((Tau_exo_d).array()/(2*k.array()*rj)+rj*(T_rel-T_rel0).array())+
                kp_fin.array()*Tau_exo_err.array();//+kd_fin.array()*Tau_exo_err_dot.array();//+
//                ki_fin.array()*Tau_exo_I.array();

        cout<<((1/rm)*((Tau_exo_d.array())/(2*k.array()*rj))).transpose()<<"\t"
            <<((1/rm)*(rj*(T_rel-T_rel0))).transpose()<<"\t"
            <<(kp_fin.array()*(Tau_exo_d-Tau_exo).array()).transpose()<<"\t"
            <<(kd_fin.array()*(Tau_exo_dot_d-Tau_exo_dot).array()).transpose()<<"\t"
            <<(ki_fin.array()*Tau_exo_I.array()).transpose()<<endl;
        cout<<"Theta_m_u ="<<Theta_m_u.transpose()*180/PI<<endl<<endl;

        Jn_val = Jn;
        Jn_dot_val = Jn_dot;
        return true;
}


// Implements a feed-forward + PID control on the finger joint torque
bool exo_finger::exo_control_FBL_finger(Vector2d &Tau_finger_d, Vector2d &Tau_finger_dot_d, Vector2d &Tau_finger_ddot_d, Vector2d &Tau_exo_d, Vector2d &Tau_exo_dot_d, Vector2d &Theta_m, Vector2d &Theta_m_dot, Vector2d &Tau_finger, Vector2d &Tau_finger_dot, Vector2d &Tau_exo, Vector2d &Tau_exo_dot, Vector2d &Theta_m_u, Matrix22d &Jn_val, Matrix22d &Jn_dot_val)
{
//    Vector2d Tau_finger_err(0,0), Tau_finger_err_dot(0,0), Tau_exo_err(0,0), Tau_exo_err_dot(0,0), v(0,0);

//        Tau_exo = 2*k.array()*rj*(Theta_m*rm-(T_rel-T_rel0)*rj).array();
//        Tau_exo_dot = 2*k.array()*rj*(Theta_m_dot*rm-T_rel_dot*rj).array();

//        exo_forward_statics(Tau_finger, Tau_finger_dot, Tau_exo, Tau_exo_dot);
//        Tau_finger_err = Tau_finger_d-Tau_finger;
//        Tau_finger_err_dot = Tau_finger_dot_d-Tau_finger_dot;

//        Tau_exo_err = Jn.transpose()*Tau_finger_err;
//        exo_jacobian_dot();
//        Tau_exo_err_dot = (Jn_dot.transpose()*Tau_finger_err+Jn.transpose()*Tau_finger_err_dot);
//        Tau_exo_I = Tau_exo_I+Tau_exo_err; //Ki should take care of dt, so not multiplying dt

//        Tau_exo_d = Jn.transpose()*Tau_finger_d;

//        v = Tau_finger_ddot_d+kp_fbl.array()*Tau_exo_err.array()+kd_fbl.array()*Tau_exo_err_dot.array()+
//                ki_fbl.array()*Tau_exo_I.array();
//        theta_m = (((1/(2*k.array()*rj))*(I_exo/K_2*(-v-K_1*exo_t_dot)-...
//                                  Tau_exo_+B*exo_t_dot))+rj*(T_rel-T_rel0).array())/rm;

//        Theta_m_u = (1/rm)*((Tau_exo_d).array()/(2*k.array()*rj)+rj*(T_rel-T_rel0).array())

//        cout<<((1/rm)*((Tau_exo_d.array())/(2*k.array()*rj))).transpose()<<"\t"
//            <<((1/rm)*(rj*(T_rel-T_rel0))).transpose()<<"\t"
//            <<(kp_fin.array()*(Tau_exo_d-Tau_exo).array()).transpose()<<"\t"
//            <<(kd_fin.array()*(Tau_exo_dot_d-Tau_exo_dot).array()).transpose()<<"\t"
//            <<(ki_fin.array()*Tau_exo_I.array()).transpose()<<endl;
//        cout<<"Theta_m_u ="<<Theta_m_u.transpose()*180/PI<<endl<<endl;

//        Jn_val = Jn;
//        Jn_dot_val = Jn_dot;
        return true;
}



// Function solving for index finger exoskeleton kinematics
bool exo_finger::exo_kinematics_estimation()
{
    bool fail_flag=false;

    double x[2];

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
        exo_t5 = exo_t2+t_DCH-x[1]-2*PI;
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

    return fail_flag;
}

bool exo_finger::exo_estimate_parameters()
{

//    dexo_x3_dt1 = J(0,0)*exo_t1_dot+J(0,1)*exo_t6r_dot;
//    dt_mcp_dt1 = J(2,1);
//    Jp[0][0] = -dexo_x3_dt1*cos(t_mcp)-l_BC*sin(exo_t1)+dt_mcp_dt1*l_CD*cos(t_mcp)+dt_mcp_dt1*exo_x3*sin(t_mcp);
//    Jp[1][1] = l_BC*cos(exo_t1)-dexo_x3_dt1*sin(t_mcp)-dt_mcp_dt1*exo_x3*cos(t_mcp)+dt_mcp_dt1*l_CD*sin(t_mcp);

//    Hp<<Jp,Matrix22d::Identity();


//    xA_hat = l_BC*cos(exo_t1)+l_CD*sin(t_mcp)-exo_x3*cos(t_mcp);
//    yA_hat = l_BC*sin(exo_t1)-l_CD*cos(t_mcp)-exo_x3*sin(t_mcp);

//    cout<<"e_x_A = "<<x_A-xA_hat<<"\t"<<"e_y_A = "<<y_A-yA_hat<<endl;


//    pip_x = l_CE*cos(exo_t5)-l_EF*cos(exo_t5+exo_t6_rel)+l_HF*sin(t_pip-t_HFG)-l_CH*sin(t_mcp+t_DCH);
//    pip_y = l_CE*sin(exo_t5)-l_EF*sin(exo_t5+exo_t6_rel)-l_HF*cos(t_pip-t_HFG)+l_CH*cos(t_mcp+t_DCH);
//    cout<<"pip_x = "<<pip_x<<"\t"<<"pip_y = "<<pip_y<<endl;

    cout<<"exo_t5_err ="<<(exo_t1-exo_t5-exo_t5_rel-PI)*180/PI<<endl;

    P_<<l_AH, l_HF, l_CD;

    // Evaluating Linearized observation matrix with respect to the uncertain parameters
    // Parameter l_AH
    l_AH = l_AH+dl_AH;
    exo_kinematics_estimation();
    exo_t5_val_l_AH(0) = exo_t5;

    l_AH = l_AH-2*dl_AH; // accounting for the addition in the above step
    exo_kinematics_estimation();
    exo_t5_val_l_AH(1) = exo_t5;

    l_AH = l_AH+dl_AH; // maintaining the value of l_AH

    // Parameter l_HF
    l_HF = l_HF+dl_HF;
    exo_kinematics_estimation();
    exo_t5_val_l_HF(0) = exo_t5;

    l_HF = l_HF-2*dl_HF; // accounting for the addition in the above step
    exo_kinematics_estimation();
    exo_t5_val_l_HF(1) = exo_t5;

    l_HF = l_HF+dl_HF; // maintaining the value of l_HF

    // Parameter l_CD
    l_CD = l_CD+dl_CD;
    exo_kinematics_estimation();
    exo_t5_val_l_CD(0) = exo_t5;

    l_CD = l_CD-2*dl_CD; // accounting for the addition in the above step
    exo_kinematics_estimation();
    exo_t5_val_l_CD(1) = exo_t5;

    l_CD = l_CD+dl_CD; // maintaining the value of l_HF

//    cout<<exo_t5_val_l_AH(0) <<"\t"<< exo_t5_val_l_AH(1) <<"\t"<< exo_t5_val_l_HF(0)<<"\t"<< exo_t5_val_l_HF(1)<<endl<<endl;


    Hp<<(exo_t5_val_l_AH(0)-exo_t5_val_l_AH(1))/(2*dl_AH),
        (exo_t5_val_l_HF(0)-exo_t5_val_l_HF(1))/(2*dl_HF),
        (exo_t5_val_l_CD(0)-exo_t5_val_l_CD(1))/(2*dl_CD);

    Kp = (Sigma_p*Hp)*(1/(Hp.transpose()*Sigma_p*Hp+Rp));

    exo_t5_rel_hat = exo_t1-exo_t5-PI;
    P = P_+Kp*(exo_t5_rel-exo_t5_rel_hat);

    Sigma_p = (Matrix33d::Identity()-Kp*Hp.transpose())*Sigma_p;

    Rp = 10000000*(exo_t5_rel-exo_t5_rel_hat)*(exo_t5_rel-exo_t5_rel_hat);
//    Rp = 1000;
    l_AH = P(0);
    l_HF = P(1);
    l_CD = P(2);
//    Kp = (Sigma_p*Hp.transpose())*(1/(Hp*Sigma_p*Hp.transpose()+Rp));

//    cout<<Hp.transpose()*Sigma_p*Hp+Rp<<endl;
    cout <<"Hp = "<<Hp.transpose()<<"\t"<<"Kp = "<<Kp.transpose()<<"\t"<<"P = "<<P.transpose()<<endl;
//            P = P_+Kp*(exo_t5_rel-);
//    Sigma_p = (Matrix22d::Identity(2,2)-Kp;
//        Rp<<
//    dxA_dt1 = -dexo_x3_dt1*cos(t_mcp)-l_BC*sin(exo_t1)+dt_mcp_dt1*l_CD*cos(t_mcp)+dt_mcp_dt1*exo_x3*sin(t_mcp);
//    dyA_dt1 = l_BC*cos(exo_t1)-dexo_x3_dt1*sin(t_mcp)-dt_mcp_dt1*exo_x3*cos(t_mcp)+dt_mcp_dt1*l_CD*sin(t_mcp);
    return true;

}

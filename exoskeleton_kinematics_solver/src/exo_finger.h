//PURPOSE : Class header file implementing the kinematics solution for the hand exoskeleton being developed in the ReNeu Robotics Lab at The University of Texas at Austin
//AUTHORS  : Priyanshu Agarwal
//CONTACT : mail2priyanshu@utexas.edu
//AFFILIATION : The University of Texas at Austin
//To DOs
//1. Add auto calibration functionality to automatically estimate the kinematic model parameters
//2.

/////////////////////////////////////////////////////////////////////
#ifndef DEFINITIONS_H
#define DEFINITIONS_H
#include "definitions.h"
#endif

#define SAMPLE_SIZE 25
#define PARAMETER_SIZE 7
#define MAXEVAL 1e8
#define MAXTIME 600

typedef Matrix<double, SAMPLE_SIZE, 1> VectorNd;

typedef Matrix<double, 4*SAMPLE_SIZE+PARAMETER_SIZE, 1> Vector4Nd;

// Error in loop closing
typedef struct
{
    double l_CE;
    double l_EF;
    VectorNd exo_t1_rel;
    VectorNd exo_t5_rel;
    VectorNd exo_t6_rel;
}data_exo_finger;

class exo_finger {
    double x_A, y_A, r_j, l_AB, l_BC, l_CD, l_CE, l_EF, l_FG, l_GH, l_FI, l_IJ, l_JK, l_GK, l_AH; // user-defined coupled model parameters
    double t_GFK, t_HFG, t_DCH, l_KF, l_HF, l_CH, l_DH; // evaluated model parameters
    double t_abad, t_mcp, t_pip, t_dip; // human finger joint angles
    double exo_abad, exo_t1, exo_t2, exo_x3, exo_t5, exo_t6, exo_t7, exo_t9, exo_t10, exo_x11; // finger exoskeleton angles/displacements
    double exo_t1_rel, exo_t5_rel, exo_t6_rel, exo_t9_rel; // relative finger exoskeleton angles measured by the sensors
  public:
    exo_finger(); // constructor to initialize model parameters and evaluate calculated model parameters
    bool exo_kinematics(double *exo_t, double *estimates); // closed-loop solution for exoskeleton kinematics
    bool exo_kin_eqns_mcp(double *x); // exoskeleton MCP chain kinematic solution
    bool exo_kin_eqns_pip(double *x); // exoskeleton PIP chain kinematic solution
    bool exo_kin_eqns_dip(double *x); // exoskeleton DIP chain kinematic solution
    bool exo_calibration();
    ~exo_finger(); // destructor
  };

//PURPOSE : Definitions file for including relevant header files and defining required constants
//AUTHORS  : Priyanshu Agarwal
//CONTACT : mail2priyanshu@utexas.edu
//AFFILIATION : The University of Texas at Austin
//To DOs
//1.Clean definitions of the motion capture (decide where to put such constants?)
//2.

/////////////////////////////////////////////////////////////////////
#ifndef DEFINITIONS_H
#define DEFINITIONS_H
#endif // DEFINITIONS_H

// Common header files//////////////////////////////////////////////
#include <stdio.h> // For printf statments in the motor control code
#include <unistd.h>
#include <fstream>
#include <stdlib.h>
#include <termio.h>
#include <iomanip>
#include <iostream>

// Maths and Optimization header files//////////////////////////////
#include <math.h>
#include <Eigen/Dense>
//#include <nlopt.hpp>
//#include <curses.h>

// General Constants////////////////////////////////////////////////
#define PI 3.141592

// Motion capture constants (change these to match your configuration)
//#define MARKER_COUNT 8
//#define SERVER_NAME "146.6.84.115"
//#define FILE_NAME_MOTION_CAPTURE_CONFIG "motion_capture_config.txt"
//#define INIT_FLAGS 0
//#define CAPTURE_FREQ 480
//#define TICK_PERIOD 1000000000
//#define TASK_PRIORITY 1
//#define STACK_SIZE 10000000
//#define FIFO 0

// Real-time Control Variables/////////////////////////////////////
//#define MOTOR_CONTROL_FREQ 500
#define CONTROL_LOOP_FREQ 500
#define ESTIMATION_FREQ 100
#define NCOUNTS 10
#define PLOT_FREQ 25
#define CPU_MAP 0xF
//#define FIFO 1

// Gnuplot/////////////////////////////////////////////////////////
#define NPOINTS 50

// Hall-sensor Calibration Variables///////////////////////////////
#define MCP_FLEX_ANG 40.0f
#define MCP_EXT_ANG 0.0f
#define MCP_PIP_FLEX_ANG 180.0f// MCP-PIP relative flexion
#define MCP_PIP_EXT_ANG 90.0f// MCP-PIP relative extension
#define PIP_FLEX_ANG 90.0f// 338.53f
#define PIP_EXT_ANG 140.0f//252.9f
//#define DIP_FLEX_ANG 30.0f//338.53f
//#define DIP_EXT_ANG 90.0f//252.9f

#define MCP_FLEX_V 2429.0f
#define MCP_EXT_V 1507.0f
#define MCP_PIP_FLEX_V 1510// MCP-PIP relative flexion //
#define MCP_PIP_EXT_V 3906// MCP-PIP relative extension //
#define PIP_FLEX_V 2000//3890.0f
#define PIP_EXT_V 940//1590.0f
//#define DIP_FLEX_V 2814//3890.0f
//#define DIP_EXT_V 630//1590.0f

// Maxon Motor Constants//////////////////////////////////////////
#define TRUE 1
#define FALSE 0
#define MOTOR_ANGLE_TO_VOLTAGE (295/(15*PI))
#define MOTOR_COUNT_TO_ANGLE (2*PI/(4*295000))

// Motor Limits
#define MCP_MOTOR_ANGLE_LIMIT_LOWER (-PI/3)
#define MCP_MOTOR_ANGLE_LIMIT_UPPER (PI/4)
#define PIP_MOTOR_ANGLE_LIMIT_LOWER (-PI/4)
#define PIP_MOTOR_ANGLE_LIMIT_UPPER (PI/5)

using namespace Eigen;
using namespace std;


typedef Matrix<double, 2, 1> Vector2d;
typedef Matrix<double, 3, 1> Vector3d;
typedef Matrix<double, 4, 1> Vector4d;
//typedef Matrix<double, SAMPLE_SIZE, 1> VectorNd;
typedef Matrix<double, 2, 2> Matrix22d;
typedef Matrix<double, 3, 3> Matrix33d;
typedef Matrix<double, 4, 2> Matrix42d;
typedef Matrix<double, 2, 4> Matrix24d;
typedef Matrix<double, 10, 2> Matrix102d;


//typedef struct {
//    VectorNd tau_m;
//    VectorNd theta_m;
//} joint_torque;

////#define SAMPLE_SIZE 10000
//#define MAXEVAL 1e8
//#define MAXTIME 600

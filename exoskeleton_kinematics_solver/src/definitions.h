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
#include <nlopt.hpp>
#include <curses.h>

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
#define MOTOR_CONTROL_FREQ 50
#define ENCODER_READ_FREQ 1000
#define PLOT_FREQ 25
#define CPU_MAP 0xF
//#define FIFO 1

// Gnuplot/////////////////////////////////////////////////////////
#define NPOINTS 50

// Hall-sensor Calibration Variables///////////////////////////////
#define MCP_FLEX_ANG 42.0f
#define MCP_EXT_ANG 0.0f
#define MCP_PIP_FLEX_ANG 180.0f// MCP-PIP relative flexion
#define MCP_PIP_EXT_ANG 90.0f// MCP-PIP relative extension
#define PIP_FLEX_ANG 90.0f// 338.53f
#define PIP_EXT_ANG 45.0f//252.9f
#define DIP_FLEX_ANG 180.0f//338.53f
#define DIP_EXT_ANG 90.0f//252.9f

#define MCP_FLEX_V 800.0f
#define MCP_EXT_V 1732.0f
#define MCP_PIP_FLEX_V 1066// MCP-PIP relative flexion
#define MCP_PIP_EXT_V 3303// MCP-PIP relative extension
#define PIP_FLEX_V 555//3890.0f
#define PIP_EXT_V 1550//1590.0f
#define DIP_FLEX_V 2814//3890.0f
#define DIP_EXT_V 630//1590.0f

// Servo Motor Constants//////////////////////////////////////////
// Control table address
//#define P_GOAL_POSITION_L	30
//#define P_GOAL_POSITION_H	31
//#define P_PRESENT_POSITION_L	36
//#define P_PRESENT_POSITION_H	37
//#define P_MOVING		46
//#define P_TORQUE_ENABLE         24

//#define DEFAULT_BAUDNUM		1 // 1Mbps
//#define MCP_ID		0
//#define PIP_ID		1


using namespace Eigen;
using namespace std;

//typedef Matrix<double, 6, 1> Vector6d;
//typedef Matrix<double, SAMPLE_SIZE, 1> VectorNd;

//typedef struct {
//    VectorNd tau_m;
//    VectorNd theta_m;
//} joint_torque;

////#define SAMPLE_SIZE 10000
//#define MAXEVAL 1e8
//#define MAXTIME 600

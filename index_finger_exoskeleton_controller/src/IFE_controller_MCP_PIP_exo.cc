//PURPOSE : Solve kinematics of the hand exoskeleton being developed in the ReNeu Robotics Lab at The University of Texas at Austin
//AUTHORS  : Priyanshu Agarwal
//CONTACT : mail2priyanshu@utexas.edu
//AFFILIATION : The University of Texas at Austin
//To DOs
//1. Make class for motion capture device
//2. Make class for motor controller
//3. Make class for sensor (NI stuff)
//4. Add automatic calibration code as function. Make a calibration procedure where the zero angle error is corrected.

// Before running the code, please check the following:
// 1. Check the sensors are calibrated and working with no interference
// 2. Check that the pose estimation code with working using GUI
// 3. Check the values of the Trel0 which is the initial angle value
// of the two actuated exoskeleton joints.
// 4. Start with the FF exo torque control and then finger torque control
// 5.
/////////////////////////////////////////////////////////////////////
#ifndef DEFINITIONS
#define DEFINITIONS
#include "definitions.h"
#endif

// Exoskeleton Kinematics/Dynamics header files
#ifndef EXO_FINGER
#define EXO_FINGER
#include "exo_finger.h"
#endif

// Encoder header files
#include "NiFpga_exoskeleton_controller.h"

//Maxon Motor header files
#include "maxon_motor.h"

// RTAI header files
#include <rtai.h>
#include <rtai_sched.h>
#include <rtai_fifos.h>
#include"rtai_nam2num.h"

// Plotting
extern "C" {
#include "gnuplot_i.h"
}

#include <termio.h>
#include <signal.h>
#include <semaphore.h>

// UDP Communication
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define FIFO 1
#define SERVER_ADDRESS  "146.6.84.187" //"146.6.84.240"
#define UDP_FREQ 1000

// Teleopration related variables
#define MCP_JOINT_RANGE 70.0
#define PIP_JOINT_RANGE 150.0

static RT_TASK *main_task, *task;
int running_flag = 1;
RTIME counter_time=0;

//// Motion capture library
//#ifndef MOTION_CAPTURE_H
//#define MOTION_CAPTURE_H
//#include "owl.h"
//#include "motion_capture.h"
//#endif // MOTION_CAPTURE_H

ofstream sensor_data_file;
ifstream recorded_data_file;
//sem_t sync_flag_1, sync_flag_2, killing_flag, thread_flag; // declare semaphore (memory location shared between threads)
double exo_t_rel[5];

double exo_pip_torque = 0;

double estimates[12]={0,0,0,0,0,0,0,0,0,0,0,0};
float pose_data[3]={0,0,0};

//int32_t encoder_data_I32[6] = {0, 0, 0, 0, 0, 0};

// Motor flag
bool disable_motor = false;
//bool disable_motor = true;

// FIFO flag
bool fifo_flag = true;
//bool fifo_flag = false;

// Control Type
//char control_type[]="FF_PIP_Exo"; //feed_forward exo PIP joint
//char control_type[]="FF_PID_PIP_Exo"; //feed_forward+PID exo PIP joint

//char control_type[]="FF_PID_MCP_Exo"; //feed_forward+PID exo MCP joint

char control_type[]="FF_PID_MCP_PIP_Exo"; //feed_forward+PID exo MCP+PIP joint

//char control_type[]="DT_MCP"; //dynamic transparency feed_forward+PID exo MCP joint
//char control_type[]="DT_PIP"; //dynamic transparency feed_forward+PID exo PIP joint
//char control_type[]="DT_MCP_PIP"; //dynamic transparency feed_forward+PID exo MCP+PIP joint

// Desired trajectory variables
double tau_f=0.5, tau_phi_mcp=0, tau_phi_pip=PI;//-PI/2;
//double tau_A_mcp=0.35; // MCP
//double tau_A_pip=0.058; // PIP
//double tau_mcp_sf = -0.1, tau_pip_sf = 0.05;

double tau_A_mcp=0.25; // MCP
double tau_A_pip=0.04; // PIP
double tau_mcp_sf = -0.1, tau_pip_sf = 0.1;

//double tau_A_pip=0.01; // PIP

/////////////////////////////////////////Index Finger Controller Thread//////////////////////////////////////////
void* IFE_controller(void *args)
{
    unsigned long task_name =nam2num("ENCODER");

    int i;
    //    bool flag_points=0;

    double exo_mcp_rel_angle=0, exo_mcp_pip_rel_angle=0, exo_pip_rel_angle = 0, exo_dip_rel_angle=0; //exo_prox_angle = 0
    RTIME old_time;
//    RTIME current_time=0;
    double t= 0;

    // NI FPGA data variables
    int32_t sensor_data_I32[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int32_t motordata_I32[6] = {-6000,-4000,0,0,0,0};

    // Finger exoskeleton related variables    
    Vector2d Theta_r(0,0), Theta_r_prev(0,0), Theta_r_dot(0,0);
    Vector2d Tau_finger_d(0,0), Tau_finger_dot_d(0,0);
    Vector2d Tau_finger(0,0), Tau_finger_dot(0,0);
    Vector2d Tau_exo_d(0,0), Tau_exo_dot_d(0,0);
    Vector2d Tau_exo(0,0), Tau_exo_dot(0,0);
    Vector2d Theta_m(0,0), Theta_m_dot(0,0); // current motor state variables
    Vector2d Theta_m_u(0,0); // desired motor position

    // Data logging variables
    bool log_data_flag = true;
    bool screen_flag = false;
    char sensor_data_filename[1000];

    cout<<"Running accuracy test!";

    if(strcmp(control_type,"FF_PIP_Exo")==0)
        sprintf(sensor_data_filename,"data/sensor_data_%s_f_%lf_tau_pip_%lf_%d.csv",control_type,tau_f,tau_A_pip,disable_motor);
    else
    {
        if(strcmp(control_type,"FF_PID_PIP_Exo")==0)
        {
            tau_A_mcp=0.0;
            sprintf(sensor_data_filename,"data/sensor_data_%s_f_%lf_tau_pip_%lf_%d.csv",control_type,tau_f,tau_A_pip,disable_motor);
        }
        else
        {
            if(strcmp(control_type,"FF_PID_MCP_Exo")==0)
            {
                tau_A_pip=0.0;
                sprintf(sensor_data_filename,"data/sensor_data_%s_f_%lf_tau_mcp_%lf_%d.csv",control_type,tau_f,tau_A_mcp,disable_motor);
            }

            else
            {

                if(strcmp(control_type,"FF_PID_MCP_PIP_Exo")==0)
                {
                    sprintf(sensor_data_filename,"data/sensor_data_%s_f_%lf_tau_mcp_%lf_tau_pip_%lf_%d.csv",control_type,tau_f,tau_A_mcp,tau_A_pip,disable_motor);
                }

                else
                {
                    if(strcmp(control_type,"DT_PIP")==0)
                    {
                        tau_A_pip=0.0;
                        sprintf(sensor_data_filename,"data/sensor_data_%s_f_%lf_tau_pip_%lf_%d.csv",control_type,tau_f,tau_A_pip,disable_motor);
                    }

                    else
                    {
                        if(strcmp(control_type,"DT_MCP_PIP")==0)
                        {
                            tau_A_mcp=0.0;
                            tau_A_pip=0.0;
                            sprintf(sensor_data_filename,"data/sensor_data_%s_f_%lf_tau_mcp_%lf_tau_pip_%lf_%d.csv",control_type,tau_f,tau_A_mcp,tau_A_pip,disable_motor);
                        }
                    }
                }
            }
        }
    }

//    sprintf(sensor_data_filename,"sensor_data.csv");

    sensor_data_file.open(sensor_data_filename);

    NiFpga_Session session;
    cout<<"Initializing..."<<endl;
    /* must be called before any other calls */
    NiFpga_Status status = NiFpga_Initialize();
    if (NiFpga_IsNotError(status))
    {
        /* opens a session, downloads the bitstream, and runs the FPGA */
        cout<<"Opening a session...\n"<<endl;
        NiFpga_MergeStatus(&status, NiFpga_Open(NiFpga_exoskeleton_controller_Bitfile,
                                                NiFpga_exoskeleton_controller_Signature,
                                                "rio://146.6.88.50/RIO0",//"rio://10.0.0.1/RIO0",
                                                NiFpga_OpenAttribute_NoRun,
                                                &session));

        if (NiFpga_IsNotError(status))
        {

            /* run the FPGA application */
            cout<<"Running the FPGA...\n"<<endl;
            NiFpga_MergeStatus(&status, NiFpga_Run(session, 0));

            if (!(task = rt_task_init_schmod(task_name, 0, 0, 0, SCHED_FIFO, CPU_MAP-1)))
            {
                cout<<"CANNOT INIT TASK"<<task_name<<endl;
                return 0;
            }

            // Reset motor position
            NiFpga_MergeStatus(&status,NiFpga_WriteBool(session,NiFpga_exoskeleton_controller_ControlBool_reset_motor_mcp,0));
            NiFpga_MergeStatus(&status,NiFpga_WriteBool(session,NiFpga_exoskeleton_controller_ControlBool_reset_motor_mcp,1));

            NiFpga_MergeStatus(&status,NiFpga_WriteBool(session,NiFpga_exoskeleton_controller_ControlBool_reset_motor_pip,0));
            NiFpga_MergeStatus(&status,NiFpga_WriteBool(session,NiFpga_exoskeleton_controller_ControlBool_reset_motor_pip,1));

//            usleep(1000000);
            // Initializing the index_finger for initial exoskeleton angles constructor
            NiFpga_MergeStatus(&status,  NiFpga_ReadArrayI32(session,NiFpga_exoskeleton_controller_IndicatorArrayI32_sensordata,sensor_data_I32,20));
            // converting voltage data to angles (FILTERED)
            exo_mcp_rel_angle = (MCP_FLEX_ANG-MCP_EXT_ANG)/(MCP_FLEX_V-MCP_EXT_V)*(sensor_data_I32[18]-MCP_EXT_V)+MCP_EXT_ANG;
            exo_pip_rel_angle = (PIP_FLEX_ANG-PIP_EXT_ANG)/(PIP_FLEX_V-PIP_EXT_V)*(sensor_data_I32[19]-PIP_EXT_V)+PIP_EXT_ANG;
            cout<<"Initializing with the following angles!"<<endl;
            cout<<exo_mcp_rel_angle<<"\t"<<exo_pip_rel_angle<<endl;
            Theta_r(0) = exo_mcp_rel_angle*PI/180;
            Theta_r(1) = exo_pip_rel_angle*PI/180;
            exo_finger index_finger(Theta_r);
            Theta_r_prev = Theta_r;

            // Going hard-real-time
            cout<<"THREAD INIT EXOSKELETON CONROLLER: name = "<<task_name<<", address = "<<task<<endl;
            mlockall(MCL_CURRENT | MCL_FUTURE);
            rt_make_hard_real_time();

            counter_time = rt_get_time_ns();
            while(running_flag)
            {
                old_time = rt_get_time_ns();
                t = (old_time-counter_time)/1e9;

                // reading encoder data from sbRIO
                NiFpga_MergeStatus(&status,NiFpga_ReadArrayI32(session,NiFpga_exoskeleton_controller_IndicatorArrayI32_sensordata,sensor_data_I32,20));

                // displaying raw sensor data
//                cout<<sensor_data_I32[0]<<"\t"<<sensor_data_I32[1]<<endl;
//                cout<<sensor_data_I32[18]<<"\t"<<sensor_data_I32[19]<<endl;

                //                // converting voltage data to angles
                //                //                exo_abd_angle = (ABD_ANG-MCP_EXT_ANG)/(MCP_FLEX_V-MCP_EXT_V)*(encoder_data_I32[0]-MCP_EXT_V)+MCP_EXT_ANG;
                //                exo_mcp_rel_angle = (MCP_FLEX_ANG-MCP_EXT_ANG)/(MCP_FLEX_V-MCP_EXT_V)*(encoder_data_I32[0]-MCP_EXT_V)+MCP_EXT_ANG;
                //                exo_mcp_pip_rel_angle = (MCP_PIP_FLEX_ANG-MCP_PIP_EXT_ANG)/(MCP_PIP_FLEX_V-MCP_PIP_EXT_V)*(encoder_data_I32[1]-MCP_PIP_EXT_V)+MCP_PIP_EXT_ANG;
                //                exo_pip_rel_angle = (PIP_FLEX_ANG-PIP_EXT_ANG)/(PIP_FLEX_V-PIP_EXT_V)*(encoder_data_I32[2]-PIP_EXT_V)+PIP_EXT_ANG;
                //                exo_dip_rel_angle = (DIP_FLEX_ANG-DIP_EXT_ANG)/(DIP_FLEX_V-DIP_EXT_V)*(encoder_data_I32[3]-DIP_EXT_V)+DIP_EXT_ANG;

//                // converting voltage data to angles (UNFILTERED)
//                exo_mcp_rel_angle = (MCP_FLEX_ANG-MCP_EXT_ANG)/(MCP_FLEX_V-MCP_EXT_V)*(sensor_data_I32[0]-MCP_EXT_V)+MCP_EXT_ANG;
//                exo_pip_rel_angle = (PIP_FLEX_ANG-PIP_EXT_ANG)/(PIP_FLEX_V-PIP_EXT_V)*(sensor_data_I32[1]-PIP_EXT_V)+PIP_EXT_ANG;
//                exo_dip_rel_angle = PI-0.1*exo_pip_rel_angle;//(DIP_FLEX_ANG-DIP_EXT_ANG)/(DIP_FLEX_V-DIP_EXT_V)*(sensor_data_I32[2]-DIP_EXT_V)+DIP_EXT_ANG;
//                cout<<exo_mcp_rel_angle<<"\t"<<exo_pip_rel_angle<<"\t"<<exo_dip_rel_angle<<endl;

                // converting voltage data to angles (FILTERED)
                exo_mcp_rel_angle = (MCP_FLEX_ANG-MCP_EXT_ANG)/(MCP_FLEX_V-MCP_EXT_V)*(sensor_data_I32[18]-MCP_EXT_V)+MCP_EXT_ANG;
                exo_pip_rel_angle = (PIP_FLEX_ANG-PIP_EXT_ANG)/(PIP_FLEX_V-PIP_EXT_V)*(sensor_data_I32[19]-PIP_EXT_V)+PIP_EXT_ANG;
                exo_dip_rel_angle = PI-0.1*exo_pip_rel_angle;//(DIP_FLEX_ANG-DIP_EXT_ANG)/(DIP_FLEX_V-DIP_EXT_V)*(sensor_data_I32[2]-DIP_EXT_V)+DIP_EXT_ANG;
                cout<<exo_mcp_rel_angle<<"\t"<<exo_pip_rel_angle<<"\t"<<exo_dip_rel_angle<<endl;
                Theta_r << exo_mcp_rel_angle*PI/180 , exo_pip_rel_angle*PI/180;

                // converting voltage data to velocities
//                Theta_r_dot(0) = ((MCP_FLEX_ANG-MCP_EXT_ANG)/(MCP_FLEX_V-MCP_EXT_V)*(sensor_data_I32[14]))*(PI/180)*1000; // exo_t1_rel (rad/s)
//                Theta_r_dot(1) = ((PIP_FLEX_ANG-PIP_EXT_ANG)/(PIP_FLEX_V-PIP_EXT_V)*(sensor_data_I32[16]))*(PI/180)*1000; // exo_t6_rel (rad/s)
                Theta_r_dot = (Theta_r-Theta_r_prev)*CONTROL_LOOP_FREQ; // rad/s
                cout<<"Theta_r_dot = "<<Theta_r_dot.transpose()*(180/PI)<<endl;

                // converting motor counts to angles
                Theta_m(0) = -sensor_data_I32[8]*MOTOR_COUNT_TO_ANGLE;
                Theta_m(1) = -sensor_data_I32[11]*MOTOR_COUNT_TO_ANGLE;

                // converting motor counts to velocity
                Theta_m_dot(0) = -sensor_data_I32[9]*MOTOR_COUNT_TO_ANGLE*1000; // rad/s
                Theta_m_dot(1) = -sensor_data_I32[12]*MOTOR_COUNT_TO_ANGLE*1000; // rad/s
                cout<<"Theta_m_dot = "<<Theta_m_dot.transpose()*(180/PI)<<endl;

                exo_t_rel[0] = exo_mcp_rel_angle*PI/180;
                exo_t_rel[1] = exo_mcp_pip_rel_angle*PI/180;
                exo_t_rel[2] = exo_pip_rel_angle*PI/180;
                exo_t_rel[3] = exo_dip_rel_angle*PI/180;
                exo_t_rel[4] = 0;

                index_finger.exo_kinematics(exo_t_rel,estimates,Theta_r_dot);
//                if(!index_finger.exo_kinematics(exo_t_rel,estimates,Theta_r_dot))
//                    running_flag = 0;
                //                index_finger.exo_jacobian();
                //                index_finger.exo_jacobian_dot();
                //                index_finger.exo_statics(Tau_finger_input, Tau_finger_dot_input);

                // Desired finger torque
                Tau_finger_d(0) = tau_A_mcp*sin(2*PI*tau_f*t+tau_phi_mcp)+tau_mcp_sf*tau_A_mcp;
                Tau_finger_d(1) = tau_A_pip*sin(2*PI*tau_f*t+tau_phi_pip)+tau_pip_sf*tau_A_pip;

                // Desired finger torque derivative
                Tau_finger_dot_d(0) = 2*PI*tau_f*tau_A_mcp*cos(2*PI*tau_f*t+tau_phi_mcp);
                Tau_finger_dot_d(1) = 2*PI*tau_f*tau_A_pip*cos(2*PI*tau_f*t+tau_phi_pip);

                //                Tau_finger_d(1) = 0.045;

                if(strcmp(control_type,"FF_PIP_Exo")==0)
                    index_finger.exo_control_FF_exo(Tau_finger_d, Tau_exo_d, Theta_m, Tau_finger, Tau_exo, Theta_m_u);
                else
                {
                    if((strcmp(control_type,"FF_PID_PIP_Exo")==0)||(strcmp(control_type,"FF_PID_MCP_Exo")==0)||
                            (strcmp(control_type,"FF_PID_MCP_PIP_Exo")==0))
                        index_finger.exo_control_FF_PID_exo(Tau_finger_d, Tau_finger_dot_d, Tau_exo_d, Tau_exo_dot_d, Theta_m, Theta_m_dot, Tau_finger, Tau_finger_dot, Tau_exo, Tau_exo_dot, Theta_m_u);
                    else
                    {
                        if(strcmp(control_type,"DT_PIP")==0)
                        {
                            Tau_finger_d(0) = 0;
                            Tau_finger_d(1) = exo_pip_torque;
                            index_finger.exo_control_FF_PID_exo(Tau_finger_d, Tau_finger_dot_d, Tau_exo_d, Tau_exo_dot_d, Theta_m, Theta_m_dot, Tau_finger, Tau_finger_dot, Tau_exo, Tau_exo_dot, Theta_m_u);
                        }

                        else
                        {
                            if(strcmp(control_type,"DT_MCP_PIP")==0)
                            {
                                Tau_finger_d(0) = 0;
                                Tau_finger_d(1) = 0;
                                index_finger.exo_control_FF_PID_exo(Tau_finger_d, Tau_finger_dot_d, Tau_exo_d, Tau_exo_dot_d, Theta_m, Theta_m_dot, Tau_finger, Tau_finger_dot, Tau_exo, Tau_exo_dot, Theta_m_u);
                            }
                        }
                    }
                }

//                cout<<"Theta_m_u ="<<Theta_m_u.transpose()*180/PI<<endl;

//                current_time = rt_get_time_ns();//clock();

//                cout<<"("<<(current_time-old_time)<<")";


                //                if(Theta_m_u(1)>MOTOR_ANGLE_LIMIT_LOWER && Theta_m_u(1)<MOTOR_ANGLE_LIMIT_UPPER)
                if(Theta_m_u(0)>MCP_MOTOR_ANGLE_LIMIT_LOWER && Theta_m_u(0)<MCP_MOTOR_ANGLE_LIMIT_UPPER &&
                        Theta_m_u(1)>PIP_MOTOR_ANGLE_LIMIT_LOWER && Theta_m_u(1)<PIP_MOTOR_ANGLE_LIMIT_UPPER)
                {
                    motordata_I32[0] = Theta_m_u(0)*MOTOR_ANGLE_TO_VOLTAGE*1000; // mV
                    motordata_I32[1] = Theta_m_u(1)*MOTOR_ANGLE_TO_VOLTAGE*1000; // mV
                    // writing motor position data to sbRIO
                    NiFpga_MergeStatus(&status,NiFpga_WriteArrayI32(session,NiFpga_exoskeleton_controller_ControlArrayI32_motordata,motordata_I32,6));
                }
                else
                {
                    cout<<"Motor angle limit crossed!"<<Theta_m_u.transpose()<<endl;
                    running_flag=0;
                }

//                // Printing the estimated angles on screen
//                for(i=0;i<12;i++)
//                {
//                    if(i==2||i==11)
//                        cout<<"\t"<<estimates[i];
//                    else
//                        cout<<"\t"<<estimates[i]*180/PI;
//                }
//                cout<<endl;

                // Printing controller data on screen
                if(screen_flag)
                {
                    cout<<"("<<t<<")"<<showpoint<<showpos<<"\t"<<Theta_m_u(0)*180/PI<<"\t"<<Theta_m_u(1)*180/PI<<"\t"
                       <<Theta_m(0)*180/PI<<"\t"<<Theta_m(1)*180/PI<<"\t"
                      <<Tau_finger_d(0)<<"\t"<<Tau_finger_d(1)<<"\t"
                     <<Tau_exo(0)<<"\t"<<Tau_exo(1)<<endl;

                    //                cout<<t<<showpoint<<showpos<<"\t"<<Theta_m_u(0)<<"\t"<<Theta_m_u(1)<<"\t"
                    //                   <<Theta_m(0)*180/PI<<"\t"<<Theta_m(1)*180/PI<<"\t"
                    //                  <<Theta_m_dot(0)*180/PI<<"\t"<<Theta_m_dot(0)*180/PI<<"\t"
                    //                 <<Tau_finger_d(0)<<"\t"<<Tau_finger_d(1)<<"\t"
                    //                <<Tau_finger_dot_d(0)<<"\t"<<Tau_finger_dot_d(1)<<"\t"
                    //                <<Tau_exo(0)<<"\t"<<Tau_exo(1)<<"\t"
                    //                <<Tau_exo_dot(0)<<"\t"<<Tau_exo_dot(1)<<"\t"<<endl;

                    //                      Theta_m_u.transpose()*180/PI<<"\t"
                    //                   <<Theta_m.transpose()*180/PI<<"\t"<<Theta_m_dot.transpose()*180/PI
                    //                   <<"\t"<<theta_j*180/PI<<"\t"<<theta_j_dot*180/PI<<"\t"<<kp*(tau_d-tau_j_hat)*180/PI
                    //                   <<"\t"<<kd*(tau_d_dot-tau_j_hat_dot)<<"\t"<<ki*tau_j_hat_I<<endl;

                }

                // writing the encoder data to the file
                if(log_data_flag)
                {
                    sensor_data_file<<t;
                    for(i=0;i<12;i++)
                        sensor_data_file<<"\t"<<estimates[i];
                    sensor_data_file<<"\t"<<exo_t_rel[0]<<"\t"<<exo_t_rel[1]<<"\t"<<exo_t_rel[2]<<"\t"<<exo_t_rel[3];
                    sensor_data_file<<"\t"<<Theta_r_dot(0)<<"\t"<<Theta_r_dot(1);

                    // writing the sensed/estimated data to the file
                    sensor_data_file<<"\t"<<Theta_m_u(0)*180/PI<<"\t"<<Theta_m_u(1)*180/PI<<"\t"
                                   <<Theta_m(0)*180/PI<<"\t"<<Theta_m(1)*180/PI<<"\t"
                                  <<Theta_m_dot(0)*180/PI<<"\t"<<Theta_m_dot(0)*180/PI<<"\t"
                                 <<Tau_finger_d(0)<<"\t"<<Tau_finger_d(1)<<"\t"
                                <<Tau_finger_dot_d(0)<<"\t"<<Tau_finger_dot_d(1)<<"\t"
                               <<Tau_finger(0)<<"\t"<<Tau_finger(1)<<"\t"
                                 <<Tau_finger_dot(0)<<"\t"<<Tau_finger_dot(1)<<"\t"
                              <<Tau_exo_d(0)<<"\t"<<Tau_exo_d(1)<<"\t"
                             <<Tau_exo_dot_d(0)<<"\t"<<Tau_exo_dot_d(1)<<"\t"
                            <<Tau_exo(0)<<"\t"<<Tau_exo(1)<<"\t"
                           <<Tau_exo_dot(0)<<"\t"<<Tau_exo_dot(1)<<"\t";

                    sensor_data_file<<endl;
                }

                // writing data to FIFO
                if(fifo_flag)
                {
                    pose_data[0] = (2*PI-estimates[3]);
                    pose_data[1] = (2*PI-estimates[7])-pose_data[0];
                    pose_data[2] = 0.3*pose_data[1];//(2*PI-estimates[11])-pose_data[0]-pose_data[1];
//                    pose_data[0] = estimates[12];
//                    pose_data[1] = estimates[13];
//                    pose_data[2] = estimates[14];
//                    cout<<pose_data[0]<<"\t"<<pose_data[1]<<"\t"<<pose_data[2]<<endl;
                    rtf_put(FIFO, (char*)&pose_data, sizeof(pose_data));
                }

                Theta_r_prev = Theta_r;
                rt_sleep_until(nano2count(old_time+1000000000/CONTROL_LOOP_FREQ));
            }

            /* close the session now that we're done */
            cout<<"Closing the session..."<<endl;

            /* must close if we successfully opened */
            NiFpga_MergeStatus(&status, NiFpga_Close(session, 0));

        }
        /* must be called after all other calls */
        cout<<"Finalizing..."<<endl;
        NiFpga_MergeStatus(&status, NiFpga_Finalize());

    }
    /* check if anything went wrong */
    else
        if (NiFpga_IsError(status))
        {
            cout<<"Error!"<<status<<endl;
            cout<<"Press <Enter> to quit..."<<endl;
            getchar();
        }

    sensor_data_file.close();

    usleep(2000000);
    rt_make_soft_real_time();
    rt_task_delete(task);
    return 0;
}


/////////////////////////////////////////Recorded Data Estimation Thread////////////////////////////////////////////////////
//void* recorded_estimation(void *args)
//{
//    unsigned long task_name =nam2num("ENCODER");

//    int i;//,j=0;
//    RTIME old_time, current_time=0;
//    double t= 0;

//    exo_finger index_finger;
//    Vector2d Theta_r_dot;
//    Vector2d Tau_finger_d, Tau_finger_dot_d;
//    Vector2d Theta_m_u;

////    double x[6]={0.015,7*PI/4,PI/4,7*PI/4,0.005,7*PI/4};
////    double x[6];

//    recorded_data_file.open("encoder_data_YY_E_high.txt");

//    if (!(task = rt_task_init_schmod(task_name, 0, 0, 0, SCHED_FIFO, CPU_MAP-1)))
//    {
//        cout<<"CANNOT INIT TASK"<<task_name<<endl;
//        return 0;
//    }

//    cout<<"THREAD INIT EXOSKELETON CONTROLLER: name = "<<task_name<<", address = "<<task<<endl;
//    mlockall(MCL_CURRENT | MCL_FUTURE);
//    rt_make_hard_real_time();

//    while(running_flag)
//    {
//        old_time = rt_get_time_ns();
//        t = (old_time-counter_time)/1e9;

//        recorded_data_file>>exo_t_rel[0]>>exo_t_rel[2]>>exo_t_rel[3];

//        Theta_r_dot(0) = 0.1; // exo_t1r_dot
//        Theta_r_dot(1) = 0.1; // exo_t6r_dot
//        Tau_finger_d(0) = 0.1;
//        Tau_finger_d(1) = 0.1;
//        Tau_finger_dot_d(0) = 0.1;
//        Tau_finger_dot_d(1) = 0.1;

//        index_finger.exo_kinematics(exo_t_rel,estimates,Theta_r_dot);
////        index_finger.exo_jacobian();
////        index_finger.exo_jacobian_dot();
////        index_finger.exo_statics(Tau_finger_input, Tau_finger_dot_input);

//        running_flag = index_finger.exo_control_FF(Tau_finger_d, Tau_finger_dot_d, Theta_m_u);

//        // writing the encoder data to the file
//        encoder_data_file<<t;
//        for(i=0;i<12;i++)
//            encoder_data_file<<"\t"<<estimates[i];
//        encoder_data_file<<"\t"<<exo_t_rel[0]<<"\t"<<exo_t_rel[2]<<"\t"<<exo_t_rel[3];
//        encoder_data_file<<endl;

//        rt_sleep_until(nano2count(old_time+1000000000/ENCODER_READ_FREQ));
//        current_time = rt_get_time_ns();//clock();

//        cout<<"("<<(current_time-old_time)<<")";

//        cout << "Theta_m_u = "<<Theta_m_u.transpose()<<endl;

//        for(i=0;i<12;i++)
//            cout<<" "<<estimates[i];
//        cout<<endl;

//    }

//    recorded_data_file.close();
//    rt_make_soft_real_time();
//    rt_task_delete(task);
//    return 0;
//}

/////////////////////////////////////////UDP Send Thread////////////////////////////////////////////////////////////
void* send_data(void *args)
{
    unsigned long task_name =nam2num("UDP_send");

    int sockfd;
    struct sockaddr_in servaddr;
//    double t=0;
    RTIME old_time;
//    RTIME current_time=0;
//    int mcp_angle;
    int pip_angle;
    uint8_t data_sent_uint8;

    sockfd=socket(AF_INET,SOCK_DGRAM,0);
    bzero(&servaddr,sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr=inet_addr(SERVER_ADDRESS);
    servaddr.sin_port=htons(52343);

    if (!(task = rt_task_init_schmod(task_name, 0, 0, 0, SCHED_FIFO, CPU_MAP-1)))
    {
        cout<<"CANNOT INIT TASK"<<task_name<<endl;
        return 0;
    }

    cout<<"THREAD INIT UDP SEND THREAD: name = "<<task_name<<", address = "<<task<<endl;
    mlockall(MCL_CURRENT | MCL_FUTURE);
    rt_make_hard_real_time();

    while(running_flag)
    {
        old_time = rt_get_time_ns();

//        // Finger MCP Joint Angle
//        mcp_angle = (((2*PI-estimates[3])*180/PI)/MCP_JOINT_RANGE)*255;
//        if(mcp_angle>255)
//            mcp_angle=255;
//        else
//        {
//            if(mcp_angle<0)
//                mcp_angle=0;
//        }
//        data_sent_uint8 = mcp_angle;

        // Finger PIP Joint Angle
        pip_angle = (((estimates[3]-estimates[7])*180/PI)/PIP_JOINT_RANGE)*255;
        if(pip_angle>255)
            pip_angle=255;
        else
        {
            if(pip_angle<0)
                pip_angle=0;
        }
        data_sent_uint8 = pip_angle;

//        cout<<"("<<(current_time-old_time)<<")";
//        cout<<"data_sent ="<<int(data_sent_uint8)<<endl;

        sendto(sockfd,(&data_sent_uint8),1,0,
               (struct sockaddr *)&servaddr,sizeof(servaddr));

//        current_time = rt_get_time_ns();//clock();

        rt_sleep_until(nano2count(old_time+1000000000/UDP_FREQ));
    }

    rt_make_soft_real_time();
    rt_task_delete(task);
    return 0;
}

/////////////////////////////////////////UDP Receive Thread////////////////////////////////////////////////////////////
void* receive_data(void *args)
{
    unsigned long task_name =nam2num("UDP_receive");

    int sockfd;
    struct sockaddr_in servaddr;
    socklen_t len;

//    double t=0;
    RTIME old_time;

    uint8_t exo_pip_torque_uint8;
    len = sizeof(servaddr);

    if ((sockfd=socket(AF_INET,SOCK_DGRAM,0)) < 0)
    {
        cerr<<"Failed to create UDP socket."<<endl;
        getchar();
        return NULL;
    }

    bzero(&servaddr,sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr=inet_addr(SERVER_ADDRESS);
    servaddr.sin_port=htons(52343);

    // time out settings
    struct timeval tv;
     tv.tv_sec = 0;
     tv.tv_usec = 1000;
     if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0)
     {
         perror("Error");
     }

     if (bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
     {
         perror("bind failed");
         return 0;
     }

    if (!(task = rt_task_init_schmod(task_name, 0, 0, 0, SCHED_FIFO, CPU_MAP-1)))
    {
        cout<<"CANNOT INIT TASK"<<task_name<<endl;
        return 0;
    }

    cout<<"THREAD INIT UDP RECEIVE THREAD: name = "<<task_name<<", address = "<<task<<endl;
    mlockall(MCL_CURRENT | MCL_FUTURE);
    rt_make_hard_real_time();

    while(running_flag)
    {
        old_time = rt_get_time_ns();
//        t = (old_time-counter_time)/1e6;

        cout<<"trying to receive!"<<endl;
//        recvfrom(sockfd, (&exo_pip_torque_uint8), 1, 0, (struct sockaddr *)&servaddr, &len);
        cout<<"data_received ="<<int(exo_pip_torque_uint8)<<endl;

        if(exo_pip_torque_uint8>255)
            exo_pip_torque_uint8=255;
        else
        {
            if(exo_pip_torque_uint8<0)
                exo_pip_torque_uint8=0;
        }

        // Exo Finger Joint Angles
        exo_pip_torque = (exo_pip_torque_uint8/tau_A_pip)*255;

//        cout<<"("<<(current_time-old_time)<<")";


//        current_time = rt_get_time_ns();//clock();

        rt_sleep_until(nano2count(old_time+1000000000/UDP_FREQ));
    }

    rt_make_soft_real_time();
    rt_task_delete(task);
    return 0;
}

int main(int argc, char *argv[])
{
    int result = 0;

    // Configuring EPOS for analog motor control
    // MCP motor
    char PortName_MCP[]="USB1";
    unsigned short nodeId_MCP = 1;
    void *keyHandle_MCP=NULL;

    //     PIP motor
    char PortName_PIP[]="USB0";
    unsigned short nodeId_PIP = 1;
    void *keyHandle_PIP=NULL;

    long TargetPosition = 70000;

    pthread_t controller_thread;
    pthread_t estimation_thread;
//    pthread_t data_send_thread;
//    pthread_t data_receive_thread;
    //    pthread_t plotting_thread;
    //    pthread capture_motion_thread;    

//    disable_motor = true;

    if(!disable_motor)
    {

        if((strcmp(control_type,"FF_MCP_Exo")==0) || (strcmp(control_type,"FF_PID_MCP_Exo")==0) ||
                (strcmp(control_type,"FF_PID_MCP_PIP_Exo")==0) || (strcmp(control_type,"DT_MCP")==0) ||
                (strcmp(control_type,"DT_MCP_PIP")==0))
        {
            // Activate MCP Motor
            closeDevice(keyHandle_MCP);
            keyHandle_MCP = activate_device(PortName_MCP, nodeId_MCP);
            Move(keyHandle_MCP, TargetPosition, nodeId_MCP);
        }

        if((strcmp(control_type,"FF_PIP_Exo")==0) || (strcmp(control_type,"FF_PID_PIP_Exo")==0) ||
                (strcmp(control_type,"FF_PID_MCP_PIP_Exo")==0) || (strcmp(control_type,"DT_PIP")==0) ||
                (strcmp(control_type,"DT_MCP_PIP")==0))
        {
            // Activate PIP Motor
            closeDevice(keyHandle_PIP);
            keyHandle_PIP = activate_device(PortName_PIP, nodeId_PIP);
            Move(keyHandle_PIP, TargetPosition, nodeId_PIP);
        }
    }

    rt_allow_nonroot_hrt();
    if (!(main_task = rt_task_init_schmod(nam2num("CTRTSK"), 0, 0, 0,SCHED_FIFO, CPU_MAP)))
    {
        cout<<"CANNOT INIT CTRTSK TASK \n"<<endl;
        exit(1);
    }
    rt_set_oneshot_mode();
    start_rt_timer(0);

    cout.precision(5); // Setting cout precision to 4
    cout.fill('0');
    cout.width(5);

    pthread_create(&controller_thread, 0, IFE_controller, (void *)(result));
    pthread_create(&estimation_thread, 0, IFE_estimation, (void *)(result));
//    pthread_create(&data_send_thread, 0, send_data, (void *)(result));
//    pthread_create(&data_receive_thread, 0, receive_data, (void *)(result));
    //    pthread_create(&controller_thread, 0, recorded_estimation, (void *)(result));
    //    pthread_create(&plotting_thread, 0, plot_data, (void *)(result));
    //    pthread_create(&capture_motion_thread, 0, capture_motion, (void *)(result));

    cout<<"Press <Enter> to stop and quit..."<<endl;
    getchar();
    running_flag = 0;
    usleep(1000000);

    stop_rt_timer();

    if(!disable_motor)
    {
        if((strcmp(control_type,"FF_MCP_Exo")==0) || (strcmp(control_type,"FF_PID_MCP_Exo")==0) ||
                (strcmp(control_type,"FF_PID_MCP_PIP_Exo")==0) || (strcmp(control_type,"DT_MCP")==0) ||
                (strcmp(control_type,"DT_MCP_PIP")==0))
        {
            // Deactivate MCP Motor
            // Disabling EPOS
            DisableDevice(keyHandle_MCP, nodeId_MCP);
        }

        if((strcmp(control_type,"FF_PIP_Exo")==0) || (strcmp(control_type,"FF_PID_PIP_Exo")==0) ||
                (strcmp(control_type,"FF_PID_MCP_PIP_Exo")==0) || (strcmp(control_type,"DT_PIP")==0) ||
                (strcmp(control_type,"DT_MCP_PIP")==0))
        {
            // Deactivate PIP Motor
            // Disabling EPOS
            DisableDevice(keyHandle_PIP, nodeId_PIP);
        }

        if((strcmp(control_type,"FF_MCP_Exo")==0) || (strcmp(control_type,"FF_PID_MCP_Exo")==0) ||
                (strcmp(control_type,"FF_PID_MCP_PIP_Exo")==0) || (strcmp(control_type,"DT_MCP")==0) ||
                (strcmp(control_type,"DT_MCP_PIP")==0))
        {
            // Deactivate MCP Motor
            // Closing device
            closeDevice(keyHandle_MCP);
        }

        if((strcmp(control_type,"FF_PIP_Exo")==0) || (strcmp(control_type,"FF_PID_PIP_Exo")==0) ||
                (strcmp(control_type,"FF_PID_MCP_PIP_Exo")==0) || (strcmp(control_type,"DT_PIP")==0) ||
                (strcmp(control_type,"DT_MCP_PIP")==0))
        {
            // Deactivate PIP Motor
            // Closing device
            closeDevice(keyHandle_PIP);
        }

    }

    usleep(1000000);
    rt_task_delete(main_task);
    return 0;
}

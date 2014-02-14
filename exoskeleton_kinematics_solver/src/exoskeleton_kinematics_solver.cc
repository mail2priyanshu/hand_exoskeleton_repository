//PURPOSE : Solve kinematics of the hand exoskeleton being developed in the ReNeu Robotics Lab at The University of Texas at Austin
//AUTHORS  : Priyanshu Agarwal
//CONTACT : mail2priyanshu@utexas.edu
//AFFILIATION : The University of Texas at Austin
//To DOs
//1. Make class for motion capture device
//2. Make class for motor controller
//3. Make class for sensor (NI stuff)

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


// Motor specific header files
//#include <dynamixel.h>

// Encoder header files
#include "NiFpga_exoskeleton_controller.h"

// RTAI header files
#include <rtai.h>
#include <rtai_sched.h>
#include <rtai_fifos.h>

// Plotting
extern "C" {
#include "gnuplot_i.h"
}

#include <termio.h>
#include <signal.h>
#include <semaphore.h>

static RT_TASK *main_task, *task;
int running_flag = 1;
RTIME counter_time=0;

//// Motion capture library
//#ifndef MOTION_CAPTURE_H
//#define MOTION_CAPTURE_H
//#include "owl.h"
//#include "motion_capture.h"
//#endif // MOTION_CAPTURE_H

ofstream encoder_data_file;
ifstream recorded_data_file;
sem_t sync_flag_1, sync_flag_2, killing_flag, thread_flag; // declare semaphore (memory location shared between threads)
double exo_t_rel[5];

double estimates[12]={0,0,0,0,0,0,0,0,0,0,0,0};

int32_t encoder_data_I32[6] = {0, 0, 0, 0, 0, 0};



void* sensing_estimation(void *args)
{
    unsigned long task_name =nam2num("ENCODER");

    int i;//,j=0;
    //    bool flag_points=0;

    double exo_mcp_rel_angle=0, exo_pip_rel_angle = 0, exo_dip_rel_angle=0; //exo_prox_angle = 0
    RTIME old_time, current_time=0;
    double t= 0;

    exo_finger index_finger;

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
                                                "rio://146.6.88.17/RIO0", //"rio://169.254.84.198/RIO0",
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

            cout<<"THREAD INIT EXOSKELETON CONROLLER: name = "<<task_name<<", address = "<<task<<endl;
            mlockall(MCL_CURRENT | MCL_FUTURE);
            rt_make_hard_real_time();

            while(running_flag)
            {
                old_time = rt_get_time_ns();
                t = (old_time-counter_time)/1e6;

                // reading encoder data from sbRIO
                NiFpga_MergeStatus(&status,  NiFpga_ReadArrayI32(session,NiFpga_exoskeleton_controller_IndicatorArrayI32_sensordata,encoder_data_I32,6));

                // converting voltage data to angles
                //                exo_abd_angle = (ABD_ANG-MCP_EXT_ANG)/(MCP_FLEX_V-MCP_EXT_V)*(encoder_data_I32[0]-MCP_EXT_V)+MCP_EXT_ANG;
                exo_mcp_rel_angle = (MCP_FLEX_ANG-MCP_EXT_ANG)/(MCP_FLEX_V-MCP_EXT_V)*(encoder_data_I32[0]-MCP_EXT_V)+MCP_EXT_ANG;
                exo_pip_rel_angle = (PIP_FLEX_ANG-PIP_EXT_ANG)/(PIP_FLEX_V-PIP_EXT_V)*(encoder_data_I32[2]-PIP_EXT_V)+PIP_EXT_ANG;
                exo_dip_rel_angle = (DIP_FLEX_ANG-DIP_EXT_ANG)/(DIP_FLEX_V-DIP_EXT_V)*(encoder_data_I32[3]-DIP_EXT_V)+DIP_EXT_ANG;


                exo_t_rel[0] = exo_mcp_rel_angle*PI/180;
                exo_t_rel[1] = 0;
                exo_t_rel[2] = exo_pip_rel_angle*PI/180;
                exo_t_rel[3] = exo_dip_rel_angle*PI/180;
                exo_t_rel[4] = 0;

                index_finger.exo_kinematics(exo_t_rel,estimates);

                rt_sleep_until(nano2count(old_time+1000000000/ENCODER_READ_FREQ));
                current_time = rt_get_time_ns();//clock();

                cout<<"("<<(current_time-old_time)<<")";

                // printing the encoder data on screen
                for(i=0;i<12;i++)
                {
                    if(i==2||i==11)
                        cout<<"\t"<<estimates[i];
                    else
                        cout<<"\t"<<estimates[i]*180/PI;
                }
                cout<<endl;

                // writing the encoder data to the file
                encoder_data_file<<t;
                for(i=0;i<12;i++)
                    encoder_data_file<<"\t"<<estimates[i];
                encoder_data_file<<"\t"<<exo_t_rel[0]<<"\t"<<exo_t_rel[2]<<"\t"<<exo_t_rel[3];
                encoder_data_file<<endl;

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
    rt_make_soft_real_time();
    rt_task_delete(task);
    return 0;
}


void* recorded_estimation(void *args)
{
    unsigned long task_name =nam2num("ENCODER");

    int i;//,j=0;
    RTIME old_time, current_time=0;
    double t= 0;

    exo_finger index_finger;

    double x[6]={0.015,7*PI/4,PI/4,7*PI/4,0.005,7*PI/4};

    if (!(task = rt_task_init_schmod(task_name, 0, 0, 0, SCHED_FIFO, CPU_MAP-1)))
    {
        cout<<"CANNOT INIT TASK"<<task_name<<endl;
        return 0;
    }


    cout<<"THREAD INIT EXOSKELETON CONTROLLER: name = "<<task_name<<", address = "<<task<<endl;
    mlockall(MCL_CURRENT | MCL_FUTURE);
    rt_make_hard_real_time();

    while(running_flag)
    {
        old_time = rt_get_time_ns();
        t = (old_time-counter_time)/1e9;

        recorded_data_file>>exo_t_rel[0]>>exo_t_rel[2]>>exo_t_rel[3];

        index_finger.exo_kinematics(exo_t_rel,estimates);

        x[0] = estimates[2];
        x[1] = estimates[3];
        x[2] = estimates[4];
        x[3] = estimates[7];
        x[4] = estimates[10];
        x[5] = estimates[11];

        rt_sleep_until(nano2count(old_time+1000000000/ENCODER_READ_FREQ));
        current_time = rt_get_time_ns();//clock();

        cout<<"("<<(current_time-old_time)<<")";

        for(i=0;i<12;i++)
            cout<<" "<<estimates[i];
        cout<<endl;

        // writing the encoder data to the file
        encoder_data_file<<t;
        for(i=0;i<12;i++)
            encoder_data_file<<"\t"<<estimates[i];
        encoder_data_file<<"\t"<<exo_t_rel[0]<<"\t"<<exo_t_rel[2]<<"\t"<<exo_t_rel[3];
        encoder_data_file<<endl;

    }

    rt_make_soft_real_time();
    rt_task_delete(task);
    return 0;
}



void* plot_data(void *args)
{
    unsigned long task_name =nam2num("PLOTTING");

    int j=0;
    double t=0;
    bool print_flag=0;
    int plot_limits[2]={-100,500};

    RTIME old_time;

    double t_window[NPOINTS], plot_data[3][NPOINTS];

    gnuplot_ctrl *h1;

    h1 = gnuplot_init();

    if (!(task = rt_task_init_schmod(task_name, 0, 0, 0, SCHED_FIFO, CPU_MAP-1)))
    {
        cout<<"CANNOT INIT TASK"<<task_name<<endl;
        return 0;
    }

    cout<<"THREAD INIT PLOTTING THREAD: name = "<<task_name<<", address = "<<task<<endl;
    mlockall(MCL_CURRENT | MCL_FUTURE);
    rt_make_hard_real_time();

    while(running_flag)
    {
        old_time = rt_get_time_ns();
        t = (old_time-counter_time)/1e6;

        t_window[j] = t;

        // Raw Sensor Data
//        plot_data[0][j] = encoder_data_I32[0];
//        plot_data[1][j] = encoder_data_I32[2];
//        plot_data[2][j] = encoder_data_I32[3];
//        plot_limits[0]=0;
//        plot_limits[1]=5000;
//        cout<<encoder_data_I32[0]<<"\t"<<encoder_data_I32[2]<<"\t"<<encoder_data_I32[3]<<endl;

        // Exoskeleton Joint Angles
//        plot_data[0][j] = exo_t_rel[0]*180/PI;
//        plot_data[1][j] = exo_t_rel[2]*180/PI;
//        plot_data[2][j] = exo_t_rel[3]*180/PI;
//        cout<<exo_t_rel[0]*180/PI<<"\t"<<exo_t_rel[2]*180/PI<<"\t"<<exo_t_rel[3]*180/PI<<endl;

        // Estimated Angles
//        plot_data[0][j] = estimates[3]*180/PI;
//        plot_data[1][j] = estimates[7]*180/PI;
//        plot_data[2][j] = estimates[11]*180/PI;
        
        // Finger Joint Angles
        plot_data[0][j] = (2*PI-estimates[3])*180/PI;
        plot_data[1][j] = (2*PI-estimates[7])*180/PI-plot_data[0][j];
        plot_data[2][j] = (2*PI-estimates[11])*180/PI-plot_data[0][j]-plot_data[1][j];

        if(j>=NPOINTS)
        {
            j=0;
        }
        else
            j++;

        gnuplot_resetplot(h1);
        gnuplot_cmd(h1,"set yrange [%d:%d]",plot_limits[0],plot_limits[1]);
        gnuplot_setstyle(h1, (char *)"lines") ;
        gnuplot_plot_xy(h1, t_window, plot_data[0], NPOINTS, (char *)"MCP");
        gnuplot_plot_xy(h1, t_window, plot_data[1], NPOINTS, (char *)"PIP") ;
        gnuplot_plot_xy(h1, t_window, plot_data[2], NPOINTS, (char *)"DIP") ;

        rt_sleep_until(nano2count(old_time+1000000000/PLOT_FREQ));
    }

    if(print_flag)
    {
        // Does not work at this point
        gnuplot_cmd(h1, "set terminal postscript");
        gnuplot_cmd(h1, "set output \"plot.ps\"");
        printf("Printing plot...");
    }
    gnuplot_close(h1);

    rt_make_soft_real_time();
    rt_task_delete(task);
    return 0;
}


int main(int argc, char *argv[])
{
    int result = 0;

    encoder_data_file.open("encoder_data.csv");
    recorded_data_file.open("encoder_data_YY_E_high.txt");

    RTIME tick_period;
    pthread_t sensing_estimation_thread, plotting_thread;
//    pthread capture_motion_thread;// ; //motor_control_thread
    rt_allow_nonroot_hrt();
    if (!(main_task = rt_task_init_schmod(nam2num("CTRTSK"), 0, 0, 0,SCHED_FIFO, CPU_MAP)))
    {
        cout<<"CANNOT INIT CTRTSK TASK \n"<<endl;
        exit(1);
    }
    rt_set_oneshot_mode();
    tick_period = start_rt_timer(0);

    cout.precision(5); // Setting cout precision to 4
    cout.fill('0');
    cout.width(5);

    counter_time = rt_get_time_ns();

//    pthread_create(&sensing_estimation_thread, 0, recorded_estimation, (void *)(result));
    pthread_create(&sensing_estimation_thread, 0, sensing_estimation, (void *)(result));
    pthread_create(&plotting_thread, 0, plot_data, (void *)(result));
    //    pthread_create(&capture_motion_thread, 0, capture_motion, (void *)(result));

    cout<<"Press <Enter> to stop and quit..."<<endl;
    getchar();
    running_flag = 0;
    usleep(1000000);


    stop_rt_timer();

    encoder_data_file.close();
    recorded_data_file.close();
    rt_task_delete(main_task);
    return 0;
}

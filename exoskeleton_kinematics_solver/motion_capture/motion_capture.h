#ifndef MOTION_CAPTURE_H
#define MOTION_CAPTURE_H

#endif // MOTION_CAPTURE_H

void owl_print_error(const char *s, int n)
{
    if(n < 0) printf("%s: %d\n", s, n);
    else if(n == OWL_NO_ERROR) printf("%s: No Error\n", s);
    else if(n == OWL_INVALID_VALUE) printf("%s: Invalid Value\n", s);
    else if(n == OWL_INVALID_ENUM) printf("%s: Invalid Enum\n", s);
    else if(n == OWL_INVALID_OPERATION) printf("%s: Invalid Operation\n", s);
    else printf("%s: 0x%x\n", s, n);
}


void* capture_motion(void *args)
{

    RT_TASK *task;
    unsigned long task_name =nam2num("MOTION");
    OWLMarker markers[32];
    int tracker;
    int err, n, MARKER_COUNT;
    char SERVER_NAME[15];

    ofstream Motion_Capture_data;   // to save file
    ifstream file_motion_capture_config;

    file_motion_capture_config.open(FILE_NAME_MOTION_CAPTURE_CONFIG);

    if(file_motion_capture_config.fail())
    {
        printf("File not found!");
        return 0;
    }


    file_motion_capture_config>>MARKER_COUNT; // read IP address and marker numbers from text
    file_motion_capture_config>>SERVER_NAME;  // read IP address and marker numbers from text

    Motion_Capture_data.open("Marker_Data.txt"); // open file to write

    RTIME old_time, current_time;

    int flag, i;
    double t;

    if(owlInit(SERVER_NAME, INIT_FLAGS) < 0) return 0;

    // create tracker 0
    tracker = 0;
    owlTrackeri(tracker, OWL_CREATE, OWL_POINT_TRACKER);

    // set markers
    for(int i = 0; i < MARKER_COUNT; i++)
        owlMarkeri(MARKER(tracker, i), OWL_SET_LED, i);

    // activate tracker
    owlTracker(tracker, OWL_ENABLE);

    // flush requests and check for errors
    if(!owlGetStatus())
    {
        owl_print_error("error in point tracker setup", owlGetError());
        return 0;
    }

    if (!(task = rt_task_init_schmod(task_name, 0, 0, 0, SCHED_FIFO, CPU_MAP-1)))
    {
        printf("CANNOT INIT TASK %lu\n", task_name);
        return 0;
    }

    printf("THREAD INIT MOTION CAPTURE: name = %lu, address = %p.\n", task_name, task);
    mlockall(MCL_CURRENT | MCL_FUTURE);

    rt_make_hard_real_time();

    // set default frequency
    owlSetFloat(OWL_FREQUENCY, OWL_MAX_FREQUENCY);

    // start streaming
    owlSetInteger(OWL_STREAMING, OWL_ENABLE);

    // main loop
    while(running_flag)
    {
        old_time = rt_get_time_ns();
        t = (old_time-counter_time)/1e6;

        // get some markers
        n = owlGetMarkers(markers, 32);

        // check for error
        if((err = owlGetError()) != OWL_NO_ERROR)
        {
            owl_print_error("error", err);
            break;
        }

        // no data yet
        if(n == 0)
        {
            continue;
        }


        if(n > 0)
        {
            Motion_Capture_data<<t<<endl;

            flag = 1;
            for(i = 0; i < n; i++)
            {
                if(markers[i].cond < 0)
                    flag = 0;
                printf("%d) %f %f %f %f\n", i, markers[i].x, markers[i].y, markers[i].z, markers[i].cond);
                Motion_Capture_data << i << "\t" << markers[i].x << "\t" << markers[i].y << "\t" << markers[i].z << "\t" << markers[i].cond << endl;

            }

            Motion_Capture_data <<endl;

        }

        rt_sleep_until(nano2count(old_time+1000000000/CAPTURE_FREQ));
        current_time = rt_get_time_ns();//clock();
        printf("(%lld)\n", (current_time-old_time));

    }

    // cleanup
    file_motion_capture_config.close();
    Motion_Capture_data.close();

    owlDone();
    rt_make_soft_real_time();
    rt_task_delete(task);
    return 0;
}

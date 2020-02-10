/*
 * Copyright 2018 The MathWorks, Inc.
 *
 * File: nuttxinitialize.c
 *
 * Abstract:
 *  This file contains RTOS thread creation functions
 *
 */

#include "nuttxinitialize.h"
#include "MW_PX4_TaskControl.h"

/***********************************************
 * Added for Simulink Threads
 ************************************************/
pthread_attr_t attr;
static baseRateInfo_t g_info;
struct sched_param g_sp;
// extern volatile boolean_T runModel ;

/***********************************************
 * Added for HRT
 ************************************************/
struct hrt_call BaseRate_HRT;
int baserate_sem_copy;


#define MW_RTOS_DEBUG 0

baseRateInfo_t EXT_sig_info;
extern pthread_t backgroundThread;
extern void exitFcn(int sig);

void setTaskPeriod(double periodInSeconds, int sigNo) {
    timer_t timerId;
    struct sigevent sev;
    struct itimerspec its;
    long stNanoSec;
    int status;

    /* Create a timer */
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = sigNo;
    sev.sigev_value.sival_ptr = &timerId;
    status = timer_create(CLOCK_REALTIME, &sev, &timerId);
    CHECK_STATUS(status, 0, "timer_create");

    /* Arm real-time scheduling timer */
    stNanoSec = (long)(periodInSeconds * 1e9);
    its.it_value.tv_sec = stNanoSec / 1000000000;
    its.it_value.tv_nsec = stNanoSec % 1000000000;
    its.it_interval.tv_sec = its.it_value.tv_sec;
    its.it_interval.tv_nsec = its.it_value.tv_nsec;
    status = timer_settime(timerId, 0, &its, NULL);
    CHECK_STATUS(status, 0, "timer_settime");
}

void print_sem_value() {
    PX4_WARN("Sem value = %d", baserate_sem_copy);
}

static void Base_HRT_Callback(void* arg) {
    int sem_value_check;

    sem_getvalue(&baserateTaskSem, &sem_value_check);
    sem_post(&baserateTaskSem);
    baserate_sem_copy = sem_value_check;
    if (sem_value_check > 1) {
#if defined(MW_HRT_CONSTRAINT) && (1 == MW_HRT_CONSTRAINT)
        if (sem_value_check > MW_SEM_WATERMARK) {
            PX4_WARN("Exiting because of Hard real time constraint violation");
            exitFcn(1); // terminate the base-rate and sub-rate threads
        }
#else
        fflush(stdout);
        sem_init(&baserateTaskSem, 0, 0);
#endif
    }
}

/* ---------------------------- */
/* Externally visible functions */
/* ---------------------------- */
void myWaitForThisEvent(int sigNo) {
    sigset_t sigMask;

    sigemptyset(&sigMask);
    sigaddset(&sigMask, sigNo);
    sigwaitinfo(&sigMask, NULL);
    //	CHECK_STATUS_NOT(status, -1, "sigwaitinfo");
}

void MW_PX4_WaitForThisEvent(void* arg) {
#ifdef EXT_MODE
    /*pace External mode thread at 10 Hz*/
    usleep(100000);

    /*Signaling is causing issue when sample time is greater than 0.1
     and model runs for more than 60 seconds. In this case model stop is not
     stopping External mode. Using usleep for now to sleep the External Mode thread.*/

    // baseRateInfo_t ext_info = *((baseRateInfo_t *)arg);
    // static boolean_T isTaskPeriodSet = false;
    // boolean_T rtmStopReq = rtmGetStopRequested(MW_StringifyDefineFunction(MODEL, _M));

    // if (!isTaskPeriodSet) {
    //     setTaskPeriod(ext_info.period, ext_info.sigNo);
    //     isTaskPeriodSet = true;
    // }

    // if (!rtmStopReq) {
    //     myWaitForThisEvent(ext_info.sigNo);
    // }

#endif
}

void myAddBlockForThisEvent(int sigNo) {
    int status;
    sigset_t sigMask;

    sigemptyset(&sigMask);
    sigaddset(&sigMask, sigNo);
    status = pthread_sigmask(SIG_BLOCK, &sigMask, NULL);
    CHECK_STATUS(status, 0, "pthread_sigmask");
}

void nuttxRTOSInit(double baseRatePeriod, int numSubrates) {
    /* This is because in Daren's original Task callback ,a sleep was added before creating the
     * threads*/
    sleep(1);

    int status;
    size_t stackSize;
    int baseRatePriority = MW_BASERATE_PRIORITY;
#ifdef MW_HAS_MULTIPLE_RATES
    int i;
    // char taskName[20];
#endif
    sched_lock();
    // status = sem_init(&termSem, 0, 0); PX4 PSP Dev: termSem no longer needed
    // CHECK_STATUS(status, 0,"sem_init:termSem");
    status = sem_init(&stopSem, 0, 0);
    CHECK_STATUS(status, 0, "sem_init:stopSem");
    status = sem_init(&baserateTaskSem, 0, 0);
    CHECK_STATUS(status, 0, "sem_init:baserateTaskSem");

    // or you can use/check: _POSIX_PRIORITY_SCHEDULING
    // _POSIX_THREAD_PRIORITY_SCHEDULING
#if !defined(_POSIX_PRIORITY_SCHEDULING)
    PX4_INFO("Priority scheduling is NOT supported by your system.");
    PX4_INFO("The generated code will not run correctly because your");
    PX4_INFO("model contains multiple rates and uses multi-tasking");
    PX4_INFO("code generation mode. You can only run the generated code");
    PX4_INFO("in single-tasking mode in your system. Open");
    PX4_INFO("Simulation -> Configuration Parameters -> Solver dialog");
    PX4_INFO("and set \"Tasking mode for periodic sample times\" parameter to SingleTasking.");
    PX4_INFO(
        "Re-build the Simulink model with the new settings and try executing the generated code "
        "again.");
    fflush(stdout);
    exit(-1);
#endif

    /* Set scheduling policy of the main thread to SCHED_FIFO */
    g_sp.sched_priority = sched_get_priority_max(SCHED_FIFO) - 50;
    status = sched_setscheduler(0, SCHED_FIFO, &g_sp);
    CHECK_STATUS(status, 0, "sched_setscheduler");

    /*Added init attribute and scheduler policy */
    pthread_attr_init(&attr);
    status = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    CHECK_STATUS(status, 0, "pthread_attr_setinheritsched");
    status = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    CHECK_STATUS(status, 0, "pthread_attr_setschedpolicy");
    // status = pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    // CHECK_STATUS(status, 0, "pthread_attr_setdetachstate");

    /* PTHREAD_STACK_MIN is the minimum stack size required to start a thread */
    stackSize = 2048 + PTHREAD_STACK_MIN;

    /*added stack attribute back in */
    status = pthread_attr_setstacksize(&attr, stackSize);
    CHECK_STATUS(status, 0, "pthread_attr_setstacksize");

    /* set up info structure */
    g_info.period = baseRatePeriod;
    g_info.sigNo = BASERATE_TIMER_SIGNAL; // SIGRTMIN;

#if MW_RTOS_DEBUG
    PX4_INFO("   stackSize = %d sched_priority = %d", stackSize, g_sp.sched_priority);
    PX4_INFO("   MW_BASERATE_PERIOD = %8.5f MW_BASERATE_PRIORITY = %d SIGRTMIN = 0x%08X",
             (double)baseRatePeriod, (int)baseRatePriority, SIGRTMIN);
    PX4_INFO("   Init info.period = %8.5f sigNo = 0x%04X", g_info.period, g_info.sigNo);
    PX4_INFO("**creating the Base Rate thread before calling pthread_create**");
    fflush(stdout);
#endif

    /* Create the Base Rate Task here */
    g_sp.sched_priority = baseRatePriority;
    status = pthread_attr_setschedparam(&attr, &g_sp);
    status = pthread_create(&baseRateThread, &attr, &baseRateTask, (void*)&g_info);
#if MW_RTOS_DEBUG
    PX4_INFO("** Base Rate Task ID = %d with Priority = %d", baseRateThread, g_sp.sched_priority);
    fflush(stdout);
#endif

/* Create sub-rate Tasks here */
#ifdef MW_HAS_MULTIPLE_RATES
#if MW_RTOS_DEBUG
    PX4_INFO("Creating sub-rate task threads\n");
    PX4_INFO("Number of sub rate tasks: %d \n", MW_NUMBER_SUBRATES);
    fflush(stdout);
#endif

    for (i = 0; i < MW_NUMBER_SUBRATES; i++) {
        taskId[i] = i;
        status = sem_init(&subrateTaskSem[i], 0, 0);
        CHECK_STATUS(status, 0, "sem_init: subrateTaskSem");
        // g_sp.sched_priority = MW_BASERATE_PRIORITY -40 + subratePriority[i] - 1;
        g_sp.sched_priority = subratePriority[i];
#if MW_RTOS_DEBUG
        PX4_INFO("MW_SUBRATE_PRIORITY = %d ", (int)g_sp.sched_priority);
#endif
        status = pthread_attr_setschedparam(&attr, &g_sp);
        CHECK_STATUS(status, 0, "pthread_attr_setschedparam");
        status = pthread_create(&subRateThread[i], &attr, &subrateTask, (void*)&taskId[i]);
        CHECK_STATUS(status, 0, "pthread_create");
#if MW_RTOS_DEBUG
        PX4_INFO("** Sub Rate Task ID = %d ", subRateThread[i]);
        fflush(stdout);
#endif
    }
#endif // End of "If Multiple Rates" check

    g_sp.sched_priority = baseRatePriority;
    status = pthread_attr_setschedparam(&attr, &g_sp);

    /* Create the Scheduler Task here */
    long stMicroSec;
    stMicroSec = (long)(g_info.period * 1e6);
    hrt_call_every(&BaseRate_HRT, stMicroSec, stMicroSec, Base_HRT_Callback,
                   NULL); // timing callback to post base-rate semaphore

/* Create the External Mode Task here */
#ifdef EXT_MODE
    /*---------- Creating EXT-mode Background Task ------------------*/
    /* Setup signal info to block for EXT Mode */
    EXT_sig_info.period = 0.1; // Sample rate at which we want Simulink to update w/ ext mode
    EXT_sig_info.sigNo = SIGRTMIN;
    /* Signaling the thread not working for low sample frequency*/
    // sigemptyset(&EXT_sig_info.sigMask);
    // sigaddset(&EXT_sig_info.sigMask, EXT_sig_info.sigNo);
    // myAddBlockForThisEvent(EXT_sig_info.sigNo);

    /* Setup thread for Custom Task */
    fflush(stdout);
    g_sp.sched_priority = 40; // default is 100
    status = pthread_attr_setschedparam(&attr, &g_sp);
    status = pthread_create(&backgroundThread, &attr, &backgroundTask, (void*)&EXT_sig_info);
    CHECK_STATUS(status, 0, "EXT Mode Background pthread_create");
/*---------------------------------------------------------*/
#endif

#if MW_RTOS_DEBUG
    PX4_INFO("px4_simulink_app : Created Simulink task threads successfully\n");
    fflush(stdout);
#endif
    sched_unlock();
}
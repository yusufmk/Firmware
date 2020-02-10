//
// Prerelease License - for engineering feedback and testing purposes
// only. Not for sale.
//
// File: ert_main.cpp
//
// Code generated for Simulink model 'TestConnExample'.
//
// Model version                  : 1.86
// Simulink Coder version         : 9.1 (R2019a) 23-Nov-2018
// C/C++ source code generated on : Fri Jan 11 08:30:05 2019
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include <stdio.h>
#include <stdlib.h>
#include "TestConnExample.h"
#include "TestConnExample_private.h"
#include "rtwtypes.h"
#include "limits.h"
#include "MW_PX4_TaskControl.h"
#include "nuttxinitialize.h"
#define UNUSED(x)                      x = x
#define NAMELEN                        16

// Function prototype declaration
void exitFcn(int sig);
void *terminateTask(void *arg);
void *baseRateTask(void *arg);
void *subrateTask(void *arg);
volatile boolean_T stopRequested = false;
volatile boolean_T runModel = true;
sem_t stopSem;
sem_t baserateTaskSem;
pthread_t schedulerThread;
pthread_t baseRateThread;
void *threadJoinStatus;
int terminatingmodel = 0;
void *baseRateTask(void *arg)
{
  runModel = (rtmGetErrorStatus(TestConnExample_M) == (NULL));
  while (runModel) {
    sem_wait(&baserateTaskSem);
    TestConnExample_step();

    // Get model outputs here
    stopRequested = !((rtmGetErrorStatus(TestConnExample_M) == (NULL)));
    runModel = !stopRequested;
  }

  runModel = 0;
  terminateTask(arg);
  pthread_exit((void *)0);
  return NULL;
}

void exitFcn(int sig)
{
  UNUSED(sig);
  rtmSetErrorStatus(TestConnExample_M, "stopping the model");
}

void *terminateTask(void *arg)
{
  UNUSED(arg);
  terminatingmodel = 1;

  {
    runModel = 0;
  }

  MW_PX4_Terminate();

  // Disable rt_OneStep() here

  // Terminate model
  TestConnExample_terminate();
  sem_post(&stopSem);
  return NULL;
}

int px4_simulink_app_task_main (int argc, char *argv[])
{
  rtmSetErrorStatus(TestConnExample_M, 0);

  // Initialize model
  TestConnExample_initialize();

  // Call RTOS Initialization function
  nuttxRTOSInit(0.1, 0);

  // Wait for stop semaphore
  sem_wait(&stopSem);

#if (MW_NUMBER_TIMER_DRIVEN_TASKS > 0)

  {
    int i;
    for (i=0; i < MW_NUMBER_TIMER_DRIVEN_TASKS; i++) {
      CHECK_STATUS(sem_destroy(&timerTaskSem[i]), 0, "sem_destroy");
    }
  }

#endif

  return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//

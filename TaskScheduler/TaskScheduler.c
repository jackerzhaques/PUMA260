//Project includes
#include "TaskScheduler.h"
#include "PriorityQueue.h"

//Tivaware includes
#include <driverlib/sysctl.h>
#include <driverlib/timer.h>
#include <inc/hw_ints.h>
#include <driverlib/interrupt.h>
#include <inc/hw_memmap.h>
#include <driverlib/rom.h>

#ifndef NULL
    #define NULL 0
#endif

static TaskScheduler    scheduler;
static Task             *tasks;
static uint16_t         nTasks;

//Does not need to be volatile because it is only used in the ISR
static PriorityQueue            queue;
static PriorityQueue*           pqueue;

/*
 * Timer ISR for the task scheduler
 *
 * Updates each timer's task
 * If the task is ready to be fired, then it is added to a queue and processed
 *
 * This timer must be given a low priority ISR because it is possible to have
 * a heavy load to process
 */
void TaskSchedulerTimer_ISR(void){
    uint16_t i;
    for(i = 0; i < nTasks; i++){
        if(tasks[i].enabled){
            tasks[i].ticks++;

            if(tasks[i].ticks >= tasks[i].maxTicks){
                tasks[i].ticks = 0;
                AddTaskToQueue(pqueue, &tasks[i]);
            }
        }
        else{
            tasks[i].ticks = 0;
        }
    }

    //Run all pending tasks
    RunAllTasks(pqueue);

    TimerIntClear(scheduler.timerBase, TIMER_TIMA_TIMEOUT);
}

void InitializeTaskScheduler(Task *pTaskList, uint16_t taskCount, uint32_t timerBase, uint32_t sysCtlTimerPeriph, uint32_t sysClkFreq, uint32_t timerIntBase){
    //Save the tasks
    tasks = pTaskList;
    nTasks = taskCount;

    //Calculate the max ticks for each task
    uint16_t i;
    for(i = 0; i < nTasks; i++){
        pTaskList[i].maxTicks = pTaskList[i].period * TASK_SCHEDULER_TICKS_IN_ONE_SECOND;
    }

    //Initialize the task scheduler
    scheduler.timerBase = timerBase;

    //Initialize the priority queue
    pqueue = &queue;
    InitializeQueue(pqueue);

    //Initialize the timer
    SysCtlPeripheralEnable(sysCtlTimerPeriph);

    //Wait for the clock to stabilize
    SysCtlDelay(100);
    IntMasterEnable();

    //Configure the timer to be a periodic 100us timer
    TimerConfigure(timerBase, TIMER_CFG_PERIODIC);

    TimerLoadSet(timerBase, TIMER_A, sysClkFreq * (TASK_SCHEDULER_TIMER_PERIOD / 1000000.0f));

    //Configure the ISR
    TimerIntRegister(timerBase, TIMER_A, TaskSchedulerTimer_ISR);
    IntEnable(timerIntBase);
    TimerIntEnable(timerBase, TIMER_TIMA_TIMEOUT);

    //Enable the timer and interrupts
    TimerEnable(timerBase, TIMER_A);
}

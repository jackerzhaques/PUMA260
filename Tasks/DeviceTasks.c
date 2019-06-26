#include "DeviceTasks.h"

//Standard includes

//Project includes
#include "Mailbox/Mailbox.h"
#include "Mailbox/MessageSubscriber.h"
#include "Mailbox/IDCD.h"
#include "TaskScheduler/TaskScheduler.h"
#include "ControlLoop/ControlGlobals.h"
#include "ControlLoop/ControlLoop.h"
#include "globals.h"
#include "EIB/Encoders.h"

//Tivaware includes
#include <driverlib/sysctl.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>

#define TASK_TIMER_BASE         TIMER2_BASE
#define TASK_TIMER_PERIPHERAL   SYSCTL_PERIPH_TIMER2
#define TASK_TIMER_INT_BASE     INT_TIMER2A

//Task and callback variables
static bool             deviceIsConnected   = false;
static PositionVector   currentSetPosition  = {0,0,0,0};

//Message callback prototypes
void Callback_HeartbeatResponse(Message m);
void Callback_SetPosition(Message m);
void Callback_SetJointAngle(Message m);
void Callback_SendEncoderStatus(Message m);
void Callback_SendJointCurrent(Message m);
void Callback_SendJointPosition(Message m);
void Callback_SendJointAngle(Message m);
void Callback_SendControlStatus(Message m);
void Callback_ProcessAutoHomeCommand(Message m);
void Callback_ProcessManualHomeCommand(Message m);

//Task callback prototypes
void Task_SendHeartbeat(void);
void Task_AngleOnChangeCheck(void);
void Task_SendJointAngles(void);

//Other function prototypes
float bytesToFloat(uint32_t bytes);
uint32_t floatToBytes(float f);


/*
 * Define the message subscriptions and their callbacks
 */
static MessageSubscription MessageSubscriptions[] = {
   {HEARTBEAT_RESPONSE_ID,  Callback_HeartbeatResponse},
   {SET_POSITION_ID,        Callback_SetPosition},
   {SET_JOINT_ANGLE_ID,     Callback_SetJointAngle},
   {GET_ENCODER_STATUS_ID,  Callback_SendEncoderStatus},
   {GET_JOINT_CURRENT_ID,   Callback_SendJointCurrent},
   {GET_POSITION_ID,        Callback_SendJointPosition},
   {GET_JOINT_ANGLE_ID,     Callback_SendJointAngle},
   {GET_CONTROL_STATUS_ID,  Callback_SendControlStatus},
   {AUTO_HOME_ID,           Callback_ProcessAutoHomeCommand},
   {MANUAL_HOME_ID,         Callback_ProcessManualHomeCommand}
};

static uint16_t nSubscriptions = sizeof(MessageSubscriptions)/sizeof(MessageSubscription);

/*
 * Define periodic tasks and their callbacks
 */
static Task taskList[] = {
   {0, 0,   5, 10,  Task_SendHeartbeat, 1},
   {0, 0, 0.5, 10,  Task_AngleOnChangeCheck, 1},
   {0, 0,   2, 8,   Task_SendJointAngles, 1},
};

static uint16_t nTasks = sizeof(taskList)/sizeof(Task);

void InitializeTasks(uint32_t sysClk){
    InitializeMailbox();
    InitializeMessageSubscriber(MessageSubscriptions, nSubscriptions);
    InitializeTaskScheduler(
            taskList,
            nTasks,
            TASK_TIMER_BASE,
            TASK_TIMER_PERIPHERAL,
            sysClk,
            TASK_TIMER_INT_BASE
    );
}

void Task_SendHeartbeat(void){
    Heartbeat message = HEARTBEAT_INIT;
    SendMessage((Message*)&message);
}

void Callback_HeartbeatResponse(Message m){
    deviceIsConnected = true;
}

void Callback_SetPosition(Message m){
    Set_Position_data *data = (Set_Position_data*)(m.data);
    float pos = bytesToFloat(data->floatPos);

    switch(data->Dimension){
        case 1:
            currentSetPosition.x = pos;
            break;
        case 2:
            currentSetPosition.y = pos;
            break;
        case 3:
            currentSetPosition.z = pos;
            break;
        case 4:
            currentSetPosition.theta = pos;
            break;
        default:
            //Do nothing
            break;
    }

    SetArmPosition(currentSetPosition);
}

void Callback_SetJointAngle(Message m){
    Set_Joint_Angle_data *data = (Set_Joint_Angle_data*)(m.data);
    float pos = bytesToFloat(data->floatDegrees);

    JOINT_POSITION jointIndex = (JOINT_POSITION)(data->Joint - 1);
    SetJointAngle(jointIndex,pos);
}

void Callback_SendEncoderStatus(Message m){

}

void Callback_SendJointCurrent(Message m){

}

void Callback_SendJointPosition(Message m){

}

void Callback_SendJointAngle(Message m){
    Task_SendJointAngles();
}

void Task_AngleOnChangeCheck(void){
    static float angles[JOINT_COUNT] = {0};

    uint8_t i;
    for(i = 0; i < JOINT_COUNT; i++){
        JOINT_POSITION joint = (JOINT_POSITION)i;
        float angle = Enc_GetJointEncoder(joint)->Degrees;

        //If the angles are different enough, send a position message
        if(fabs(angle - angles[i]) >= 0.01){
            Get_Joint_Angle_Response message = GET_JOINT_ANGLE_RESPONSE_INIT;
            message.data.angleBytes = floatToBytes(angle);
            message.data.Joint = (i + 1);
            SendMessage((Message*)(&message));
            angles[i] = angle;
        }
    }
}

void Task_SendJointAngles(void){
    uint8_t i;
    for(i = 0; i < JOINT_COUNT; i++){
        JOINT_POSITION joint = (JOINT_POSITION)i;
        float angle = Enc_GetJointEncoder(joint)->Degrees;
        Get_Joint_Angle_Response message = GET_JOINT_ANGLE_RESPONSE_INIT;
        message.data.angleBytes = floatToBytes(angle);
        message.data.Joint = (i + 1);
        SendMessage((Message*)(&message));
    }
}

void Callback_SendControlStatus(Message m){

}

void Callback_ProcessAutoHomeCommand(Message m){

}

void Callback_ProcessManualHomeCommand(Message m){
    //Reset all joints position/target
    sPID* pids = GetPositionPIDs();
    uint8_t i = 0;
    for(i = 0; i < JOINT_COUNT; i++){
        JOINT_POSITION joint = (JOINT_POSITION)i;
        pids[i].Target = 0;
        Enc_ResetEncoder(joint);
    }

    Home_Response message = HOME_RESPONSE_INIT;
    SendMessage((Message*)&message);
}

float bytesToFloat(uint32_t bytes){
    float f = 0;
    memcpy(&f, &bytes, sizeof(bytes));
    return f;
}

uint32_t floatToBytes(float f){
    uint32_t bytes;
    memcpy(&bytes, &f, sizeof(f));
    return bytes;
}

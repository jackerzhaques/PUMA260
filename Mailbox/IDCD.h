#ifndef IDCD_H
#define IDCD_H

#define HEARTBEAT_ID                 1
#define HEARTBEAT_DLC                0
#define HEARTBEAT_DATA_INIT          {}
#define HEARTBEAT_INIT               {HEARTBEAT_ID, HEARTBEAT_DLC, HEARTBEAT_DATA_INIT}

typedef struct Heartbeat_data_tag{

} Heartbeat_data;

typedef struct Heartbeat_tag{
  uint16_t ID;
  uint8_t DLC;
  Heartbeat_data data;
} Heartbeat;


#define HEARTBEAT_RESPONSE_ID                 2
#define HEARTBEAT_RESPONSE_DLC                0
#define HEARTBEAT_RESPONSE_DATA_INIT          {}
#define HEARTBEAT_RESPONSE_INIT               {HEARTBEAT_RESPONSE_ID, HEARTBEAT_RESPONSE_DLC, HEARTBEAT_RESPONSE_DATA_INIT}

typedef struct Heartbeat_Response_data_tag{

} Heartbeat_Response_data;

typedef struct Heartbeat_Response_tag{
  uint16_t ID;
  uint8_t DLC;
  Heartbeat_Response_data data;
} Heartbeat_Response;


#define SET_POSITION_ID                 3
#define SET_POSITION_DLC                5
#define SET_POSITION_DATA_INIT          {0,0}
#define SET_POSITION_INIT               {SET_POSITION_ID, SET_POSITION_DLC, SET_POSITION_DATA_INIT}

typedef struct Set_Position_data_tag{
      uint32_t floatPos;
      uint8_t Dimension;
} Set_Position_data;

typedef struct Set_Position_tag{
  uint16_t ID;
  uint8_t DLC;
  Set_Position_data data;
} Set_Position;


#define SET_JOINT_ANGLE_ID                 4
#define SET_JOINT_ANGLE_DLC                5
#define SET_JOINT_ANGLE_DATA_INIT          {0,0}
#define SET_JOINT_ANGLE_INIT               {SET_JOINT_ANGLE_ID, SET_JOINT_ANGLE_DLC, SET_JOINT_ANGLE_DATA_INIT}

typedef struct Set_Joint_Angle_data_tag{
      uint32_t floatDegrees;
      uint8_t Joint;
} Set_Joint_Angle_data;

typedef struct Set_Joint_Angle_tag{
  uint16_t ID;
  uint8_t DLC;
  Set_Joint_Angle_data data;
} Set_Joint_Angle;


#define GET_ENCODER_STATUS_ID                 5
#define GET_ENCODER_STATUS_DLC                0
#define GET_ENCODER_STATUS_DATA_INIT          {}
#define GET_ENCODER_STATUS_INIT               {GET_ENCODER_STATUS_ID, GET_ENCODER_STATUS_DLC, GET_ENCODER_STATUS_DATA_INIT}

typedef struct Get_Encoder_Status_data_tag{

} Get_Encoder_Status_data;

typedef struct Get_Encoder_Status_tag{
  uint16_t ID;
  uint8_t DLC;
  Get_Encoder_Status_data data;
} Get_Encoder_Status;


#define GET_ENCODER_STATUS_RESPONSE_ID                 6
#define GET_ENCODER_STATUS_RESPONSE_DLC                1
#define GET_ENCODER_STATUS_RESPONSE_DATA_INIT          {0}
#define GET_ENCODER_STATUS_RESPONSE_INIT               {GET_ENCODER_STATUS_RESPONSE_ID, GET_ENCODER_STATUS_RESPONSE_DLC, GET_ENCODER_STATUS_RESPONSE_DATA_INIT}

typedef struct Get_Encoder_Status_Response_data_tag{
      uint8_t Joint;
} Get_Encoder_Status_Response_data;

typedef struct Get_Encoder_Status_Response_tag{
  uint16_t ID;
  uint8_t DLC;
  Get_Encoder_Status_Response_data data;
} Get_Encoder_Status_Response;


#define GET_JOINT_CURRENT_ID                 7
#define GET_JOINT_CURRENT_DLC                1
#define GET_JOINT_CURRENT_DATA_INIT          {0}
#define GET_JOINT_CURRENT_INIT               {GET_JOINT_CURRENT_ID, GET_JOINT_CURRENT_DLC, GET_JOINT_CURRENT_DATA_INIT}

typedef struct Get_Joint_Current_data_tag{
      uint8_t Joint;
} Get_Joint_Current_data;

typedef struct Get_Joint_Current_tag{
  uint16_t ID;
  uint8_t DLC;
  Get_Joint_Current_data data;
} Get_Joint_Current;


#define GET_JOINT_CURRENT_RESPONSE_ID                 8
#define GET_JOINT_CURRENT_RESPONSE_DLC                5
#define GET_JOINT_CURRENT_RESPONSE_DATA_INIT          {0,0}
#define GET_JOINT_CURRENT_RESPONSE_INIT               {GET_JOINT_CURRENT_RESPONSE_ID, GET_JOINT_CURRENT_RESPONSE_DLC, GET_JOINT_CURRENT_RESPONSE_DATA_INIT}

typedef struct Get_Joint_Current_Response_data_tag{
      uint32_t current;
      uint8_t Joint;
} Get_Joint_Current_Response_data;

typedef struct Get_Joint_Current_Response_tag{
  uint16_t ID;
  uint8_t DLC;
  Get_Joint_Current_Response_data data;
} Get_Joint_Current_Response;


#define GET_POSITION_ID                 9
#define GET_POSITION_DLC                1
#define GET_POSITION_DATA_INIT          {0}
#define GET_POSITION_INIT               {GET_POSITION_ID, GET_POSITION_DLC, GET_POSITION_DATA_INIT}

typedef struct Get_Position_data_tag{
      uint8_t Dimension;
} Get_Position_data;

typedef struct Get_Position_tag{
  uint16_t ID;
  uint8_t DLC;
  Get_Position_data data;
} Get_Position;


#define GET_POSITION_RESPONSE_ID                 10
#define GET_POSITION_RESPONSE_DLC                5
#define GET_POSITION_RESPONSE_DATA_INIT          {0,0}
#define GET_POSITION_RESPONSE_INIT               {GET_POSITION_RESPONSE_ID, GET_POSITION_RESPONSE_DLC, GET_POSITION_RESPONSE_DATA_INIT}

typedef struct Get_Position_Response_data_tag{
      uint32_t Position;
      uint8_t Dimension;
} Get_Position_Response_data;

typedef struct Get_Position_Response_tag{
  uint16_t ID;
  uint8_t DLC;
  Get_Position_Response_data data;
} Get_Position_Response;


#define GET_JOINT_ANGLE_ID                 11
#define GET_JOINT_ANGLE_DLC                1
#define GET_JOINT_ANGLE_DATA_INIT          {0}
#define GET_JOINT_ANGLE_INIT               {GET_JOINT_ANGLE_ID, GET_JOINT_ANGLE_DLC, GET_JOINT_ANGLE_DATA_INIT}

typedef struct Get_Joint_Angle_data_tag{
      uint8_t Joint;
} Get_Joint_Angle_data;

typedef struct Get_Joint_Angle_tag{
  uint16_t ID;
  uint8_t DLC;
  Get_Joint_Angle_data data;
} Get_Joint_Angle;


#define GET_JOINT_ANGLE_RESPONSE_ID                 12
#define GET_JOINT_ANGLE_RESPONSE_DLC                5
#define GET_JOINT_ANGLE_RESPONSE_DATA_INIT          {0,0}
#define GET_JOINT_ANGLE_RESPONSE_INIT               {GET_JOINT_ANGLE_RESPONSE_ID, GET_JOINT_ANGLE_RESPONSE_DLC, GET_JOINT_ANGLE_RESPONSE_DATA_INIT}

typedef struct Get_Joint_Angle_Response_data_tag{
      uint32_t angleBytes;
      uint8_t Joint;
} Get_Joint_Angle_Response_data;

typedef struct Get_Joint_Angle_Response_tag{
  uint16_t ID;
  uint8_t DLC;
  Get_Joint_Angle_Response_data data;
} Get_Joint_Angle_Response;


#define GET_CONTROL_STATUS_ID                 13
#define GET_CONTROL_STATUS_DLC                0
#define GET_CONTROL_STATUS_DATA_INIT          {}
#define GET_CONTROL_STATUS_INIT               {GET_CONTROL_STATUS_ID, GET_CONTROL_STATUS_DLC, GET_CONTROL_STATUS_DATA_INIT}

typedef struct Get_Control_Status_data_tag{

} Get_Control_Status_data;

typedef struct Get_Control_Status_tag{
  uint16_t ID;
  uint8_t DLC;
  union {
      Get_Control_Status_data data;
      Get_Control_Status_data *pData;
  };
} Get_Control_Status;


#define GET_CONTROL_STATUS_RESPONSE_ID                 14
#define GET_CONTROL_STATUS_RESPONSE_DLC                1
#define GET_CONTROL_STATUS_RESPONSE_DATA_INIT          {0}
#define GET_CONTROL_STATUS_RESPONSE_INIT               {GET_CONTROL_STATUS_RESPONSE_ID, GET_CONTROL_STATUS_RESPONSE_DLC, GET_CONTROL_STATUS_RESPONSE_DATA_INIT}

typedef struct Get_Control_Status_Response_data_tag{
      uint8_t status;
} Get_Control_Status_Response_data;

typedef struct Get_Control_Status_Response_tag{
  uint16_t ID;
  uint8_t DLC;
  Get_Control_Status_Response_data data;
} Get_Control_Status_Response;


#define AUTO_HOME_ID                 15
#define AUTO_HOME_DLC                0
#define AUTO_HOME_DATA_INIT          {}
#define AUTO_HOME_INIT               {AUTO_HOME_ID, AUTO_HOME_DLC, AUTO_HOME_DATA_INIT}

typedef struct Auto_Home_data_tag{

} Auto_Home_data;

typedef struct Auto_Home_tag{
  uint16_t ID;
  uint8_t DLC;
  Auto_Home_data data;
} Auto_Home;


#define MANUAL_HOME_ID                 16
#define MANUAL_HOME_DLC                0
#define MANUAL_HOME_DATA_INIT          {}
#define MANUAL_HOME_INIT               {MANUAL_HOME_ID, MANUAL_HOME_DLC, MANUAL_HOME_DATA_INIT}

typedef struct Manual_Home_data_tag{

} Manual_Home_data;

typedef struct Manual_Home_tag{
  uint16_t ID;
  uint8_t DLC;
  Manual_Home_data data;
} Manual_Home;


#define HOME_RESPONSE_ID                 17
#define HOME_RESPONSE_DLC                0
#define HOME_RESPONSE_DATA_INIT          {}
#define HOME_RESPONSE_INIT               {HOME_RESPONSE_ID, HOME_RESPONSE_DLC, HOME_RESPONSE_DATA_INIT}

typedef struct Home_Response_data_tag{

} Home_Response_data;

typedef struct Home_Response_tag{
  uint16_t ID;
  uint8_t DLC;
  Home_Response_data data;
} Home_Response;


#endif // IDCD_H

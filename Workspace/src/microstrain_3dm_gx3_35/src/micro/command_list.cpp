#include "micro/microstrain_comm.h"

/**
 * Templates of MIP packet
 * There use an 'x' as the end flag for the converting from char* to std::string in set_commands() function.
 */

// Headers
char Base_Command_Header[] = {
    static_cast<char> (MIP_PACKET_SYNC1),
    static_cast<char> (MIP_PACKET_SYNC2),
    static_cast<char> (BASE_COMMAND_SET),
    static_cast<char> (0x02), //Payload length (default)
    'x'
};

char DM_Command_Header[] = {
    static_cast<char> (MIP_PACKET_SYNC1),
    static_cast<char> (MIP_PACKET_SYNC2),
    static_cast<char> (DM_COMMAND_SET),
    static_cast<char> (0x00), //Payload length (mannual)
    'x'
};

char Sys_Command_Header[] = {
    static_cast<char> (MIP_PACKET_SYNC1),
    static_cast<char> (MIP_PACKET_SYNC2),
    static_cast<char> (SYS_COMMAND_SET),
    static_cast<char> (0x04), //Payload length (default)
    'x'
};

// Fields
char Set_To_Idle[] = {
    static_cast<char> (LEN_OF_FIELD(0x02)),
    static_cast<char> (SET_TO_IDLE),
    'x'
    };

char Resume[] = {
    static_cast<char> (LEN_OF_FIELD(0x02)),
    static_cast<char> (RESUME),
    'x'
};

char Device_Reset[] = {
    static_cast<char> (LEN_OF_FIELD(0x02)),
    static_cast<char> (DEV_RESET),
    'x'
};

char Disable_AHRS_Data_Stream[] = {
    static_cast<char> (LEN_OF_FIELD(0x05)),      //Field length
    static_cast<char> (EN_DEV_CONT_DATA_STREAM), //Cmd Desc.
    static_cast<char> (APPLY_NEW_SETTINGS),      //Action (Apply)--
    static_cast<char> (INDEX_OF_DEVICE(0x01)),   //Device (AHRS)--
    static_cast<char> (DATA_STREAM_ON(0x00)),    //Stream (Off)--
    'x'
};

char Disable_GPS_Data_Stream[] = {
    static_cast<char> (LEN_OF_FIELD(0x05)),      //Field length
    static_cast<char> (EN_DEV_CONT_DATA_STREAM), //Cmd Desc.
    static_cast<char> (APPLY_NEW_SETTINGS),      //Action (Apply)--
    static_cast<char> (INDEX_OF_DEVICE(0x02)),   //Device (GPS)--
    static_cast<char> (DATA_STREAM_ON(0x00)),    //Stream (Off)--
    'x'
};

char Set_UART_BAUD_Rate[] = {
    static_cast<char> (LEN_OF_FIELD(0x07)),
    static_cast<char> (UART_BAUD_RATE),
    static_cast<char> (APPLY_NEW_SETTINGS),
    static_cast<char> (BAUD_RATE_DEFAULT_BYTE_1), //Baud rate h
    static_cast<char> (BAUD_RATE_DEFAULT_BYTE_2), //Baud rate h
    static_cast<char> (BAUD_RATE_DEFAULT_BYTE_3), //Baud rate l
    static_cast<char> (BAUD_RATE_DEFAULT_BYTE_4), //Baud rate l (115200)--
    'x'
};

char Set_AHRS_Message_Format[] = {
    static_cast<char> (LEN_OF_FIELD(0x0D)),
    static_cast<char> (AHRS_MESSAGE_FORMAT),
    static_cast<char> (APPLY_NEW_SETTINGS),
    static_cast<char> (0x03),                //Desc. count--
    static_cast<char> (SCALED_ACC_VECTOR),   //1st Desc. (Acc)--
    static_cast<char> (0x00),                //Rate dec h
    static_cast<char> (AHRS_DATA_RATE(100)), //Rate dec l (100Hz)--
    static_cast<char> (SCALED_GYRO_VECTOR),  //2nd Desc. (Gyro)--
    static_cast<char> (0x00),                //Rate dec h
    static_cast<char> (AHRS_DATA_RATE(100)), //Rate dec l (100Hz)--
    static_cast<char> (INTERNAL_TIME_STAMP), //3rd Desc. (Timestamp)--
    //static_cast<char> (QUATERNION),          //3rd Desc. (Quaternion)--
    static_cast<char> (0x00),                //Rate dec h
    static_cast<char> (AHRS_DATA_RATE(100)), //Rate dec l (100Hz)--
    'x'
};

char Set_GPS_Message_Format[] = {
    static_cast<char> (LEN_OF_FIELD(0x0A)),
    static_cast<char> (GPS_MESSAGE_FORMAT),
    static_cast<char> (APPLY_NEW_SETTINGS),
    static_cast<char> (0x02),             //Desc. count--
    static_cast<char> (ECEF_POSITION),    //ECEF Pos Desc.--
    static_cast<char> (0x00),             //Rate dec h
    static_cast<char> (GPS_DATA_RATE(1)), //Rate dec l (1Hz)--
    static_cast<char> (ECEF_VELOCITY),    //ECEF Vel Desc.--
    static_cast<char> (0x00),             //Rate dec h
    static_cast<char> (GPS_DATA_RATE(1)), //Rate dec l (1Hz)--
    'x'
};

char Save_AHRS_Message_Format[] = {
    static_cast<char> (LEN_OF_FIELD(0x04)),
    static_cast<char> (AHRS_MESSAGE_FORMAT),
    static_cast<char> (SAVE_CURRENT_SETTINGS_AS_STARTUP_SETTINGS),
    static_cast<char> (0x00), //Desc. count--
    'x'
};

char Save_GPS_Message_Format[] = {
    static_cast<char> (LEN_OF_FIELD(0x04)),
    static_cast<char> (GPS_MESSAGE_FORMAT),
    static_cast<char> (SAVE_CURRENT_SETTINGS_AS_STARTUP_SETTINGS),
    static_cast<char> (0x00), //Desc. count--
    'x'
};

char Enable_AHRS_Data_Stream[] = {
    static_cast<char> (LEN_OF_FIELD(0x05)),
    static_cast<char> (EN_DEV_CONT_DATA_STREAM),
    static_cast<char> (APPLY_NEW_SETTINGS),
    static_cast<char> (INDEX_OF_DEVICE(0x01)), //Device (AHRS)--
    static_cast<char> (DATA_STREAM_ON(0x01)),  //Stream (On)--
    'x'
};

char Enable_GPS_Data_Stream[] = {
    static_cast<char> (LEN_OF_FIELD(0x05)),
    static_cast<char> (EN_DEV_CONT_DATA_STREAM),
    static_cast<char> (APPLY_NEW_SETTINGS),
    static_cast<char> (INDEX_OF_DEVICE(0x02)), //Device (GPS)--
    static_cast<char> (DATA_STREAM_ON(0x01)),  //Stream (On)--
    'x'
};


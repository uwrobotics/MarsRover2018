#ifndef __microstrain_comm__
#define __microstrain_comm__

/**
 * Define all needed constants
 * Check the MIP protocol pdf for info on how to add settings.
 */

/**
 * Macros for MIP Protocol
 */

// Base Command Set
#define BASE_COMMAND_SET   0x01

#define PING               0x01
#define SET_TO_IDLE        0x02
#define GET_DEV_INFO       0x03
#define GET_DEV_DESC       0x04
#define DEV_BUILT_IN_TEST  0x05
#define RESUME             0x06
#define DEV_RESET          0x7E

#define BASE_COMMAND_REPLY 0xF1

// 3DM Command Set
#define DM_COMMAND_SET               0x0C

#define POLL_AHRS_DATA               0x01
#define POLL_GPS_DATA                0x02
#define GET_AHRS_DATA_RATE_BASE      0x06
#define GET_GPS_DATA_RATE_BASE       0x07
#define AHRS_MESSAGE_FORMAT          0x08
#define GPS_MESSAGE_FORMAT           0x09
#define EN_DEV_CONT_DATA_STREAM      0x11
#define SAVE_DEV_STARTUP_SETTING     0x30
#define GPS_DYNAMICS_MODE            0x34
#define AHRS_SIG_CONDITIONAL_SETTING 0x35
#define AHRS_TIME_STAMP              0x36
#define UART_BAUD_RATE               0x40

#define DM_COMMAND_REPLY             0xF1

// System Command Set
#define SYS_COMMAND_SET    0x7F

#define COMMUNICATION_MODE 0x10

#define SYS_COMMAND_REPLY  0xF1

// AHRS Data Sets
#define AHRS_DATA_SET            0x80

#define RAW_ACC_VECTOR           0x01
#define RAW_GYRO_VECTOR          0x02
#define RAW_MAG_VECTOR           0x03
#define SCALED_ACC_VECTOR        0x04
#define SCALED_GYRO_VECTOR       0x05
#define SCALED_MAG_VECTOR        0x06
#define DELTA_THETA_VECTOR       0x07
#define DELTA_VEL_VECTOR         0x08
#define ORIENTATION_MATRIX       0x09
#define QUATERNION               0x0A
#define ORIENATION_UPDATE_MATRIX 0x0B
#define EULER_ANGLES             0x0C
#define INTERNAL_TIME_STAMP      0x0E
#define BEACONED_TIME_STAMP      0x0F
#define STABILIZED_MAG_VECTOR    0x10 //North
#define STABILIZED_ACC_VECTOR    0x11 //Up
#define GPS_TIME_STAMP           0x12
#define WRAPPED_RAW_GX3_25_DATA_PACKET 0x82

// GPS Data Set
#define GPS_DATA_SET       0x81

#define LLH_POSITION       0x03
#define ECEF_POSITION      0x04
#define NED_VELOCITY       0x05
#define ECEF_VELOCITY      0x06
#define DOP_DATA           0x07
#define UTC_TIME           0x08
#define GPS_TIME           0x09
#define CLOCK_INFO         0x0A
#define GPS_FIX_INFO       0x0B
#define SPACE_VEHICLE_INFO 0x0C
#define HARDWARE_STATUS    0x0D
#define WRAPPED_RAW_NMEA_PACKET 0x01
#define WRAPPED_RAW_UBX_PACKET  0x02

// MIP Packet Header
#define MIP_PACKET_SYNC1 0x75 //"u"
#define MIP_PACKET_SYNC2 0x65 //"e"

// MIP Packet Field
#define LEN_OF_FIELD(x)       (x)
#define NUM_OF_DESCRIPTORS(x) (x)
#define INDEX_OF_DEVICE(x)    (x)
#define DATA_STREAM_ON(x)     (x)

// MIP Packet Length
#define LEN_PACKET_HEADER   0x04
#define LEN_PACKET_CHECKSUM 0x02

#define LEN_REPLY_HEADER    0x04 //ACK/NACK
#define LEN_REPLY_CHECKSUM  0x02 //ACK/NACK

// MIP Error Codes
#define MIP_ACK_NACK_ERROR_NONE              0x00
#define MIP_ACK_NACK_ERROR_UNKNOWN_COMMAND   0x01
#define MIP_ACK_NACK_ERROR_CHECKSUM_INVALID  0x02
#define MIP_ACK_NACK_ERROR_PARAMETER_INVALID 0x03
#define MIP_ACK_NACK_ERROR_COMMAND_FAILED    0x04
#define MIP_ACK_NACK_ERROR_COMMAND_TIMEOUT   0x05

// MIP Function Selectors
#define APPLY_NEW_SETTINGS                        0x01
#define READ_BACK_CURRENT_SETTINGS                0x02
#define SAVE_CURRENT_SETTINGS_AS_STARTUP_SETTINGS 0x03
#define LOAD_SAVED_STARTUP_SETTINGS               0x04
#define LOAD_FACTORY_DEFAULT_SETTINGS             0x05

/**
 * Some Relative Variables
 */

// BAUD Rate
#define BAUD_RATE_DEFAULT 115200
#define BAUD_RATE_MED     460800
#define BAUD_RATE_HIGH    921600

#define BAUD_RATE_DEFAULT_BYTE_1 0x00 //115200
#define BAUD_RATE_DEFAULT_BYTE_2 0x01
#define BAUD_RATE_DEFAULT_BYTE_3 0xC2
#define BAUD_RATE_DEFAULT_BYTE_4 0x00

// Data Rate
#define DATA_RATE_HIGH    1000
#define DATA_RATE_MED     500
#define DATA_RATE_DEFAULT 100

#define AHRS_DATA_RATE(x) (1000 / x)
#define GPS_DATA_RATE(x)  (4 / x)

// Others
#define GRAVITY 9.80665
#define INPUT_BUFFER_SIZE 1024

/**
 * Templates for MIP packet
 */

// Headers
extern char Base_Command_Header[];
extern char DM_Command_Header[];
extern char Sys_Command_Header[];

// Fields
extern char Set_To_Idle[];
extern char Resume[];
extern char Device_Reset[];

extern char Disable_AHRS_Data_Stream[];
extern char Disable_GPS_Data_Stream[];
extern char Set_UART_BAUD_Rate[];
extern char Set_AHRS_Message_Format[];
extern char Set_GPS_Message_Format[];
extern char Save_AHRS_Message_Format[];
extern char Save_GPS_Message_Format[];
extern char Enable_AHRS_Data_Stream[];
extern char Enable_GPS_Data_Stream[];

#endif

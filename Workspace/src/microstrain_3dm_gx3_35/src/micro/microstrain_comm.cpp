#include <termios.h> // terminal io (serial port) interface
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
#include <assert.h>
#include <signal.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <ros/ros.h>
#include <ros/time.h>
#include "sensor_msgs/Imu.h"

#include <glib.h>
#include <gio/gio.h>

#include "micro/microstrain_comm.h"
#include "micro/util.h"

#include "libbot/timestamp.h"
#include "libbot/rotations.h"
#include "libbot/small_linalg.h"
#include "libbot/ringbuf.h"

using namespace std;

typedef unsigned char Byte;

// Global reference to Glib main loop
static GMainLoop* mainloop = NULL;

// Global ROS publisher
ros::Publisher imu_data_pub_;

// Core app self structure
class app_t {
public:
    // Com port
    int comm; //file descriptor
    char comm_port_name[255];

    // Buffers: com port (buffer)-->read_buffer-->input_buffer
    Byte input_buffer[INPUT_BUFFER_SIZE];
    BotRingBuf* read_buffer;

    // MIP Packet
    char current_segment; //in header or payload
    int  expected_segment_length; //packet length

    // Setting flags
    bool verbose;
    bool debug;
    bool little_endian;

    // Timer
    int64_t utime;
    int64_t utime_prev;

    // IMU ROS message
    sensor_msgs::Imu imu_data;

    // Sync
    bot_timestamp_sync_state* sync;
    bool do_sync;
};

/**
 * sig_action()
 * Callback function that will exit the glib loop.
 */
static void sig_action(int signal, siginfo_t* s, void* user)
{
    // Kill the glib main loop...
    if (g_main_loop_is_running(mainloop)) {
        g_main_loop_quit(mainloop);
    }
}

/**
 * install_signal_handler()
 * Set our signal callback, and have it called on SIGINT, SIGTERM, SIGKILL, SIGHUP.
 */
void install_signal_handler()
{
    struct sigaction action;
    action.sa_sigaction = sig_action;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;

    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
    sigaction(SIGKILL, &action, NULL);
    sigaction(SIGHUP, &action, NULL);
}

/**
 * setup_com_port()
 * Set the com port with user-defined parameters.
 */
int setup_com_port(int comPort, speed_t baudRate) {
    struct termios options;

    // Get the current options
    tcgetattr(comPort, &options);

    // Set the baud rate (default is 115200)
    cfsetospeed(&options, baudRate);
    cfsetispeed(&options, baudRate);

    // Set the number of data bits
    options.c_cflag &= ~CSIZE; //mask the character size bits
    options.c_cflag |= CS8;

    // Set the number of stop bits to 1
    options.c_cflag &= ~CSTOPB;

    // Set parity to none
    options.c_cflag &= ~PARENB;

    // Set for non-canonical (raw processing, no echo, etc.)
    options.c_iflag = IGNPAR; //ignore parity check
    options.c_oflag = 0;      //raw output
    options.c_lflag = 0;      //raw input

    // time-outs -- won't work with NDELAY option in the call to open
    options.c_cc[VMIN]  = 0;   //block reading until RX x characers. If x = 0, it is non-blocking.
    options.c_cc[VTIME] = 100; //Inter-Character Timer -- i.e. timeout= x*.1 s

    // Set local mode and enable the receiver
    options.c_cflag |= (CLOCAL | CREAD);

    // Purge com port I\O buffers
    tcflush(comPort, TCIOFLUSH);

    // Set the new options
    int status = tcsetattr(comPort, TCSANOW, &options);

    if (status != 0) {
        fprintf(stderr, "Error: failed in configuring com port\n");
        return status;
    }

    // Purge com port I\O buffers
    tcflush(comPort, TCIOFLUSH);

    return comPort;
}

/**
 * open_com_port()
 * Open a com port with the right settings.
 */
int open_com_port(const char* comPortPath, speed_t baudRate) {
    int comPort = open(comPortPath, O_RDWR | O_NOCTTY);

    if (comPort == -1) {
        fprintf(stderr, "Error: could not open the com port %x : %i\n", comPortPath, errno);
        return -1;
    }

    return setup_com_port(comPort, baudRate);
}

/**
 * scandev()
 * Scan all com ports to find the attached microstrain devices.
 */
bool scandev(char* comm_port_name) {
    FILE *instream;
    char dev_names[255][255]; //allows for up to 256 devices with path links up to 255 characters long each
    int dev_count = 0;
    int usr_choice = 0; //default
    int i, j;

    cout << "Searching for MicroStrain devices..." << endl;

    char command[] = "find /dev/serial -print | grep -i microstrain"; //search /dev/serial for microstrain devices

    instream = popen(command, "r"); //execute piped command in read mode
    if (!instream) {
        cout << "Error: failed in opening pipeline : " << command << endl;
        return false;
    }

    // Load the char array of device addresses
    for (i = 0; i < 255 && (fgets(dev_names[i], sizeof(dev_names[i]), instream)); ++i) {
        ++dev_count;
    }

    for (i = 0; i < dev_count; ++i) {
        for (j = 0; j < sizeof(dev_names[i]); ++j) {
            if (dev_names[i][j] == '\n') {
                dev_names[i][j] = '\0'; //replaces newline inserted by pipe reader with the char array terminator character
                break; //breaks loop after replacement
            }
        }
        cout << "MicroStrain devices found: " << i << " : " << dev_names[i] << endl;
    }

    // Select the device and connect to it
    if (dev_count > 0) {
        if (dev_count > 1) {
            fprintf(stderr, "Please choose a device to connect (0 to %i):\n", dev_count - 1);
            while (scanf("%i", &usr_choice) == 0 || usr_choice < 0 || usr_choice > dev_count - 1) { //check that there's input and in the correct range
                fprintf(stderr, "Invalid choice... Please choose one between 0 and %d:\n", dev_count - 1);
                getchar(); //clear carriage return from keyboard buffer after invalid choice
            }
        }

        strcpy(comm_port_name, dev_names[usr_choice]);
        return true;
    } else {
        fprintf(stderr, "No MicroStrain devices found\n");
        return false;
    }
}

/**
 * cksum()
 * Use two-byte Fletcher ckecksum algorithm.
 */
unsigned short cksum(const Byte* packet_bytes, int packet_length) {
    uint8_t checksum_byte1 = 0;
    uint8_t checksum_byte2 = 0;

    for (int i = 0; i < packet_length - 2; ++i) {
        checksum_byte1 += packet_bytes[i];
        checksum_byte2 += checksum_byte1;
    }

    return ((uint16_t) checksum_byte1 << 8) + ((uint16_t) checksum_byte2);
}

/**
 * handle_error()
 * Parse the error byte in the payload of MIP packet.
 */
bool handle_error(char* command_name, Byte error_code) {
    if (error_code == MIP_ACK_NACK_ERROR_NONE) {
        fprintf(stderr, "Received [%s] command echo : no error\n", command_name);
        return true;
    }
    if (error_code == MIP_ACK_NACK_ERROR_UNKNOWN_COMMAND) {
        fprintf(stderr, "Received [%s] command echo : unknown command\n", command_name);
        return false;
    }
    if (error_code == MIP_ACK_NACK_ERROR_CHECKSUM_INVALID) {
        fprintf(stderr, "Received [%s] command echo : checksum invalid\n", command_name);
        return false;
    }
    if (error_code == MIP_ACK_NACK_ERROR_PARAMETER_INVALID) {
        fprintf(stderr, "Received [%s] command echo : parameter invalid\n", command_name);
        return false;
    }
    if (error_code == MIP_ACK_NACK_ERROR_COMMAND_FAILED) {
        fprintf(stderr, "Received [%s] command echo : command failed\n", command_name);
        return false;
    }
    if (error_code == MIP_ACK_NACK_ERROR_COMMAND_TIMEOUT) {
        fprintf(stderr, "Received [%s] command echo : command timeout\n", command_name);
        return false;
    }
}

/**
 * handle_message()
 * Parse the payload bytes in MIP packet.
 */
bool handle_message(app_t* app) {
    bool success = false;

    uint8_t header_byte_set_desc;
    uint8_t header_byte_payload_length;

    uint8_t field_1_byte_length, field_2_byte_length,   field_3_byte_length; //TODO: field_4_...
    uint8_t field_1_byte_cmd_desc, field_2_byte_cmd_desc, field_3_byte_cmd_desc; //TODO: field_4_...

    // Command set reply
    uint8_t field_1_byte_cmd_echo, field_2_byte_cmd_echo, field_3_byte_cmd_echo; //TODO: field_4_...
    uint8_t field_1_byte_err_code, field_2_byte_err_code, field_3_byte_err_code; //TODO: field_4_...

    // Data set reply
    uint8_t field_1_byte_data, field_2_byte_data, field_3_byte_data; //TODO: field_4_...

    uint8_t len_payload;
    uint8_t len_field_1, len_field_2, len_field_3; //TODO: len_field_4

    float float_vals[9] = {0}; //for the longest data format (36-byte) convert

    int ins_timer;
    int64_t utime = bot_timestamp_now(); //get current time stamp
    int64_t utime_nosyn = utime;

    if (app->verbose)
        print_array_char_hex((unsigned char*) app->input_buffer, app->expected_segment_length);

    // Byte indices(index) in MIP packet
    header_byte_set_desc = 2;
    header_byte_payload_length = 3;
    len_payload = (uint8_t) app->input_buffer[header_byte_payload_length];

    field_1_byte_length   = 4;
    field_1_byte_cmd_desc = 5;
    field_1_byte_cmd_echo = 6; //for command set reply
    field_1_byte_err_code = 7; //for command set reply
    field_1_byte_data     = 6; //for data set reply
    len_field_1 = (uint8_t) app->input_buffer[field_1_byte_length];

    if (len_payload - len_field_1 != 0) {
        field_2_byte_length   = field_1_byte_length + len_field_1;
        field_2_byte_cmd_desc = field_2_byte_length + 1;
        field_2_byte_cmd_echo = field_2_byte_length + 2;
        field_2_byte_err_code = field_2_byte_length + 3;
        field_2_byte_data     = field_2_byte_length + 2;
        len_field_2 = (uint8_t) app->input_buffer[field_2_byte_length];

        if (len_payload - len_field_1 - len_field_2 != 0) {
            field_3_byte_length   = field_2_byte_length + len_field_2;
            field_3_byte_cmd_desc = field_3_byte_length + 1;
            field_3_byte_cmd_echo = field_3_byte_length + 2;
            field_3_byte_err_code = field_3_byte_length + 3;
            field_3_byte_data     = field_3_byte_length + 2;
            len_field_3 = (uint8_t) app->input_buffer[field_3_byte_length];

            // TODO: field_4_...
            //if (len_payload - len_field_1 - len_field_2 - len_field_3 != 0){
                 //... ...
            //}
        }
    }

    // Parse the payload byte by byte
	switch (app->input_buffer[header_byte_set_desc]) {
        case BASE_COMMAND_SET: {
            if (app->input_buffer[field_1_byte_cmd_desc] == BASE_COMMAND_REPLY) {//for Reply Field 1: ACK/NACK
                switch (app->input_buffer[field_1_byte_cmd_echo]) {
                    case PING:
                        success = handle_error("Ping", app->input_buffer[field_1_byte_err_code]);
                        break;
                    case SET_TO_IDLE:
                        success = handle_error("Set To Idle", app->input_buffer[field_1_byte_err_code]);
                        break;
                    case GET_DEV_INFO:
                        success = handle_error("Get Device Info", app->input_buffer[field_1_byte_err_code]);
                        break;
                    case GET_DEV_DESC:
                        success = handle_error("Get Device Descriptor", app->input_buffer[field_1_byte_err_code]);
                        break;
                    case DEV_BUILT_IN_TEST:
                        success = handle_error("Device Built-in Test", app->input_buffer[field_1_byte_err_code]);
                        break;
                    case RESUME:
                        success = handle_error("Resume", app->input_buffer[field_1_byte_err_code]);
                        break;
                    case DEV_RESET:
                        success = handle_error("Device Reset", app->input_buffer[field_1_byte_err_code]);
                        break;
                    default:
                        fprintf(stderr, "Base Command Reply : nothing\n");
                        break;
                }
            }

            //if (app->input_buffer[3 + field_1_length + 2] == 0x81/0x83/...) {//TODO: for Reply Field 2: ... ...
            //
            //}

            break;
        }

        case DM_COMMAND_SET: {
            if (app->input_buffer[field_1_byte_cmd_desc] == BASE_COMMAND_REPLY) {//for Reply Field 1: ACK/NACK
                switch (app->input_buffer[field_1_byte_cmd_echo]) {
                    case AHRS_MESSAGE_FORMAT:
                        success = handle_error("AHRS Message Format", app->input_buffer[field_1_byte_err_code]);
                        break;
                    case GPS_MESSAGE_FORMAT:
                        success = handle_error("GPS Message Format", app->input_buffer[field_1_byte_err_code]);
                        break;
                    case EN_DEV_CONT_DATA_STREAM:
                        success = handle_error("Enable Device Continuous Data Stream", app->input_buffer[field_1_byte_err_code]);
                        break;
                    case SAVE_DEV_STARTUP_SETTING:
                        success = handle_error("Save Device Startup Setting", app->input_buffer[field_1_byte_err_code]);
                        break;
                    case UART_BAUD_RATE:
                        success = handle_error("UART Baud Rate", app->input_buffer[field_1_byte_err_code]);
                        break;
                    default:
                        fprintf(stderr, "DM Command Reply : nothing\n");
                        break;
                }
            }

            break;
        }

        case SYS_COMMAND_SET: {
            if (app->input_buffer[field_1_byte_cmd_desc] == BASE_COMMAND_REPLY) {//for Reply Field 1: ACK/NACK
                switch (app->input_buffer[field_1_byte_cmd_echo]) {
                    case COMMUNICATION_MODE:
                        success = handle_error("Communication Mode", app->input_buffer[field_1_byte_err_code]);
                        break;
                    default:
                        fprintf(stderr, "System Command Reply : nothing\n");
                        break;
                }
            }

            break;
        }

        case AHRS_DATA_SET: {
            switch (app->input_buffer[field_1_byte_cmd_desc]) {
                case SCALED_ACC_VECTOR: {
                    pack32BitFloats(float_vals, &app->input_buffer[field_1_byte_data], 3, app->little_endian);
                    cout << "acc: " << float_vals[0] * GRAVITY << ", " << float_vals[1] * GRAVITY << ", " << float_vals[2] * GRAVITY << endl;
                    app->imu_data.linear_acceleration.x = float_vals[0] * GRAVITY; //~g * 9.8m/s
                    app->imu_data.linear_acceleration.y = float_vals[1] * GRAVITY; //~g * 9.8m/s
                    app->imu_data.linear_acceleration.z = float_vals[2] * GRAVITY; //~g * 9.8m/s

                    break;
                }
                //case ...:
                    //... ...
                    //break;
                default:
                    break;
            }
            switch (app->input_buffer[field_2_byte_cmd_desc]) {
                case SCALED_GYRO_VECTOR: {
                    pack32BitFloats(float_vals, &app->input_buffer[field_2_byte_data], 3, app->little_endian);
                    cout << "gyro: " << float_vals[0] << ", " << float_vals[1] << ", " << float_vals[2] << endl;
                    app->imu_data.angular_velocity.x = float_vals[0]; //rad/s
                    app->imu_data.angular_velocity.y = float_vals[1]; //rad/s
                    app->imu_data.angular_velocity.z = float_vals[2]; //rad/s

                    break;
                }
                //case ...:
                    //... ...
                    //break;
                default:
                    break;
            }
            switch (app->input_buffer[field_3_byte_cmd_desc]) {
                case INTERNAL_TIME_STAMP: {
                    ins_timer = make32UnsignedInt(&app->input_buffer[field_3_byte_data], app->little_endian);
                    cout << "timestamp: " << ins_timer << endl;

                    if (app->do_sync) {
                        app->utime = bot_timestamp_sync(app->sync, ins_timer, utime);
                    } else {
                        app->utime = utime_nosyn;
                    }
                    //cout << "timestamp: " << app-utime << endl;

                    break;
                }
                case QUATERNION: {
                    pack32BitFloats(float_vals, &app->input_buffer[field_3_byte_data], 4, app->little_endian);
                    cout << "quaternion: " << float_vals[1] << ", " << float_vals[2] << ", " << float_vals[3] << ", " << float_vals[0] << endl;
                    app->imu_data.orientation.x = float_vals[1];
                    app->imu_data.orientation.y = float_vals[2];
                    app->imu_data.orientation.z = float_vals[3];
                    app->imu_data.orientation.w = float_vals[0];

                    break;
                                 }
                default:
                    break;
            }
            
            app->imu_data.header.stamp = ros::Time::now();
	    
            // Publish to imu topic
            imu_data_pub_.publish(app->imu_data);

            success = true;
            break;
        }

        case GPS_DATA_SET: {
            switch (app->input_buffer[field_1_byte_cmd_desc]) {
                case ECEF_POSITION:
                    //... ...
                    break;
                default:
                    break;
            }
            switch (app->input_buffer[field_2_byte_cmd_desc]) {
                case ECEF_VELOCITY:
                    //... ...
                    break;
                default:
                    break;
            }

            success = true;
            break;
        }

        default: {
            if (app->debug)
                fprintf(stderr, "Error: unknown message with the starting byte : 0x%x\n", app->input_buffer[0]);
            break;
        }
    }

    return success;
}

/**
 * unpack_packets()
 * It gets data packets out of ring buffer.
 * It has two states, either parsing the header bytes or parsing payload and checksum bytes.
 * It will be waiting until it has all expected bytes before taking appropriate action.
 */
void unpack_packets(app_t* app) {
    while (bot_ringbuf_available(app->read_buffer) >= app->expected_segment_length) {

        switch (app->current_segment) {
        case 'h':
            bot_ringbuf_peek(app->read_buffer, 4, app->input_buffer); //peek packet header
            if (app->verbose)
                fprintf(stderr, "Received packet with the starting byte : 0x%x\n", app->input_buffer[0]);

            // Parse the header
            if ((app->input_buffer[0] == MIP_PACKET_SYNC1) && (app->input_buffer[1] == MIP_PACKET_SYNC2)) {
                app->expected_segment_length = LEN_PACKET_HEADER + app->input_buffer[3] + LEN_PACKET_CHECKSUM;
                app->current_segment = 'p';
            } else {
                if (app->debug)
                    fprintf(stderr, "Error: unknown packet with the starting byte : 0x%x\n", app->input_buffer[0]);

                // Read out one byte and continue, until we find a match one
                bot_ringbuf_read(app->read_buffer, 1, app->input_buffer);
                app->current_segment = 'h';
                break;
            }
            break;

        case 'p':
            bot_ringbuf_read(app->read_buffer, app->expected_segment_length, app->input_buffer); //read whole packet

            unsigned short inpacket_cksum = make16UnsignedInt(&app->input_buffer[app->expected_segment_length - 2], app->little_endian);
            unsigned short computed_cksum = cksum(app->input_buffer, app->expected_segment_length);

            if (computed_cksum != inpacket_cksum) {
                if (app->debug)
                    fprintf(stderr, "Error: failed in checksum! got : %d, expected : %d\n", inpacket_cksum, computed_cksum);
                app->current_segment = 'h';
                app->expected_segment_length = 4;
                break;
            }

            if (app->verbose)
                fprintf(stderr, "Passed checksum, handling message...\n");
            bool success = handle_message(app);

            if (!success && app->debug)
                fprintf(stderr, "Error: failed in handling message\n");

            app->current_segment = 'h';
            app->expected_segment_length = 4;
            break;
        }
    }
}

/**
 * set_commands()
 * It generates the commands for AHRS and GPS settings.
 * It uses std::string to edit the message so that user can change the contents easily.
 */
bool set_commands(app_t* app, const char* command_header, const char* command_field_1, const char* command_field_2) {
    string command_t;
    string command_field_1_t, command_field_2_t;
    string len_cmd_field_1_t, len_cmd_field_2_t;
    string checksum_byte_t;

    // 1. Edit command header
    for (int i = 0; command_header[i] != 'x'; ++i) {
        if (app->debug)
            fprintf(stderr, "%x ", command_header[i]);
        command_t += command_header[i];
    }
    fprintf(stderr, "\n");

    if (app->debug)
        cout << "length of command header: " << command_t.size() << endl;

    // 2. Edit 1st command field
    if (command_field_1 != 0) {
        for (int j = 0; command_field_1[j] != 'x'; ++j) {
            if (app->debug)
                fprintf(stderr, "%x ", command_field_1[j]);
            command_field_1_t += command_field_1[j];
        }
        fprintf(stderr, "\n");

        len_cmd_field_1_t += (unsigned char) command_field_1_t.size();
        if (app->debug)
            cout << "length of 1st command field: " << command_field_1_t.size() << endl;

        command_t.replace(3, 1, len_cmd_field_1_t); //reset the payload length
        command_t += command_field_1_t; //add to command
    }

    // 3. Edit 2nd command field
    if (command_field_2 != 0) {
        for (int k = 0; command_field_2[k] != 'x'; ++k) {
            if (app->debug)
                fprintf(stderr, "%x ", command_field_2[k]);
            command_field_2_t += command_field_2[k];
        }
        fprintf(stderr, "\n");

        len_cmd_field_2_t += (unsigned char) (command_field_1_t.size() + command_field_2_t.size());
        if (app->debug)
            cout << "length of 2nd command field: " << command_field_2_t.size() << endl;

        command_t.replace(3, 1, len_cmd_field_2_t); //reset the payload length
        command_t += command_field_2_t; //add to command
    } else {
        if(app->debug)
            cout << "length of 2nd command field: " << 0 << endl;
    }

    // 4. Edit checksum bytes
    uint8_t checksum_byte_1 = 0;
    uint8_t checksum_byte_2 = 0;

    for (string::iterator it = command_t.begin(); it != command_t.end(); ++it) {
        checksum_byte_1 += *it;
        checksum_byte_2 += checksum_byte_1;
    }
    checksum_byte_t += (unsigned char) checksum_byte_1;
    checksum_byte_t += (unsigned char) checksum_byte_2;

    if (app->debug) {
        fprintf(stderr, "%x %x\n ", checksum_byte_1, checksum_byte_2);
        cout << "length of checksum: " << checksum_byte_t.size() << endl;
    }

    command_t += checksum_byte_t; //add to command

    // 5. Send the command
    char* command = new char [command_t.size() + 1]; //allocated on heap; +1 means add "\0" to the end
    memcpy(command, command_t.c_str(), command_t.size() + 1);

    if (app->debug) {
        for (int n = 0; n < command_t.size(); ++n) {
            fprintf(stderr, "%x ", command[n]);
        }
        fprintf(stderr,"\n");

        cout << "length of command: " << command_t.size() << endl;
    }

    if (write(app->comm, command, command_t.size() + 1) != (command_t.size() + 1)) {
        cout << "Error: failed in sending the command" << endl;
        return false;
    }

    delete[] command;

    // 6. Read the command reply
    if (read(app->comm, app->input_buffer, LEN_REPLY_HEADER) == -1) {
        cout << "Error: failed in receiving the header of command echo" << endl;
        return false;
    }

    uint8_t len_reply_payload = (uint8_t) app->input_buffer[3];
    app->expected_segment_length = LEN_REPLY_HEADER + len_reply_payload + LEN_REPLY_CHECKSUM;

    if (read(app->comm, &app->input_buffer[LEN_REPLY_HEADER], len_reply_payload + LEN_REPLY_CHECKSUM) == -1) {
        cout << "Error: failed in receiving the rest of command echo" << endl;
        return false;
    }

    if (app->debug) {
        for (int l = 0; l < app->expected_segment_length; ++l) {
            fprintf(stderr, "%x " , app->input_buffer[l]);
        }
        fprintf(stderr, "\n");

        cout << "length of reply: " << app->expected_segment_length << endl;
    }

    // 7. Parse the command reply
    if (app->input_buffer[0] == MIP_PACKET_SYNC1 && app->input_buffer[1] == MIP_PACKET_SYNC2) {
        unsigned short inpacket_cksum = make16UnsignedInt(&app->input_buffer[app->expected_segment_length - 2], app->little_endian);
        unsigned short computed_cksum = cksum(app->input_buffer, app->expected_segment_length);

        if (computed_cksum != inpacket_cksum) {
            fprintf(stderr, "Error: failed in checksum! got : %d, expected : %d\n", inpacket_cksum, computed_cksum);
            return false;
        }

        if (handle_message(app) == false) {
            cout << "Error: failed in handling echo message" << endl;
            return false;
        }
    } else {
        fprintf(stderr, "Error: unknown echo with the starting byte : 0x%x\n", app->input_buffer[0]);
        exit(1);
    }

   return true;
}

/**
 * serial_read_handler()
 * Callback function from the glib main loop
 * It reads serial bytes from ardu as they become available from g_io_watch, then
 * These bytes are writen to a ring buffer and the unpack_packets() function is called.
 */
static gboolean serial_read_handler(GIOChannel* source, GIOCondition condition, void* user) {
    // Check to see if the user has requested a stop
    if (!ros::ok()) {
        g_main_loop_quit(mainloop);
        return true;
    }

    app_t* app = (app_t*) user;

    static uint8_t middle_buffer[INPUT_BUFFER_SIZE];

    // Get the number of bytes available
    int available = 0;

    if (ioctl(app->comm, FIONREAD, &available) != 0) {
        if (!app->debug)
            fprintf(stderr, "Warning: ioctl check for bytes available didn't return 0, breaking read\n");
        return true;
    }

    if (available > INPUT_BUFFER_SIZE) {
        if (!app->debug)
            fprintf(stderr, "Warning: too many bytes available: %d, flushing input buffer\n", available);
        tcflush(app->comm, TCIFLUSH);
        return true;
    }

    int num_read = read(app->comm, middle_buffer, available);

    if (num_read != available) {
        if (!app->debug)
            fprintf(stderr, "Warning: read %d of %d available bytes\n", num_read, available);
    }

    if (num_read > 0) {
        bot_ringbuf_write(app->read_buffer, num_read, middle_buffer);
    }

    unpack_packets(app);

    return true;
}

/**
 * main()
 * It first reads in all parameter information, then
 * After setting the configuration of the driver, the IMU is set to continuous mode.
 * This main glib loop is what waits for data from the IMU until the process is ended
 */
int main(int argc, char** argv) {
    // ROS init
    ros::init(argc, argv, "microstrain_comm_35");
    ros::NodeHandle nh("~");

    // ROS publisher
    ros::NodeHandle imu_node_handle("imu");
    imu_data_pub_ = imu_node_handle.advertise<sensor_msgs::Imu>("data", 100);

    mainloop = g_main_loop_new(NULL, FALSE);

    // Install signal handler
    install_signal_handler();

    app_t* app = new app_t();
    app->little_endian = systemLittleEndianCheck();
    app->current_segment = 'h';
    app->expected_segment_length = 4;

    // Default settings
    string user_comm_port_name;
    bool dev_init;

    // Get params from the config file, or command line
    nh.param("verbose", app->verbose, false);
    nh.param("debug", app->debug, false);
    nh.param("com_port", user_comm_port_name, string(""));
    nh.param("time_sync", app->do_sync, true);
    nh.param("init", dev_init, false);

    app->utime_prev = bot_timestamp_now();
    app->read_buffer = bot_ringbuf_create(INPUT_BUFFER_SIZE);
    app->sync = bot_timestamp_sync_init(62500, (int64_t) 68719 * 62500, 1.001);

    // Use user specified port if there is one
    if (user_comm_port_name == "")
        scandev(app->comm_port_name);
    else
        strcpy(app->comm_port_name, user_comm_port_name.c_str());

    // Initialize com port at default baud rate
    app->comm = open_com_port(app->comm_port_name, BAUD_RATE_DEFAULT);
    if (app->comm < 0)
        exit(1);

    // Startup state machine
    if (dev_init) {
    cout << "MicroStrain IMU initialization begin..." << endl;

        if (!set_commands(app, DM_Command_Header, Disable_AHRS_Data_Stream, Disable_GPS_Data_Stream)) {
            cout << "ERROR in initialization: disableDataStream failed" << endl;
            exit(1);
        }

        if (!set_commands(app, DM_Command_Header, Set_AHRS_Message_Format, 0)) {
            cout << "ERROR in initialization: setImuDataStreamFormat failed" << endl;
            exit(1);
        }

        if (!set_commands(app, DM_Command_Header, Set_GPS_Message_Format, 0)) {
            cout << "ERROR in initialization: setGpsDataStreamFormat failed" << endl;
            exit(1);
        }

        if (!set_commands(app, DM_Command_Header, Save_AHRS_Message_Format, Save_GPS_Message_Format)) {
            cout << "ERROR in initialization: saveDataStreamFormat failed" << endl;
            exit(1);
        }

        if (!set_commands(app, DM_Command_Header, Enable_AHRS_Data_Stream, Enable_GPS_Data_Stream)) {
            cout << "ERROR in initialization: enableDataStream failed" << endl;
            exit(1);
        }

        cout << "initialization is done" << endl;
    } else {
        if (!set_commands(app, Base_Command_Header, Resume, 0)) {
            cout << "ERROR in initialization: resume device failed" << endl;
            exit(1);
        }
    }

    // Create a glib channel and run main thread
    GIOChannel* ioc = g_io_channel_unix_new(app->comm);
    g_io_add_watch_full(ioc, G_PRIORITY_HIGH, G_IO_IN, (GIOFunc)serial_read_handler, (void*)app, NULL);
    g_main_loop_run(mainloop);

    // If quit...
    // Set IMU to idle state
    set_commands(app, Base_Command_Header, Set_To_Idle, 0);

    // Close com port
    close(app->comm);

    delete app;

    return 0;
}


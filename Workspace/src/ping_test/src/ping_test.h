#ifndef PING_TEST_H
#define PING_TEST_H

#define PING_NUM_ATTEMPTS 5
#define PING_REQUIRED_SUCCESSES 4
#define PING_BYTES_PER_TEST 32
#define PING_TIME_BETWEEN_TESTS 30*1000
#define PING_REQUEST_TOPIC "ping_test/ping_request"
#define PING_REPLY_TOPIC "ping_test/ping_reply"
#define PING_CLIENT_RESULT_TOPIC "ping_test/ping_client_results"
#define PING_SERVER_RESULT_TOPIC "ping_test/ping_server_results"

#define PING_CLIENT_WAIT_TIME 1*1000
#define PING_SERVER_WAIT_TIME 2*1000

#define GPS_SENSOR_TOPIC "/gps/filtered"
//TODO: We need to figure out how to manage what gps goal we want,
//maybe in master control?
#define GPS_TARGET_TOPIC "/local_planner/goal_gps"

#endif


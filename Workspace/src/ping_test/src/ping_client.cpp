#include "ping_test.h"
#include "timer.h"
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <iostream>
#include <mutex>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8MultiArray.h>
#include <stdlib.h>
#include <thread>

std::mutex msgMutex;
std::mutex resultMutex;
std::condition_variable resultCondvar;

std_msgs::UInt8MultiArray requestMsg;

enum result_t { unfinished, success, timeout, failure };

result_t result = result_t::unfinished;
using namespace std;
Timer t;

unsigned int timestamp() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch())
      .count();
}

void timer_expiry(const boost::system::error_code &ec) {
  if (ec == boost::asio::error::operation_aborted) {
    // No-op
  } else {
    std::unique_lock<std::mutex> lock(resultMutex);
    if (result == result_t::unfinished) {
      result = result_t::timeout;
      lock.unlock();
    } else {
      lock.unlock();
    }
  }
}

void replyCallback(const std_msgs::UInt8MultiArray::ConstPtr &reply) {
  std::unique_lock<std::mutex> resultLock(resultMutex);
  if (result != result_t::unfinished) {
    // this is for flushing out leftover messages from a failed previous test
    return;
  }
  resultLock.unlock();

  bool success = true;
  t.cancel();
  std::unique_lock<std::mutex> msgLock(msgMutex);
  for (int i = 0; i < PING_BYTES_PER_TEST; i++) {
    if (reply->data[i] != requestMsg.data[i]) {
      success = false;
      break;
    }
  }
  msgLock.unlock();

  resultLock.lock();
  if (success) {
    result = result_t::success;
  } else {
    result = result_t::failure;
  }
  resultMutex.unlock();
}

int main(int argc, char **argv) {
  // Initializes ROS, and sets up a node
  ros::init(argc, argv, "ping_test_client");
  ros::NodeHandle nh;

  // Set up publishers and subscribers
  ros::Publisher pingPub =
      nh.advertise<std_msgs::UInt8MultiArray>(PING_REQUEST_TOPIC, 1);
  ros::Subscriber sub = nh.subscribe(PING_REPLY_TOPIC, 1, replyCallback);
  ros::Publisher resultPub =
      nh.advertise<std_msgs::Bool>(PING_CLIENT_RESULT_TOPIC, 1);

  // Sets up the random number generator
  srand(time(0));

  // Sleep for a bit so the publisher can set up
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  while (ros::ok()) {
    int numSuccesses = 0;
    int numTimeouts = 0;
    int numFailures = 0;
    for (int requestNum = 0; requestNum < PING_NUM_ATTEMPTS; requestNum++) {
      requestMsg.data.clear();
      requestMsg.data.reserve(PING_BYTES_PER_TEST);
      for (int i = 0; i < PING_BYTES_PER_TEST; i++) {
        requestMsg.data.push_back((uint8_t)(255 * (rand() / double(RAND_MAX))));
      }
      cout << "Sending " << PING_BYTES_PER_TEST << "  bytes of data:     ";
      t.start(PING_CLIENT_WAIT_TIME, &timer_expiry);
      auto start = std::chrono::high_resolution_clock::now();
      ros::spinOnce(); // flush out any leftover message
      pingPub.publish(requestMsg);
      std::unique_lock<std::mutex> lk(resultMutex);
      result = result_t::unfinished;

      while (result == result_t::unfinished) {
        lk.unlock();
        if (!ros::ok()) {
          // If the program has been killed, stop running
          return 0;
        }
        ros::spinOnce();
        lk.lock();
      }

      auto finish = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = finish - start;

      std::cout << "(Time: " << std::fixed << std::setprecision(5)
                << elapsed.count() << " s) ";

      switch (result) {
      case result_t::timeout:
        cout << "waiting for a reply timed out\n";
        numTimeouts++;
        break;
      case result_t::success:
        cout << "ping succeeded\n";
        numSuccesses++;
        break;
      case result_t::failure:
        cout << "Received erroneous data\n";
        numFailures++;
        break;
      default:

        break;
      }
      lk.unlock();
    }

    cout << "received " << numSuccesses << " out of " << PING_NUM_ATTEMPTS
         << " successful replies\n";
    // Send a message telling success or failure of the ping test
    std_msgs::Bool resultMsg;

    if (numSuccesses < PING_REQUIRED_SUCCESSES) {
      resultMsg.data = false;

    } else {
      resultMsg.data = true;
    }
    resultPub.publish(resultMsg);

    std::this_thread::sleep_for(
        std::chrono::milliseconds(PING_TIME_BETWEEN_TESTS));
  }
}

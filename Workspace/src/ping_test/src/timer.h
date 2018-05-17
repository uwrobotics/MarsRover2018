#ifndef TIMER_H
#define TIMER_H

#include "boost/asio.hpp"
#include <thread>
#include <mutex>
#include <condition_variable>

class Timer {
public:
    Timer() {
        dl = new boost::asio::deadline_timer(io);
        started = false;
        restart = false;
    }
    ~Timer() {
        cancel();
        delete dl;
    }
    void start(unsigned int milliseconds,
               void (*expiry_handler) (const boost::system::error_code &))
    {
        if (timerThread.joinable()) {
            timerThread.join();
        }
        timerThread = std::thread(&Timer::timer_operation, this, milliseconds, expiry_handler);
        std::unique_lock<std::mutex> lk(mutex);
        condvar.wait(lk, [this]{return this->started;});

    }

    //ONLY Call this from a timer expiry callback
    void restartTimer() {
        std::unique_lock<std::mutex> lk(mutex);
        restart = true;
        lk.unlock();
    }

    void cancel() {
        dl->cancel();
        if (timerThread.joinable()) {
            timerThread.join();
        }
    }

private:

    boost::asio::deadline_timer* dl;

    void timer_operation(unsigned int milliseconds, void (*expiry_handler) (const boost::system::error_code &)) {
        std::unique_lock<std::mutex> lk(mutex);
        do {
            dl->expires_from_now(boost::posix_time::milliseconds(milliseconds));
            dl->async_wait(boost::bind(expiry_handler, boost::asio::placeholders::error));
            started = true;
            restart = false;
            lk.unlock();
            condvar.notify_all();
            io.run_one();
            io.reset();
            lk.lock();
        } while (restart);
        started = false;
        lk.unlock();
    }

    boost::asio::io_service io;
    std::thread timerThread;
    bool started;
    bool restart;
    std::mutex mutex;
    std::condition_variable condvar;

};

#endif

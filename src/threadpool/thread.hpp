#pragma once

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <queue>

class Thread{
    public:
        Thread();
        ~Thread();
        Thread(const Thread&) = delete;
        Thread &operator=(const Thread&) = delete;
        Thread(Thread&&) = delete;
        Thread &operator=(Thread&&) = delete;

        int32_t getWeight();
        void addToQueue(const std::function<void()>& func, const int32_t& weight);
        bool busy();

    private:
        std::thread worker;
        std::queue<std::pair<std::function<void()>, int32_t>> tasks;
        std::mutex tasksMutex;
        std::condition_variable condition;
        bool terminated = false;
        int32_t weight = 0;
        std::mutex weightMutex;

        void run();
};
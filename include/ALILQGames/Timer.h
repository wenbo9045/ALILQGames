#pragma once
#include <iostream>
#include <memory>
#include <chrono>

/*
    Timer class for checking the performance of a function in us, ms
    Usage:
    {
        Timer timer;
        Foo();
    }
*/

class Timer
{
    // Scope based class
    public:
        // When object gets created, start the timer
        Timer()
        {
            m_StartTimepoint = std::chrono::high_resolution_clock::now();
        }

        // When object gets destroyed, stop the timer
        ~Timer()
        {
            Stop();
        }

        void Stop()
        {
            auto endTimepoint = std::chrono::high_resolution_clock::now();

            auto start = std::chrono::time_point_cast<std::chrono::microseconds>(m_StartTimepoint).time_since_epoch().count();
            auto end = std::chrono::time_point_cast<std::chrono::microseconds>(endTimepoint).time_since_epoch().count();

            auto duration = end - start;
            double ms = duration * 0.001;

            std::cout << duration << "us (" << ms << "ms)\n";

        }
    private:
        std::chrono::time_point< std::chrono::high_resolution_clock> m_StartTimepoint;
};
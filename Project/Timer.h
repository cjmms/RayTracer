#pragma once

// Try to use this to profile intersection detection
// https://stackoverflow.com/questions/728068/how-to-calculate-a-time-difference-in-c

#include <iostream>
#include <chrono>

class Timer
{
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    inline double elapsed() const {
        return std::chrono::duration_cast<milli_>
            (clock_::now() - beg_).count();
    }

private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::milli> milli_;
    std::chrono::time_point<clock_> beg_;
};
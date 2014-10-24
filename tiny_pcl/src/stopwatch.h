#ifndef STOPWATCH_H
#define STOPWATCH_H

#include <vector>

//! @class StopWatch
//! @brief Helper class for measuring execution times more or less accurate
class StopWatch
{
public:
    StopWatch(bool start = false);
    void start();
    double pause();
    void reset();
    void resetStart();
    double getTimeSum();
    double now();

private:
    bool running_;
    double starttime_;
    double time_sum_;
};

#endif // STOPWATCH_H

#include "stopwatch.h"
#include <boost/date_time/posix_time/posix_time.hpp>

StopWatch::StopWatch(bool start)
{
    reset();
    if( start )
        this->start();
}

//-----------------------------------------------------------------------------
double StopWatch::now()
{
    return boost::posix_time::microsec_clock::local_time().time_of_day().total_milliseconds();
}

//-----------------------------------------------------------------------------
void StopWatch::start()
{
    if(running_)
        throw std::string("start called twice");
    starttime_ = now();
    running_ = true;
}

//-----------------------------------------------------------------------------
double StopWatch::pause()
{
    running_ = false;
    time_sum_ += now() - starttime_;
    return time_sum_;
}

//-----------------------------------------------------------------------------
void StopWatch::reset()
{
    starttime_ = 0;
    time_sum_ = 0;
    running_ = false;
}

//-----------------------------------------------------------------------------
double StopWatch::getTimeSum()
{
    if(running_)
        return time_sum_ + now() - starttime_;
    else
        return time_sum_;
}

//-----------------------------------------------------------------------------
void StopWatch::resetStart()
{
    reset();
    start();
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

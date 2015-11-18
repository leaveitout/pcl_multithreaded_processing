//
// Created by sean on 18/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_TIMER_HPP
#define PCL_MULTI_THREADED_PROCESSING_TIMER_HPP

#include <string>
#include <pcl/common/time.h>

#include "Logger.hpp"

class Timer {
private:
    double last_;
    std::string description_;
    int count_;
    size_t interval_millis_;


public:
    Timer(const std::string& description, size_t interval_millis = 1000) :
            description_ (description),
            interval_millis_ (interval_millis) {
        last_ = pcl::getTime();
    }

    void reset() {
        last_ = pcl::getTime();
    }

    bool time() {
        double now = pcl::getTime();
        ++count_;
        if (now - last_ > static_cast<double>(interval_millis_)/1000) {
            std::stringstream ss;
            ss << "Average framerate (" << description_ << "): " << double(count_)/(now - last_) << " Hz." << std::endl;
            Logger::log(Logger::INFO, std::move(ss.str()));
            count_ = 0;
            last_ = now;
            return true;
        }
        return false;
    }
};

#endif //PCL_MULTI_THREADED_PROCESSING_TIMER_HPP

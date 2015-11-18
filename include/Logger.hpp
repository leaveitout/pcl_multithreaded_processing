//
// Created by sean on 17/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_LOGGER_HPP
#define PCL_MULTI_THREADED_PROCESSING_LOGGER_HPP

#include <string>
#include <pcl/console/print.h>

using namespace pcl::console;

namespace Logger {

    enum Level
    {
        ALWAYS = L_ALWAYS,
        ERROR = L_ERROR,
        WARN = L_WARN,
        INFO = L_INFO,
        DEBUG = L_DEBUG,
        VERBOSE = L_VERBOSE
    };

    void log(Logger::Level level, const char *format, ...);

    inline void log(Logger::Level level, const std::string& str) {
        log(level, str.c_str());
    }

    void log(const char* format, ...);

    inline void log(const std::string& str) {
        log(str.c_str());
    }
}

#endif //PCL_MULTI_THREADED_PROCESSING_LOGGER_HPP

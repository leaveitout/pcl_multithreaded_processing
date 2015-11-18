//
// Created by sean on 17/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_APP_HPP
#define PCL_MULTI_THREADED_PROCESSING_APP_HPP

class App {
private:
    void parseArguments();

    void printHelp();

public:
    App(int argc, char **argv);

    int exec();
};

#endif //PCL_MULTI_THREADED_PROCESSING_APP_HPP

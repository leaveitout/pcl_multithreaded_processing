//
// Created by sean on 18/11/15.
//

#include "CloudProcessor.hpp"

#ifndef PCL_MULTI_THREADED_PROCESSING_CLOUDRECORDER_HPP
#define PCL_MULTI_THREADED_PROCESSING_CLOUDRECORDER_HPP

#endif //PCL_MULTI_THREADED_PROCESSING_CLOUDRECORDER_HPP

template <typename PointType>
class CloudRecorder : public CloudProcessor<PointType> {

    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

public:
    CloudRecorder(std::shared_ptr<Buffer<CloudConstPtr>> cloud_buffer, size_t id) : CloudProcessor<PointType>(cloud_buffer, id) {}

protected:
    virtual void processCloud(CloudConstPtr cloud) override {
//        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::stringstream ss;
        ss << cloud->header;
        Logger::log(Logger::INFO, ss.str());
    }
};

//
// Created by sean on 18/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_CLOUDPROCESSOR_HPP
#define PCL_MULTI_THREADED_PROCESSING_CLOUDPROCESSOR_HPP

#include <pcl/point_cloud.h>
#include <thread>

#include "Buffer.hpp"
#include "Timer.hpp"

template <typename PointType>
class CloudProcessor {
public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

protected:
    std::shared_ptr<Buffer<CloudConstPtr>> cloud_buffer_;

private:
    size_t  id_;

    std::shared_ptr<Timer> processing_timer_;

    std::shared_ptr<std::thread> thread_;
    std::mutex mutex_;
    bool is_done_;

    // This is the method that should be overridden for different processes
    void processAllClouds() {
        initializeThreadResources();
        mutex_.lock();
        while(!is_done_) {
            mutex_.unlock();
            getCloudAndProcess();
            mutex_.lock();
        }
        mutex_.unlock();

        Logger::log(Logger::INFO, "Processing remaining %ld clouds in the buffer...\n", cloud_buffer_->getSize());

        while(!cloud_buffer_->isEmpty())
            getCloudAndProcess();
    }

    void getCloudAndProcess() {
        CloudConstPtr cloud = cloud_buffer_->getFront();
        if(cloud) {
            processCloud(cloud);
            if (processing_timer_->time())
                Logger::log(Logger::INFO, "Processing Cloud Buffer size: %zd.\n", cloud_buffer_->getSize());
        }
    }

protected:
    virtual void processCloud(CloudConstPtr cloud) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    virtual void initializeThreadResources() { }

public:
    CloudProcessor(std::shared_ptr<Buffer<CloudConstPtr>> cloud_buffer, size_t id) :
            cloud_buffer_ (cloud_buffer),
            id_ (id),
            is_done_ (false) { }


    void start() {
        is_done_ = false;
        std::stringstream ss;
        ss << "Device " << id_ << ", write callback";
        processing_timer_.reset (new Timer(ss.str()));
        thread_.reset (new std::thread(&CloudProcessor::processAllClouds, this));
    }

    void stop () {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            is_done_ = true;
        }
//        thread_->join();
        Logger::log(Logger::INFO, "Consumer done.\n");
    }

    ~CloudProcessor() {
        thread_->join();
    }


};
#endif //PCL_MULTI_THREADED_PROCESSING_CLOUDPROCESSOR_HPP

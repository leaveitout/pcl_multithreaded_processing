//
// Created by sean on 25/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_MULTITIMEDCLOUDPRODUCER_HPP
#define PCL_MULTI_THREADED_PROCESSING_MULTITIMEDCLOUDPRODUCER_HPP

#include <pcl/io/grabber.h>
#include <memory>
#include <mutex>
#include "MultiTimedCloud.hpp"
#include "Buffer.hpp"
#include "Timer.hpp"

template <typename PointType>
class MultiTimedCloudProducer {
    typedef std::shared_ptr<MultiTimedCloud<PointType>> MultiTimedCloudPtr;

    // TODO: Get this FPS using grabber.getFrameRate()
    static constexpr double FPS = 30;
    static constexpr int MILLISEC_PER_SEC = 1000;
    static constexpr double ACCEPTABLE_RATIO = 0.5;
    static constexpr size_t TIMEOUT = static_cast<size_t>((1/FPS) * MILLISEC_PER_SEC * ACCEPTABLE_RATIO);

public:
    MultiTimedCloudProducer(std::vector<std::shared_ptr<pcl::Grabber>> grabbers, size_t id) :
            grabbers_ (grabbers),
            id_ (id) {
        auto ms = std::chrono::duration_cast< std::chrono::milliseconds >(
                std::chrono::system_clock::now().time_since_epoch() );
        first_set_stamp = (size_t) ms.count();
//        Logger::log(Logger::INFO, "Timeout: %d", TIMEOUT);
        buffer_ = std::make_shared<Buffer<MultiTimedCloudPtr>>(500);
        cloud_connections_ = std::make_shared<std::vector<boost::signals2::connection>>(grabbers_.size());
        cloud_timers_ = std::make_shared<std::vector<std::shared_ptr<Timer>>>(grabbers_.size());

        for(size_t idx = 0; idx < grabbers_.size(); ++idx) {
            if(grabbers_.at(idx)->providesCallback<void (const typename pcl::PointCloud<PointType>::ConstPtr&)>()) {
                boost::function<void (const typename pcl::PointCloud<PointType>::ConstPtr&)> cloud_callback =
                        boost::bind(&MultiTimedCloudProducer::cloudCallback, this, _1, idx);
                cloud_connections_->at(idx) = grabbers_.at(idx)->registerCallback (cloud_callback);
            }
            else
                Logger::log(Logger::ERROR, "Cloud callback not provided by grabber %d\n", id);

        }
        temp_buffer_element_ = std::make_shared<MultiTimedCloud<PointType>>(grabbers_.size());
    }

    void start() {
        std::vector<std::thread> threads(grabbers_.size());
//        auto start_time_point = std::chrono::system_clock::now() + std::chrono::seconds(2);
        for(size_t idx = 0; idx < grabbers_.size(); ++idx)
            threads.at(idx) = std::thread(&MultiTimedCloudProducer<PointType>::threadedStart, this, idx);

        for(auto& thread : threads)
            thread.join();

        for(size_t idx = 0; idx < grabbers_.size(); ++idx) {
            std::stringstream ss;
            ss << "MultiGrabber id: " << id_ << ", Camera : " << idx << "Cloud Callback";
            cloud_timers_->at(idx) = std::make_shared<Timer>(ss.str());
        }

    }

    void threadedStart(size_t idx) {//, std::chrono::system_clock::time_point start_time_point) {
//        std::stringstream ss;
//        ss << "MultiGrabber id: " << id_ << ", Camera : " << idx;
//        cloud_timers_->at(idx) = std::make_shared<Timer>(ss.str() + " Cloud Callback");
//        std::this_thread::sleep_until(start_time_point);
        grabbers_.at(idx)->start();
        auto ms = std::chrono::duration_cast< std::chrono::milliseconds >(
                std::chrono::system_clock::now().time_since_epoch() );
        size_t start_time = (size_t) ms.count();
        std::stringstream ss1;
        ss1 << "Grabber " << idx << ", started at " << start_time << "." << std::endl;
        Logger::log(Logger::INFO, ss1.str());
    }

    void stop() {
        for(size_t idx = 0; idx < grabbers_.size(); ++idx)
            cloud_connections_->at(idx).disconnect();

        for(size_t idx = 0; idx < grabbers_.size(); ++idx) {
            grabbers_.at(idx)->stop();
            std::stringstream ss;
            ss << "MultiGrabber id: " << id_ << ", Camera : " << idx << ", Producer done." << std::endl;
            Logger::log(Logger::INFO, ss.str());
        }
    }

    std::shared_ptr<Buffer<MultiTimedCloudPtr>> getBuffer() const {
        return buffer_;
    }

private:
    void cloudCallback (const typename pcl::PointCloud<PointType>::ConstPtr& cloud, size_t index) {
        auto ms = std::chrono::duration_cast< std::chrono::milliseconds >(
                std::chrono::system_clock::now().time_since_epoch() );
        size_t timestamp = (size_t) ms.count();

        auto timed_cloud = std::make_shared<TimedCloud<PointType>>(cloud, timestamp);

        bool overwrite = false;
        bool pushed = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!temp_buffer_element_->isTimedCloudSetAt(index)) {
                if(isBufferElementFullyEmpty()) {
                    temp_buffer_element_->setTimedCloudAt(timed_cloud, index);
                    first_set_stamp = timestamp;
                }
                else {
                    if (isItTimeToPushElement(timestamp)) {
                        Logger::log(Logger::WARN, "Pushed, timeout!!\n");
                        temp_buffer_element_->printTimings();
                        overwrite = buffer_->pushBack(temp_buffer_element_);
                        pushed = true;
                        temp_buffer_element_ = std::make_shared<MultiTimedCloud<PointType>>(grabbers_.size());
                        temp_buffer_element_->setTimedCloudAt(timed_cloud, index);
                        first_set_stamp = timestamp;
                    }
                    else {
                        temp_buffer_element_->setTimedCloudAt(timed_cloud, index);
                        if (isBufferElementFullySet()) {
                            Logger::log(Logger::WARN, "Pushed, fully set!!\n");
                            temp_buffer_element_->printTimings();
                            overwrite = buffer_->pushBack(temp_buffer_element_);
                            pushed = true;
                            temp_buffer_element_ = std::make_shared<MultiTimedCloud<PointType>>(grabbers_.size());
                        }
                    }
                }
            }
            else {
                Logger::log(Logger::WARN, "Pushed, already set!!\n");
                temp_buffer_element_->printTimings();
                overwrite = buffer_->pushBack(temp_buffer_element_);
                pushed = true;
                temp_buffer_element_ = std::make_shared<MultiTimedCloud<PointType>>(grabbers_.size());
                temp_buffer_element_->setTimedCloudAt(timed_cloud, index);
                first_set_stamp = timestamp;
            }
        }

        if(pushed) {
            if(overwrite)
                Logger::log(Logger::WARN, "Warning! CloudAndImage Buffer was full, overwriting data!\n");
        }

        if(cloud_timers_->at(index)->time())
            Logger::log(Logger::INFO, "MultiCloudImage Buffer size: %zd.\n", buffer_->getSize());
    }

    bool isBufferElementFullySet() {
        for(size_t idx = 0; idx < grabbers_.size(); ++idx) {
            if(!temp_buffer_element_->isTimedCloudSetAt(idx))
                return false;
        }
        // TODO: Need to check the timings first_set_stamp
        return true;
    }

    bool isBufferElementFullyEmpty() {
        for(size_t idx = 0; idx < grabbers_.size(); ++idx) {
            if(temp_buffer_element_->isTimedCloudSetAt(idx))
                return false;
        }
        return true;

    }

    bool isItTimeToPushElement(size_t latest_stamp) {
        // TODO: Calculate the timeout using the frame rates of the grabbers
        if( latest_stamp - first_set_stamp > TIMEOUT ) {
            Logger::log(Logger::WARN, "Reached timeout!\n");
            return true;
        }
        else
            return false;
    }

    size_t calculateVariance() {

    }

    std::vector<std::shared_ptr<pcl::Grabber>> grabbers_;
    std::shared_ptr<Buffer<MultiTimedCloudPtr>> buffer_;
    size_t id_;
    std::shared_ptr<std::vector<boost::signals2::connection>> cloud_connections_;
    std::shared_ptr<std::vector<std::shared_ptr<Timer>>> cloud_timers_;
    MultiTimedCloudPtr temp_buffer_element_;
    std::mutex mutex_;
    size_t first_set_stamp;

};
#endif //PCL_MULTI_THREADED_PROCESSING_MULTITIMEDCLOUDPRODUCER_HPP

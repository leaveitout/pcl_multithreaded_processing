//
// Created by sean on 25/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_MULTICLOUDPRODUCER_HPP
#define PCL_MULTI_THREADED_PROCESSING_MULTICLOUDPRODUCER_HPP

#include <pcl/io/grabber.h>
#include <memory>
#include <mutex>
#include "MultiCloud.hpp"
#include "Buffer.hpp"
#include "Timer.hpp"

template <typename PointType>
class MultiCloudProducer {
    typedef std::shared_ptr<MultiCloud<PointType>> MultiCloudPtr;

    static constexpr double FPS = 30;
    static constexpr int MICROSEC_PER_SEC = 1000000;
    static constexpr double ACCEPTABLE_RATIO = 0.5;
    static constexpr size_t TIMEOUT = static_cast<size_t>((1/FPS) * MICROSEC_PER_SEC * ACCEPTABLE_RATIO);

public:
    MultiCloudProducer(std::vector<std::shared_ptr<pcl::Grabber>> grabbers, size_t id) :
            grabbers_ (grabbers),
            id_ (id) {

        first_set_stamp = 0;
        buffer_ = std::make_shared<Buffer<MultiCloudPtr>>();
        cloud_connections_ = std::make_shared<std::vector<boost::signals2::connection>>(grabbers_.size());
        cloud_timers_ = std::make_shared<std::vector<std::shared_ptr<Timer>>>(grabbers_.size());

        for(size_t idx = 0; idx < grabbers_.size(); ++idx) {
            if(grabbers_.at(idx)->providesCallback<void (const typename pcl::PointCloud<PointType>::ConstPtr&)>()) {
                boost::function<void (const typename pcl::PointCloud<PointType>::ConstPtr&)> cloud_callback =
                        boost::bind(&MultiCloudProducer::cloudCallback, this, _1, idx);
                cloud_connections_->at(idx) = grabbers_.at(idx)->registerCallback (cloud_callback);
            }
            else
                Logger::log(Logger::ERROR, "Cloud callback not provided by grabber %d\n", id);

        }
        temp_buffer_element_ = std::make_shared<MultiCloud<PointType>>(grabbers_.size());
    }

    void start() {
        for(size_t idx = 0; idx < grabbers_.size(); ++idx) {
            std::stringstream ss;
            ss << "MultiGrabber id: " << id_ << ", Camera : " << idx;
            cloud_timers_->at(idx) = std::make_shared<Timer>(ss.str() + " Cloud Callback");
            grabbers_.at(idx)->start();
        }
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

    std::shared_ptr<Buffer<MultiCloudPtr>> getBuffer() const {
        return buffer_;
    }

private:
    void cloudCallback (const typename pcl::PointCloud<PointType>::ConstPtr& cloud, size_t index) {

//        cloud->header.stamp = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());
//        typename pcl::PointCloud<PointType>::ConstPtr cloud2 = const_cast<pcl::PointCloud<PointType>::ConstPtr>(cloud);

        std::stringstream ss;
        ss << "Camera " << index << ", seq=" << cloud->header.seq << ", stamp=" << cloud->header.stamp << std::endl;
        Logger::log(Logger::INFO, ss.str());
        bool overwrite = false;
        bool pushed = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!temp_buffer_element_->isCloudSetAt(index)) {
                if (isItTimeToPushElement(cloud->header.stamp)) {
                    overwrite = buffer_->pushBack(temp_buffer_element_);
                    pushed = true;
                    temp_buffer_element_ = std::make_shared<MultiCloud<PointType>>(grabbers_.size());
                    temp_buffer_element_->setCloudAt(cloud, index);
                    first_set_stamp = cloud->header.stamp;
                }
                else {
                    if(isBufferElementFullyEmpty()) {
                        temp_buffer_element_->setCloudAt(cloud, index);
                        first_set_stamp = cloud->header.stamp;
                    }
                    else {
                        temp_buffer_element_->setCloudAt(cloud, index);
                        if (isBufferElementFullySet()) {
                            overwrite = buffer_->pushBack(temp_buffer_element_);
                            pushed = true;
                            temp_buffer_element_ = std::make_shared<MultiCloud<PointType>>(grabbers_.size());
                        }
                    }
                }
            }
            else {
                overwrite = buffer_->pushBack(temp_buffer_element_);
                pushed = true;
                temp_buffer_element_ = std::make_shared<MultiCloud<PointType>>(grabbers_.size());
                temp_buffer_element_->setCloudAt(cloud, index);
                first_set_stamp = cloud->header.stamp;
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
            if(!temp_buffer_element_->isCloudSetAt(idx))
                return false;
        }
        // TODO: Need to check the timings first_set_stamp
        return true;
    }

    bool isBufferElementFullyEmpty() {
        for(size_t idx = 0; idx < grabbers_.size(); ++idx) {
            if(temp_buffer_element_->isCloudSetAt(idx))
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

    }

    size_t calculateVariance() {

    }

    std::vector<std::shared_ptr<pcl::Grabber>> grabbers_;
    std::shared_ptr<Buffer<MultiCloudPtr>> buffer_;
    size_t id_;
    std::shared_ptr<std::vector<boost::signals2::connection>> cloud_connections_;
    std::shared_ptr<std::vector<std::shared_ptr<Timer>>> cloud_timers_;
    MultiCloudPtr temp_buffer_element_;
    std::mutex mutex_;
    size_t first_set_stamp;

};

#endif //PCL_MULTI_THREADED_PROCESSING_MULTICLOUDPRODUCER_HPP

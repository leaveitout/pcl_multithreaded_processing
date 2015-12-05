//
// Created by sean on 24/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_MULTICLOUDANDIMAGEPRODUCER_HPP
#define PCL_MULTI_THREADED_PROCESSING_MULTICLOUDANDIMAGEPRODUCER_HPP

#include <pcl/io/grabber.h>
#include "MultiCloudAndImage.hpp"
#include "Buffer.hpp"
#include "Timer.hpp"

template <typename PointType>
class MultiCloudAndImageProducer {
    typedef std::shared_ptr<MultiCloudAndImage<PointType>> MultiCloudAndImagePtr;

private:
    std::vector<std::shared_ptr<pcl::Grabber>> grabbers_;
    std::shared_ptr<Buffer<MultiCloudAndImagePtr>> buffer_;
    size_t id_;
    std::shared_ptr<std::vector<bool>> provides_images_;
    std::shared_ptr<std::vector<boost::signals2::connection>> cloud_connections_;
    std::shared_ptr<std::vector<boost::signals2::connection>> image_connections_;
    std::shared_ptr<std::vector<std::shared_ptr<Timer>>> cloud_timers_;
    std::shared_ptr<std::vector<std::shared_ptr<Timer>>> image_timers_;
    MultiCloudAndImagePtr temp_buffer_element_;
    std::mutex mutex_;
    size_t first_set_stamp;

    void cloudCallback (const typename pcl::PointCloud<PointType>::ConstPtr& cloud, size_t index) {
        bool overwrite = false;
        bool pushed = false;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (!temp_buffer_element_->isCloudSetAt(index)) {
                temp_buffer_element_->setCloudAt(cloud, index);
                if(isBufferElementFullyEmpty())
                    first_set_stamp = cloud->header.stamp;
                else if (isBufferElementFullySet() || isItTimeToPushElement(cloud->header.stamp)) {
                    overwrite = buffer_->pushBack(temp_buffer_element_);
                    pushed = true;
                    temp_buffer_element_ = std::make_shared<MultiCloudAndImage<PointType>>();
//                    first_set_stamp = cloud->header.stamp;
                }
            }
            else {
                overwrite = buffer_->pushBack(temp_buffer_element_);
                pushed = true;
                temp_buffer_element_ = std::make_shared<MultiCloudAndImage<PointType>>();
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
        for(size_t idx = 0; idx < grabbers_.size(); ++idx) {
            if(provides_images_->at(idx))
                if(!temp_buffer_element_->isImageSetAt(idx))
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
        for(size_t idx = 0; idx < grabbers_.size(); ++idx) {
            if(provides_images_->at(idx))
                if(temp_buffer_element_->isImageSetAt(idx))
                    return false;
        }
        // TODO: Need to check the timings first_set_stamp
        return true;

    }

    bool isItTimeToPushElement(size_t latest_stamp) {
        // TODO: Calculate the timeout using the frame rates of the grabbers
        size_t timeout = static_cast<size_t>((static_cast<double>(1)/30) * 1000000 * 0.5);
        return latest_stamp - first_set_stamp > timeout;
    }

    void imageCallback(const pcl::io::Image::Ptr& image, size_t index) {

        bool overwrite = false;
        bool pushed = false;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (!temp_buffer_element_->isImageSetAt(index)) {
                temp_buffer_element_->setImageAt(image, index);
                if (isBufferElementFullySet()) {
                    overwrite = buffer_->pushBack(temp_buffer_element_);
                    pushed = true;
                    temp_buffer_element_ = std::make_shared<MultiCloudAndImage<PointType>>();
                }
            }
            else {
                overwrite = buffer_->pushBack(temp_buffer_element_);
                pushed = true;
                temp_buffer_element_ = std::make_shared<MultiCloudAndImage<PointType>>();
                temp_buffer_element_->setImageAt(image, index);
            }
        }

        if(pushed) {
            if(overwrite)
                Logger::log(Logger::WARN, "Warning! CloudAndImage Buffer was full, overwriting data!\n");
        }

        if(image_timers_->at(index)->time())
            Logger::log(Logger::INFO, "MultiCloudImage Buffer size: %zd.\n", buffer_->getSize());
    }

public:
    MultiCloudAndImageProducer(std::vector<std::shared_ptr<pcl::Grabber>> grabbers, size_t id) :
            grabbers_ (grabbers),
            id_ (id) {

        provides_images_ = std::make_shared<std::vector<bool>>(grabbers_.size());
        cloud_connections_ = std::make_shared<std::vector<boost::signals2::connection>>(grabbers_.size());
        image_connections_ = std::make_shared<std::vector<boost::signals2::connection>>(grabbers_.size());
        cloud_timers_ = std::make_shared<std::vector<std::shared_ptr<Timer>>>(grabbers_.size());
        image_timers_ = std::make_shared<std::vector<std::shared_ptr<Timer>>>(grabbers_.size());


        for(size_t idx = 0; idx < grabbers_.size(); ++idx) {
            if(grabbers_.at(idx)->providesCallback<void (const typename pcl::PointCloud<PointType>::ConstPtr&)>()) {
                boost::function<void (const typename pcl::PointCloud<PointType>::ConstPtr&)> cloud_callback =
                        boost::bind(&MultiCloudAndImageProducer::cloudCallback, this, _1, idx);
                cloud_connections_->at(idx) = grabbers_.at(idx)->registerCallback (cloud_callback);
            }
            else
                Logger::log(Logger::ERROR, "Cloud callback not provided by grabber %d\n", id);

            if(grabbers_.at(idx)->providesCallback<void (const pcl::io::Image::Ptr&)>()) {
                boost::function<void(const pcl::io::Image::Ptr &)> image_callback =
                        boost::bind(&MultiCloudAndImageProducer::imageCallback, this, _1, idx);
                image_connections_->at(idx) = grabbers_.at(idx)->registerCallback(image_callback);
                provides_images_->at(idx) = true;
            }
            else
                Logger::log(Logger::ERROR, "Image callback not provided by grabber %d\n", id);
        }
        temp_buffer_element_ = std::make_shared<MultiCloudAndImage<PointType>>();
    }

    void start() {
        for(size_t idx = 0; idx < grabbers_.size(); ++idx) {
            std::stringstream ss;
            ss << "MultiGrabber id: " << id_ << ", Camera : " << idx;
            cloud_timers_->at(idx).reset(new Timer(ss.str() + "Cloud Callback"));
            if (provides_images_)
                image_timers_->at(idx).reset(new Timer(ss.str() + "Image Callback"));
            grabbers_.at(idx)->start();
        }
    }

    void stop() {
        for(size_t idx = 0; idx < grabbers_.size(); ++idx) {
            grabbers_.at(idx)->stop();
            cloud_connections_->at(idx).disconnect();
            if (provides_images_->at(idx))
                image_connections_->at(idx).disconnect();
            std::stringstream ss;
            ss << "MultiGrabber id: " << id_ << ", Camera : " << idx << ", Producer done." << std::endl;
            Logger::log(Logger::INFO, ss.str());
        }
    }

    std::shared_ptr<Buffer<MultiCloudAndImagePtr>> getBuffer() const {
        return buffer_;
    }


};

#endif //PCL_MULTI_THREADED_PROCESSING_MULTICLOUDANDIMAGEPRODUCER_HPP

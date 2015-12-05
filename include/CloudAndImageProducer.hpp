//
// Created by sean on 23/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_CLOUDANDIMAGEPRODUCER_HPP
#define PCL_MULTI_THREADED_PROCESSING_CLOUDANDIMAGEPRODUCER_HPP

#include <pcl/io/grabber.h>
#include <pcl/io/image.h>
#include "Buffer.hpp"
#include "CloudAndImage.hpp"
#include "Logger.hpp"
#include "Timer.hpp"

template <typename PointType>
class CloudAndImageProducer {
    typedef std::shared_ptr<CloudAndImage<PointType>> CloudAndImagePtr;

private:
    std::shared_ptr<pcl::Grabber> grabber_;
    std::shared_ptr<Buffer<CloudAndImagePtr>> buffer_;
    size_t id_;
    bool provides_images_;
    boost::signals2::connection cloud_connection_;
    boost::signals2::connection image_connection_;
    std::shared_ptr<Timer> cloud_timer_;
    std::shared_ptr<Timer> image_timer_;
    CloudAndImagePtr temp_buffer_element_;
    std::mutex mutex_;

    void cloudCallback (const typename pcl::PointCloud<PointType>::ConstPtr& cloud) {
        bool overwrite = false;
        bool pushed = false;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (!temp_buffer_element_->isCloudSet()) {
                temp_buffer_element_->setCloud(cloud);
                if (temp_buffer_element_->isImageSet() || !provides_images_) {
                    overwrite = buffer_->pushBack(temp_buffer_element_);
                    pushed = true;
                    temp_buffer_element_ = std::make_shared<CloudAndImage<PointType>>();
                }
            }
            else {
                overwrite = buffer_->pushBack(temp_buffer_element_);
                pushed = true;
                temp_buffer_element_ = std::make_shared<CloudAndImage<PointType>>();
                temp_buffer_element_->setCloud(cloud);
            }
        }

        if(pushed) {
            if(overwrite)
                Logger::log(Logger::WARN, "Warning! CloudAndImage Buffer was full, overwriting data!\n");
        }

        if(cloud_timer_->time())
            Logger::log(Logger::INFO, "CloudImage Buffer size: %zd.\n", buffer_->getSize());
    }

    void imageCallback(const pcl::io::Image::Ptr& image) {
        bool overwrite = false;
        bool pushed = false;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (!temp_buffer_element_->isImageSet()) {
                temp_buffer_element_->setImage(image);
                if (temp_buffer_element_->isCloudSet()) {
                    overwrite = buffer_->pushBack(temp_buffer_element_);
                    pushed = true;
                    temp_buffer_element_ = std::make_shared<CloudAndImage<PointType>>();
                }
            }
            else {
                overwrite = buffer_->pushBack(temp_buffer_element_);
                pushed = true;
                temp_buffer_element_ = std::make_shared<CloudAndImage<PointType>>();
                temp_buffer_element_->setImage(image);
            }
        }

        if(pushed) {
            if(overwrite)
                Logger::log(Logger::WARN, "Warning! CloudAndImage Buffer was full, overwriting data!\n");
        }

        if(image_timer_->time())
            Logger::log(Logger::INFO, "CloudImage Buffer size: %zd.\n", buffer_->getSize());
    }

public:
    CloudAndImageProducer(std::shared_ptr<pcl::Grabber> grabber, size_t id) : grabber_ (grabber), id_ (id) {
        buffer_ = std::make_shared<Buffer<CloudAndImagePtr>>();
        if(grabber_->providesCallback<void (const typename pcl::PointCloud<PointType>::ConstPtr&)>()) {
            boost::function<void (const typename pcl::PointCloud<PointType>::ConstPtr&)> cloud_callback =
                     boost::bind(&CloudAndImageProducer::cloudCallback, this, _1);
            cloud_connection_ = grabber_->registerCallback (cloud_callback);
        }
        else
            Logger::log(Logger::ERROR, "Cloud callback not provided by grabber %d\n", id);

        if(grabber_->providesCallback<void (const pcl::io::Image::Ptr&)>()) {
            boost::function<void(const pcl::io::Image::Ptr &)> image_callback =
                    boost::bind(&CloudAndImageProducer::imageCallback, this, _1);
            image_connection_ = grabber_->registerCallback(image_callback);
            provides_images_ = true;
        }
        else
            Logger::log(Logger::ERROR, "Image callback not provided by grabber %d\n", id);

        temp_buffer_element_ = std::make_shared<CloudAndImage<PointType>>();
    }

    void start() {
        std::stringstream ss;
        ss << "Device " << id_ << ", ";
        cloud_timer_.reset(new Timer(ss.str() + "Cloud Callback"));
        if(provides_images_)
            image_timer_.reset(new Timer(ss.str() + "Image Callback"));
        grabber_->start();
    }

    void stop() {
        grabber_->stop();
        cloud_connection_.disconnect();
        if(provides_images_)
            image_connection_.disconnect();
        Logger::log(Logger::INFO, "Producer done.\n");
    }

    std::shared_ptr<Buffer<CloudAndImagePtr>> getBuffer() const {
        return buffer_;
    }

};
#endif //PCL_MULTI_THREADED_PROCESSING_CLOUDANDIMAGEPRODUCER_HPP

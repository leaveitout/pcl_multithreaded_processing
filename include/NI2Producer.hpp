//
// Created by sean on 18/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_NI2PRODUCER_HPP
#define PCL_MULTI_THREADED_PROCESSING_NI2PRODUCER_HPP

#include <pcl/io/openni2_grabber.h>
#include "Buffer.hpp"
#include "Logger.hpp"
#include "Timer.hpp"


template <typename PointType>
class NI2Producer {
private:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    typedef pcl::io::openni2::Image NI2Image;
    typedef NI2Image::Ptr NI2ImagePtr;
    typedef pcl::io::OpenNI2Grabber NI2Grabber;

    NI2Grabber::Ptr grabber_;
    std::shared_ptr<Buffer<CloudConstPtr>> cloud_buffer_;
    std::shared_ptr<Buffer<NI2ImagePtr>> image_buffer_;
    size_t id_;
    bool provides_images_;
    boost::signals2::connection cloud_connection_;
    boost::signals2::connection image_connection_;
    std::shared_ptr<Timer> cloud_timer_;
    std::shared_ptr<Timer> image_timer_;

    void cloudCallback (const CloudConstPtr& cloud) {
        // TODO: Remove next line once checked
//        std::stringstream ss;
//        ss << "Producer " << id_ << " operating on thread " << boost::this_thread::get_id() << std::endl;
//        Logger::log(Logger::DEBUG, ss.str());

        if(cloud_buffer_->pushBack(cloud))
            Logger::log(Logger::WARN, "Warning! Cloud Buffer was full, overwriting data!\n");

        if(cloud_timer_->time())
            Logger::log(Logger::INFO, "Cloud Buffer size: %zd.\n", cloud_buffer_->getSize());
    }

    void imageCallback(const NI2ImagePtr& image) {
        if(image_buffer_->pushBack(image))
            Logger::log(Logger::WARN, "Warning! Image Buffer was full, overwriting data!\n");

        if(image_timer_->time())
            Logger::log(Logger::INFO, "Image Buffer size: %zd.\n", cloud_buffer_->getSize());
    }

public:
    NI2Producer(NI2Grabber::Ptr grabber, size_t id, bool provide_images = false) :
            grabber_ (grabber),
            id_ (id),
            provides_images_ (provide_images) {
        cloud_buffer_ = std::make_shared<Buffer<CloudConstPtr>>();
        boost::function<void (const CloudConstPtr&)> cloud_callback = boost::bind(&NI2Producer::cloudCallback, this, _1);
        cloud_connection_ = grabber_->registerCallback (cloud_callback);

        if(provides_images_ && grabber_->providesCallback<void (const NI2ImagePtr&)>()) {
            image_buffer_ = std::make_shared<Buffer<NI2ImagePtr>>();
            boost::function<void(const NI2ImagePtr &)> image_callback = boost::bind(&NI2Producer::imageCallback, this, _1);
            image_connection_ = grabber_->registerCallback(image_callback);
        }
        else
            image_buffer_ = nullptr;
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

    std::shared_ptr<Buffer<CloudConstPtr>> getCloudBuffer() const {
        return cloud_buffer_;
    }

    std::shared_ptr<Buffer<NI2ImagePtr>> getImageBuffer() const {
        return image_buffer_;
    }
};
#endif //PCL_MULTI_THREADED_PROCESSING_NI2PRODUCER_HPP

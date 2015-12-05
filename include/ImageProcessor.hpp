//
// Created by sean on 18/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_IMAGEPROCESSOR_HPP
#define PCL_MULTI_THREADED_PROCESSING_IMAGEPROCESSOR_HPP


#include <memory>
#include <mutex>
#include <thread>
#include <pcl/io/openni2/openni2_device.h>

#include "Buffer.hpp"
#include "Timer.hpp"

class ImageProcessor {

    typedef pcl::io::openni2::Image NI2Image;
    typedef NI2Image::Ptr NI2ImagePtr;

public:
    ImageProcessor(std::shared_ptr<Buffer<NI2ImagePtr>> image_buffer, size_t id) :
            image_buffer_ (image_buffer),
            id_ (id),
            is_done_ (false) { }

    void start();

    void stop();

    ~ImageProcessor();

protected:
    virtual void processImage(NI2ImagePtr image) {}

    virtual void initializeThreadResources()  {}

private:
    void processAllImages();

    void getImageAndProcess();


protected:
    std::shared_ptr<Buffer<NI2ImagePtr>> image_buffer_;

private:
    size_t id_;

    std::shared_ptr<Timer> processing_timer_;

    std::shared_ptr<std::thread> thread_;
    std::mutex mutex_;
    bool is_done_;


};


#endif //PCL_MULTI_THREADED_PROCESSING_IMAGEPROCESSOR_HPP

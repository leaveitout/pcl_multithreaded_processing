//
// Created by sean on 19/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_NI2IMAGEVIEWER_HPP
#define PCL_MULTI_THREADED_PROCESSING_NI2IMAGEVIEWER_HPP


#include <pcl/io/openni2/openni2_device.h>
#include <pcl/visualization/image_viewer.h>

#include "Processor.hpp"

typedef pcl::io::openni2::Image NI2Image;
typedef NI2Image::Ptr NI2ImagePtr;


class NI2ImageViewer : public Processor<NI2ImagePtr> {

public:
    NI2ImageViewer(const std::shared_ptr<Buffer<NI2ImagePtr>> buffer, size_t id, std::string description);

protected:
    virtual void initializeThreadResources() override;

    virtual void processBufferElement(NI2ImagePtr element) override;

    void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *);

    void mouseCallback(const pcl::visualization::MouseEvent &mouse_event, void *);

private:
    pcl::visualization::ImageViewer::Ptr image_viewer_;
};



#endif //PCL_MULTI_THREADED_PROCESSING_NI2IMAGEVIEWER_HPP

//
// Created by sean on 20/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_CLOUDIMAGEVIEWER_HPP
#define PCL_MULTI_THREADED_PROCESSING_CLOUDIMAGEVIEWER_HPP


#include <memory>
#include <pcl/visualization/image_viewer.h>
#include "Processor.hpp"
#include "CloudAndImage.hpp"

template <typename PointType>
class CloudImageViewer : public Processor<std::shared_ptr<CloudAndImage<PointType>>> {
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    typedef typename pcl::io::Image Image;
    typedef typename Image::Ptr ImagePtr;
    typedef typename std::shared_ptr<CloudAndImage<PointType>> CloudAndImagePtr;

private:
    pcl::visualization::ImageViewer::Ptr image_viewer_;
    pcl::visualization::PCLVisualizer::Ptr cloud_viewer_;

public:
    CloudImageViewer(const std::shared_ptr<Buffer<CloudAndImagePtr>> buffer, size_t id, std::string description) :
            Processor<CloudAndImagePtr>(buffer, id, description) {
    }

protected:
    virtual void initializeThreadResources() override {
        std::stringstream ss1;
        ss1 << "Cloud Viewer : " << this->getDescription();
        cloud_viewer_.reset(new pcl::visualization::PCLVisualizer(ss1.str()));
        cloud_viewer_->registerMouseCallback(&CloudImageViewer::mouseCallback, *this);
        cloud_viewer_->registerKeyboardCallback(&CloudImageViewer::keyboardCallback, *this);
        std::stringstream ss2;
        ss2 << "Image Viewer : " << this->getDescription();
        image_viewer_.reset(new pcl::visualization::ImageViewer(ss2.str()));
        image_viewer_->registerMouseCallback(&CloudImageViewer::mouseCallback, *this);
        image_viewer_->registerKeyboardCallback(&CloudImageViewer::keyboardCallback, *this);
    }


    virtual void processBufferElement(CloudAndImagePtr element) override {
        cloud_viewer_->spinOnce();

        if(element->getCloud())
            if (!cloud_viewer_->updatePointCloud(element->getCloud(), "OpenNICloud")) {
                cloud_viewer_->setPosition(0, 0);
                cloud_viewer_->setSize(element->getCloud()->width, element->getCloud()->height);
                cloud_viewer_->addPointCloud(element->getCloud(), "OpenNICloud");
                cloud_viewer_->resetCameraViewpoint("OpenNICloud");
                cloud_viewer_->setCameraPosition(
                        0, 0, 0,        // Position
                        0, 0, 1,        // Viewpoint
                        0, -1, 0);    // Up
            }

        if(element->getImage())
            if(element->getImage()->getEncoding() == pcl::io::openni2::Image::RGB)
                image_viewer_->addRGBImage((const unsigned char*) element->getImage()->getData(),
                                           element->getImage()->getWidth(),
                                           element->getImage()->getHeight());
        image_viewer_->spinOnce();
    }

    void mouseCallback(const pcl::visualization::MouseEvent &mouse_event, void *) {
        if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress &&
            mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton) {
            Logger::log(Logger::INFO, "Left button pressed @ (%i, %i).\n", mouse_event.getX(), mouse_event.getY());
        }
    }

    void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *) {
        std::string special_key = event.getKeyCode() ? "" : "special ";
        if (event.keyDown())
            Logger::log(Logger::INFO, "The %skey %c was pressed.", special_key.c_str(), event.getKeyCode());
        else
            Logger::log(Logger::INFO, "The %skey %c was released.", special_key.c_str(), event.getKeyCode());
    }

};
#endif //PCL_MULTI_THREADED_PROCESSING_CLOUDIMAGEVIEWER_HPP


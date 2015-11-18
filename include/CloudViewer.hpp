//
// Created by sean on 18/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_CLOUDVIEWER_HPP
#define PCL_MULTI_THREADED_PROCESSING_CLOUDVIEWER_HPP

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/openni2_grabber.h>
#include <thread>
#include "Logger.hpp"
#include "Buffer.hpp"
#include "CloudProcessor.hpp"

template <typename PointType>
class CloudViewer : public CloudProcessor<PointType> {
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    pcl::visualization::PCLVisualizer::Ptr cloud_viewer_;

    void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *) {
        std::string special_key = event.getKeyCode() ? "" : "special ";
        if (event.keyDown())
            Logger::log(Logger::INFO, "The %skey %c was pressed.", special_key.c_str(), event.getKeyCode());
        else
            Logger::log(Logger::INFO, "The %skey %c was released.", special_key.c_str(), event.getKeyCode());
    }

    void mouseCallback(const pcl::visualization::MouseEvent &mouse_event, void *) {
        if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress &&
            mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton) {
            Logger::log(Logger::INFO, "Left button pressed @ (%i, %i).\n", mouse_event.getX(), mouse_event.getY());
        }
    }

public:
    CloudViewer(std::shared_ptr<Buffer<CloudConstPtr>> cloud_buffer, size_t id) :
            CloudProcessor<PointType>(cloud_buffer, id) {
    }

protected:
    virtual void processCloud(CloudConstPtr cloud) override {
        cloud_viewer_->spinOnce();

        if (!cloud_viewer_->updatePointCloud(cloud, "OpenNICloud")) {
            cloud_viewer_->setSize(cloud->width, cloud->height);
            cloud_viewer_->setPosition(0, 0);
            cloud_viewer_->addPointCloud(cloud, "OpenNICloud");
            cloud_viewer_->resetCameraViewpoint("OpenNICloud");
            cloud_viewer_->setCameraPosition(
                    0, 0, 0,        // Position
                    0, 0, 1,        // Viewpoint
                    0, -1, 0);    // Up
        }

        // TODO; Figure out how to achieve this
//        if(cloud_viewer_->wasStopped()) {
//            this->cloud_buffer_.reset(new Buffer<CloudConstPtr>());
//            this->stop();
//            cloud_viewer_->close();
//        }
    }

    virtual void initializeThreadResources() override {
        cloud_viewer_.reset(new pcl::visualization::PCLVisualizer("PCL OpenNI2 Cloud"));
        cloud_viewer_->registerMouseCallback(&CloudViewer::mouseCallback, *this);
        cloud_viewer_->registerKeyboardCallback(&CloudViewer::keyboardCallback, *this);
    }
};

#endif //PCL_MULTI_THREADED_PROCESSING_CLOUDVIEWER_HPP

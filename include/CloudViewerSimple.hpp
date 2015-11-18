//
// Created by sean on 18/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_CLOUDVIEWERSIMPLE_HPP
#define PCL_MULTI_THREADED_PROCESSING_CLOUDVIEWERSIMPLE_HPP

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/openni2_grabber.h>

#include <thread>
#include "Logger.hpp"
#include "Buffer.hpp"

template <typename PointType>
class CloudViewerSimple {
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    std::shared_ptr<Buffer<CloudConstPtr>> cloud_buffer_;
    pcl::visualization::PCLVisualizer::Ptr cloud_viewer_;
    std::shared_ptr<std::thread> thread_;
    size_t id_;

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
    CloudViewerSimple(std::shared_ptr<Buffer<CloudConstPtr>> buffer, size_t id) :
            cloud_buffer_ (buffer),
            id_ (id) {
    }

    void start() {
        thread_.reset(new std::thread(&CloudViewerSimple::run, this));
    }

    void stop() {
        thread_->join();

    }

    void run() {
        cloud_viewer_.reset(new pcl::visualization::PCLVisualizer("PCL OpenNI2 Cloud"));
        cloud_viewer_->setPosition(0, 0);
        cloud_viewer_->setSize(640, 480);
        cloud_viewer_->registerMouseCallback(&CloudViewerSimple::mouseCallback, *this);
        cloud_viewer_->registerKeyboardCallback(&CloudViewerSimple::keyboardCallback, *this);
        while (!cloud_viewer_->wasStopped()) {
            cloud_viewer_->spinOnce();

            CloudConstPtr cloud;
            cloud = cloud_buffer_->getFront();

            if (cloud) {
                if (!cloud_viewer_->updatePointCloud(cloud, "OpenNICloud")) {
                    cloud_viewer_->addPointCloud(cloud, "OpenNICloud");
                    cloud_viewer_->resetCameraViewpoint("OpenNICloud");
                    cloud_viewer_->setCameraPosition(
                            0, 0, 0,        // Position
                            0, 0, 1,        // Viewpoint
                            0, -1, 0);    // Up
                }
            }
        }
    }

};
#endif //PCL_MULTI_THREADED_PROCESSING_CLOUDVIEWERSIMPLE_HPP

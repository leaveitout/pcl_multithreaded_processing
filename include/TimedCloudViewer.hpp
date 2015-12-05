//
// Created by sean on 04/12/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_TIMEDCLOUDVIEWER_HPP
#define PCL_MULTI_THREADED_PROCESSING_TIMEDCLOUDVIEWER_HPP

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/point_cloud_image_extractors.h>
#include "TimedCloud.hpp"
#include "Processor.hpp"

template<typename PointType>
class TimedCloudViewer : public Processor<std::shared_ptr<TimedCloud<PointType>>> {
    typedef std::shared_ptr<TimedCloud<PointType>> TimedCloudPtr;

public:
    TimedCloudViewer(const std::shared_ptr<Buffer<TimedCloudPtr>> timed_cloud_buffer,
                     size_t id,
                     std::string description) :
            Processor<TimedCloudPtr>(timed_cloud_buffer, id, description) {
    }

protected:
    virtual void initializeThreadResources() {

    }

    virtual void processBufferElement(TimedCloudPtr element) {
        std::call_once(flag, [&]() {
            cloud_viewer_ = boost::make_shared<pcl::visualization::PCLVisualizer>();
            cloud_viewer_->registerMouseCallback(&TimedCloudViewer::mouseCallback, *this);
            cloud_viewer_->registerKeyboardCallback(&TimedCloudViewer::keyboardCallback, *this);
            cloud_viewer_->setCameraFieldOfView(1.02259994f);

//            image_viewer_ = boost::make_shared<pcl::visualization::ImageViewer>();
//            image_viewer_->registerMouseCallback(&TimedCloudViewer::mouseCallback, *this);
//            image_viewer_->registerKeyboardCallback(&TimedCloudViewer::keyboardCallback, *this);
        });

        cloud_viewer_->spinOnce();
        if(element) {
            auto cloud = element->getCloud();
            if(cloud) {
                if(!cloud_viewer_->updatePointCloud(cloud, "OpenNICloud")) {
                    cloud_viewer_->setPosition(0, 0);
                    cloud_viewer_->setSize(640, 480);
                    cloud_viewer_->addPointCloud(cloud, "OpenNICloud");
                    cloud_viewer_->resetCameraViewpoint("OpenNICloud");
                    cloud_viewer_->setCameraPosition(
                            0, 0, 0,        // Position
                            0, 0, 1,        // Viewpoint
                            0, -1, 0);    // Up
                }

//                pcl::io::PointCloudImageExtractorFromRGBField<PointType> extractor;
//                pcl::PCLImage image;
//                extractor.setPaintNaNsWithBlack(false);
//                auto extracted = extractor.extract(*cloud, image);
//                if(extracted)
//                    image_viewer_->addRGBImage((const unsigned char*) &image.data[0], image.width, image.height);
            }
        }
//        image_viewer_->spinOnce();
    }

private:
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

    pcl::visualization::PCLVisualizer::Ptr cloud_viewer_;
    pcl::visualization::ImageViewer::Ptr image_viewer_;
    std::once_flag flag;
};

#endif //PCL_MULTI_THREADED_PROCESSING_TIMEDCLOUDVIEWER_HPP

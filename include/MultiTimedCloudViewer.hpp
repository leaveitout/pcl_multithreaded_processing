//
// Created by sean on 02/12/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_MULTITIMEDCLOUDVIEWER_HPP
#define PCL_MULTI_THREADED_PROCESSING_MULTITIMEDCLOUDVIEWER_HPP


#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/point_cloud_image_extractors.h>
#include "MultiTimedCloud.hpp"
#include "Processor.hpp"

template<typename PointType>
class MultiTimedCloudViewer : public Processor<std::shared_ptr<MultiTimedCloud<PointType>>> {
    typedef std::shared_ptr<MultiTimedCloud<PointType>> MultiTimedCloudPtr;

public:
    MultiTimedCloudViewer(const std::shared_ptr<Buffer<MultiTimedCloudPtr>> multi_timed_cloud_buffer,
                          size_t id,
                          std::string description) :
            Processor<MultiTimedCloudPtr>(multi_timed_cloud_buffer, id, description) {
    }

protected:
    virtual void initializeThreadResources() {
//        for(auto& viz : cloud_viewers_) {
//            viz.reset(new pcl::visualization::PCLVisualizer("PCL OpenNI2 Cloud"));
//
//        }
    }

    virtual void processBufferElement(MultiTimedCloudPtr element) {
        std::call_once(flag, [&]() {
            num_clouds_in_element = element->getSize();
            for (int i = 0; i < num_clouds_in_element; ++i) {
                pcl::visualization::PCLVisualizer::Ptr viz = boost::make_shared<pcl::visualization::PCLVisualizer>();
                cloud_viewers_.push_back(viz);
                viz->registerMouseCallback(&MultiTimedCloudViewer::mouseCallback, *this);
                viz->registerKeyboardCallback(&MultiTimedCloudViewer::keyboardCallback, *this);
                viz->setCameraFieldOfView (1.02259994f);

                pcl::visualization::ImageViewer::Ptr img_viz = boost::make_shared<pcl::visualization::ImageViewer>();
                image_viewers_.push_back(img_viz);
                img_viz->registerMouseCallback(&MultiTimedCloudViewer::mouseCallback, *this);
                img_viz->registerKeyboardCallback(&MultiTimedCloudViewer::keyboardCallback, *this);
            }
        });

        for (size_t idx = 0; idx < num_clouds_in_element; ++idx) {
            auto viz = cloud_viewers_.at(idx);
            auto img_viz = image_viewers_.at(idx);
            viz->spinOnce();
            auto timed_cloud = element->getTimedCloudAt(idx);
            if (timed_cloud) {
                auto cloud = timed_cloud->getCloud();
                if (cloud) {
                    if (!viz->updatePointCloud(cloud, "OpenNICloud")) {
                        viz->setPosition(0, 0);
                        viz->setSize(cloud->width, cloud->height);
                        viz->addPointCloud(cloud, "OpenNICloud");
                        viz->resetCameraViewpoint("OpenNICloud");
                        viz->setCameraPosition(
                                0, 0, 0,        // Position
                                0, 0, 1,        // Viewpoint
                                0, -1, 0);    // Up
                        viz->addText(std::to_string(idx), 0, 0, "Camera Index");
                    }


                    pcl::io::PointCloudImageExtractorFromRGBField<PointType> extractor;
                    pcl::PCLImage image;
                    extractor.setPaintNaNsWithBlack(false);
                    auto extracted = extractor.extract(*(element->getTimedCloudAt(idx)->getCloud()), image);
                    if(extracted)
                        img_viz->addRGBImage((const unsigned char*) &image.data[0], image.width, image.height);
                }
            }
            img_viz->spinOnce();
        }
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

    std::vector<pcl::visualization::PCLVisualizer::Ptr> cloud_viewers_;
    std::vector<pcl::visualization::ImageViewer::Ptr> image_viewers_;
    size_t num_clouds_in_element;
    std::once_flag flag;
};

#endif //PCL_MULTI_THREADED_PROCESSING_MULTITIMEDCLOUDVIEWER_HPP

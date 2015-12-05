//
// Created by sean on 05/12/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_INDEXEDTIMEDCLOUDVIEWER_HPP
#define PCL_MULTI_THREADED_PROCESSING_INDEXEDTIMEDCLOUDVIEWER_HPP

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/point_cloud_image_extractors.h>
#include "IndexedTimedCloud.hpp"
#include "Processor.hpp"
#include "Palette.hpp"

template<typename PointType>
class IndexedTimedCloudViewer : public Processor<std::shared_ptr<IndexedTimedCloud<PointType>>> {
    typedef std::shared_ptr<IndexedTimedCloud<PointType>> IndexedTimedCloudPtr;

public:
    IndexedTimedCloudViewer(const std::shared_ptr<Buffer<IndexedTimedCloudPtr>> timed_cloud_buffer,
                     size_t id,
                     std::string description) :
            Processor<IndexedTimedCloudPtr>(timed_cloud_buffer, id, description) {
        palette_ = std::make_shared<Palette>(8);
    }

protected:
    virtual void initializeThreadResources() {

    }

    virtual void processBufferElement(IndexedTimedCloudPtr element) {
        std::call_once(flag, [&]() {
            cloud_viewer_ = boost::make_shared<pcl::visualization::PCLVisualizer>();
            cloud_viewer_->registerMouseCallback(&IndexedTimedCloudViewer::mouseCallback, *this);
            cloud_viewer_->registerKeyboardCallback(&IndexedTimedCloudViewer::keyboardCallback, *this);
            cloud_viewer_->setCameraFieldOfView(1.02259994f);
        });

        cloud_viewer_->spinOnce();
        cloud_viewer_->removeAllShapes();
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
                else {
                    auto indices_ptr = element->getIndices();
                    if(indices_ptr) {
                        const auto& indices = indices_ptr->indices;
                        for(size_t index_num = 0; index_num < indices.size(); ++index_num) {
                            const auto& pt = cloud->at(indices.at(index_num));
                            if (!pcl_isfinite (pt.x) || !pcl_isfinite (pt.y) || !pcl_isfinite (pt.z))
                                continue;
                            auto color = palette_->getColorAt(index_num);
                            std::stringstream ss;
                            ss << "sphere" << index_num;
                            cloud_viewer_->addSphere(pt, 0.05, color[0], color[1], color[2], ss.str());
//                            cloud_viewer_->addSphere(pt, 0.05, 255, 255, 255, ss.str());
                        }
                    }
                }
            }
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

    pcl::visualization::PCLVisualizer::Ptr cloud_viewer_;
    pcl::visualization::ImageViewer::Ptr image_viewer_;
    std::shared_ptr<Palette> palette_;
    std::once_flag flag;
};

#endif //PCL_MULTI_THREADED_PROCESSING_INDEXEDTIMEDCLOUDVIEWER_HPP

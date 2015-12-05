//
// Created by sean on 25/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_MULTICLOUDVIEWER_HPP
#define PCL_MULTI_THREADED_PROCESSING_MULTICLOUDVIEWER_HPP

#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <boost/smart_ptr/make_shared_object.hpp>
#include "Logger.hpp"
#include "Buffer.hpp"
#include "Processor.hpp"
#include "MultiCloud.hpp"

template <typename PointType>
class MultiCloudViewer : public Processor<std::shared_ptr<MultiCloud<PointType>>> {
    typedef std::shared_ptr<MultiCloud<PointType>> MultiCloudPtr;

public:
    MultiCloudViewer(const std::shared_ptr<Buffer<MultiCloudPtr>> multi_cloud_buffer,
                     size_t num_clouds_in_buffer_element,
                     size_t id,
                     std::string description) :
            Processor<MultiCloudPtr>(multi_cloud_buffer, id, description) {
        for(int i = 0; i < num_clouds_in_buffer_element; ++i) {
            pcl::visualization::PCLVisualizer::Ptr viz = boost::make_shared<pcl::visualization::PCLVisualizer>();
            cloud_viewers_.push_back(viz);
        }
    }

protected:
    virtual void initializeThreadResources() {

    }

    virtual void processBufferElement(std::shared_ptr<MultiCloud<PointType>> element) {

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

};

#endif //PCL_MULTI_THREADED_PROCESSING_MULTICLOUDVIEWER_HPP


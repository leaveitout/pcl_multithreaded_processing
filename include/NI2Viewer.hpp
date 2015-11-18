//
// Created by sean on 17/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_NI2VIEWER_HPP
#define PCL_MULTI_THREADED_PROCESSING_NI2VIEWER_HPP


#include <mutex>
#include <memory>

#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include "Logger.hpp"
#include "Buffer.hpp"

template <typename PointType>
class NI2Viewer {

    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    typedef pcl::io::openni2::Image NI2Image;
    typedef NI2Image::Ptr NI2ImagePtr;
    typedef pcl::io::OpenNI2Grabber NI2Grabber;

    NI2Grabber::Ptr grabber_;

    pcl::visualization::PCLVisualizer::Ptr cloud_viewer_;
    pcl::visualization::ImageViewer::Ptr image_viewer_;

    CloudConstPtr cloud_;
    std::mutex cloud_mutex_;

    NI2ImagePtr image_;
    std::mutex image_mutex_;

    boost::signals2::connection cloud_connection_;
    boost::signals2::connection image_connection_;

    void cloudCallback(const CloudConstPtr& cloud) {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        cloud_ = cloud;
    }

    void cloudProcess() {
        // TODO: Figure out how to do the cloud processing in a separate thread
        // Use of a condition variable to process cloud
    }

    void imageCallback(const NI2ImagePtr& image) {
        std::lock_guard<std::mutex> lock(image_mutex_);
        image_ = image;
    }

    void imageProcess() {
        // TODO: Figure out how to do the image processing in a separate thread
    }

    void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*) {
        std::string special_key = event.getKeyCode() ? "" : "special ";
        if(event.keyDown())
            Logger::log(Logger::INFO, "The %skey %c was pressed.", special_key.c_str(), event.getKeyCode() );
        else
            Logger::log(Logger::INFO, "The %skey %c was released.", special_key.c_str(), event.getKeyCode() );
    }

    void mouseCallback(const pcl::visualization::MouseEvent& mouse_event, void*) {
        if (mouse_event.getType () == pcl::visualization::MouseEvent::MouseButtonPress &&
            mouse_event.getButton () == pcl::visualization::MouseEvent::LeftButton) {
            Logger::log(Logger::INFO, "Left button pressed @ (%i, %i).\n", mouse_event.getX(), mouse_event.getY());
        }
    }

public:
    NI2Viewer(NI2Grabber::Ptr grabber) :
            grabber_ (grabber),
            cloud_viewer_ (new pcl::visualization::PCLVisualizer("PCL OpenNI2 Cloud")){
        cloud_viewer_->setPosition(0, 0);
        cloud_viewer_->setSize(640, 480);
    }

    void run() {
        cloud_viewer_->registerMouseCallback(&NI2Viewer::mouseCallback, *this);
        cloud_viewer_->registerKeyboardCallback(&NI2Viewer::keyboardCallback, *this);

        boost::function<void (const CloudConstPtr&) > cloud_callback = boost::bind(&NI2Viewer::cloudCallback, this, _1);
        cloud_connection_ = grabber_->registerCallback(cloud_callback);

        if (grabber_->providesCallback<void (const boost::shared_ptr<pcl::io::openni2::Image>&)>())
        {
            image_viewer_.reset (new pcl::visualization::ImageViewer ("PCL OpenNI image"));
            image_viewer_->registerMouseCallback (&NI2Viewer::mouseCallback, *this);
            image_viewer_->registerKeyboardCallback (&NI2Viewer::keyboardCallback, *this);
            boost::function<void (const boost::shared_ptr<pcl::io::openni2::Image>&) > image_callback =
                    boost::bind(&NI2Viewer::imageCallback, this, _1);
            image_connection_ = grabber_->registerCallback(image_callback);
            image_viewer_->setPosition(645, 0);
            image_viewer_->setSize(640, 480);
        }

        while (!cloud_viewer_->wasStopped() && (image_viewer_ && !image_viewer_->wasStopped())) {
            boost::shared_ptr<pcl::io::openni2::Image> image;
            CloudConstPtr cloud;

            cloud_viewer_->spinOnce();

            // See if we can get a cloud
            if (cloud_mutex_.try_lock()) {
                cloud_.swap(cloud);
                cloud_mutex_.unlock();
            }

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

            // See if we can get an image
            if (image_mutex_.try_lock()) {
                image_.swap(image);
                image_mutex_.unlock();
            }

            if (image) {
                image_viewer_->addRGBImage((const unsigned char *) image->getData(),
                                           image->getWidth(), image->getHeight());
                image_viewer_->spinOnce();
            }
        }
    }

    ~NI2Viewer() {
        cloud_connection_.disconnect();
        image_connection_.disconnect();
    }


};


#endif //PCL_MULTI_THREADED_PROCESSING_NI2VIEWER_HPP

//
// Created by sean on 17/11/15.
//

#include "App.hpp"

#include <pcl/io/openni2_grabber.h>
#include <Logger.hpp>
#include <NI2Viewer.hpp>
#include <NI2Producer.hpp>
#include <thread>
#include <CloudProcessor.hpp>
#include <CloudRecorder.hpp>
#include <CloudViewer.hpp>
#include <CloudViewerSimple.hpp>

typedef pcl::io::OpenNI2Grabber NI2Grabber;
typedef pcl::io::openni2::OpenNI2DeviceManager NI2DeviceManager;

App::App(int argc, char **argv) {
    // Parse arguments
    // Check that cameras are connected
}

int App::exec() {
    auto deviceManager = NI2DeviceManager::getInstance ();
    if (deviceManager->getNumOfConnectedDevices () > 0) {
        auto device = deviceManager->getAnyDevice ();
        Logger::log(Logger::INFO, "Device ID not set, using default device: %s\n", device->getStringID ().c_str());
        NI2Grabber::Ptr grabber = boost::make_shared<NI2Grabber>(device->getStringID());
//        NI2Producer<pcl::PointXYZRGBA> producer(grabber, 0, cloud_buffer, image_buffer);
        NI2Producer<pcl::PointXYZRGBA> producer(grabber, 0);
//        CloudRecorder<pcl::PointXYZRGBA> processor(producer.getCloudBuffer(), 0);
        CloudViewer<pcl::PointXYZRGBA> viewer(producer.getCloudBuffer(), 0);

        producer.start();
        viewer.start();
//        processor.start();

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        producer.stop();
        viewer.stop();
//        processor.stop();

//        grabber->start();

//        std::unique_ptr<NI2Viewer<pcl::PointXYZRGBA>> viewer;
//        viewer.reset(new NI2Viewer<pcl::PointXYZRGBA>(grabber));
//        viewer->run();
//
//        grabber->stop();
    }
    else {
        Logger::log(Logger::INFO, "No compatible devices attached.\n");
    }

    return 0;
}

//
// Created by sean on 17/11/15.
//

#include "App.hpp"

#include <pcl/io/openni2_grabber.h>
#include <Logger.hpp>
//#include <NI2Viewer.hpp>
//#include <NI2Producer.hpp>
#include <thread>
//#include <MultiCloudProducer.hpp>
#include "MultiTimedCloudProducer.hpp"
#include "TimedCloudViewer.hpp"
#include "IndexedTimedCloudViewer.hpp"
#include "CalibratorNode.hpp"
//#include <MultiTimedCloudViewer.hpp>
//#include <CloudViewer.hpp>
//#include <NI2ImageViewer.hpp>
//#include "CloudAndImageProducer.hpp"
//#include "CloudImageViewer.hpp"

typedef pcl::io::OpenNI2Grabber NI2Grabber;
typedef pcl::io::openni2::OpenNI2DeviceManager NI2DeviceManager;

App::App(int argc, char **argv) {
    // Parse arguments
    // Check that cameras are connected
}

int App::exec() {
    auto device_manager = NI2DeviceManager::getInstance ();
    if (device_manager->getNumOfConnectedDevices () > 0) {
//        auto device = device_manager->getAnyDevice ();
//        Logger::log(Logger::INFO, "Device ID not set, using default device: %s\n", device->getStringID ().c_str());
//        std::shared_ptr<NI2Grabber> grabber = std::make_shared<NI2Grabber>(device->getStringID());
//        CloudAndImageProducer<pcl::PointXYZRGBA> producer(grabber, 0);
//        CloudImageViewer<pcl::PointXYZRGBA> viewer(producer.getBuffer(), 0, "Camera 0");

        std::vector<std::shared_ptr<pcl::Grabber>> grabbers;

        size_t num_devices = device_manager->getNumOfConnectedDevices();
        for (size_t i = 1; i <= num_devices; ++i) {
            std::stringstream ss;
            ss << '#' << i;
            auto grabber = std::make_shared<NI2Grabber>(ss.str());
            if(grabber->isRunning())
                Logger::log(Logger::INFO, "Grabber is running\n");
            grabbers.push_back(grabber);
        }
        MultiTimedCloudProducer<pcl::PointXYZRGBA> producer(grabbers, 0);
        CalibratorNode<pcl::PointXYZRGBA> calibrator(producer.getBuffer(), 0, "Calibrator Node");
//        MultiTimedCloudViewer<pcl::PointXYZRGBA> viewer(producer.getBuffer(), 0, "Viewer");
//        TimedCloudViewer<pcl::PointXYZRGBA> viewer(calibrator.getOutputBuffer(), 0, "Calibrated Cloud Viewer");
        IndexedTimedCloudViewer<pcl::PointXYZRGBA> viewer(calibrator.getOutputBuffer(), 0, "Indexed Cloud Viewer");

        producer.start();
        calibrator.start();
        viewer.start();
        std::this_thread::sleep_for(std::chrono::seconds(5));
        producer.stop();
        calibrator.stop();
        viewer.stop();
    }
    else {
        Logger::log(Logger::INFO, "No compatible devices attached.\n");
    }

    return 0;
}

////
//// Created by sean on 18/11/15.
////
//
//#ifndef PCL_MULTI_THREADED_PROCESSING_CLOUDPROCESSOR_HPP
//#define PCL_MULTI_THREADED_PROCESSING_CLOUDPROCESSOR_HPP
//
//#include <pcl/point_cloud.h>
//#include <thread>
//
//#include "Buffer.hpp"
//#include "Timer.hpp"
//#include "Processor.hpp"
//
//template <typename PointType>
//class CloudProcessor : public Processor {
//public:
//    typedef pcl::PointCloud<PointType> Cloud;
//    typedef typename Cloud::ConstPtr CloudConstPtr;
//
//    std::shared_ptr<Buffer<CloudConstPtr>> cloud_buffer_;
//
//// This is the method that should be overridden for different processes
//void processAllClouds() {
//    initializeThreadResources();
//    mutex_.lock();
//    while(!is_done_) {
//        mutex_.unlock();
//        getCloudAndProcess();
//        mutex_.lock();
//    }
//    mutex_.unlock();
//
//    Logger::log(Logger::INFO, "Processing remaining %ld clouds in the buffer...\n", cloud_buffer_->getSize());
//
//    while(!cloud_buffer_->isEmpty())
//        getCloudAndProcess();
//}
//
//private:
//
//    void getCloudAndProcess() {
//        CloudConstPtr cloud = cloud_buffer_->getFront();
//        if(cloud) {
//            processCloud(cloud);
//            if (processing_timer_->time())
//                Logger::log(Logger::INFO, "Processing Cloud Buffer size: %zd.\n", cloud_buffer_->getSize());
//        }
//    }
//
//protected:
//    virtual void processCloud(CloudConstPtr cloud) {
//        std::this_thread::sleep_for(std::chrono::milliseconds(50));
//    }
//
//
//};
//#endif //PCL_MULTI_THREADED_PROCESSING_CLOUDPROCESSOR_HPP

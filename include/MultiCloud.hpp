//
// Created by sean on 25/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_MULTICLOUD_HPP
#define PCL_MULTI_THREADED_PROCESSING_MULTICLOUD_HPP

#include <pcl/point_cloud.h>
#include <mutex>

template <typename PointType>
class MultiCloud {
public:
    // TODO: Investigate the need for synchronicity
    MultiCloud(size_t size) :
            size_ (size) {
        clouds_ = std::make_shared<std::vector<typename pcl::PointCloud<PointType>::ConstPtr>>(size_);
    }

    typename pcl::PointCloud<PointType>::ConstPtr getCloudAt(size_t i) const {
//        std::lock_guard<std::mutex> lock(mutex_);
        if(i < clouds_->size())
            return clouds_->at(i);
        else
            return nullptr;
    }

    void setCloudAt(const typename pcl::PointCloud<PointType>::ConstPtr cloud, size_t i) {
//        std::lock_guard<std::mutex> lock(mutex_);
        if(i < clouds_->size())
            clouds_->at(i) = cloud;
    }

    inline bool isCloudSetAt(size_t i) const {
        return clouds_->at(i) ? true : false;
    }

private:
    MultiCloud(const MultiCloud&) = delete;
    MultiCloud& operator = (const MultiCloud&) = delete;
    size_t size_;
    std::shared_ptr<std::vector<typename pcl::PointCloud<PointType>::ConstPtr>> clouds_;
//    std::mutex mutex_;

};
#endif //PCL_MULTI_THREADED_PROCESSING_MULTICLOUD_HPP

//
// Created by sean on 23/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_MULTICLOUDANDIMAGE_HPP
#define PCL_MULTI_THREADED_PROCESSING_MULTICLOUDANDIMAGE_HPP

#include <pcl/io/image.h>
#include <pcl/point_cloud.h>
#include <mutex>

template <typename PointType>
class MultiCloudAndImage {
    const pcl::io::Image::Ptr &getImageAt(size_t i) const {
//        std::lock_guard<std::mutex> lock(mutex_);
        if(i < images_->size())
            return images_->at(i);
        else
            return nullptr;
    }

    const pcl::PointCloud<PointType>::ConstPtr &getCloudAt(size_t i) const {
//        std::lock_guard<std::mutex> lock(mutex_);
        if(i < clouds_->size())
            return clouds_->at(i);
        else
            return nullptr;
    }

    MultiCloudAndImage(size_t size) :
            size_ (size) {
        images_ = std::make_shared<std::vector<pcl::io::Image::Ptr>>(size_);
        clouds_ = std::make_shared<std::vector<pcl::PointCloud<PointType>::ConstPtr>>(size_);
//        images_.reset(new std::vector<pcl::io::Image::ConstPtr>(size));
//        clouds_.reset(new std::vector<pcl::PointCloud<PointType>::ConstPtr>(size));
    }

public:
    inline bool isImageSetAt(size_t i) const {
        return images_->at(i) ? true : false;
    }

    void setCloudAt(const pcl::PointCloud<PointType>::ConstPtr &cloud, size_t i) {
//        std::lock_guard<std::mutex> lock(mutex_);
        if(i < clouds_->size())
            clouds_->at(i) = cloud;
    }

    inline bool isCloudSetAt(size_t i) const {
        return clouds_->at(i) ? true : false;
    }

    void setImageAt(const pcl::io::Image::Ptr &image, size_t i) {
//        std::lock_guard<std::mutex> lock(mutex_);
        if(i < images_->size())
            images_->at(i) = image;
    }

private:
    size_t size_;
    std::unique_ptr<std::vector<pcl::io::Image::Ptr>> images_;
    std::unique_ptr<std::vector<pcl::PointCloud<PointType>::ConstPtr>> clouds_;
    std::mutex mutex_;
};

#endif //PCL_MULTI_THREADED_PROCESSING_MULTICLOUDANDIMAGE_HPP

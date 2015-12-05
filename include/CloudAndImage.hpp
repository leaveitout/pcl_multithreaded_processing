//
// Created by sean on 23/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_CLOUDANDIMAGE_HPP
#define PCL_MULTI_THREADED_PROCESSING_CLOUDANDIMAGE_HPP

#include <pcl/io/image.h>
#include <pcl/point_cloud.h>

template <typename PointType>
class CloudAndImage {
public:
    const pcl::io::Image::Ptr &getImage() const {
        return image_;
    }

    void setImage(const pcl::io::Image::Ptr &image) {
        image_ = image;
    }

    inline bool isImageSet() const {
        return image_ ? true : false;
    }

    typename pcl::PointCloud<PointType>::ConstPtr getCloud() const {
        return cloud_;
    }

    void setCloud(const typename pcl::PointCloud<PointType>::ConstPtr &cloud) {
        cloud_ = cloud;
    }

    inline bool isCloudSet() const {
        return cloud_ ? true : false;
    }

    CloudAndImage(typename pcl::PointCloud<PointType>::ConstPtr cloud, pcl::io::Image::Ptr image) :
            cloud_ (cloud),
            image_ (image) {
    }

    CloudAndImage(typename pcl::PointCloud<PointType>::ConstPtr cloud) :
            cloud_ (cloud),
            image_ (nullptr) {
    }

    CloudAndImage(pcl::io::Image::Ptr image) :
            cloud_ (nullptr),
            image_ (image) {
    }

    CloudAndImage() : image_(nullptr), cloud_(nullptr) {
    }

private:
    pcl::io::Image::Ptr image_;
    typename pcl::PointCloud<PointType>::ConstPtr cloud_;
};

#endif //PCL_MULTI_THREADED_PROCESSING_CLOUDANDIMAGE_HPP

//
// Created by sean on 25/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_TIMEDCLOUD_HPP
#define PCL_MULTI_THREADED_PROCESSING_TIMEDCLOUD_HPP

#include <pcl/point_cloud.h>

template <typename PointType>
class TimedCloud {
public:
    TimedCloud(typename pcl::PointCloud<PointType>::ConstPtr cloud, size_t timestamp) :
            cloud_ (cloud), stamp_ (timestamp) {
    }

    size_t getStamp() const {
        return stamp_;
    }

    const typename pcl::PointCloud<PointType>::ConstPtr& getCloud() const {
        return cloud_;
    }

private:
    TimedCloud(const TimedCloud&) = delete;
    TimedCloud& operator = (const TimedCloud&) = delete;

    typename pcl::PointCloud<PointType>::ConstPtr cloud_;
    size_t stamp_;

};

#endif //PCL_MULTI_THREADED_PROCESSING_TIMEDCLOUD_HPP

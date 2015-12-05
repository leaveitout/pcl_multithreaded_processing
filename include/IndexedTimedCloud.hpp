//
// Created by sean on 05/12/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_INDEXEDTIMEDCLOUD_HPP
#define PCL_MULTI_THREADED_PROCESSING_INDEXEDTIMEDCLOUD_HPP

#include <pcl/PointIndices.h>
#include "TimedCloud.hpp"

template <typename PointType>
class IndexedTimedCloud : public TimedCloud<PointType> {
public:
    IndexedTimedCloud(typename pcl::PointCloud<PointType>::ConstPtr cloud,
                      size_t timestamp,
                      pcl::PointIndices::Ptr indices) :
            TimedCloud<PointType>(cloud, timestamp),
            indices_ (indices) {
    }

    pcl::PointIndicesPtr getIndices() const {
        return indices_;
    }

private:
    pcl::PointIndicesPtr indices_;
};

#endif //PCL_MULTI_THREADED_PROCESSING_INDEXEDTIMEDCLOUD_HPP

//
// Created by sean on 25/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_MULTITIMEDCLOUD_HPP
#define PCL_MULTI_THREADED_PROCESSING_MULTITIMEDCLOUD_HPP

#include "TimedCloud.hpp"

template <typename PointType>
class MultiTimedCloud {
public:
    MultiTimedCloud(size_t size) {
        timed_clouds_ = std::make_shared<std::vector<std::shared_ptr<TimedCloud<PointType>>>>(size);
    }

    std::shared_ptr<TimedCloud<PointType>> getTimedCloudAt(size_t i) const {
        if(i < timed_clouds_->size())
            return timed_clouds_->at(i);
        else
            return nullptr;
    }

    void setTimedCloudAt(std::shared_ptr<TimedCloud<PointType>> timed_cloud, size_t i) {
        if(i < timed_clouds_->size())
            timed_clouds_->at(i) = timed_cloud;
    }

    bool isTimedCloudSetAt(size_t i) const {
        return timed_clouds_->at(i) ? true : false;
    }

    inline size_t getSize() const {
        return timed_clouds_->size();
    }

    void printTimings() const {
        std::stringstream ss;
        ss << "Timed Cloud Buffer Element Timings:" << std::endl;
        for(auto tc: *timed_clouds_) {
            if(tc) {
                ss << tc->getStamp();
            }
            else {
                ss << "----------";
            }
            ss << std::endl;
        }
        ss << std::endl;
        Logger::log(Logger::INFO, ss.str());
    }

    size_t getLatestTimestamp() const {
        size_t max = 0;
        for(const auto& timed_cloud : *timed_clouds_)
            if(timed_cloud->getStamp() > max)
                max = timed_cloud->getStamp();
        return max;
        // TODO: Fix this so that it return the correct type
//        return std::max(timed_clouds_->begin(), timed_clouds_->end(), compareTimings());
    }

private:
    struct compareTimings {
        inline bool operator()(const std::shared_ptr<TimedCloud<PointType>> &timed_cloud1,
                            const std::shared_ptr<TimedCloud<PointType>> &timed_cloud2) {
            return timed_cloud1->getStamp() < timed_cloud2->getStamp();
        }
    };

    MultiTimedCloud(const MultiTimedCloud&) = delete;
    MultiTimedCloud& operator = (const MultiTimedCloud&) = delete;
    std::shared_ptr<std::vector<std::shared_ptr<TimedCloud<PointType>>>> timed_clouds_;
};

#endif //PCL_MULTI_THREADED_PROCESSING_MULTITIMEDCLOUD_HPP

//
// Created by sean on 26/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_CALIBRATOR_NODE_HPP
#define PCL_MULTI_THREADED_PROCESSING_CALIBRATOR_NODE_HPP

#include <pcl/io/image.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/point_cloud_image_extractors.h>
#include <pcl/correspondence.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include "MultiTimedCloud.hpp"
#include "ProcessorProducer.hpp"
#include "SquareDetector.hpp"
#include "CameraId.hpp"

template <typename PointType>
class CalibratorNode : public ProcessorProducer<std::shared_ptr<MultiTimedCloud<PointType>>,
                                                std::shared_ptr<IndexedTimedCloud<PointType>>> {

    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    typedef std::shared_ptr<TimedCloud<PointType>> TimedCloudPtr;
    typedef std::shared_ptr<MultiTimedCloud<PointType>> MultiTimedCloudPtr;
    typedef std::shared_ptr<IndexedTimedCloud<PointType>> IndexedTimedCloudPtr;

public:
    CalibratorNode(const std::shared_ptr<Buffer<MultiTimedCloudPtr>> input_buffer,
                   size_t id,
                   std::string description) :
            ProcessorProducer<MultiTimedCloudPtr, IndexedTimedCloudPtr>(input_buffer, id, description) {
        palette_ = std::make_shared<Palette>(8);
    }

protected:
    virtual void processBufferElement(MultiTimedCloudPtr element) override {
        std::call_once(flag, [&](){
            num_clouds_in_element = element->getSize();
            transformations_ = std::make_shared<std::vector<Eigen::Matrix4f>>(num_clouds_in_element-1);
        });
        // Check that we have a full element
        for(size_t idx = 0; idx < num_clouds_in_element; ++idx) {
            if(!element->getTimedCloudAt(idx))
                return;
        }

        pcl::io::PointCloudImageExtractorFromRGBField<PointType> extractor;
        pcl::ExtractIndices<PointType> extract;
        pcl::PCLImage image;
        std::vector<CloudPtr> correspondence_clouds(num_clouds_in_element);
        std::vector<pcl::PointIndices::Ptr> point_indices(num_clouds_in_element);
        for(size_t idx = 0; idx < num_clouds_in_element; ++idx) {

            // Generate images from all clouds
            extractor.setPaintNaNsWithBlack(false);
            auto extracted = extractor.extract(*(element->getTimedCloudAt(idx)->getCloud()), image);
            std::stringstream ssa;
            ssa << "Extracted image : " << image << std::endl;
            Logger::log(Logger::INFO, ssa.str());

            if(extracted) {
                 //Detect Squares in the Images
                point_indices.at(idx) = SquareDetector::getPointIndicesOfCorners(image, CameraId::center, palette_);
//                for(auto& index : point_indices)
//                    element->getTimedCloudAt(idx)->getCloud()->points.at(index
                if(point_indices.at(idx)) {
                    correspondence_clouds.at(idx).reset(new pcl::PointCloud<PointType>);

                    // Get correspondences
                    extract.setInputCloud(element->getTimedCloudAt(idx)->getCloud());
                    extract.setIndices(point_indices.at(idx));
//                    extract.setNegative(false);
                    extract.filter(*correspondence_clouds.at(idx));
                    std::stringstream ss;
                    ss << "Correspondence Cloud " << idx << ": " <<
                            correspondence_clouds.at(idx)->width * correspondence_clouds.at(idx)->height <<
                            " data points." << std::endl;
                    Logger::log(Logger::INFO, ss.str());
                }
            }
        }

//        auto target = correspondence_clouds.at(0);
//        if(target)
//            if(target->size() > 0)
//                for(size_t cloud_num = 1; cloud_num < num_clouds_in_element; ++cloud_num) {
//                    auto source = correspondence_clouds.at(cloud_num);
//                    if(source)
//                        if(source->size() == target->size()) {
//        //                    pcl::PointCorrespondences3DVector correspondences3DVector;
//        //                    pcl::PointCorrespondence3D correspondence3D;
//                            pcl::Correspondences correspondences;
//                            for(int idx = 0; idx < target->size(); ++idx)
//                                correspondences.push_back(pcl::Correspondence(idx, idx, 5.0));
//                            Eigen::Matrix4f transformation;
//                            pcl::registration::TransformationEstimationSVD<PointType, PointType> svd;
////                            svd.estimateRigidTransformation(*source, *target, correspondences, transformation);
//                            svd.estimateRigidTransformation(*source, *target, transformation);
//                            std::stringstream ss;
//                            ss << "Transformation Matrix (" << cloud_num << "): " << std::endl << transformation << std::endl;
//                            Logger::log(Logger::INFO, ss.str());
//                        }
//                }
        bool use_svd = true;

        auto target = element->getTimedCloudAt(0)->getCloud();
        auto target_indices = point_indices.at(0);
        if(target && target_indices)
            if(target_indices->indices.size() > 0)
                for(size_t cloud_num = 1; cloud_num < num_clouds_in_element; ++cloud_num) {
                    auto source = element->getTimedCloudAt(cloud_num)->getCloud();
                    auto source_indices = point_indices.at(cloud_num);
                    if(source && source_indices)
                        if(source_indices->indices.size() == target_indices->indices.size()) {
                            Eigen::Matrix4f transformation;
                            if(use_svd) {
                                pcl::registration::TransformationEstimationSVD<PointType, PointType> svd;
                                svd.estimateRigidTransformation(*source,
                                                                source_indices->indices,
                                                                *target,
                                                                target_indices->indices,
                                                                transformation);
//                                svd.estimateRigidTransformation(*target,
//                                                                target_indices->indices,
//                                                                *source,
//                                                                source_indices->indices,
//                                                                transformation);
                            }
                            else {
                                pcl::registration::TransformationEstimationLM<PointType, PointType> svd;
                                svd.estimateRigidTransformation(*source,
                                                                source_indices->indices,
                                                                *target,
                                                                target_indices->indices,
                                                                transformation);
                            }
//                            svd.estimateRigidTransformation(*target, target_indices->indices, *source, source_indices->indices, transformation);
                            std::stringstream ss;
                            ss << "Transformation Matrix (" << cloud_num << "): " << std::endl << transformation << std::endl;
                            Logger::log(Logger::INFO, ss.str());
                            if(!transformation.hasNaN() && !transformation.isIdentity())
                                transformations_->at(cloud_num-1) = transformation;
                        }
                }
//        auto source_cloud = element->getTimedCloudAt(0)->getCloud();
//        auto target_cloud = element->getTimedCloudAt(1)->getCloud();
//        Eigen::Matrix4f transformation;
//        RANSACRegister(source_cloud, target_cloud, transformation);

        // Transform cloud and concatenate these into result cloud
        CloudPtr result_cloud = boost::make_shared<Cloud>();
        *result_cloud = *(element->getTimedCloudAt(0)->getCloud());
        drawIndices(result_cloud, point_indices.at(0)->indices, palette_);

        for(size_t cloud_num = 1; cloud_num < num_clouds_in_element; ++cloud_num) {
            if (!transformations_->at(cloud_num - 1).isZero()) {
                CloudPtr transformed_cloud = boost::make_shared<Cloud>();
                auto source = element->getTimedCloudAt(cloud_num)->getCloud();
//                Eigen::Matrix4f inverse = transformations_->at(cloud_num - 1).inverse();
//                pcl::transformPointCloud(*source, *transformed_cloud, inverse);
                pcl::transformPointCloud(*source, *transformed_cloud, transformations_->at(cloud_num - 1));
//                drawIndices(transformed_cloud, point_indices.at(cloud_num)->indices, palette_);
//                pcl::transformPointCloud(*source, *transformed_cloud, transformation);

                *result_cloud += (*transformed_cloud);
            }
        }


        // Push new cloud to output buffer
        CloudConstPtr result_const_cloud = result_cloud;
        size_t latest_timestamp = element->getLatestTimestamp();
        TimedCloudPtr result_timed_cloud = std::make_shared<TimedCloud<PointType>>(result_const_cloud, latest_timestamp);
//        this->output_buffer_->pushBack(result_timed_cloud);
        auto first_cloud = (element->getTimedCloudAt(0)->getCloud());
        auto first_timestamp = (element->getTimedCloudAt(0)->getStamp());
        auto first_indices = point_indices.at(0);
        IndexedTimedCloudPtr result =
                std::make_shared<IndexedTimedCloud<PointType>>(first_cloud, first_timestamp, first_indices);
        this->output_buffer_->pushBack(result);

    }

private:
    size_t RANSACRegister(typename pcl::PointCloud<PointType>::ConstPtr& cloudA,
                       typename pcl::PointCloud<PointType>::ConstPtr& cloudB,
                       Eigen::Matrix4f& transformation)
    {
        typename pcl::SampleConsensusModelRegistration<PointType>::Ptr sac_model(new pcl::SampleConsensusModelRegistration<PointType>(cloudA));
        sac_model->setInputTarget(cloudB);

        pcl::RandomSampleConsensus<PointType> ransac(sac_model);
        //pcl::LeastMedianSquares<pcl::PointNormal> ransac(sac_model); //might as well try these out too!
        //pcl::ProgressiveSampleConsensus<pcl::PointNormal> ransac(sac_model);
        ransac.setDistanceThreshold(0.05);

        //upping the verbosity level to see some info
        pcl::console::VERBOSITY_LEVEL vblvl = pcl::console::getVerbosityLevel();
        pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_DEBUG);
        ransac.computeModel(1);
        pcl::console::setVerbosityLevel(vblvl);

        Eigen::VectorXf coeffs;
        ransac.getModelCoefficients(coeffs);
        assert(coeffs.size() == 16);
        transformation = Eigen::Map<Eigen::Matrix4f>(coeffs.data(),4,4);

        vector<int> inliers; ransac.getInliers(inliers);
        return inliers.size();
    }

    void drawIndices(CloudPtr cloud, std::vector<int> indices, std::shared_ptr<Palette> palette) {
        int color_index = 0;
        for(const auto& index : indices) {
            auto row = index / cloud->width;
            auto col = index % cloud->width;
            int offset = 5;
            for(int r=row-offset; r<row+offset; r++)
                for(int c=col-offset; c<col+offset; c++) {
                    if (!pcl_isfinite (cloud->points.at(index).x) ||
                            !pcl_isfinite (cloud->points[index].y) ||
                            !pcl_isfinite (cloud->points[index].z))
                        continue;
                    cv::Scalar color = palette->getColorAt(color_index);
                    cloud->points.at(index).r = color[0];
                    cloud->points.at(index).g = color[1];
                    cloud->points.at(index).b = color[2];
                }
            color_index++;
        }
    }

    size_t num_clouds_in_element;
    std::shared_ptr<std::vector<Eigen::Matrix4f>> transformations_;
    std::once_flag flag;
    std::shared_ptr<Palette> palette_;
};

#endif //PCL_MULTI_THREADED_PROCESSING_CALIBRATORNODE_HPP


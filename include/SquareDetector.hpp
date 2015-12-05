//
// Created by sean on 30/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_SQUAREDETECTOR_HPP
#define PCL_MULTI_THREADED_PROCESSING_SQUAREDETECTOR_HPP

#include <memory>
#include <pcl/PointIndices.h>
#include <pcl/io/image.h>
#include <pcl/PCLImage.h>
#include "KeypointSorter.hpp"
#include "Palette.hpp"

using namespace std;
using namespace cv;

class SquareDetector {
public:
    static pcl::PointIndices::Ptr getPointIndicesOfCorners(pcl::PCLImage& image,
                                                           string camera_id,
                                                           shared_ptr<Palette> palette = nullptr);

    static pcl::PointIndices::Ptr getPointIndicesOfCorners(Mat image,
                                                           string camera_id,
                                                           shared_ptr<Palette> palette = nullptr);
    static shared_ptr<vector<vector<Point2d>>> detectSquareCorners(Mat image, string camera_id);
    static void drawSquareCorners(Mat image,
                                  shared_ptr<vector<vector<Point2d>>> squareCorners,
                                  shared_ptr<Palette> palette = nullptr);

private:
    // TODO: Create Mat objects to store allocated images
    static vector<vector<Point>> detectSquares(Mat image);
    static vector<Point2d> detectCorners(Mat image);
    static shared_ptr<vector<vector<Point2d>>> resolveCornersToSquares(const vector<Point2d> &corners,
                                                                       const vector<vector<Point>> &squares);


    static double angle(Point2d pt1, Point2d pt2, Point2d pt0);
    static double dist(Point2d pt0, Point2d pt1);
    static Scalar getRandomColor();
    static int getOpenCVType(std::string type);

    static constexpr int CHANNEL_DEPTH = 255;

    // Parameters for Square contours detection
    static constexpr double MAX_CONTOUR_AREA = 10000;
    static constexpr double MIN_CONTOUR_AREA = 100;
    static constexpr double APPROX_POLY_LENGTH_RATIO = 0.02;
    static constexpr double MAX_COSINE_LIMIT = 0.3;

    // Parameters for corners / contours resolution
    static constexpr double MAX_DIST_CORNER_TO_CONTOUR_PT = 5.0;

    // Parameters for Shi-Tomasi algorithm
    static constexpr int MAX_CORNERS = 20;
    static constexpr double QUALITY_LEVEL = 0.01;
    static constexpr double MIN_DISTANCE = 10.0;
    static constexpr double DEFAULT_K = 0.04;
    static constexpr bool USE_HARRIS = false;
    static constexpr int BLOCK_SIZE = 3;

    // Drawing constexprants
    static constexpr int CIRCLE_RADIUS = 4;
    static constexpr int CIRCLE_THICKNESS = -1; // Filled Circle
};


#endif //PCL_MULTI_THREADED_PROCESSING_SQUAREDETECTOR_HPP

//
// Created by sean on 27/11/15.
//

#include "Palette.hpp"

Palette::Palette(int num_colors) {
    if(num_colors <=0)
        num_colors = DEFAULT_NUM_COLORS;

    cv::Scalar default_color(DEFAULT_HUE, DEFAULT_SATURATION, DEFAULT_VALUE);
    cv::Mat palette(1, num_colors, CV_8UC3, default_color);

    for(int i=0; i<num_colors; i++) {
        uchar hue = static_cast<uchar>(round(static_cast<double>(i * MAX_HUE)/num_colors));
        palette.at<cv::Vec3b>(cv::Point(i,0))[0] = hue;
    }

    cv::cvtColor(palette, palette, CV_HSV2RGB);

    for(int i=0; i<num_colors; i++) {
        cv::Vec3b &pixel = palette.at<cv::Vec3b>(cv::Point(i, 0));
        cv::Scalar color(pixel[0], pixel[1], pixel[2]);
        colors_.push_back(color);
    }

    // TODO: Something better than just shuffling
    std::default_random_engine engine(std::random_device{}());
    std::shuffle(std::begin(colors_), std::end(colors_), engine);
}

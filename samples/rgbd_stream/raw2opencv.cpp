#include <opencv2/opencv.hpp>
#include <xv-sdk.h>

#include <cstring>
#include "colors.h"

cv::Mat raw_to_opencv( std::shared_ptr<const xv::DepthColorImage> rgbd)
{
    cv::Mat out;
    out = cv::Mat::zeros(rgbd->height, rgbd->width*2, CV_8UC3);
    auto w = rgbd->width;

    if (rgbd->height>0 && rgbd->width>0) {
        float dmax = 7.5;
        const auto tmp_d = reinterpret_cast<std::uint8_t const*>(rgbd->data.get()+3);
        for (unsigned int i=0; i< rgbd->height*rgbd->width; i++) {
            const auto &d = *reinterpret_cast<float const*>(tmp_d + i*(3+sizeof(float)));
            if( d < 0.01 || d > 9.9 ) {
                out.at<cv::Vec3b>(i / w, i % rgbd->width) = 0;
            } else {
                unsigned int u = static_cast<unsigned int>( std::max(0.0f, std::min(255.0f,  d * 255.0f / dmax )));
                const auto &cc = colors.at(u);
                out.at<cv::Vec3b>( i/ w, i%rgbd->width ) = cv::Vec3b(cc.at(2), cc.at(1), cc.at(0) );
            }
        }
        const auto tmp_rgb = reinterpret_cast<std::uint8_t const*>(rgbd->data.get());
        for (unsigned int i=0; i< rgbd->height*rgbd->width; i++) {
            const auto rgb = reinterpret_cast<std::uint8_t const*>(tmp_rgb + i*(3+sizeof(float)));
            out.at<cv::Vec3b>( i/ w, (i%rgbd->width) + rgbd->width) = cv::Vec3b(rgb[0], rgb[1], rgb[2]);
        }
    }
    return out;
}

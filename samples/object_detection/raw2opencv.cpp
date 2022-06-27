#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <xv-sdk.h>

#include <cstring>
#include <sstream>
#include <iomanip>
#ifndef WIN32
#include <arpa/inet.h>
#endif

#include "colors.h"

static const size_t keypointsNumber = 18;

static const cv::Scalar cvColors[keypointsNumber] = {
        cv::Scalar(255, 0, 0), cv::Scalar(255, 85, 0), cv::Scalar(255, 170, 0),
        cv::Scalar(255, 255, 0), cv::Scalar(170, 255, 0), cv::Scalar(85, 255, 0),
        cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 85), cv::Scalar(0, 255, 170),
        cv::Scalar(0, 255, 255), cv::Scalar(0, 170, 255), cv::Scalar(0, 85, 255),
        cv::Scalar(0, 0, 255), cv::Scalar(85, 0, 255), cv::Scalar(170, 0, 255),
        cv::Scalar(255, 0, 255), cv::Scalar(255, 0, 170), cv::Scalar(255, 0, 85)
};
static const std::pair<int, int> limbKeypointsIds[] = {
    {1, 2},  {1, 5},   {2, 3},
    {3, 4},  {5, 6},   {6, 7},
    {1, 8},  {8, 9},   {9, 10},
    {1, 11}, {11, 12}, {12, 13},
    {1, 0},  {0, 14},  {14, 16},
    {0, 15}, {15, 17}
};

void display_objects( cv::Mat &input, const std::vector<xv::Object> &objects, bool flip )
{
    for(unsigned int i=0; i<objects.size(); i++ ){
        const auto obj = objects.at(i);

        if( obj.shape == xv::Object::Shape::BoundingBox ){
            cv::Rect r(obj.x,obj.y,obj.width,obj.height);
            if( flip ){
                r = cv::Rect( input.cols - obj.x - obj.width, input.rows - obj.y - obj.height, obj.width, obj.height );
            }
            cv::rectangle(input, r, cv::Scalar(0,255,0) );
            std::stringstream stream;
            stream << obj.type << ": " << std::setprecision(3) << obj.confidence;
            std::string str = stream.str();
            cv::putText(input, str, (r.br() + r.tl())*0.5, cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0) );
        }else if( obj.shape == xv::Object::Shape::Human ){
            const int stickWidth = 4;
            CV_Assert(obj.keypoints.size() == keypointsNumber);

            for (size_t keypointIdx = 0; keypointIdx < obj.keypoints.size(); keypointIdx++) {
                if (obj.keypoints[keypointIdx].x == -1.0 && obj.keypoints[keypointIdx].y == -1.0) {
                    float x = obj.keypoints[keypointIdx].x;
                    float y = obj.keypoints[keypointIdx].y;
                    if( flip ){
                        x = input.cols - x;
                        y = input.rows - y;
                    }
                    cv::circle(input, cv::Point(x,y), 4, cvColors[keypointIdx], -1);
                }
            }

            for (const auto& limbKeypointsId : limbKeypointsIds) {
                std::pair<cv::Point2f, cv::Point2f> limbKeypoints(
                            cv::Point2f(obj.keypoints[limbKeypointsId.first].x,obj.keypoints[limbKeypointsId.first].y),
                        cv::Point2f(obj.keypoints[limbKeypointsId.second].x,obj.keypoints[limbKeypointsId.second].y)
                        );
                if ( (limbKeypoints.first.x == -1.0 && limbKeypoints.first.y == -1.0) || (limbKeypoints.second.x == -1.0 && limbKeypoints.second.y == -1.0)){
                    continue;
                }

                float meanX = (limbKeypoints.first.x + limbKeypoints.second.x) / 2;
                float meanY = (limbKeypoints.first.y + limbKeypoints.second.y) / 2;
                if( flip ){
                    meanX = input.cols - meanX;
                    meanY = input.rows - meanY;
                }
                cv::Point difference = limbKeypoints.first - limbKeypoints.second;
                double length = std::sqrt(difference.x * difference.x + difference.y * difference.y);
                int angle = static_cast<int>(std::atan2(difference.y, difference.x) * 180 / CV_PI);
                std::vector<cv::Point> polygon;
                cv::ellipse2Poly(cv::Point2d(meanX, meanY), cv::Size2d(length / 2, stickWidth), angle, 0, 360, 1, polygon);
                cv::fillConvexPoly(input, polygon, cvColors[limbKeypointsId.second]);
            }
        }
    }
}

cv::Mat raw_to_opencv(const xv::GrayScaleImage &stereo, const std::vector<xv::Object> &objects, bool flip)
{
    int width = stereo.width;
    int height = stereo.height;

    cv::Mat out = cv::Mat::zeros(height, width, CV_8UC1);

    if( stereo.data != nullptr ){
        std::memcpy(out.data, stereo.data.get(), static_cast<size_t>(width * height));
    }

    cv::cvtColor(out, out, cv::COLOR_GRAY2BGR);

    display_objects( out, objects, flip );

    return out;
}

cv::Mat raw_to_opencv(std::shared_ptr<const xv::ColorImage> rgb, const std::vector<xv::Object> &objects, bool flip )
{
    cv::Mat img;
    switch (rgb->codec) {
    case xv::ColorImage::Codec::YUV420p: {
        img = cv::Mat::zeros(rgb->height, rgb->width, CV_8UC3);
        auto raw = rgb->data.get();
        auto rawImg = cv::Mat(rgb->height * 3 / 2, rgb->width, CV_8UC1, const_cast<unsigned char*>(raw));
        cv::cvtColor(rawImg, img, cv::COLOR_YUV2BGR_I420);
        break;
    }
    case xv::ColorImage::Codec::YUYV: {
        img = cv::Mat::zeros(rgb->height, rgb->width, CV_8UC3);
        auto raw = rgb->data.get();
        auto rawImg = cv::Mat(rgb->height, rgb->width, CV_8UC2, const_cast<unsigned char*>(raw));
        cv::cvtColor(rawImg, img, cv::COLOR_YUV2BGR_YUYV);
        break;
    }
    case xv::ColorImage::Codec::JPEG: {
        cv::Mat raw(1, rgb->width*rgb->height, CV_8UC1, const_cast<unsigned char*>(rgb->data.get()));
        img = cv::imdecode(raw, cv::IMREAD_COLOR);
        break;
    }
    }

    display_objects( img, objects, flip );

    return img;
}



cv::Mat raw_to_opencv(std::shared_ptr<const xv::DepthImage> tof, const std::vector<xv::Object> &objects, bool flip)
{
    cv::Mat out;
    out = cv::Mat::zeros(tof->height, tof->width, CV_8UC3);
    float dmax = 7.5;
    const auto tmp_d = reinterpret_cast<float const*>(tof->data.get());
    for (unsigned int i=0; i< tof->height*tof->width; i++) {
        const auto &d = tmp_d[i];
        unsigned int u = static_cast<unsigned int>( std::max(0.0f, std::min(255.0f,  d * 255.0f / dmax )));
        const auto &cc = colors.at(u);
        out.at<cv::Vec3b>( i/tof->width, i%tof->width ) = cv::Vec3b(cc.at(2), cc.at(1), cc.at(0) );
    }

#if 0
    case XSlam::tof::Depth_16:{
        out = cv::Mat::zeros(tof->height, tof->width, CV_8UC3);
        float dmax = 7.5;
        const auto tmp_d = reinterpret_cast<short const*>(tof->data.get());
        for (unsigned int i=0; i< tof->height*tof->width; i++) {
            const auto &d = tmp_d[i];
            unsigned int u = static_cast<unsigned int>( std::max(0.0f, std::min(255.0f,  d * 255.0f / dmax )));
            const auto &cc = colors.at(u);
            out.at<cv::Vec3b>( i/tof->width, i%tof->width ) = cv::Vec3b(cc.at(2), cc.at(1), cc.at(0) );
        }
        break;
    }
    case XSlam::tof::Raw: {
        out = cv::Mat::zeros(tof->height, tof->width, CV_8UC3);
        auto raw = tof->data.get();
        // as cvtColor should not change the input data, we can const_cast here ...
        auto rawImg = cv::Mat(tof->height, tof->width, CV_8UC2, const_cast<unsigned char*>(raw));
        cv::cvtColor(rawImg, out, cv::COLOR_YUV2BGR_UYVY);
        break;
    }
    case XSlam::tof::IR: {
        out = cv::Mat::zeros(tof->height, tof->width, CV_8UC3);

        float dmax = 2191/4;

        auto tmp_d = reinterpret_cast<unsigned short const*>(tof->data.get());

        for (unsigned int i=0; i< tof->height*tof->width; i++) {

            unsigned short d = tmp_d[i];

            unsigned int u = static_cast<unsigned int>( std::max(0.0f, std::min(255.0f,  d * 255.0f / dmax )));
            if( u < 15 )
                u = 0;
            const auto &cc = colors.at(u);
            out.at<cv::Vec3b>( i/tof->width, i%tof->width ) = cv::Vec3b(cc.at(2), cc.at(1),cc.at(0) );

            /*short u = tof->ir.get()[i];
            cvtof.at<cv::Vec3b>( i/tof->width, i%tof->width ) = cv::Vec3s(u*255/2191,u*255/2191,u*255/2191);*/
        }
        break;
    }
    default:{
        return cv::Mat();
    }
    }
#endif

    display_objects( out, objects, flip );

    return out;
}

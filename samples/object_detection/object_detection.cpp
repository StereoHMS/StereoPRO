#include <xv-sdk.h>

#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <thread>
#include <signal.h>


static std::vector<xv::Object> objects;

#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>
#include <atomic>
cv::Mat raw_to_opencv( std::shared_ptr<const xv::ColorImage> rgb, const std::vector<xv::Object> &objects, bool flip);
cv::Mat raw_to_opencv( std::shared_ptr<const xv::DepthImage> tof, const std::vector<xv::Object> &objects, bool flip);
cv::Mat raw_to_opencv( std::shared_ptr<const xv::GrayScaleImage> tof_ir, const std::vector<xv::Object> &objects, bool flip);
cv::Mat raw_to_opencv( const xv::GrayScaleImage &stereo, const std::vector<xv::Object> &objects, bool flip);

std::atomic_bool s_newStereoFrame(false);
std::shared_ptr<const xv::FisheyeImages> s_stereo = nullptr;
std::atomic_bool s_newRgbFrame(false);
std::shared_ptr<const xv::ColorImage> s_rgb = nullptr;
std::atomic_bool s_newTofFrame(false);
std::shared_ptr<const xv::DepthImage> s_tof = nullptr;

void display( xv::ObjectDetector::Source cnn_source );

#endif

bool s_stop = false;

void signal_handler(int /*sig*/)
{
    s_stop = true;
}

bool s_flipRgb = false;
bool s_flipStereo = false;
bool s_flipTof = false;

#include "fps_count.hpp"

std::ostream& operator<<(std::ostream& o, const xv::Object & obj)
{
    o << " " << obj.typeID << " (" << obj.type << ") " << (obj.shape==xv::Object::Shape::BoundingBox?"bounding box":"human") << " [" << obj.x << "," << obj.y << "," << obj.width << "x" << obj.height << "]";
    return o;
}


int main( int argc, char* argv[] ) try
{
    auto devices = xv::getDevices(10.);

    if (devices.empty()) {
        std::cout << "Timeout: no device found" << std::endl;
        return EXIT_FAILURE;
    }

    auto device = devices.begin()->second;

    if( !device->objectDetector() ){
        std::cout << "Object detector not available" << std::endl;
        return EXIT_FAILURE;
    }


    xv::ObjectDetector::Source cnn_source = xv::ObjectDetector::Source::LEFT;
    std::string cnn_model = "CNN_model_2085.bin";
    std::string cnn_descriptor = "CNN_descriptor_2085.txt";


    if( argc > 1 ){
        cnn_descriptor = argv[1];
    }

    std::cout << "SDK version: " << xv::version() << std::endl;

    signal(SIGINT, signal_handler);


    device->objectDetector()->registerCallback( [](std::vector<xv::Object> objs){
        objects = objs;
        std::cout << objects.size() << " objects" << std::endl;
        for(int i=0;i<objs.size();i++){
            std::cout << objs.at(i) << std::endl;
        }
    });

    //vsc.setObjectDetectionSource( cnn_source );
    //vsc.setObjectDetectionModel( cnn_model );

    //uvc->setStereoMode( XSlam::UVC::StereoMode::IMAGES );
    //device->setStereoMode( XSlam::UVC::StereoMode::IMAGES_AND_DATA );
    device->objectDetector()->setDescriptor( cnn_descriptor );

    cnn_source = device->objectDetector()->getSource();

    auto descriptor = device->objectDetector()->getDescriptor();
    s_flipRgb = descriptor.flipRgb;
    s_flipStereo = descriptor.flipStereo;
    s_flipTof = descriptor.flipTof;


    // Left or Right source

    if( cnn_source == xv::ObjectDetector::Source::LEFT ||  cnn_source == xv::ObjectDetector::Source::RIGHT ){

        device->fisheyeCameras()->registerCallback( [](xv::FisheyeImages const & stereo){
            static FpsCount fc;
            fc.tic();
            static int k = 0;
            if(k++%50==0){
                if( stereo.images.size() > 0 ){
                    const auto &img = stereo.images.cbegin();
                    std::cout << "STEREO " << img->width << "x" << img->height << "@" << std::round(fc.fps()) << "fps" << std::endl;
                }
            }
        });

#ifdef USE_OPENCV
        device->fisheyeCameras()->registerCallback( [](xv::FisheyeImages const & stereo){
            s_stereo = std::make_shared<xv::FisheyeImages>(stereo);
            s_newStereoFrame = true;
        });
#endif
    }

    // RGB source

    if( cnn_source == xv::ObjectDetector::Source::RGB ){

        device->colorCamera()->registerCallback( [](xv::ColorImage const & rgb){
            static FpsCount fc;
            fc.tic();
            static int k = 0;
            if(k++%50==0){
                std::cout << "RGB " << rgb.width << "x" << rgb.height << "@" << std::round(fc.fps()) << "fps" << std::endl;
            }
        });

#ifdef USE_OPENCV
        device->colorCamera()->registerCallback( [](xv::ColorImage const & rgb){
            s_rgb = std::make_shared<xv::ColorImage>(rgb);
            s_newRgbFrame = true;
        });
#endif
    }

    if( cnn_source == xv::ObjectDetector::Source::TOF ){

        device->tofCamera()->registerCallback([](xv::DepthImage const & tof){
            static FpsCount fc;
            fc.tic();
            static int k = 0;
            if(k++%50==0){
                std::cout << "TOF " << tof.width << "x" << tof.height << "@" << std::round(fc.fps()) << "fps" << std::endl;
            }
        });

#ifdef USE_OPENCV
        device->tofCamera()->registerCallback([](xv::DepthImage const & tof){
            s_tof = std::make_shared<xv::DepthImage>(tof);
            s_newTofFrame = true;
        });
#endif
    }


#ifdef USE_OPENCV
    //Display in thread to not slow down callback
    s_stop = false;
    std::thread t(display, cnn_source);
#endif


    while( !s_stop ){
        std::this_thread::sleep_for( std::chrono::milliseconds(100) );
    }

#ifdef USE_OPENCV
    s_stop = true;
    if (t.joinable()) {
        t.join();
    }
#endif

    return EXIT_SUCCESS;
}
catch( const std::exception &e){
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}








#ifdef USE_OPENCV
void display( xv::ObjectDetector::Source cnn_source )
{
    std::string name;
    switch( cnn_source ){
    case xv::ObjectDetector::Source::LEFT: name = "Left"; break;
    case xv::ObjectDetector::Source::RIGHT: name = "Right"; break;
    case xv::ObjectDetector::Source::RGB: name = "Rgb"; break;
    case xv::ObjectDetector::Source::TOF: name = "Tof"; break;
    }

    cv::namedWindow(name);
    cv::moveWindow(name, 20, 20);
    cv::waitKey(1);

    while( !s_stop ){
        switch( cnn_source ){
        case xv::ObjectDetector::Source::LEFT: {
            if (s_newStereoFrame) {
                s_newStereoFrame = false;
                auto stereo = s_stereo;
                if (stereo.get() != nullptr && stereo->images.size()>0 ) {
                    const auto &tmp = stereo->images.at(0);
                    auto img = raw_to_opencv(tmp, objects, s_flipStereo);
                    cv::imshow(name, img);
                }
            }
            break;
        }
        case xv::ObjectDetector::Source::RIGHT: {
            if (s_newStereoFrame) {
                s_newStereoFrame = false;
                auto stereo = s_stereo;
                if (stereo.get() != nullptr && stereo->images.size()>1 ) {
                    const auto &tmp = stereo->images.at(1);
                    auto img = raw_to_opencv(tmp, objects, s_flipStereo);
                    cv::imshow(name, img);
                }
            }
            break;
        }
        case xv::ObjectDetector::Source::RGB: {
            if (s_newRgbFrame) {
                s_newRgbFrame = false;
                auto rgb = s_rgb;
                if (rgb.get() != nullptr) {
                    auto img = raw_to_opencv(rgb, objects, s_flipRgb);
                    if( !img.empty() ){
                        cv::imshow(name, img);
                    }
                }
            }
            break;
        }
        case xv::ObjectDetector::Source::TOF: {
            if (s_newTofFrame) {
                s_newTofFrame = false;
                auto tof = s_tof;
                if (tof.get() != nullptr) {
                    auto img = raw_to_opencv(tof, objects, s_flipTof);
                    if( !img.empty() ){
                        cv::imshow(name, img);
                    }
                }
            }
            break;
        }
        }

        int key = cv::waitKey(1);
        if( key == 'q' ){
            s_stop = true;
        }
    }
}
#endif

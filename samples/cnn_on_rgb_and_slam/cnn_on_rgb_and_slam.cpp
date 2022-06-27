#include <xv-sdk.h>

#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <thread>
#include <signal.h>

static std::vector<xv::Object> objects;


// #define USE_CNN_RAW_DATA
#ifdef USE_CNN_RAW_DATA
bool valid(float in)
{
    if(std::isnan(in)) return false;
    if(in < 0 || in > 1) return false;
    return true;
}

std::vector<xv::Object> parse_tensorflow(std::shared_ptr<xv::CnnRawWrapper> cnn_raw, int image_width=-1, int image_height=-1)
{
    static std::vector<std::string> CLASSES_MAP = {"background",
        "aeroplane", "bicycle", "bird", "boat",
        "bottle", "bus", "car", "cat", "chair",
        "cow", "diningtable", "dog", "horse",
        "motorbike", "person", "pottedplant",
        "sheep", "sofa", "train", "tvmonitor"};
    std::shared_ptr<float> cnn_out_raw = cnn_raw->raw_data;
    int max_elements = cnn_raw->raw_data_length / 7;
    
    std::vector<xv::Object> detections;

    int i = 0;
    while (/*valid*/ float(cnn_out_raw.get()[i * 7]) != -1.0f && i + 1 < max_elements){
        xv::Object d;
        d.shape = xv::Object::Shape::BoundingBox;

        d.typeID = int(cnn_out_raw.get()[i * 7 + 1]);

        if( d.typeID >=0 && d.typeID < CLASSES_MAP.size() ){

            d.type = CLASSES_MAP.at( d.typeID );

            bool ok = true;

            for(int j=2;j<7;j++){
                if( !valid( cnn_out_raw.get()[i * 7 + j] ) ){
                    ok = false;
                    break;
                }
            }

            if( ok ){
                d.confidence = float(cnn_out_raw.get()[i * 7 + 2]);
                d.x = float(cnn_out_raw.get()[i * 7 + 3]);
                d.y = float(cnn_out_raw.get()[i * 7 + 4]);
                d.width = float(cnn_out_raw.get()[i * 7 + 5]) - d.x;
                d.height = float(cnn_out_raw.get()[i * 7 + 6]) - d.y;
#ifdef USE_OPENCV
                d.x *= float(image_width);
                d.y *= float(image_height);
                d.width *= float(image_width);
                d.height *= float(image_height);
#endif
                if( d.confidence > 0.5 ){
                    detections.push_back(d);
                }
            }
        }
        i++;
    }
    return detections;
}
#endif


#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>
#include <atomic>
cv::Mat raw_to_opencv(std::shared_ptr<const xv::ColorImage> rgb, const std::vector<xv::Object> &objects, bool flip = false);
cv::Mat raw_to_opencv(std::shared_ptr<const xv::FisheyeImages> stereo);
std::atomic_bool s_newRgbFrame(false);
std::shared_ptr<const xv::ColorImage> s_rgb = nullptr;
std::atomic_bool s_newStereoFrame(false);
std::shared_ptr<const xv::FisheyeImages> s_stereo = nullptr;
void display();
#endif

bool s_stop = false;
void signal_handler(int /*sig*/)
{
    s_stop = true;
}

bool s_flipRgb = false;
bool s_flipStereo = false;

#include "fps_count.hpp"

int main( int argc, char* argv[] ) try
{
    signal(SIGINT, signal_handler);

    std::string cnn_model = "";
    std::string cnn_descriptor = "";

    if( argc > 1 ){
        cnn_descriptor = argv[1];
    }else{
        std::cerr << "Usage:" << std::endl << argv[0] << " <cnn_descriptor_file> [cnn_model_file]" << std::endl;
        return EXIT_FAILURE;
    }

    if( argc > 2 ){
        cnn_model = argv[2];
    }

    auto devices = xv::getDevices(10., std::string(), &s_stop);

    if (devices.empty()) {
        std::cout << "Timeout: no device found\n";
        return EXIT_FAILURE;
    }


    auto device = devices.begin()->second;

    if (!device->colorCamera()) {
        std::cerr << "No RGB camera" << std::endl;
        return EXIT_FAILURE;
    }
    if (!device->slam()) {
        std::cerr << "No SLAM" << std::endl;
        return EXIT_FAILURE;
    }
    if (!device->objectDetector()) {
        std::cerr << "No object detector" << std::endl;
        return EXIT_FAILURE;
    }

    //vsc->setRgbResolution( XSlam::VSC::RgbResolution::RGB_1920x1080 );
    device->colorCamera()->setResolution( xv::ColorCamera::Resolution::RGB_1280x720 );

    device->slam()->registerCallback( [](const xv::Pose& pose){
        static FpsCount fc;
        fc.tic();
        static int k=0;
        if(k++%100==0){
            std::cout << "slam-pose" << "@" << std::round(fc.fps()) << "fps"
            << " p=(" << pose.x() << " " << pose.y() << " " << pose.z()
            //<< " ), r=(" << pose->pitch << " " << pose->yaw << " " << pose->roll << " )"
            << ", Confidence= " << (int)pose.confidence()
            //<< ", Status= " << pose->status
            << std::endl;
        }
    });

#ifdef USE_CNN_RAW_DATA
    device->objectDetector()->registerCnnRawCallback( [](std::shared_ptr<xv::CnnRawWrapper> cnn_raw){
#ifdef USE_OPENCV
        if(s_rgb.get() != nullptr)
            objects = parse_tensorflow(cnn_raw, s_rgb->width, s_rgb->height);
#else
        objects = parse_tensorflow(cnn_raw);
#endif
    });
#else
    device->objectDetector()->registerCallback( [](std::vector<xv::Object> objs){
        objects = objs;
    });
#endif

    device->objectDetector()->setDescriptor( cnn_descriptor );

    auto descriptor = device->objectDetector()->getDescriptor();
    s_flipRgb = descriptor.flipRgb;
    //s_flipStereo = descriptor.flip_stereo;

    if( !cnn_model.empty() ){
        device->objectDetector()->setSource( xv::ObjectDetector::Source::RGB );
        device->objectDetector()->setModel( cnn_model );
    }

    device->colorCamera()->registerCallback( [](xv::ColorImage const & rgb){
        static FpsCount fc;
        fc.tic();
        static int k = 0;
        if(k++%50==0){
            std::cout << "RGB " << rgb.width << "x" << rgb.height << "@" << std::round(fc.fps()) << "fps" << std::endl;
        }
    });

    device->colorCamera()->start();
    device->slam()->start();

#ifdef USE_OPENCV

    device->fisheyeCameras()->registerCallback( [](xv::FisheyeImages const & stereo){
        s_stereo = std::make_shared<xv::FisheyeImages>(stereo);
        s_newStereoFrame = true;
    });

    device->colorCamera()->registerCallback( [](xv::ColorImage const & rgb){
        s_rgb = std::make_shared<xv::ColorImage>(rgb);
        s_newRgbFrame = true;
    });

    //Display in thread to not slow down callback
    s_stop = false;
    std::thread t(display);

    // Wait for 'q' key
    while( !s_stop ){
        std::this_thread::sleep_for( std::chrono::milliseconds(100) );
    }
    if (t.joinable()) {
        t.join();
    }
#else
    while( !s_stop ){
        std::this_thread::sleep_for( std::chrono::milliseconds(100) );
    }
#endif

    //device->colorCamera()->stop();
    device->slam()->stop();

	return EXIT_SUCCESS;
}
catch( const std::exception &e){
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}








#ifdef USE_OPENCV
void display()
{
    std::string name = "Rgb";

    cv::namedWindow(name);
    //cv::moveWindow(name, 20, 20);
    cv::waitKey(1);


    //cv::namedWindow("left");
    //cv::waitKey(1);

    while( !s_stop ){
        if (s_newRgbFrame) {
            s_newRgbFrame = false;
            auto rgb = s_rgb;
            if (rgb.get() != nullptr) {
                auto img = raw_to_opencv(rgb, objects, s_flipRgb);
                if( !img.empty() ){
                    //cv::resize(img, img, cv::Size( img.cols * 0.5, img.rows * 0.5) );
                    cv::imshow(name, img);
                }
            }
        }
        if (s_newStereoFrame) {
            s_newStereoFrame = false;
            auto stereo = s_stereo;
            if (stereo.get() != nullptr) {
                auto img = raw_to_opencv(stereo);
                if( !img.empty() ){
                    //cv::resize(img, img, cv::Size( img.cols * 0.5, img.rows * 0.5) );
                    cv::imshow("left", img);
                }
            }
        }
        int key = cv::waitKey(1);
        if( key == 'q' ){
            s_stop = true;
        }
    }
}
#endif

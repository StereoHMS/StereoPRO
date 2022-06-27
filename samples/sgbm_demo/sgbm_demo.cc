#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <xv-sdk.h>
#include <string>
#include <memory>
#include <iostream>
#include <mutex>
#include <thread>
#include <fstream>
#include <signal.h>
#include <chrono>


#include "xv-sdk-ex.h"

//enable fillholes
// #define USE_FILLHOLES

using std::cerr;
using std::cin;
using std::cout;
using std::endl;
using std::ifstream;
using std::mutex;
using std::ofstream;
using std::string;
using std::thread;
using std::vector;

using std::shared_ptr;
using std::make_shared;


using cv::Mat;
using cv::Scalar;
using cv::Point;
using cv::Size;

using cv::imshow;
using cv::putText;
using cv::setMouseCallback;
using cv::namedWindow;
using cv::medianBlur;


Mat raw_to_opencv(shared_ptr<const xv::SgbmImage> sbgm_image);
std::pair<cv::Mat,cv::Mat> raw_to_opencv(std::shared_ptr<const xv::FisheyeImages> stereo);



string json = "";
#ifdef USE_FILLHOLES
int d1 = 200;
int d2 = 10000;
#endif

static struct xv::sgbm_config global_config = {
    1, //enable_dewarp
    1.0, //dewarp_zoom_factor
    0, //enable_disparity
    1, //enable_depth
    0, //enable_point_cloud
    0.11285, //baseline
    69, //fov
    255, //disparity_confidence_threshold
    {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, //homography
    1, //enable_gamma
    2.2, //gamma_value
    0, //enable_gaussian
    0, //mode
    65535, //max_distance
    100, //min_distance
};

#ifdef USE_FILLHOLES
/**
 * @brief Fill the holes
 * @param inputImage: depth input image (uint16_t values)
 * @param isHole: function to determine if a value of the image correspond to a hole
 * @param holeSize: size of the biggest holes to fill (in pxl)
 * @return the input image with holes filled
 */
template <class InputImage = xv::SgbmImage>
InputImage fillHoles(InputImage const& inputImage, std::function<bool(uint16_t)> isHole, float holeSize=1) {

    // pointer on input data (assume input data is float)
    const auto inputImagePtr = reinterpret_cast<uint16_t const*>(inputImage.data.get());

    // binary image of holes
    cv::Mat holesBinaryImage = cv::Mat::zeros(inputImage.height, inputImage.width, CV_16UC1);
    for (std::size_t i=0; i< inputImage.height*inputImage.width; i++) {
        const auto &d = inputImagePtr[i];
        // cout<<"d "<<d<<endl;
        if( isHole(d)) {
            holesBinaryImage.at<uint16_t>(i / inputImage.width, i % inputImage.width) = 0;
        } else {
            holesBinaryImage.at<uint16_t>(i / inputImage.width, i % inputImage.width) = 65535;
        }
    }

    // Create structuring element
    int elementSize = (holeSize+1)/2;
    cv::Mat element = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(2 * elementSize + 1,
                         2 * elementSize + 1),
                cv::Point(elementSize, elementSize));
    cv::Mat holesFilledBinaryImage;

    // Closing to fill the holes
    cv::morphologyEx(holesBinaryImage, holesFilledBinaryImage,
                     cv::MORPH_CLOSE, element,
                     cv::Point(-1, -1), 2);

    cv::Mat onlyFilledHolesImage;
    // extract only the filled holes in a binary image
    {
        cv::Mat tmp;
        cv::bitwise_not(holesBinaryImage, tmp);
        cv::bitwise_and(tmp, holesFilledBinaryImage, onlyFilledHolesImage);
    }

    auto median = [] (std::vector<uint16_t> &v)
    {
        size_t n = v.size() / 2;
        std::nth_element(v.begin(), v.begin()+n, v.end());
        return v[n];
    };

    // copy the input image to output image
    InputImage outputImage = inputImage;
    std::shared_ptr<std::uint8_t> data(new std::uint8_t[inputImage.dataSize], std::default_delete<std::uint8_t[]>());
    std::memcpy(data.get(), inputImage.data.get(), inputImage.dataSize);

    const auto outputImagePtr = reinterpret_cast<uint16_t*>(data.get());

    for (std::size_t c=0; c< inputImage.height*inputImage.width; c++) {
        auto i = c / inputImage.width;
        auto j = c % inputImage.width;

        // if it is a filled hole, compute the median to fill the depth image data
        if( onlyFilledHolesImage.at<uint16_t>(i, j)>0 ) {
            std::vector<uint16_t> pixels;
            // compute the median based on non-hole input pixels
            for (int ii=-holeSize; ii<holeSize; ++ii) {
                if (i+ii<0 || i+ii>inputImage.height) continue;
                for (int jj=-holeSize; jj<holeSize; ++jj) {
                    if (j+jj<0 || j+jj>inputImage.width) continue;
                    uint16_t v = inputImagePtr[(i+ii)*inputImage.width+(j+jj)];
                    if (!isHole(v)) {
                        pixels.push_back(v);
                    }
                }
            }
            if (!pixels.empty())
                outputImagePtr[c] = median(pixels);
        }
    }
    outputImage.data = data;

    return outputImage;
}
#endif


void dump(const char *name, const void *data, int len)
{
    auto f = std::ofstream(name);
    if (f.is_open()) {
        f.write((const char *)data, len);
    } else {
        std::cerr << "cannot write to " << name << std::endl;
    }
}


int s_x=-1,s_y=-1;
static int k = 0;
void onMouse(int event,int x,int y,int flag,void* _user_data)
{
    switch (event)
    {
    case cv::EVENT_MOUSEMOVE:
        s_x = x;
        s_y = y;
        break;
    default:
        break;
    }
}


bool s_stop = false;

shared_ptr<const xv::FisheyeImages> s_ptr_stereo = nullptr;
shared_ptr<const xv::SgbmImage> s_ptr_sgbm = nullptr;

mutex s_mtx_stereo;
mutex s_mtx_sgbm;

void Display()
{
    namedWindow("Depth");
    namedWindow("Left");
    namedWindow("Right");

    setMouseCallback("Depth",onMouse,0);

    while (!s_stop)
    {
        shared_ptr<const xv::FisheyeImages> ptr_stereo = nullptr;
        shared_ptr<const xv::SgbmImage> ptr_sgbm = nullptr;

        s_mtx_stereo.lock();
        ptr_stereo = s_ptr_stereo;
        s_mtx_stereo.unlock();

        if(ptr_stereo)
        {
            auto imgs = raw_to_opencv(ptr_stereo);
            cv::imshow("Left", imgs.first);
            cv::imshow("Right", imgs.second);
            
        }
        s_mtx_sgbm.lock();
        ptr_sgbm = s_ptr_sgbm;
        s_mtx_sgbm.unlock();


         if(ptr_sgbm)
        {
            if(ptr_sgbm->type == xv::SgbmImage::Type::Depth)
            {   
            #ifdef USE_FILLHOLES
                {
                    int hole = 1;
                    xv::SgbmImage img = fillHoles(*ptr_sgbm.get(), [](uint16_t d){return d < d1  || d > d2; }, hole);
                    ptr_sgbm = make_shared<xv::SgbmImage>(img);
                }
            #endif
                cv::Mat img = raw_to_opencv(ptr_sgbm);               
                char text[256];
                uint16_t* p16 = (uint16_t*)ptr_sgbm->data.get();

                if( s_x!=-1 && ( s_x > 0 && s_x < ptr_sgbm->width && s_y > 0 && s_y < ptr_sgbm->height ) )
                {
                    int width = ptr_sgbm->width;
                    int height = ptr_sgbm->height;
                    if(p16[s_x+s_y*width]==0)
                    {
                        if(k++%20==0)
                        {
                            memset(text,0,256);
                            sprintf(text,"x:%d,y:%d depth:%d mm",s_x,s_y,p16[s_x+s_y*width]);
                        } 
                    }
                    else
                    {
                        memset(text,0,256);
                        sprintf(text,"x:%d,y:%d depth:%d mm",s_x,s_y,p16[s_x+s_y*width]);
                    }
                        
                    putText(img,text,Point(25,height-30),cv::FONT_HERSHEY_TRIPLEX,0.5,Scalar(0,0,255));
                }
                imshow("Depth",img);
            }
            
        }
        cv::waitKey(1);
    }
    

}

std::shared_ptr<xv::PointCloud> s_ptr_pointcloud;

mutex s_mtx_pointcloud;
ofstream ofs;

void printPointCloud()
{
    char buff[128];
    auto time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    struct tm *ptm = localtime(&time);
    sprintf(buff, "pointcloud-%d-%d-%d-%d-%d.txt", ptm->tm_year+1900,ptm->tm_mon+1,ptm->tm_mday,ptm->tm_hour,ptm->tm_min);
    ofs.open(buff, std::ios::out);
    while (!s_stop)
    {
        std::shared_ptr<xv::PointCloud> pointcloud = nullptr;
        s_mtx_pointcloud.lock();
        pointcloud = s_ptr_pointcloud;
        s_mtx_pointcloud.unlock();
        if(pointcloud)
        {
            auto points = pointcloud->points;
            for (auto iter = points.begin(); iter != points.end();iter++)
            {
                auto point = *iter;
                char buff[128];
                sprintf(buff, "x:%f ,y:%f ,z:%f\n", point[0], point[1], point[2]);
                ofs << buff;
            }
        }

    }
}

void handle(int sig)
{
    cout << "kill program" << endl;
    s_stop = true;
    ofs.close();
    signal(sig, SIG_DFL);
    raise(sig);
}

long findInvalid(const xv::SgbmImage& image)
{
    uint16_t *p16 = (uint16_t *)image.data.get();
    vector<uint16_t> data(p16,p16+image.width*image.height);
    return count_if(data.begin(), data.end(), [](uint16_t v){
        return ( v==65535 || v==0 );
    });
}

int main(int argc ,char** argv)
{
    auto devices = xv::getDevices(10., json);
    if(devices.empty())
    {
        cerr<<"Timeout: no device found"<<endl;
    }

    auto device = devices.begin()->second;
#ifdef USE_FILLHOLES
    d1 = atoi(argv[1]);
    d2 = atoi(argv[2]);
#endif

    xv::setLogLevel(xv::LogLevel::debug);
    if(device->fisheyeCameras())
    {
        device->fisheyeCameras()->registerCallback([](xv::FisheyeImages const & stereo){
            s_mtx_stereo.lock();
            s_ptr_stereo = make_shared<const xv::FisheyeImages>(stereo);
            s_mtx_stereo.unlock();        
        });
        std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->setResolutionMode(xv::FisheyeCamerasEx::ResolutionMode::HIGH);
        //         std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->setResolutionMode(xv::FisheyeCamerasEx::ResolutionMode::MEDIUM);
        device->fisheyeCameras()->start();
    }
    
    if(device->sgbmCamera())
    {
        device->sgbmCamera()->registerCallback([=](const xv::SgbmImage& sgbm_image){
            if(sgbm_image.type == xv::SgbmImage::Type::Depth)
            {  
                s_mtx_sgbm.lock();
                s_ptr_sgbm = make_shared<const xv::SgbmImage>(sgbm_image);
                s_mtx_sgbm.unlock();
                long invalidNum = findInvalid(sgbm_image);
                cout << "invalid : " << (100.0 * invalidNum) / (sgbm_image.width * sgbm_image.height) << "%" << endl;
                auto pointcloud = device->sgbmCamera()->depthImageToPointCloud(sgbm_image);
                s_mtx_pointcloud.lock();
                s_ptr_pointcloud = pointcloud;
                s_mtx_pointcloud.unlock();
            }
        });
        device->sgbmCamera()->start(global_config);
    }
    signal(SIGINT,handle);
    thread t(Display);
    thread t2(printPointCloud);

    // cin.get();
    int count = 0;
    while (cin.get())
    {
        device->sgbmCamera()->stop();
        device->fisheyeCameras()->stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        auto mode = count++ % 2 ? xv::SgbmCamera::Resolution::SGBM_640x480 : xv::SgbmCamera::Resolution::SGBM_1280x720;
//        std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->setResolutionMode(mode);
        device->sgbmCamera()->setSgbmResolution(mode);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        device->fisheyeCameras()->start();
        device->sgbmCamera()->start(global_config);
    }
    t.join();
    t2.join();
    device->fisheyeCameras()->stop();
    // device->sgbmCamera()->stop();

    return 0;
}

#define _USE_MATH_DEFINES

#include <cstdlib>
#include <thread>
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <cmath>
#include <mutex>
#include <signal.h>

#include <xv-sdk.h>
#include "colors.h"
#include "fps_count.hpp"


bool s_stop = false;

#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>

cv::Mat raw_to_opencv( std::shared_ptr<const xv::DepthColorImage> rgbd);


std::map<std::string, std::shared_ptr<const xv::DepthColorImage>> s_depthColor;

std::mutex s_mtx;

void display() {
    cv::waitKey(1);

    while( !s_stop ){
        {
            std::lock_guard<std::mutex> lock(s_mtx);
            for (auto& pair : s_depthColor) {
                auto depthColor = pair.second;
                if (depthColor) {
                    cv::Mat img = raw_to_opencv(depthColor);
                    if (img.rows>0 && img.cols>0)
                        cv::imshow("RGBD (depth) " + pair.first, img);
                }
            }
        }
        cv::waitKey(10);
    }
}
#endif


std::string timeShowStr(double hostTimestamp) {
    char s[1024];
    double now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()*1e-6;
    std::sprintf(s, " (host=%.4f now=%.4f delay=%.4f) ", hostTimestamp, now, now-hostTimestamp);
    return std::string(s);
}

int main( int argc, char* argv[] ) try
{
    std::cout << "xvsdk version: " << xv::version() << std::endl;

    //xv::setLogLevel(xv::LogLevel(0));
    xv::setLogLevel(xv::LogLevel::debug);

    std::string json = "";
    if (argc == 2) {
        std::ifstream ifs( argv[1] );
        if( !ifs.is_open() ){
            std::cerr << "Failed to open: " << argv[1] << std::endl;
            return -1;
        }

        std::stringstream fbuf;
        fbuf << ifs.rdbuf();
        json = fbuf.str();
    }

    /// Multi-device support
    xv::registerPlugEventCallback([&](std::shared_ptr<xv::Device> device, xv::PlugEventType type)
    {
      const auto deviceId = device->id();

      if (type == xv::PlugEventType::Unplug) {
          std::cout << " == Device left (" << deviceId << ") ==" << std::endl;
          return;
      }

      if (type == xv::PlugEventType::Plugin) {
          std::lock_guard<std::mutex> lock(s_mtx);
          if (s_depthColor.find(deviceId) != s_depthColor.end()) {
              std::cout << " == Device replugged (" << deviceId << ") ==" << std::endl;
              return;
          }
      }

      std::cout << " == New device plugged (" << deviceId << ") ==" << std::endl;

      if (device->colorCamera()) {
          device->colorCamera()->setResolution(xv::ColorCamera::Resolution::RGB_1280x720);
          device->colorCamera()->start();
      } else {
          std::cout << "No RGB camera.\n";
      }

      if (device->tofCamera()) {

          device->tofCamera()->start();

          device->tofCamera()->registerColorDepthImageCallback([deviceId](const xv::DepthColorImage& depthColor){
              static FpsCount fc;
              fc.tic();
              static int k = 0;
              if(k++%15==0){
                  std::cout << "RGBD tof " << timeShowStr(depthColor.hostTimestamp)
                            << depthColor.width << "x" << depthColor.height << "@"
                            << std::round(fc.fps()) << "fps (" << deviceId << ")"
                            << std::endl;
              }
          });
      }

      std::cout << " == Device initialized (" << device->id() << ") ==" << std::endl;

#ifdef USE_OPENCV
      /// Display in thread to not slow down callbacks
      if (device->tofCamera()) {
          device->tofCamera()->registerColorDepthImageCallback([&,deviceId](const xv::DepthColorImage& depthColor){
              std::lock_guard<std::mutex> lock(s_mtx);
              s_depthColor[deviceId] = std::make_shared<xv::DepthColorImage>(depthColor);
          });
      }
#endif
    }, json);


#ifdef USE_OPENCV
    std::thread t(display);
#endif

    std::cout << " ################## " << std::endl;
    std::cout << "        Start       " << std::endl;
    std::cout << " ################## " << std::endl;

    std::cerr << "ENTER to stop" << std::endl;
    std::cin.get();

    s_stop = true;

    std::cout << " ################## " << std::endl;
    std::cout << "        Stop        " << std::endl;
    std::cout << " ################## " << std::endl;


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

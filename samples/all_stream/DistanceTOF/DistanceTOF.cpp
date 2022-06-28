#define _USE_MATH_DEFINES

#include <cstdlib>
#include <thread>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <mutex>
#include <signal.h>
#include <cstring>
#include <sys/timeb.h>
#include <xv-sdk.h>
#include "colors.h"
#include <opencv2/opencv.hpp>
#include "fps_count.hpp"

#define TOF_WINDOW_NAME "TOF"
#define USE_EX
using namespace std;

static std::map<std::string, int> enableDevMap;

cv::Mat raw_to_opencv_tof(std::shared_ptr<const xv::DepthImage> tof)
{
	cv::Mat out;
	if (tof->height > 0 && tof->width > 0) {
		out = cv::Mat::zeros(tof->height, tof->width, CV_8UC3);

		if (tof->type == xv::DepthImage::Type::Depth_32) {
			float dmax = 7.5;
			const auto tmp_d = reinterpret_cast<float const*>(tof->data.get());
			for (unsigned int i = 0; i < tof->height*tof->width; i++) {
				const auto &d = tmp_d[i];
				if (d < 0.01 || d > 9.9) {
					out.at<cv::Vec3b>(i / tof->width, i % tof->width) = 0;
				}
				else {
					unsigned int u = static_cast<unsigned int>(std::max(0.0f, std::min(255.0f, d * 255.0f / dmax)));
					const auto &cc = colors.at(u);
					out.at<cv::Vec3b>(i / tof->width, i%tof->width) = cv::Vec3b(cc.at(2), cc.at(1), cc.at(0));
				}
			}
		}
		else if (tof->type == xv::DepthImage::Type::Depth_16) {
			// #define DEPTH_RANGE_20M_SF       7494  //7.494m
			// #define DEPTH_RANGE_60M_SF       2494  //2.494m
			// #define DEPTH_RANGE_100M_SF      1498  //1.498m
			// #define DEPTH_RANGE_120M_SF      1249  //1.249m
			float dmax = 2494.0; // maybe 7494,2494,1498,1249 see mode_manage.h in sony toflib
			const auto tmp_d = reinterpret_cast<int16_t const*>(tof->data.get());
			for (unsigned int i = 0; i < tof->height*tof->width; i++) {
				const auto &d = tmp_d[i];
				unsigned int u = static_cast<unsigned int>(std::max(0.0f, std::min(255.0f, d * 255.0f / dmax)));
				const auto &cc = colors.at(u);
				out.at<cv::Vec3b>(i / tof->width, i%tof->width) = cv::Vec3b(cc.at(2), cc.at(1), cc.at(0));
			}
		}


	}

	return out;
}

#ifdef USE_EX
#include "xv-sdk-ex.h"
#endif
#ifdef USE_PRIVATE
#include "xv-sdk-private.h"
#endif

std::shared_ptr<const xv::DepthImage> s_tof = nullptr;

std::mutex s_mtx_tof;
std::string timeShowStr(std::int64_t edgeTimestampUs, double hostTimestamp) {
	char s[1024];
	double now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()*1e-6;
	std::sprintf(s, " (device=%lld host=%.4f now=%.4f delay=%.4f) ", (long long)edgeTimestampUs, hostTimestamp, now, now - hostTimestamp);
	return std::string(s);
}
std::string timeShowStr(double hostTimestamp) {
	char s[1024];
	double now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()*1e-6;
	std::sprintf(s, " (host=%.4f now=%.4f delay=%.4f) ", hostTimestamp, now, now - hostTimestamp);
	return std::string(s);
}
void display() {
	while (true) {
		std::shared_ptr<const xv::DepthImage> tof = nullptr;

		if (enableDevMap["tof"]) {
			s_mtx_tof.lock();
			tof = s_tof;
			s_mtx_tof.unlock();
			if (tof) {

				const auto tmp_d = reinterpret_cast<int16_t const*>(tof->data.get());
				const int pixel_count = 100;
				const int radius = 5;
				unsigned int p1[pixel_count] = { 0 };
				int index_p1 = 0;
				for (int i = -radius; i < radius; i++)
					for (int j = -radius; j < radius; j++) {
						int row = tof->height / 2 + i;
						int col = tof->width / 2 + j;
						p1[index_p1] = tmp_d[row * tof->width + col];
						index_p1++;
					}
				std::sort(p1, p1 + pixel_count);// min 2 max
				unsigned int sum_p1 = 0;
				for (int i = 0; i < pixel_count; i++) {
					sum_p1 += p1[i];
				}
				unsigned int average = sum_p1 / pixel_count;
				std::cout << "  average : " << average << "  min : " << p1[0]
					<< "  max : " << p1[pixel_count - 1] << "  median  : " << p1[pixel_count / 2 - 1] << " max min diff value  :"
					<< p1[pixel_count - 1] - p1[0] << std::endl;

				cv::Mat img = raw_to_opencv_tof(tof);
				cv::putText(img, "center avg depth : " + std::to_string(average), cv::Point2i(img.cols/2, img.rows-20), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255, 255, 255), 1, 2, false);
				cv::circle(img, cv::Point(640 / 2, 480 / 2), 1, cv::Scalar(255, 255, 255), 0);

				if (img.rows > 0 && img.cols > 0)
				{
					cv::namedWindow(TOF_WINDOW_NAME, cv::WINDOW_NORMAL);
					cv::imshow(TOF_WINDOW_NAME, img);
				}
			}

		}
		cv::waitKey(1);
	}
}


int main(int argc, char* argv[]) try
{
	std::cout << "xvsdk version: " << xv::version() << std::endl;

	xv::setLogLevel(xv::LogLevel::debug);

	std::string json = "";

	enableDevMap["tof"] = true;
	enableDevMap["tof_mode"] = 3;
	auto devices = xv::getDevices(10., json);

	std::ofstream ofs;
	if (devices.empty())
	{
		std::cout << "Timeout: no device found\n";
		return EXIT_FAILURE;
	}

	auto device = devices.begin()->second;

	std::cout << device->id() << std::endl;


	enableDevMap["tof"] &= device->tofCamera() != nullptr;
	if (enableDevMap["tof"]) {

		bool ret = device->tofCamera()->setLibWorkMode(static_cast<xv::TofCamera::SonyTofLibMode>(enableDevMap["tof_mode"]));
		if (!ret)
		{
			std::cout << "setLibWorkMode failed" << std::endl;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		device->tofCamera()->registerCallback([&](xv::DepthImage const & tof) {
			if (tof.type == xv::DepthImage::Type::Depth_16 || tof.type == xv::DepthImage::Type::Depth_32) {
				static FpsCount fc;
				fc.tic();
				static int k = 0;
				if (k++ % 15 == 0) {
					std::cout << "tof      " << timeShowStr(tof.edgeTimestampUs, tof.hostTimestamp)
						<< tof.width << "x" << tof.height << "@" << std::round(fc.fps()) << "fps" << std::endl;
				}
			}
		});
		device->tofCamera()->start();

	}



	if (enableDevMap["tof"]) {
		device->tofCamera()->registerCallback([](xv::DepthImage const & tof) {
			if (tof.type == xv::DepthImage::Type::Depth_16 || tof.type == xv::DepthImage::Type::Depth_32) {
				std::lock_guard<std::mutex> l(s_mtx_tof);
				s_tof = std::make_shared<xv::DepthImage>(tof);
			}
		});
	}

	std::thread t(display);


	std::cout << " ################## " << std::endl;
	std::cout << "        Start       " << std::endl;
	std::cout << " ################## " << std::endl;


	std::cerr << "ENTER to stop" << std::endl;
	std::cin.get();


	std::cout << " ################## " << std::endl;
	std::cout << "        Stop        " << std::endl;
	std::cout << " ################## " << std::endl;
}
catch (const std::exception &e) {
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

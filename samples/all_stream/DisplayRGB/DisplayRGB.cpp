
#include <cstdlib>
#include <thread>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <mutex>
#include <signal.h>
#include <cstring>
#include <xv-sdk.h>
#include "colors.h"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include "fps_count.hpp"



std::shared_ptr<const xv::ColorImage> s_rgb = nullptr;
std::mutex s_mtx_rgb;

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
cv::Mat raw_to_opencv_rgb(std::shared_ptr<const xv::ColorImage> rgb)
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
	return img;
}


void rgbCallback(xv::ColorImage const& rgb) {
	static FpsCount fc;
	fc.tic();
	static int k = 0;
	if (k++ % 10 == 0) {
		std::cout << rgb.width << "x" << rgb.height << "@" << std::round(fc.fps()) << "fps" << std::endl;
	}
	//std::shared_ptr<const xv::ColorImage> rgb_data = nullptr;
	s_rgb = std::make_shared<xv::ColorImage>(rgb);

}
void display()
{
	cv::namedWindow("RGB");
	cv::moveWindow("RGB", 20, 462);

	while (true) {
		std::shared_ptr<const xv::ColorImage> rgb = nullptr;
		if (true) {
			s_mtx_rgb.lock();
			rgb = s_rgb;
			s_mtx_rgb.unlock();
			if (rgb && rgb->width > 0 && rgb->height > 0) {
				cv::Mat img = raw_to_opencv_rgb(rgb);
				cv::imshow("RGB", img);
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

	auto devices = xv::getDevices(10., json);

	if (devices.empty())
	{
		std::cout << "Timeout: no device found\n";
		return EXIT_FAILURE;
	}

	auto device = devices.begin()->second;
	int rgbId = -1;
	std::cout << device->id() << std::endl;
	device->colorCamera()->start();
	rgbId = device->colorCamera()->registerCallback(rgbCallback);

	std::thread t(display);
	std::cout << " ################## " << std::endl;
	std::cout << "        Start       " << std::endl;
	std::cout << " ################## " << std::endl;


	std::cerr << "ENTER to stop" << std::endl;
	std::cin.get();

	std::cout << " ################## " << std::endl;
	std::cout << "        Stop        " << std::endl;
	std::cout << " ################## " << std::endl;

	return EXIT_SUCCESS;
}
catch (const std::exception &e) {
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

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

#include <xv-sdk.h>
#include "colors.h"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include "xv-sdk-ex.h"

#define USE_EX

static std::map<std::string, int> enableDevMap;


cv::Mat raw_to_opencv(std::shared_ptr<const xv::ColorImage> rgb);
cv::Mat raw_to_opencv(std::shared_ptr<const xv::DepthImage> tof);
cv::Mat raw_to_opencv(std::shared_ptr<const xv::DepthColorImage> rgbd);


std::shared_ptr<const xv::ColorImage> s_rgb = nullptr;
std::shared_ptr<const xv::DepthImage> s_tof = nullptr;
std::shared_ptr<const xv::DepthColorImage> s_depthColor = nullptr;


std::mutex s_mtx_rgb;
std::mutex s_mtx_tof;
std::mutex s_mtx_depthColor;


cv::Mat raw_to_opencv(std::shared_ptr<const xv::ColorImage> rgb)
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
cv::Mat raw_to_opencv(std::shared_ptr<const xv::DepthImage> tof)
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
		else if (tof->type == xv::DepthImage::Type::IR) {
			out = cv::Mat::zeros(tof->height, tof->width, CV_8UC3);
			float dmax = 2494.0; // maybe 7494,2494,1498,1249 see mode_manage.h in sony toflib
			auto tmp_d = reinterpret_cast<unsigned short const*>(tof->data.get());
			for (unsigned int i = 0; i < tof->height*tof->width; i++) {
				unsigned short d = tmp_d[i];
				unsigned int u = static_cast<unsigned int>(std::max(0.0f, std::min(255.0f, d * 255.0f / dmax)));
				if (u < 15)
					u = 0;
				const auto &cc = colors.at(u);
				out.at<cv::Vec3b>(i / tof->width, i%tof->width) = cv::Vec3b(cc.at(2), cc.at(1), cc.at(0));
				/*short u = tof->ir.get()[i];
				cvtof.at<cv::Vec3b>( i/tof->width, i%tof->width ) = cv::Vec3s(u*255/2191,u*255/2191,u*255/2191);*/
			}
		}
	}
	return out;
}
cv::Mat raw_to_opencv(std::shared_ptr<const xv::DepthColorImage> rgbd)
{
	cv::Mat out;
	out = cv::Mat::zeros(rgbd->height, rgbd->width * 2, CV_8UC3);
	auto w = rgbd->width;

	if (rgbd->height > 0 && rgbd->width > 0) {
		float dmax = 7.5;


		const auto tmp_d = reinterpret_cast<std::uint8_t const*>(rgbd->data.get() + 3);


		for (unsigned int i = 0; i < rgbd->height*rgbd->width; i++) {
			const auto &d = *reinterpret_cast<float const*>(tmp_d + i * (3 + sizeof(float)));
			if (d < 0.01 || d > 9.9) {
				out.at<cv::Vec3b>(i / w, i % rgbd->width) = 0;
			}
			else {
				unsigned int u = static_cast<unsigned int>(std::max(0.0f, std::min(255.0f, d * 255.0f / dmax)));
				const auto &cc = colors.at(u);
				out.at<cv::Vec3b>(i / w, i%rgbd->width) = cv::Vec3b(cc.at(2), cc.at(1), cc.at(0));
			}
		}

		const auto tmp_rgb = reinterpret_cast<std::uint8_t const*>(rgbd->data.get());

		for (unsigned int i = 0; i < rgbd->height*rgbd->width; i++) {
			const auto rgb = reinterpret_cast<std::uint8_t const*>(tmp_rgb + i * (3 + sizeof(float)));
			out.at<cv::Vec3b>(i / w, (i%rgbd->width) + rgbd->width) = cv::Vec3b(rgb[2], rgb[1], rgb[0]);
		}
	}
	return out;
}


void display() {
	if (enableDevMap["rgb"]) {
		cv::namedWindow("RGB");
		cv::moveWindow("RGB", 20, 462);
	}
	if (enableDevMap["tof"]) {
		cv::namedWindow("TOF");
		cv::moveWindow("TOF", 500, 462);

	}
	cv::waitKey(1);

	while (true) {
		std::shared_ptr<const xv::ColorImage> rgb = nullptr;
		std::shared_ptr<const xv::DepthImage> tof = nullptr;
		std::shared_ptr<const xv::GrayScaleImage> ir = nullptr;

		if (enableDevMap["rgb"]) {
			s_mtx_rgb.lock();
			rgb = s_rgb;
			s_mtx_rgb.unlock();
			if (rgb && rgb->width > 0 && rgb->height > 0) {
				cv::Mat img = raw_to_opencv(rgb);
				cv::resize(img, img, cv::Size(), 0.25, 0.25);
				cv::imshow("RGB", img);
			}
		}
		if (enableDevMap["tof"]) {
			s_mtx_tof.lock();
			tof = s_tof;
			s_mtx_tof.unlock();
			if (tof) {
				cv::Mat img = raw_to_opencv(tof);
				if (img.rows > 0 && img.cols > 0)
					cv::imshow("TOF", img);
			}
			s_mtx_depthColor.lock();
			auto depthColor = s_depthColor;
			s_mtx_depthColor.unlock();
			if (depthColor) {
				cv::Mat img = raw_to_opencv(depthColor);
				if (img.rows > 0 && img.cols > 0)
					cv::imshow("RGBD (depth)", img);
			}
		}

		cv::waitKey(1);
	}
}


#include "fps_count.hpp"

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

int main(int argc, char* argv[]) try
{
	std::cout << "xvsdk version: " << xv::version() << std::endl;

	xv::setLogLevel(xv::LogLevel::debug);

	std::string json = "";

	enableDevMap["rgb"] = true;
	enableDevMap["tof"] = true;
	enableDevMap["tof_mode"] = 3;//default lablize sf

	auto devices = xv::getDevices(10., json);

	if (devices.empty())
	{
		std::cout << "Timeout: no device found\n";
		return EXIT_FAILURE;
	}

	auto device = devices.begin()->second;

	enableDevMap["rgb"] &= device->colorCamera() != nullptr;
	enableDevMap["tof"] &= device->tofCamera() != nullptr;
	if (enableDevMap["rgb"])
	{
		device->colorCamera()->registerCallback([](xv::ColorImage const & rgb) {
			static FpsCount fc;
			fc.tic();
			static int k = 0;
			if (k++ % 25 == 0) {
				std::cout << "rgb      " << timeShowStr(rgb.edgeTimestampUs, rgb.hostTimestamp)
					<< rgb.width << "x" << rgb.height << "@" << std::round(fc.fps()) << "fps" << std::endl;
			}
		});
		device->colorCamera()->setResolution(xv::ColorCamera::Resolution::RGB_640x480);
		device->colorCamera()->start();
	}
	else
	{
		std::cout << "No RGB camera.\n";
	}
	if (device->colorCamera()) {
		device->colorCamera()->registerCallback([&device](xv::ColorImage const & im) {
			s_mtx_rgb.lock();
			s_rgb = std::make_shared<xv::ColorImage>(im);
			s_mtx_rgb.unlock();
		});
	}

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

		device->tofCamera()->registerColorDepthImageCallback([](const xv::DepthColorImage& depthColor) {
			static FpsCount fc;
			fc.tic();
			static int k = 0;
			if (k++ % 15 == 0) {
				std::cout << "RGBD     " << timeShowStr(depthColor.hostTimestamp)
					<< depthColor.width << "x" << depthColor.height << "@" << std::round(fc.fps()) << "fps" << std::endl;
			}
		});

	}
	if (enableDevMap["tof"]) {
		device->tofCamera()->registerCallback([](xv::DepthImage const & tof) {
			if (tof.type == xv::DepthImage::Type::Depth_16 || tof.type == xv::DepthImage::Type::Depth_32) {
				std::lock_guard<std::mutex> l(s_mtx_tof);
				s_tof = std::make_shared<xv::DepthImage>(tof);
			}

		});


		device->tofCamera()->registerColorDepthImageCallback([](const xv::DepthColorImage& depthColor) {
			s_mtx_depthColor.lock();
			s_depthColor = std::make_shared<xv::DepthColorImage>(depthColor);
			s_mtx_depthColor.unlock();
		});

	}


	std::cout << " == Initialized ==" << std::endl;

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

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

#define USE_EX

static std::map<std::string, int> enableDevMap;


#ifdef USE_EX
#include "xv-sdk-ex.h"
#endif
#ifdef USE_PRIVATE
#include "xv-sdk-private.h"
#endif
#include <opencv2/opencv.hpp>


std::pair<cv::Mat, cv::Mat> raw_to_opencv(std::shared_ptr<const xv::FisheyeImages> stereo);


#ifdef USE_EX

std::pair<cv::Mat, cv::Mat> raw_to_opencv(std::shared_ptr<const xv::FisheyeImages> stereo, std::shared_ptr<const xv::FisheyeKeyPoints<2, 32>> keypoints, std::shared_ptr<const std::vector<std::pair<int, std::array<xv::Vector2d, 4>>>> tags)
{
	cv::Mat left;
	cv::Mat right;

	if (stereo) {
		auto const& leftInput = stereo->images[0];
		auto const& rightInput = stereo->images[1];
		if (leftInput.data != nullptr) {
			left = cv::Mat::zeros(leftInput.height, leftInput.width, CV_8UC1);
			std::memcpy(left.data, leftInput.data.get(), static_cast<size_t>(left.rows*left.cols));
		}
		if (rightInput.data != nullptr) {
			right = cv::Mat::zeros(rightInput.height, rightInput.width, CV_8UC1);
			std::memcpy(right.data, rightInput.data.get(), static_cast<size_t>(right.rows*right.cols));
		}
	}
	else {
		left = cv::Mat::zeros(400, 640, CV_8UC1);
		right = cv::Mat::zeros(400, 640, CV_8UC1);
	}

	cv::cvtColor(left, left, cv::COLOR_GRAY2BGR);
	cv::cvtColor(right, right, cv::COLOR_GRAY2BGR);

	if (keypoints) {
		const int size = 2;
		int s = 0;
		for (unsigned int i = 0; i < keypoints->descriptors[s].size; i++) {
			auto p = keypoints->descriptors[s].keypoints.get() + i * 2;
			cv::Point pt(*p, *(p + 1));
			cv::line(left, pt - cv::Point(size, 0), pt + cv::Point(size, 0), cv::Scalar(0, 0, 255));
			cv::line(left, pt - cv::Point(0, size), pt + cv::Point(0, size), cv::Scalar(0, 0, 255));
		}
		s = 1;
		for (unsigned int i = 0; i < keypoints->descriptors[s].size; i++) {
			auto p = keypoints->descriptors[s].keypoints.get() + i * 2;
			cv::Point pt(*p, *(p + 1));
			cv::line(right, pt - cv::Point(size, 0), pt + cv::Point(size, 0), cv::Scalar(0, 0, 255));
			cv::line(right, pt - cv::Point(0, size), pt + cv::Point(0, size), cv::Scalar(0, 0, 255));
		}
	}

	return { left, right };
}

std::array<cv::Mat, 4> raw_to_opencv(std::shared_ptr<const xv::FisheyeImages> stereo, std::shared_ptr<const xv::FisheyeKeyPoints<4, 32>> keypoints, std::shared_ptr<const std::vector<std::pair<int, std::array<xv::Vector2d, 4>>>> tags)
{

	std::array<cv::Mat, 4> images;

	for (auto& im : images) {
		im = cv::Mat::zeros(400, 640, CV_8UC1);
	}

	if (stereo) {
		std::size_t i = 0;
		for (auto& im : stereo->images) {
			if (im.data != nullptr && i < images.size()) {
				std::memcpy(images[i].data, im.data.get(), static_cast<size_t>(images[i].rows*images[i].cols));
				cv::cvtColor(images[i], images[i], cv::COLOR_GRAY2BGR);
			}
			++i;
		}
	}
	else {
		for (auto& im : images) {
			im = cv::Mat::zeros(400, 640, CV_8UC1);
		}
	}

	if (keypoints) {

		const int size = 2;
		std::size_t ii = 0;
		for (auto& descriptors : keypoints->descriptors) {
			for (unsigned int i = 0; i < descriptors.size; i++) {
				auto p = descriptors.keypoints.get() + i * 2;
				cv::Point pt(*p, *(p + 1));
				cv::line(images[ii], pt - cv::Point(size, 0), pt + cv::Point(size, 0), cv::Scalar(0, 0, 255));
				cv::line(images[ii], pt - cv::Point(0, size), pt + cv::Point(0, size), cv::Scalar(0, 0, 255));
			}
			++ii;
		}
	}

	
	return images;
}

#endif


std::shared_ptr<const xv::FisheyeImages> s_stereo = nullptr;

#ifdef USE_EX
std::shared_ptr<const xv::FisheyeKeyPoints<2, 32>> s_keypoints = nullptr;
std::shared_ptr<const xv::FisheyeKeyPoints<4, 32>> s_keypoints4cam = nullptr;
std::mutex s_mtx_tags;
#endif

std::mutex s_mtx_stereo;

void display() {
	if (enableDevMap["fisheye"]) {
		cv::namedWindow("Left");
		cv::moveWindow("Left", 20, 20);
		cv::namedWindow("Right");
		cv::moveWindow("Right", 660, 20);
	}

	cv::waitKey(1);

	while (true) {
		std::shared_ptr<const xv::FisheyeImages> stereo = nullptr;
#ifdef USE_EX
		std::shared_ptr<const xv::FisheyeKeyPoints<2, 32>> keypoints = nullptr;
		std::shared_ptr<const xv::FisheyeKeyPoints<4, 32>> keypoints4cam = nullptr;
		std::shared_ptr<const std::vector<std::pair<int, std::array<xv::Vector2d, 4>>>> tags;
#endif
		if (enableDevMap["fisheye"]) {
			s_mtx_stereo.lock();
			stereo = s_stereo;
#ifdef USE_EX
			keypoints = s_keypoints;
			keypoints4cam = s_keypoints4cam;
			s_mtx_tags.lock();
			s_mtx_tags.unlock();
#endif
			s_mtx_stereo.unlock();

#ifdef USE_EX
			if (keypoints) {
				auto imgs = raw_to_opencv(stereo, keypoints, tags);
				cv::imshow("Left", imgs.first);
				cv::imshow("Right", imgs.second);
			}
	
#else
			if (stereo) {
				auto imgs = raw_to_opencv(stereo);
				cv::imshow("Left", imgs.first);
				cv::imshow("Right", imgs.second);
			}
#endif
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
	enableDevMap["fisheye"] = true;
	enableDevMap["slam"] = true;
	enableDevMap["imu"] = true;

	auto devices = xv::getDevices(10., json);

	if (devices.empty())
	{
		std::cout << "Timeout: no device found\n";
		return EXIT_FAILURE;
	}

	auto device = devices.begin()->second;

	enableDevMap["fisheye"] &= device->fisheyeCameras() != nullptr;
	enableDevMap["slam"] &= device->slam() != nullptr;
	enableDevMap["imu"] &= device->imuSensor() != nullptr;


	if (enableDevMap["fisheye"]) {
#ifdef USE_EX
		std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->registerKeyPointsCallback([](const xv::FisheyeKeyPoints<2, 32>& keypoints) {
			static FpsCount fc;
			fc.tic();
			static int k = 0;
			if (k++ % 50 == 0) {
				std::cout << "keypoints  " << timeShowStr(keypoints.edgeTimestampUs, keypoints.hostTimestamp) << keypoints.descriptors[0].size << ":" << keypoints.descriptors[1].size << "@" << std::round(fc.fps()) << "fps" << std::endl;
			}
		});
#endif
		device->fisheyeCameras()->registerCallback([](xv::FisheyeImages const & stereo) {
			static FpsCount fc;
			fc.tic();
			static int k = 0;
			if (k++ % 50 == 0) {
				std::cout << "stereo   " << timeShowStr(stereo.edgeTimestampUs, stereo.hostTimestamp) << stereo.images[0].width << "x" << stereo.images[0].height << "@" << std::round(fc.fps()) << "fps" << std::endl;
			}
		});


	}




#ifdef USE_EX
	// Must set device to edge mode to make both edge and mixed slam work.
	std::dynamic_pointer_cast<xv::DeviceEx>(device)->slam2()->start(xv::Slam::Mode::Edge);
#endif


	std::cout << " == Initialized ==" << std::endl;

	//Display in thread to not slow down callbacks

	if (enableDevMap["fisheye"]) {
		device->fisheyeCameras()->registerCallback([&device](xv::FisheyeImages const & stereo) {
			s_mtx_stereo.lock();
			s_stereo = std::make_shared<xv::FisheyeImages>(stereo);
			s_mtx_stereo.unlock();
		});
#ifdef USE_EX
		std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->registerKeyPointsCallback([](const xv::FisheyeKeyPoints<2, 32>& keypoints) {
			s_mtx_stereo.lock();
			s_keypoints = std::make_shared<xv::FisheyeKeyPoints<2, 32>>(keypoints);
			s_mtx_stereo.unlock();
		});
		std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->registerKeyPointsCallback([](const xv::FisheyeKeyPoints<4, 32>& keypoints) {
			s_mtx_stereo.lock();
			s_keypoints4cam = std::make_shared<xv::FisheyeKeyPoints<4, 32>>(keypoints);
			s_mtx_stereo.unlock();
		});
#endif
	}


	std::thread t1(display);




	std::cout << " ################## " << std::endl;
	std::cout << "        Start       " << std::endl;
	std::cout << " ################## " << std::endl;


	std::cerr << "ENTER to stop" << std::endl;
	std::cin.get();


	std::cout << " ################## " << std::endl;
	std::cout << "        Stop        " << std::endl;
	std::cout << " ################## " << std::endl;

#ifdef USE_EX
	if (std::dynamic_pointer_cast<xv::DeviceEx>(device)->slam2())
		std::dynamic_pointer_cast<xv::DeviceEx>(device)->slam2()->stop();
#endif

	if (device->slam())
		device->slam()->stop();



	return EXIT_SUCCESS;
}
catch (const std::exception &e) {
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

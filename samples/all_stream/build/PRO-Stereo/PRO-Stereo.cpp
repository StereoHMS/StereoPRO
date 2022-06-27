﻿#define _USE_MATH_DEFINES

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
//#define USE_PRIVATE

bool s_stop = false;
static std::map<std::string, int> enableDevMap;

static struct xv::sgbm_config global_config = {
	1 ,//enable_dewarp
	1.0, //dewarp_zoom_factor
	0, //enable_disparity
	1, //enable_depth
	0, //enable_point_cloud
	0.08, //baseline
	96, //fov
	255, //disparity_confidence_threshold
	{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, //homography
	1, //enable_gamma
	2.2, //gamma_value
	0, //enable_gaussian
	0, //mode
	8000, //max_distance
	100, //min_distance
};


#ifdef USE_EX
#include "xv-sdk-ex.h"
#endif
#ifdef USE_PRIVATE
#include "xv-sdk-private.h"
#endif


#include <opencv2/opencv.hpp>


std::pair<cv::Mat, cv::Mat> raw_to_opencv(std::shared_ptr<const xv::FisheyeImages> stereo);

void add_tags(cv::Mat& im, std::vector<std::pair<int, std::array<xv::Vector2d, 4>>> const& tags) {
	for (auto const& t : tags) {
		auto const& pts = t.second;
		std::size_t i = 0;
		cv::line(im, cv::Point(pts[i][0], pts[i][1]), cv::Point(pts[i + 1][0], pts[i + 1][1]), cv::Scalar(0, 255, 0)); i++;
		cv::line(im, cv::Point(pts[i][0], pts[i][1]), cv::Point(pts[i + 1][0], pts[i + 1][1]), cv::Scalar(0, 255, 0)); i++;
		cv::line(im, cv::Point(pts[i][0], pts[i][1]), cv::Point(pts[i + 1][0], pts[i + 1][1]), cv::Scalar(0, 255, 0)); i++;
		cv::line(im, cv::Point(pts[i][0], pts[i][1]), cv::Point(pts[0][0], pts[0][1]), cv::Scalar(0, 255, 0));
		cv::putText(im, std::to_string(t.first),
			cv::Point(pts[i][0], pts[i][1]), //top-left position
			cv::FONT_HERSHEY_DUPLEX,
			0.25,
			CV_RGB(0, 255, 0), //font color
			1.);
	}
}

#ifdef USE_EX

void add_tags(cv::Mat& im, std::vector<xv::TagDetection> const& tags) {
	for (auto const& t : tags) {
		auto const& pts = t.corners;
		std::size_t i = 0;
		cv::line(im, cv::Point(pts[i][0], pts[i][1]), cv::Point(pts[i + 1][0], pts[i + 1][1]), cv::Scalar(0, 255, 0)); i++;
		cv::line(im, cv::Point(pts[i][0], pts[i][1]), cv::Point(pts[i + 1][0], pts[i + 1][1]), cv::Scalar(0, 255, 0)); i++;
		cv::line(im, cv::Point(pts[i][0], pts[i][1]), cv::Point(pts[i + 1][0], pts[i + 1][1]), cv::Scalar(0, 255, 0)); i++;
		cv::line(im, cv::Point(pts[i][0], pts[i][1]), cv::Point(pts[0][0], pts[0][1]), cv::Scalar(0, 255, 0));
		cv::putText(im, std::to_string(t.tagId),
			cv::Point(pts[i][0], pts[i][1]), //top-left position
			cv::FONT_HERSHEY_DUPLEX,
			0.25,
			CV_RGB(0, 255, 0), //font color
			1.);
	}
}

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

	/*if (tags) {
		add_tags(left, *tags);
	}*/

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

	if (tags) {
		add_tags(images[0], *tags);
	}

	return images;
}

std::pair<cv::Mat, cv::Mat> raw_to_opencv(std::shared_ptr<const xv::EyetrackingImage> eyetracking)
{
	cv::Mat left;
	cv::Mat right;

	if (eyetracking) {
		auto const& leftInput = eyetracking->images[0];
		auto const& rightInput = eyetracking->images[1];
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

	return { left, right };
}

#endif


std::shared_ptr<const xv::ColorImage> s_rgb = nullptr;
std::shared_ptr<const xv::DepthImage> s_tof = nullptr;
std::shared_ptr<const xv::GrayScaleImage> s_ir = nullptr;
std::shared_ptr<const xv::FisheyeImages> s_stereo = nullptr;
std::shared_ptr<const xv::DepthColorImage> s_depthColor = nullptr;
std::shared_ptr<const xv::SgbmImage> s_ptr_sgbm = nullptr;
std::shared_ptr<const xv::EyetrackingImage> s_eyetracking = nullptr;

#ifdef USE_EX
std::shared_ptr<const xv::FisheyeKeyPoints<2, 32>> s_keypoints = nullptr;
std::shared_ptr<const xv::FisheyeKeyPoints<4, 32>> s_keypoints4cam = nullptr;
std::mutex s_mtx_tags;
std::shared_ptr<const std::vector<std::pair<int, std::array<xv::Vector2d, 4>>>> s_tags;
std::mutex s_mtx_rgb_tags;
xv::GrayScaleImage s_rgb_gray;
std::vector<xv::TagDetection> s_rgb_tags;
#endif

std::mutex s_mtx_rgb;
std::mutex s_mtx_tof;
std::mutex s_mtx_depthColor;
std::mutex s_mtx_ir;
std::mutex s_mtx_stereo;
std::mutex s_mtx_sgbm;
std::mutex s_mtx_eyetracking;

void display() {
	if (enableDevMap["fisheye"]) {
		cv::namedWindow("Left");
		cv::moveWindow("Left", 20, 20);
		cv::namedWindow("Right");
		cv::moveWindow("Right", 660, 20);
	}

	cv::waitKey(1);

	while (!s_stop) {
		std::shared_ptr<const xv::FisheyeImages> stereo = nullptr;
#ifdef USE_EX
		std::shared_ptr<const xv::FisheyeKeyPoints<2, 32>> keypoints = nullptr;
		std::shared_ptr<const xv::FisheyeKeyPoints<4, 32>> keypoints4cam = nullptr;
		std::shared_ptr<const std::vector<std::pair<int, std::array<xv::Vector2d, 4>>>> tags;
		decltype (s_rgb_tags) rgb_tags;
#endif
		if (enableDevMap["fisheye"]) {
			s_mtx_stereo.lock();
			stereo = s_stereo;
#ifdef USE_EX
			keypoints = s_keypoints;
			keypoints4cam = s_keypoints4cam;
			s_mtx_tags.lock();
			tags = s_tags;
			s_mtx_tags.unlock();
			s_mtx_rgb_tags.lock();
			rgb_tags = s_rgb_tags;
			s_mtx_rgb_tags.unlock();
#endif
			s_mtx_stereo.unlock();

#ifdef USE_EX
			if (keypoints) {
				auto imgs = raw_to_opencv(stereo, keypoints, tags);
				cv::imshow("Left", imgs.first);
				cv::imshow("Right", imgs.second);
			}
			/*if (keypoints4cam) {
				auto imgs = raw_to_opencv(stereo, keypoints4cam, tags);
				for (std::size_t i = 0; i < imgs.size(); ++i) {
					cv::imshow("Cam" + std::to_string(i), imgs[i]);
				}
			}*/
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
	if (argc > 1 && *argv[1] != '\0') {
		std::ifstream ifs(argv[1]);
		if (!ifs.is_open()) {
			std::cerr << "Failed to open: " << argv[1] << std::endl;
		}
		else
		{
			std::stringstream fbuf;
			fbuf << ifs.rdbuf();
			json = fbuf.str();
		}
	}
	enableDevMap["fisheye"] = true;
	enableDevMap["slam"] = true;
	enableDevMap["imu"] = true;
	enableDevMap["eyetracking"] = true;
	enableDevMap["sync"] = false;
	enableDevMap["dewarp"] = true;
	enableDevMap["VGA"] = true;
	enableDevMap["720P"] = false;
	if (argc == 3)
	{
		std::string enableDevStr(argv[2]);
		enableDevStr += " ";
		int index, index2;
		while (true)
		{
			index = enableDevStr.find(' ');
			if (index == std::string::npos)
			{
				break;
			}
			auto one = enableDevStr.substr(0, index);
			index2 = one.find(':');
			auto key = one.substr(0, index2);
			auto value = one.substr(index2 + 1, one.size() - index2 - 1) == "1" ? true : false;
			enableDevStr = enableDevStr.substr(index + 1);
			std::cout << key << " : " << value << std::endl;
			enableDevMap[key] = value;
		}
	}

	auto devices = xv::getDevices(10., json);

	std::ofstream ofs;
	if (devices.empty())
	{
		std::cout << "Timeout: no device found\n";
		return EXIT_FAILURE;
	}

	auto device = devices.begin()->second;

	enableDevMap["fisheye"] &= device->fisheyeCameras() != nullptr;
	enableDevMap["slam"] &= device->slam() != nullptr;
	enableDevMap["imu"] &= device->imuSensor() != nullptr;


	std::string tagDetectorId;
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
#ifdef USE_EX
		tagDetectorId = std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->startTagDetector(device->slam(), "36h11", 0.0639, 50.);

		if (enableDevMap["VGA"])
		{
			std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->setResolutionMode(xv::FisheyeCamerasEx::ResolutionMode::MEDIUM);
		}
		if (enableDevMap["720P"])
		{
			std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->setResolutionMode(xv::FisheyeCamerasEx::ResolutionMode::HIGH);
		}
		// device->fisheyeCameras()->setStereoResolutionMode(xv::ResolutionMode::R_720P);
		device->fisheyeCameras()->start();
	}

#endif


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
#ifdef USE_EX
			/*s_mtx_tags.lock();
			auto tags = std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->detectTags(stereo.images[0], "36h11");
			s_tags = std::make_shared<std::vector<std::pair<int, std::array<xv::Vector2d, 4>>>>(tags);
			s_mtx_tags.unlock();*/
#endif
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


	s_stop = false;
	std::thread t1(display);




	std::cout << " ################## " << std::endl;
	std::cout << "        Start       " << std::endl;
	std::cout << " ################## " << std::endl;

#ifdef USE_EX
	if (!tagDetectorId.empty()) {
		std::cerr << "ENTER to stop tag detection" << std::endl;
		std::cin.get();
		std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->stopTagDetector(tagDetectorId);
	}

	std::cerr << "ENTER to switch FE to HIGH res" << std::endl;
	std::cin.get();

	std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->setResolutionMode(xv::FisheyeCamerasEx::ResolutionMode::HIGH);

	std::cerr << "ENTER to switch FE to MEDIUM res" << std::endl;
	std::cin.get();

	std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->setResolutionMode(xv::FisheyeCamerasEx::ResolutionMode::MEDIUM);

#endif

	std::cerr << "ENTER to stop" << std::endl;
	std::cin.get();

	s_stop = true;

	std::cout << " ################## " << std::endl;
	std::cout << "        Stop        " << std::endl;
	std::cout << " ################## " << std::endl;

#ifdef USE_EX
	if (std::dynamic_pointer_cast<xv::DeviceEx>(device)->slam2())
		std::dynamic_pointer_cast<xv::DeviceEx>(device)->slam2()->stop();
#endif

	if (device->slam())
		device->slam()->stop();


#ifdef USE_OPENCV
	s_stop = true;
	if (t.joinable()) {
		t.join();
	}
#endif
	ofs.close();
	return EXIT_SUCCESS;
}
catch (const std::exception &e) {
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

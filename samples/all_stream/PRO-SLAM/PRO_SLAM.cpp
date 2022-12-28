#define _USE_MATH_DEFINES
#define _CRT_SECURE_NO_WARNINGS
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
#include <xv-sdk-ex.h>

#include <opencv2/opencv.hpp>
#include "fps_count.hpp"




std::shared_ptr<const xv::FisheyeKeyPoints<2, 32>> s_keypoints = nullptr;
std::shared_ptr<const xv::FisheyeKeyPoints<4, 32>> s_keypoints4cam = nullptr;




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
void startGetPose(std::shared_ptr<xv::Slam> slam)
{
	
	
	
		double prediction = 0.005;

		long n = 0;
		long nb_ok = 0;
		xv::Pose pose;
		while (true) {
			std::this_thread::sleep_for(std::chrono::milliseconds(2));

			bool ok = slam->getPose(pose, prediction);

			if (ok) {
				++nb_ok;
				static int k = 0;
				if (k++ % 100 == 0) {
					auto t = pose.translation();
					auto r = xv::rotationToPitchYawRoll(pose.rotation());
					if (true) {
						//std::cout << "slam-get-pose [" << p->x << "," << p->y << "," << p->z << "]" << std::endl;
						std::cout << std::setprecision(5) << "slam-get-pose"
							<< " p=(" << t[0] << " " << t[1] << " " << t[2]
							<< " ), r=(" << r[0] << " " << r[1] << " " << r[2] << " )"
							<< ", Confidence= " << pose.confidence()
							<< std::endl;
					}
				}
			}
			n++;
		}
		/*if (enable_output_log) {
			std::cout << "Nb get pose ok: " << 100.0 * double(nb_ok) / n << "% (" << nb_ok << "/" << n << ")" << std::endl;
		}*/
		

}
int main(int argc, char* argv[]) try
{
	std::cout << "xvsdk version: " << xv::version() << std::endl;

	xv::setLogLevel(xv::LogLevel::debug);

	std::string json = "";


	auto devices = xv::getDevices(10., json);

	xv::setLogLevel(xv::LogLevel::debug);

	if (devices.empty())
	{
		std::cout << "Timeout: no device found\n";
		return EXIT_FAILURE;
	}

	auto device = devices.begin()->second;

	device->slam() != nullptr;
	int cb_imu = -1;
	if (device->imuSensor())
	{
		cb_imu = device->imuSensor()->registerCallback([](xv::Imu const & imu) {});
		device->imuSensor()->start();
	}
	else
	{
		std::cerr << "No imu " << std::endl;
		return EXIT_FAILURE;
	}
	int cb_fisheye = -1;
	int cb_keypoints = -1;
	std::mutex mutex_images;
	xv::FisheyeImages images;

	std::mutex mutex_keypoints;
	std::shared_ptr<const xv::FisheyeKeyPoints<2, 32>> keypoints;
	if (device->fisheyeCameras())
	{
		cb_fisheye = device->fisheyeCameras()->registerCallback([&](xv::FisheyeImages const & im)
			{
				std::lock_guard<std::mutex> l(mutex_images);

				//images = im;
			});

		cb_keypoints = std::dynamic_pointer_cast<xv::FisheyeCamerasEx>(device->fisheyeCameras())->registerKeyPointsCallback([&](const xv::FisheyeKeyPoints<2, 32>& kps)
			{
				std::lock_guard<std::mutex> l(mutex_keypoints);
				keypoints = std::make_shared<xv::FisheyeKeyPoints<2, 32>>(kps);
			});

		device->fisheyeCameras()->start();
	}
	else
	{
		std::cerr << "No fisheyes" << std::endl;
		return EXIT_FAILURE;
	}
	auto slam = device->slam();
	std::shared_ptr<const xv::SlamMap> map(new xv::SlamMap);
	std::vector<xv::Pose> shared_poses, poses;
	std::mutex mutex_map, mutex_poses;

	std::mutex mutex_str;
	std::string localize_str;


	// start mix slam
	device->slam()->start(xv::Slam::Mode::Mixed);

	// get mixed 6dof
	startGetPose(device->slam());



	int getkey = 0;
	std::cerr << "ENTER 'q' to exit" << std::endl;

	std::cin >> getkey;
	slam->stop();

	device->imuSensor()->unregisterCallback(cb_imu);
	return 0;
}

catch (const std::exception &e) {
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

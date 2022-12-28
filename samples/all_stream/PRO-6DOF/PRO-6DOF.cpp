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
#include "opencv2/imgproc.hpp"
#include <xv-sdk.h>
#include "colors.h"

#include "xv-sdk-ex.h"

#include <opencv2/opencv.hpp>
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


	auto devices = xv::getDevices(10., json);

	xv::setLogLevel(xv::LogLevel::debug);

	if (devices.empty())
	{
		std::cout << "Timeout: no device found\n";
		return EXIT_FAILURE;
	}

	auto device = devices.begin()->second;
	std::cout << "---------------------------" << std::endl;

	device->slam() != nullptr;


	if (true)
	{
		if (std::dynamic_pointer_cast<xv::DeviceEx>(device)->slam2()) {
			std::dynamic_pointer_cast<xv::DeviceEx>(device)->slam2()->registerCallback([](const xv::Pose& pose) {
				static FpsCount fc;
				fc.tic();
				static int k = 0;
				if (k++ % 500 == 0) {
					auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
					if (true)
					{
						std::cout << "edge-pose" << timeShowStr(pose.edgeTimestampUs(), pose.hostTimestamp()) << "@" << std::round(fc.fps()) << "fps" << " (" << pose.x() << "," << pose.y() << "," << pose.z() << ") (" << pitchYawRoll[0] * 180 / M_PI << "," << pitchYawRoll[1] * 180 / M_PI << "," << pitchYawRoll[2] * 180 / M_PI << ")" << pose.confidence() << std::endl;
					}
				}
				});
			std::dynamic_pointer_cast<xv::DeviceEx>(device)->slam2()->start(xv::Slam::Mode::Edge);
		}
		else {
			std::cout << "No edge in camera.\n";
		}
	}std::cerr << "ENTER 'q' to exit" << std::endl;
	int getkey = -1;
	std::cin >> getkey;


}
catch (const std::exception &e) {
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

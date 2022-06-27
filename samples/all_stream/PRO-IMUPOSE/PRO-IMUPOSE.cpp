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
#include <iomanip>
#include <xv-sdk.h>
#include "colors.h"

#define USE_EX
//#define USE_PRIVATE

bool s_stop = false;
static std::map<std::string, int> enableDevMap;



#ifdef USE_EX
#include "xv-sdk-ex.h"
#endif
#ifdef USE_PRIVATE
#include "xv-sdk-private.h"
#endif

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

std::thread tpos;
bool stop = false;
double t = -1;
void stopGetPose()
{
	stop = true;
	if (tpos.joinable())
		tpos.join();
}
void startGetPose(std::shared_ptr<xv::Slam> slam)
{
	stopGetPose();
	stop = false;
	tpos = std::thread([slam] {
		double prediction = 0.005;

		long n = 0;
		long nb_ok = 0;
		xv::Pose pose;
		while (!stop) {
			
			std::this_thread::sleep_for(std::chrono::milliseconds(2));

			bool ok = slam->getPose(pose, prediction);
			if (ok) {
				std::cout << (bool)ok << std::endl;

				++nb_ok;
				static int k = 0;
				if (k++ % 100 == 0) {
					auto t = pose.translation();
					auto r = xv::rotationToPitchYawRoll(pose.rotation());

					//std::cout << "slam-get-pose [" << p->x << "," << p->y << "," << p->z << "]" << std::endl;
					std::cout << std::setprecision(5) << "slam-get-pose"
						<< " p=(" << t[0] << " " << t[1] << " " << t[2]
						<< " ), r=(" << r[0] << " " << r[1] << " " << r[2] << " )"
						<< ", Confidence= " << pose.confidence()
						<< std::endl;
				}
			}
			n++;
		}
		std::cout << "Nb get pose ok: " << 100.0 * double(nb_ok) / n << "% (" << nb_ok << "/" << n << ")" << std::endl;
	});

}

void startslam(std::shared_ptr<xv::Device> device)
{
	
	device->imuSensor()->registerCallback([](xv::Imu const & imu) {
			if (true)
			{
				std::cout << "imu      " << timeShowStr(imu.edgeTimestampUs, imu.hostTimestamp) << "@" << "" << "fps" << " Accel(" << imu.accel[0] << "," << imu.accel[1] << "," << imu.accel[2] << "), Gyro(" << imu.gyro[0] << "," << imu.gyro[1] << "," << imu.gyro[2] << ")" << std::endl;
			}
		
	});

}
void orientationCallback(xv::Orientation const& o)
{
	static FpsCount fc;
	fc.tic();
	static int k = 0;
	if (k++ % 100 == 0) {
		auto& q = o.quaternion();
		std::cout << "orientation" << "@" << std::round(fc.fps()) << "fps"
			<< " 3dof=(" << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << "),"
			<< std::endl;
	}
}
int main(int argc, char* argv[]) 
{
	std::shared_ptr<xv::Device> device = nullptr;
	xv::setLogLevel(xv::LogLevel::debug);
	std::string json = "";

	std::cout << "xvsdk version: " << xv::version() << std::endl;

	// return a map of devices with serial number as key, wait at most x seconds if no device detected
	auto devices = xv::getDevices(10., json);

	// if no device: quit
	if (devices.empty()) {
		std::cerr << "Timeout for device detection." << std::endl;
		return EXIT_FAILURE;
	}

	// take the first device in the map
	device = devices.begin()->second;

	std::cout << "device id : "<<device->id() << std::endl;
	xv::setLogLevel(xv::LogLevel(1));
	int imuId = -1;
	if (!device->slam()) {
		std::cerr << "Host SLAM algorithm not supported." << std::endl;
		return EXIT_FAILURE;
	}
	
	std::cout << "-----------start SLAM------------" << std::endl;
	
		if (true) {
			device->imuSensor()->registerCallback([](xv::Imu const & imu) {
				static FpsCount fc;
				fc.tic();
				static int k = 0;
				if (k++ % 500 == 0) {
					if (true)
					{
						std::cout << "imu      " << timeShowStr(imu.edgeTimestampUs, imu.hostTimestamp) << "@" << std::round(fc.fps()) << "fps" << " Accel(" << imu.accel[0] << "," << imu.accel[1] << "," << imu.accel[2] << "), Gyro(" << imu.gyro[0] << "," << imu.gyro[1] << "," << imu.gyro[2] << ")" << std::endl;
					}
				}
			});
		}
		device->slam()->stop();

		
	
	

	/*device->slam()->start(xv::Slam::Mode::Edge);
	std::cout << "-----------start SLAM------------" << std::endl;

	startGetPose(device->slam());*/

	
}


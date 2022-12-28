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
#include <math.h>

#include "colors.h"


#define USE_EX

bool s_stop = false;
static std::map<std::string, int> enableDevMap;

#ifdef USE_EX
#include "xv-sdk-ex.h"
#endif
#ifdef USE_PRIVATE
#include "xv-sdk-private.h"
#endif
#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>


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

std::shared_ptr<const xv::DepthImage> s_tof = nullptr;


std::mutex s_mtx_tof;
std::mutex s_mtx_depthColor;


void display() {
	
	if (enableDevMap["tof"]) {
		cv::namedWindow("TOF");
		cv::moveWindow("TOF", 500, 462);
	}
	cv::waitKey(1);

	while (!s_stop) {
		std::shared_ptr<const xv::DepthImage> tof = nullptr;

		if (enableDevMap["tof"]) {
			s_mtx_tof.lock();
			tof = s_tof;
			s_mtx_tof.unlock();
			if (tof) {
				cv::Mat img = raw_to_opencv(tof);
				if (img.rows > 0 && img.cols > 0)
					cv::imshow("TOF", img);
			}

		}

		cv::waitKey(1);
	}
}
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
std::string timeShowStr(std::int64_t edgeTimestampUs, double hostTimestamp, bool test) {
	char s[1024];
	double now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()*1e-6;
	std::sprintf(s, " %.4f ", now);

	return std::string(s);
}
std::string timeShowStr(double hostTimestamp, int i)
{
	char s[1024];
	double now = hostTimestamp;
	std::sprintf(s, " %.4f ", now);

	return std::string(s);

}
std::string timeShowStr()
{
	char s[1024];
	double now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()*1e-6;
	std::sprintf(s, " %.4f ", now);
	return std::string(s);
}
void showToftime(const xv::DepthImage& tof)
{
	std::cout << tof.hostTimestamp << std::endl;
}
void writepointcloud(std::ofstream &ofs, char buff[128])
{
	//ofs << std::fixed << std::setprecision(8) <<time<<buff<< std::endl;
	ofs << buff << std::endl;
}
int main(int argc, char* argv[]) try
{
	bool isExit = false;
	int my_time = 0;

	std::cout << "xvsdk version: " << xv::version() << std::endl;

	xv::setLogLevel(xv::LogLevel::debug);

	std::string json = "";
	
	enableDevMap["tof"] = true;
	enableDevMap["tof_mode"] = 3;//default lablize sf
	enableDevMap["tof_point_cloud"] = true;


	auto devices = xv::getDevices(10., json);
	if (enableDevMap["log"])
	{
		xv::setLogLevel(xv::LogLevel::debug);
	}
	std::ofstream ofs;
	//std::ofstream ofsply;
	if (devices.empty())
	{
		std::cout << "Timeout: no device found\n";
		return EXIT_FAILURE;
	}

	auto device = devices.begin()->second;
	std::cout << "---------------------------" << std::endl;
	enableDevMap["tof"] &= device->tofCamera() != nullptr;

	if (enableDevMap["tof"]) {

	
		device->tofCamera()->start();
		device->tofCamera()->setSonyTofSetting(xv::TofCamera::SonyTofLibMode::M2MIX_DF,
			xv::TofCamera::Resolution::QVGA,
			xv::TofCamera::Framerate::FPS_30);
		
		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		device->tofCamera()->registerCallback([&](xv::DepthImage const & tof) {
			if (tof.type == xv::DepthImage::Type::Depth_16 || tof.type == xv::DepthImage::Type::Depth_32) {
				static FpsCount fc;
				fc.tic();
				static int k = 0;

				if (tof.dataSize > 0 && k++ % 5 == 0)
				{

					std::string pathname = "./tof_pointcloud_";
					std::string pathtime = timeShowStr(tof.hostTimestamp, 1) + std::to_string(k);
					std::string pathl = ".pcd";
					std::string path = pathname + pathtime + pathl;
					std::ofstream fout_pc_name(path);

					auto points = device->tofCamera()->depthImageToPointCloud(tof)->points;


					int length = points.size();
					fout_pc_name << "# .PCD v0.7 - Point Cloud Data file format" << std::endl;
					fout_pc_name << "VERSION 0.7" << std::endl;
					fout_pc_name << "FIELDS x y z" << std::endl;
					fout_pc_name << "SIZE 4 4 4" << std::endl;
					fout_pc_name << "TYPE F F F" << std::endl;
					fout_pc_name << "COUNT 1 1 1" << std::endl;
					fout_pc_name << "WIDTH " << length << std::endl;
					fout_pc_name << "HEIGHT 1" << std::endl;
					fout_pc_name << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
					fout_pc_name << "POINTS " << length << std::endl;
					fout_pc_name << "DATA ascii" << std::endl;
					for (auto iter = points.begin(); iter != points.end(); iter++)
					{
						auto point = *iter;
						float x = point[0] * 0.001;
						float y = point[1] * 0.001;
						float z = point[2] * 0.001;

						if (z >= 10.0)
						{
							x = 0;
							y = 0;
							z = 0;
						}
						fout_pc_name << x << " " << y << " " << z << std::endl;

					}
					fout_pc_name.close();
				}
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
	}

	std::thread t(display);
	std::cout << " == Initialized ==" << std::endl;

	
	std::cout << " ################## " << std::endl;
	std::cout << "        Start       " << std::endl;
	std::cout << " ################## " << std::endl;

	std::string getkey;

	std::cerr << "ENTER to stop" << std::endl;
	std::cin.get();

	s_stop = true;

	std::cout << " ################## " << std::endl;
	std::cout << "        Stop        " << std::endl;
	std::cout << " ################## " << std::endl;


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

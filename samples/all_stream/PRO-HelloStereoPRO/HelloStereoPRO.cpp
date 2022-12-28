
#include <xv-sdk.h>
#include <iostream>

int main()
{
	std::cout << "xvsdk version: " << xv::version() << std::endl;

	//xv::setLogLevel(xv::LogLevel::debug);
	std::string json = "";
	auto devices = xv::getDevices(10., json);
	auto device = devices.begin()->second;
	std::cout <<"id : "<< device->id() << std::endl;
	std::cout << "Hello StereoPRO" << std::endl;
	
}

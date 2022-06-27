#include <xv-sdk.h>

#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <thread>
#include <mutex>
#include <cstring>
#include <signal.h>

static bool stop = false;

void signal_handler(int /*sig*/)
{
    stop = true;
}

static const int HID_REPORT_SIZE = 63;
static const int HID_BUF_OFFSET = 3;

float rotation_norm(float const* v)
{
    int i;
    float sn = 0.f;
    for (i = 0 ; i < 9 ; ++i)
        sn += v[i]*v[i];
    return std::sqrt(sn);
}


std::ostream& operator<<(std::ostream& o, xv::Calibration const& c);
std::ostream& operator<<(std::ostream& o, std::vector<xv::Calibration> const& c);



int main( int /*argc*/, char* /*argv*/[] ) try
{
    std::cout << "Version: " << xv::version().toString() << std::endl;

    auto devices = xv::getDevices(10.);

    if (devices.empty()) {
        std::cout << "Timeout: no device found\n";
        return EXIT_FAILURE;
    }

    signal(SIGINT, signal_handler);

    auto device = devices.begin()->second;

    std::cout << "Stereo:" << std::endl;
    std::cout << device->fisheyeCameras()->calibration() << std::endl;

    return EXIT_SUCCESS;
}
catch( const std::exception &e){
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

#include <iterator>

template<class F, std::size_t N>
std::ostream& operator<<(std::ostream& o, const std::array<F,N> &v)
{
    o << "[";
    for(int i=0;i<N;i++){
        o << v.at(i);
        if( i<N-1){
            o << ", ";
        }
    }
    o << "]";
    return o;
}

std::ostream& operator<<(std::ostream& o, const std::vector<xv::Calibration> &calibs)
{
    for( auto c: calibs){
        std::cout << c << std::endl;
    }
    return o;
}

std::ostream& operator<<(std::ostream& o, const xv::UnifiedCameraModel &m)
{
    o << "{";
    o << "w=" << m.w << ", ";
    o << "h=" << m.h << ", ";
    o << "fx=" << m.fx << ", ";
    o << "fy=" << m.fy << ", ";
    o << "u0=" << m.u0 << ", ";
    o << "v0=" << m.v0 << ", ";
    o << "xi=" << m.xi;
    o << "}";
    return o;
}

std::ostream& operator<<(std::ostream& o, const xv::PolynomialDistortionCameraModel &m)
{
    o << "{";
    o << "w=" << m.w << ", ";
    o << "h=" << m.h << ", ";
    o << "fx=" << m.fx << ", ";
    o << "fy=" << m.fy << ", ";
    o << "u0=" << m.u0 << ", ";
    o << "v0=" << m.v0 << ", ";
    o << "distor=" << m.distor;
    o << "}";
    return o;
}

std::ostream& operator<<(std::ostream& o, xv::Calibration const& c)
{
    o << "Calibration:" << std::endl;
    o << " R:" << c.pose.rotation() << std::endl;
    o << " T: " << c.pose.translation() << std::endl;
    for(int i=0;i<c.ucm.size();i++){
        o << "UCM" << i << ": " << c.ucm.at(i) << std::endl;
    }
    for(int i=0;i<c.pdcm.size();i++){
        o << "PDCM" << i << ": " << c.pdcm.at(i) << std::endl;
    }
    return o;
}

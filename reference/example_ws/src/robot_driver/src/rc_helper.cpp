//
// Created by ryan on 18-8-30.
//

#include <robot_driver/rc_helper.h>
#include <math.h>
using namespace std;

namespace rc_driver_ros {
    RCHelper::RCHelper() {

    }

    RCHelper::~RCHelper() {
    }

    int16_t RCHelper::getSpeedDEC(double speed, double scale)
    {
        double _rpm = speed * scale;
        int16_t _dec;
        if (_rpm >= 0)
           _dec = floor(_rpm);
        else
           _dec = ceil(_rpm);
        return _dec;    
    }

    boost::array<uint8_t, 2> RCHelper::convert_speed_array_(int16_t speedDEC)
    {
        boost::array<uint8_t, 2> bytes;
        int16_t n = speedDEC;

        bytes[0] = (n >> 8) & 0xFF;
        bytes[1] = n & 0xFF;

        return bytes;
    }

    boost::array<uint8_t, 2> RCHelper::convert_data_array_(uint16_t data)
    {
        boost::array<uint8_t, 2> bytes;
        int16_t n = data;

        bytes[0] = (n >> 8) & 0xFF;
        bytes[1] = n & 0xFF;

        return bytes;
    }

    boost::array<uint8_t, 4> RCHelper::convert_4data_array_(uint32_t data)
    {
        boost::array<uint8_t, 4> bytes;
        uint32_t n = data;

        bytes[0] = (n >> 24) & 0xFF;
        bytes[1] = (n >> 16) & 0xFF;
        bytes[2] = (n >> 8) & 0xFF;
        bytes[3] = n & 0xFF;

        return bytes;
    }

    double RCHelper::getVelocityFromDEC(int16_t dec_, double wheel_meter, double dec_ratio)
    {
        double _vel = (dec_*(pi*wheel_meter))/(dec_ratio);
        return _vel;
    }

}

//
// Created by James on 2020-11-7.
// 用于底盘的数值计算
//
#ifndef ROS_RC_HELPER_H
#define ROS_RC_HELPER_H

#include     <stdio.h>      /*标准输入输出定义*/
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*Unix 标准函数定义*/
#include     <sys/types.h>
#include     <sys/stat.h>
#include     <fcntl.h>      /*文件控制定义*/
#include     <termios.h>    /*PPSIX 终端控制定义*/
#include     <errno.h>      /*错误号定义*/
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>
#include <sensor_msgs/BatteryState.h>
#include <boost/thread.hpp>
#include "rc_values.h"

namespace rc_driver_ros {

    class RCHelper {
    public:
        RCHelper();

        ~RCHelper();

        /**
         * @brief  用于设置速度时的单位换算
         * @param  speed 目标线速度
         * @param  rad 轮半径
         */
        int16_t getSpeedDEC(double speed, double scale);
        boost::array<uint8_t, 2> convert_speed_array_(int16_t speedDEC);
        boost::array<uint8_t, 2> convert_data_array_(uint16_t data);
        boost::array<uint8_t, 4> convert_4data_array_(uint32_t data);
        double getVelocityFromDEC(int16_t dec_, double wheel_meter, double dec_ratio);
    };
}
#endif //

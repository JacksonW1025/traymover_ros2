//
// Created by James on 2020-11-08.
//
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
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <map_msgs/DriverEntry.h>
#include <map_msgs/DriverOdomEntry.h>
#include <map_msgs/DriverDebugEntry.h>
#include <map_msgs/RobotSonarEntrys.h>
#include <map_msgs/AndroidCmd.h>
#include <map_msgs/TestDriver.h>
#include <map_msgs/CleanData.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include "rc_values.h"
#include "rc_helper.h"
#include <ros/ros.h>

namespace rc_driver_ros {
    class Driver_ros
    {
    public:
        Driver_ros(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io);

        virtual ~Driver_ros();    

        int    chassisType_;

        //std::vector<char> hexStringToByteArray(std::string hex);
        //void hexToBytes(const std::string& hex,BYTE* bytes);
    private:
        std::string port_; ///< @brief The serial port the driver is attached to
        uint32_t baud_rate_; ///< @brief The baud rate for the serial connection
        boost::asio::serial_port serial_; ///< @brief Actual serial port object for reading/writing to the battery set
        boost::thread* retrieve_thread_;
        boost::thread* status_thread_;
        ros::Subscriber  cmd_sub_,navi_cmd_sub_,config_update_sub,reInit_motor_sub_,motionmode_sub_;
        ros::Publisher driverinfo_pub_,driverdebuginfo_pub_,navi_cmd_pub_,motor_error_code_pub_;
        RCHelper* rchelper_;
        ros::Time last_time, last_cmdvel_time_, last_navi_cmdvel_time_;

        bool backward_mode_10cm_;
        int  collision_detect_;
        double cliff_data_[3];

        uint8_t lastLeftAlarm_, lastRightAlarm_;

        //float dec_ratio_;
        float speedx_, speedxold_;
        float speedth_, speedthold_;
        float lwheel_pwm_;
        float rwheel_pwm_;
        double odom_rate_;
        double x;
        double y;

        double th1_old_;
        double th_z_;

        int32_t left_encoder_;
        int32_t right_encoder_;
        uint8_t motor_move_alm_;
        uint8_t motor_stop_alm_;
        uint8_t motor_move_count_;
        uint8_t motor_stop_count_;

        float  speed_ratio_;

        double test_odom_distance_, test_odom_speed_, test_odom_rotate_, test_odom_time_;
        int    test_left_encoder_, test_right_encoder_, test_left_encoder_old_, test_right_encoder_old_, encoder_info_l_, encoder_info_r_;
        int    navigation_pause_;
        bool   useSonar_, useLidarCharge_;
        
        bool   cmdvel_change_;
        double cmdvel_timeout_;
        double wheel_diameter_;
        double wheel_track_;
        double motor_gear_reduction_;
        int motor_tick_meter_ratio_;
        double start_rotation_limit_w_;
        double accel_limit_;
        double sonar_height_;
        double sonar_maxval_;
        double line_scale_;
        double motor_line_scale_;
        double rotate_scale_;

        double right_pwm_scale_;
        double left_pwm_scale_;
        double motor_rotate_scale_;
        int stop_rotate_;
        double scan_obstacle_distance_;

        bool      stm32_update_;
        uint8_t   serial_id_;

        map_msgs::DriverDebugEntry driver_debuginfo_;
        map_msgs::DriverEntry   driver_data_;
        _sonars_data  sonars_data_[7];
        _send_motordata send_motordata_, old_send_motordata_;
        _send_motorstatus send_motorstatus_;

        bool driver_path_debug_;
        boost::recursive_mutex run_mutex_;

        int control_mode_;
        int speed_dwa_;
        bool force_reboot_;
        bool autoReInitMotor_;
        ros::Time last_enable_motor_time_;

        int16_t reInitMotorStatus_ = 0; //电机重新初始化标志


        ///内部方法
        bool Check_Baud(void);
        void Set_Motor_Reset(void);
        void Set_Motor_CodeClear(void);
        void Set_Motor_AlarmClear(void);
        void Set_MotorScale(uint16_t leftencoder, uint16_t rightencoder);
        void Set_MotorFilter(uint16_t filter);
        void Get_Motor_Version(void);
        void Set_MotorPWM(int16_t leftenpwm, int16_t rightenpwm);
        void _CHKS_Cal(boost::array<uint8_t, 10> &_origin);
        bool _CHKS_check(uint8_t len, boost::array<uint8_t, 40> _origin);
        void cmd_CB(const geometry_msgs::Twist::ConstPtr& twist);
        void navi_cmd_CB(const geometry_msgs::Twist::ConstPtr& twist);
        void  reinitMotor_CB(const std_msgs::Int16::ConstPtr &msg);
        void Motionmode_CB(const std_msgs::Int16::ConstPtr& msg);
        void Set_Motor_Alarmled_Switch(uint8_t ledswitch);
        void read_odom(void);
        bool read_buffer_data(msg_id_t msg_ID, uint8_t len,  boost::array<uint8_t, 40> &_origin);
        void check_status(void);
        void setZeroVelocity(void);
        bool getdatafromdriver(msg_id_t msg_ID, boost::array<uint8_t, 40> &_origin);
        void config_update_CB(const std_msgs::Int32::ConstPtr& msg);
    };
}

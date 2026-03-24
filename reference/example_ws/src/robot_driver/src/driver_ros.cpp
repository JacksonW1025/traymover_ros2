//
// Created by James on 2020-11-06.
// 用于天使大机器人底盘的驱动
//
#include "robot_driver/driver_ros.h"
#include <boost/exception/all.hpp>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"

using namespace std;
namespace rc_driver_ros {
	Driver_ros::Driver_ros(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io)
	    :port_(port), baud_rate_(baud_rate),x(0.0), y(0.0), serial_(io, port_){
		ros::NodeHandle nh;
		ros::NodeHandle private_nh("~");
		serial_.set_option(boost::asio::serial_port::baud_rate(baud_rate_));
		serial_.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
		serial_.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
		serial_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
		serial_.set_option(boost::asio::serial_port::character_size(8));
        ros::Rate r(1.0);

        navigation_pause_ = 0;
        cmdvel_change_ = false;
        stm32_update_ = false;
        backward_mode_10cm_ = false;
        driver_path_debug_ = false;

        motor_stop_alm_ = 0;
        motor_move_alm_ = 0;
        motor_move_count_ = 0;
        motor_stop_count_ = 0;
        memset(&driver_data_, 0, sizeof(map_msgs::DriverEntry));
        memset(&driver_debuginfo_, 0, sizeof(map_msgs::DriverDebugEntry));
        memset(&send_motordata_, 0, sizeof(_send_motordata));
        memset(&old_send_motordata_, 0, sizeof(_send_motordata));
		memset(&send_motorstatus_, 0, sizeof(_send_motorstatus)); 
        
		rchelper_ = new rc_driver_ros::RCHelper();

        driverinfo_pub_ = nh.advertise<map_msgs::DriverEntry>("driver_info", 5);
        driverdebuginfo_pub_ = nh.advertise<map_msgs::DriverDebugEntry>("driverdebug_info", 1);       
        navi_cmd_pub_ = nh.advertise<geometry_msgs::Twist>("navi_cmd_vel", 10);
        motor_error_code_pub_ = nh.advertise<std_msgs::UInt8MultiArray>("/motor_error_code", 1,true); //发送左右电机的错误码

        cmd_sub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, boost::bind(&Driver_ros::cmd_CB, this, _1));
        navi_cmd_sub_ = nh.subscribe<geometry_msgs::Twist>("/navi_cmd_vel", 1, boost::bind(&Driver_ros::navi_cmd_CB, this, _1));
        motionmode_sub_ = nh.subscribe<std_msgs::Int16>("/motion_mode", 1, boost::bind(&Driver_ros::Motionmode_CB, this, _1)); 
        reInit_motor_sub_ = nh.subscribe<std_msgs::Int16>("/reInit_motor", 1, boost::bind(&Driver_ros::reinitMotor_CB, this, _1));

		private_nh.param("odom_rate", odom_rate_, 50.0);
		private_nh.param("test_odom_distance", test_odom_distance_, 1.0);
		private_nh.param("test_odom_speed", test_odom_speed_, 0.5);
		private_nh.param("test_odom_rotate", test_odom_rotate_, 360.0);
        test_odom_time_ = 0;
        test_left_encoder_ = 0;
        test_right_encoder_ = 0;
        encoder_info_l_ = 0;
        encoder_info_r_ = 0;
        th_z_ = 0;

        speed_ratio_ = 1.0;

        //left_pwm_scale_ = 0.9827;
        //right_pwm_scale_ = 0.9907;
        motor_line_scale_ = 1.0;
        motor_rotate_scale_ = 1.0;
        
        private_nh.param("cmdvel_timeout", cmdvel_timeout_, 0.5);

        Set_MotorScale(motor_line_scale_, motor_rotate_scale_);
        private_nh.param("left_pwm_scale", left_pwm_scale_, 1.0);
		private_nh.param("right_pwm_scale", right_pwm_scale_, 1.0);
        
        private_nh.param("speed_dwa", speed_dwa_, 3);
        private_nh.param("autoReInitMotor", autoReInitMotor_, true);
        private_nh.param("chassisType", chassisType_, 10);
        private_nh.param("useLidarCharge", useLidarCharge_, true);
        private_nh.param("stop_rotate", stop_rotate_, 0);
        private_nh.param("scan_obstacle_distance", scan_obstacle_distance_, 0.3);

		private_nh.param("wheel_diameter", wheel_diameter_, 0.125);
        send_motordata_.wheel_diameter = (uint8_t)(wheel_diameter_ * 1000);
        //方形底盘 0.455，圆形底盘0.405米
		private_nh.param("wheel_track", wheel_track_, 0.355);
        send_motordata_.wheel_track = (uint16_t)(wheel_track_ * 1000);
		private_nh.param("motor_gear_reduction", motor_gear_reduction_, 600.0);
        send_motordata_.gear_reduction = (uint32_t)(motor_gear_reduction_);
		
        // 对中菱电机值为1， 对和利时，配置为10
        private_nh.param("tick_meter_ratio", motor_tick_meter_ratio_, 1);
        send_motordata_.tick_meter_ratio = (uint16_t)(motor_tick_meter_ratio_);
        x = 0;
        y = 0;
        force_reboot_ = false;

        uint8_t test_count = 0;
        while ((Check_Baud() == false) && (stm32_update_ == false) && (test_count < 5)) {
            test_count++;
            ROS_INFO("baud is not correct, driver init fail!  retry again");
            r.sleep();
        }  

        //Set_Motor_Reset();
        Set_Motor_CodeClear();
        Set_Motor_AlarmClear();
        Set_Motor_Alarmled_Switch(1);

        lastLeftAlarm_ = 0;
        lastRightAlarm_ = 0;
        last_enable_motor_time_ = ros::Time::now();

		retrieve_thread_ = new boost::thread(boost::bind(&Driver_ros::read_odom, this));
        status_thread_ = new boost::thread(boost::bind(&Driver_ros::check_status, this));


        config_update_sub = nh.subscribe<std_msgs::Int32>("/config_update", 5, boost::bind(&Driver_ros::config_update_CB, this, _1));

        ROS_INFO("connected on rikibase port, baud is %d, driver initialize finished", baud_rate_);
    }

    Driver_ros::~Driver_ros(){

        retrieve_thread_->interrupt();
        retrieve_thread_->join();
        delete retrieve_thread_;

        status_thread_->interrupt();
        status_thread_->join();
        delete status_thread_;

        setZeroVelocity();
    }

    //检查与驱动器通信的波特率是否与预期的波特率匹配
    bool Driver_ros::Check_Baud(void) {
        // baud
        uint32_t baud = 0;
        bool driver_result = false;
        boost::array<uint8_t, 40> response_bytes;
        
        driver_result = getdatafromdriver(MSG_ID_GET_BAUD, response_bytes); 
        if (driver_result){
            baud = (int32_t)response_bytes[8]+((int32_t)response_bytes[7]<<8) + ((int32_t)response_bytes[6]<<16) + ((int32_t)response_bytes[5]<<24);
        }

        ROS_INFO("-----------------Check_Baud, baud: %d-----------------------------", baud);
       
        if (baud == baud_rate_)
                return true;
        else
            return false;
    }

    //设置报警led
    void Driver_ros::Set_Motor_Alarmled_Switch(uint8_t ledswitch) {

        send_motordata_.alarmled_switch = ledswitch;
        ROS_INFO("-----------------Set_Motor_Alarmled_Switch %d is called-----------------------------", ledswitch);

    }

    //设置电机重置状态
    void Driver_ros::Set_Motor_Reset(void) {
        send_motorstatus_.motor_reset = 1;
        ROS_INFO("-----------------Set_Motor_Reset is called-----------------------------");
    }

    //清除电机的错误码
    void Driver_ros::Set_Motor_CodeClear(void) {
        send_motorstatus_.clear_motorcode = 1;
        ROS_INFO("-----------------Set_Motor_CodeClear is called-----------------------------");
    }

    //清除电机的报警信号
    void Driver_ros::Set_Motor_AlarmClear(void){
        send_motorstatus_.clear_motoralarm = 1;
        ROS_INFO("-----------------Set_Motor_AlarmClear is called-----------------------------");
    }
    
    //获取电机版本信息
    void Driver_ros::Get_Motor_Version(void){
        send_motorstatus_.get_ver = 1;
        ROS_INFO("-----------------Get_Motor_Version is called-----------------------------");
    }

    //设置左右电机的线性和旋转比例
    void Driver_ros::Set_MotorScale(uint16_t leftscale, uint16_t rightscale){
        send_motordata_.leftmotor_scale = leftscale;
        send_motordata_.rightmotor_scale = rightscale;
        ROS_INFO("-----------------Set_MotorScale is called, leftmotor_scale: %d, rightmotor_scale: %d-----------------------------", send_motordata_.leftmotor_scale, send_motordata_.rightmotor_scale);
    }

    //设置电机的滤波器参数
    void Driver_ros::Set_MotorFilter(uint16_t filter){        
        send_motordata_.motor_filter = filter;
        ROS_INFO("-----------------Set_MotorFilter is called, motor_encoderscale_: %d-----------------------------", send_motordata_.motor_filter);
    }

    //设置左右电机的PWM
    void Driver_ros::Set_MotorPWM(int16_t leftenpwm, int16_t rightenpwm) {
        send_motordata_.leftPWM = leftenpwm;
        send_motordata_.rightPWM = rightenpwm;
        ROS_INFO("-----------------Set_MotorPWM is called, leftenpwm: %d, rightenpwm: %d-----------------------------", send_motordata_.leftPWM, send_motordata_.rightPWM);
    }

    //处理改变自动充电模式、手动模式的改变
    void Driver_ros::Motionmode_CB(const std_msgs::Int16::ConstPtr &msg) {
        if (msg->data == 1) {
            // autocharge mode
            if (useLidarCharge_ == false){
                driver_data_.Mode = 1;
                send_motordata_.mode = 1;
            }
        }
        else if (msg->data == 3) {
            // out of mode
            driver_data_.Mode = 3;
            send_motordata_.mode = 3;
        } 
        else {
            // manu mode
            driver_data_.Mode  = 2;
            send_motordata_.mode = 2; 
        }
        ROS_INFO("-----------------Motionmode_CB called mode: %d-----------------------------", msg->data);
    }

    void  Driver_ros::reinitMotor_CB(const std_msgs::Int16::ConstPtr &msg) {
        reInitMotorStatus_ = msg->data;
    }

    //更新配置参数 线性和旋转比例
    void Driver_ros::config_update_CB(const std_msgs::Int32::ConstPtr& msg){
        if (msg->data ==1){
            ros::NodeHandle private_nh("~");
            private_nh.param("left_pwm_scale", left_pwm_scale_, 1.0);
            private_nh.param("right_pwm_scale", right_pwm_scale_, 1.0);
        }
    }

    // cmd_vel发布的速度，需要做smooth处理
    void Driver_ros::cmd_CB(const geometry_msgs::Twist::ConstPtr &twist_) {
        // 判断当前速度值
        geometry_msgs::Twist cmd_vel;
        double max_x=0, max_th=0, min_x=0, min_th=0;
        int run_mode = 0;   //  0 stop 1 forward 2 backword 3 turn left 4 turn right
        ros::Time current_time = ros::Time::now();

        if(twist_->linear.x > 0)
            run_mode = 1;
        else if(twist_->linear.x < 0)
            run_mode = 2;
        else if(twist_->angular.z > 0)
            run_mode = 3;
        else if(twist_->angular.z < 0)
            run_mode = 4;
        else
            run_mode = 0;

        float dt = (current_time - last_cmdvel_time_).toSec();
        if (dt > 1.0){
            speedth_ = 0.0;
            speedx_ = 0.0;
            control_mode_ = 0;
            last_cmdvel_time_ = current_time;
            dt = 0;
        }

        control_mode_ = 1;    // 手动控制模式

        if (run_mode == 1){
            // 前进
            max_x = std::max(0.1, speedx_ + 0.1 * dt);
            min_x = speedx_ - 1.2 * dt;
            cmd_vel.linear.x = std::min(std::max(twist_->linear.x, min_x), max_x);
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
        }else if (run_mode == 2){
            // 后退
             if (driver_data_.ChargeStatus ==1 || driver_data_.ChargeStatus ==3){
                //自动充电中,禁止后退
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.0;
            }else{
                min_x = std::min(-0.1, speedx_ - 0.1 * dt);
                max_x = speedx_ + 1.2 * dt;
                cmd_vel.linear.x = std::min(std::max(twist_->linear.x, min_x), max_x);
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.0;
            }
        }else if (run_mode == 3){
            // 左转
            max_th = std::max(0.1, speedth_ + 0.5 * dt);
            min_th = speedth_ - 1.2 * dt;
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = std::min(std::max(twist_->angular.z, min_th), max_th);
        }else if (run_mode == 4){
            // 右转
            min_th = std::min(-0.1, speedth_ - 0.5 * dt);
            max_th = speedth_ + 1.2 * dt;
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = std::min(std::max(twist_->angular.z, min_th), max_th);
        }      

        if(twist_->linear.x == 0){
            run_mode = 0;
            speedx_ = speedx_ / 2;
            cmd_vel.linear.x = speedx_;
        }
        if(twist_->angular.z == 0){
            run_mode = 0;
            speedth_ = speedth_ / 2;
            cmd_vel.angular.z = speedth_;
        }

        navi_cmd_pub_.publish(cmd_vel);
        last_cmdvel_time_ = ros::Time::now();

        ROS_INFO("Driver, publish cmd_vel: (%f, %f), dt: %f", cmd_vel.linear.x, cmd_vel.angular.z, dt);
    }

    ///获取navi_vel命令
    void Driver_ros::navi_cmd_CB(const geometry_msgs::Twist::ConstPtr &twist_) {
        geometry_msgs::Twist twist = *twist_;
        float rc_vel_pid = twist.linear.x;
        float rc_rot_pid = twist.angular.z;
        bool result = false;

        float rc_vel = rc_vel_pid;
        float rc_rot = rc_rot_pid;

        // 记录当前速度
        speedx_ = rc_vel;
        speedth_ = rc_rot;

        last_navi_cmdvel_time_ = ros::Time::now();
        cmdvel_change_ = true;

        if (driver_data_.ChargeStatus ==2 || driver_data_.ChargeStatus ==4){
            //手动充电中,停止动作
            send_motordata_.leftPWM  = 0;
            send_motordata_.rightPWM = 0; 
        }else{
            if (driver_data_.ChargeStatus ==1 || driver_data_.ChargeStatus ==3){
                // 在充电中，先出桩
                if (rc_vel_pid < 0 || rc_rot_pid !=0){
                    return;
                }
            }
        }

        if ((driver_data_.EmergencyStatus == 1) || (navigation_pause_ == 1) || (driver_data_.MotorEnabled == 1)) {
            // 按下了紧急暂停按钮
            rc_vel = 0;
            rc_rot = 0;
            speed_ratio_ = 0.0;
        }
		
		//collision_detect
        if((collision_detect_ == 1) && (backward_mode_10cm_ == false) && (rc_vel > 0)){
            //backward 10cm
            rc_vel = 0;
            rc_rot = 0;
            backward_mode_10cm_ = true;
            test_left_encoder_ = test_odom_distance_ * 10000;
            test_right_encoder_ = test_odom_distance_ * 10000;
            ROS_WARN("WARNING:Collision sensor has triggered, Backward! rc_vel: %f, rc_rot: %f", rc_vel, rc_rot);
        }else if(backward_mode_10cm_ == true){
            rc_vel = -0.2;
            rc_rot = 0;
            if(test_left_encoder_ <= 0){
                rc_vel = 0;
                rc_rot = 0;
                backward_mode_10cm_ = false;
            }
        }
        // 更新当前速度
        speedx_ = rc_vel;
        speedth_ = rc_rot;

        speedxold_ = speedx_;
        speedthold_ = speedth_;
		
	    //设置驱动PWM
        if (driver_data_.ChargeStatus ==2 || driver_data_.ChargeStatus ==4){
            //手动充电中,停止动作
            send_motordata_.leftPWM  = 0;
            send_motordata_.rightPWM = 0; 
        }else{
            send_motordata_.leftPWM  = rc_vel * 1000;
            send_motordata_.rightPWM = rc_rot * 1000;            
        }       
        //ROS_INFO("wheel_pwm_: (%f,%f), motorPWM(%d, %d)", lwheel_pwm_, rwheel_pwm_, send_motordata_.leftPWM ,  send_motordata_.rightPWM);
     }

    ///定时读取odom
    void Driver_ros::read_odom(void){
        bool result = false;
        uint8_t return_type = 0;
        int16_t _tempdata=0;
        uint16_t _utempdata=0;
        int16_t encoder_l=0,encoder_r=0, speedx=0, speedth=0; 
        double dt;
        double testdt;
        ros::NodeHandle n;        
        
        std_msgs::Float32 testpublishdata;
        std_msgs::Int16 outofchargetdata;
        boost::array<uint8_t, 40> response_bytes;
        uint8_t print_count = 0;
        uint8_t end_test  = 0;

        last_time = ros::Time::now();

        ros::Rate r(odom_rate_);
        ros::Rate imu_rate(1);
          
        bool read_driver_result = false;

        while (n.ok()) {
            if (stm32_update_ == false){ 
                boost::recursive_mutex::scoped_lock _lock(run_mutex_);
                try {
                    ros::Time current_time = ros::Time::now(); 

                    float cmd_dt = (current_time - last_cmdvel_time_).toSec();
                    if (cmd_dt > 1.0){
                        control_mode_ = 0;
                    }
    
                    //查询               
                    read_driver_result = getdatafromdriver(MSG_ID_GET_MOTOR_DATA, response_bytes);
                    return_type = response_bytes[4];
                    if ((return_type == w_success)) {
                        //   多位数据，高位在前，低位在后
                        // data:[0 1  2    3    4         5         6       7         8         9          10       1112    1314    1516  1718 1920 2122  2324   2526    2728   2930    3132   33    34        3536      37    3839]    
                        //       head len cmd result emergerncy initstatus power chargestatus leftAlarm rightAlarm leftenc rightenc front  up  left right left_M right_M back  speedx  speedth mode  reversed  version check tail
                        if (read_driver_result){
                            // get Emergency 
                            driver_data_.EmergencyStatus  = response_bytes[5];
                            // get Motor init
                            driver_data_.MotorInitStatus  = response_bytes[6];
                            // get Power
                            driver_data_.Power            = response_bytes[7];
                            // get charge status
                            driver_data_.ChargeStatus     = response_bytes[8]; 
                        
                            // get leftmotor Alarm
                            driver_data_.LeftAlarm  = response_bytes[9];
                            // get rightmotor ALarm
                            driver_data_.RightAlarm = response_bytes[10];
                            if(lastLeftAlarm_ != driver_data_.LeftAlarm || lastRightAlarm_ != driver_data_.RightAlarm){
                                // 发布电机报警
                                std_msgs::UInt8MultiArray motor_error_code;
                                motor_error_code.data.clear();
                                motor_error_code.data.push_back(driver_data_.LeftAlarm);
                                motor_error_code.data.push_back(driver_data_.RightAlarm);
                                motor_error_code_pub_.publish(motor_error_code);
                                lastLeftAlarm_ = driver_data_.LeftAlarm;
                                lastRightAlarm_ = driver_data_.RightAlarm;
                                last_enable_motor_time_ = ros::Time::now();
                            }
                            // calculate stop alarm and move alarm
                            driver_data_.MoveAlarm  =  motor_move_alm_;
                            driver_data_.StopAlarm  =  motor_stop_alm_;

                            //  encoder data
                            _tempdata = ((int16_t)response_bytes[11]<<8) + ((int16_t)response_bytes[12]);
                            driver_data_.LeftEncoder = _tempdata;
                            encoder_l = _tempdata;
                            encoder_l = int16_t((double(encoder_l)*1000.0/left_pwm_scale_)/1000.0);
                            left_encoder_ += encoder_l;

                            _tempdata = ((int16_t)response_bytes[13]<<8) + ((int16_t)response_bytes[14]);
                            driver_data_.RightEncoder = _tempdata;
                            encoder_r = _tempdata;
                            encoder_r = int16_t((double(encoder_r)*1000.0/right_pwm_scale_)/1000.0);

                            right_encoder_ += encoder_r;                    

                            //testdt = (ros::Time::now() - last_test_time_).toSec();
                            test_left_encoder_ += encoder_l;
                            test_right_encoder_ += encoder_r;

                            encoder_info_l_ += encoder_l;
                            encoder_info_r_ += encoder_r;

                            // PWM current data
                            _tempdata = ((int16_t)response_bytes[29]<<8) + ((int16_t)response_bytes[30]);
                            driver_data_.Speedx = _tempdata;
                            speedx = _tempdata;

                            _tempdata = ((int16_t)response_bytes[31]<<8) + ((int16_t)response_bytes[32]);
                            driver_data_.Speedth = _tempdata;
                            speedth = _tempdata;

                            //ROS_INFO("getspeed: (%d,%d), setspeed: (%f, %f)", driver_data_.Speedx, driver_data_.Speedth, send_motordata_.leftPWM, send_motordata_.rightPWM );

                            // get motor mode
                            if (response_bytes[33] == 2){
                                if (driver_data_.Mode < 1 || driver_data_.Mode > 2){
                                    driver_data_.Mode  = response_bytes[33];
                                } 
                            }else if (response_bytes[33] < 1 || response_bytes[33] > 2){
                                if (driver_data_.Mode == 1 || driver_data_.Mode == 2){
                                    driver_data_.Mode  = response_bytes[33];
                                } 
                            }                    
                        
                            //colision detect
                            if(((response_bytes[34] >> 7) & 0x01) == 0x01) {   
                                //collision detected
                                collision_detect_ = 1;
                            }else{
                                collision_detect_ = 0;
                            }

                            // get dipan version
                            _tempdata = ((int16_t)response_bytes[35]<<8) + ((int16_t)response_bytes[36]);
                            driver_data_.Version  = _tempdata;

                            test_right_encoder_old_ = test_right_encoder_;
                            test_left_encoder_old_ = test_left_encoder_;

                            // 导航主板重启
                            if (force_reboot_)
                                driver_data_.Shutdown = 2;
                            else    
                                driver_data_.Shutdown = response_bytes[28];

                            // 电机锁
                            driver_data_.MotorEnabled = response_bytes[27];
                        }
                    }

                    driverinfo_pub_.publish(driver_data_);

                dt = (current_time - last_navi_cmdvel_time_).toSec();
                if ((dt > cmdvel_timeout_) && (cmdvel_change_ == true)) {
                    cmdvel_change_ = false;

                    lwheel_pwm_ = 0;
                    rwheel_pwm_ = 0;
                    send_motordata_.leftPWM = 0;
                    send_motordata_.rightPWM = 0;
                    ROS_INFO("cmd setting timeout, dt = %f > cmdvel_timeout = %f", dt, cmdvel_timeout_);
                }
               
                if (control_mode_ == 0  && (driver_data_.ChargeStatus > 0 && driver_data_.ChargeStatus <5)){
                    //设置驱动PWM，在充电中，不是手动控制或者进行出桩活动，都要立马停止
                    lwheel_pwm_ = 0;
                    rwheel_pwm_ = 0;
                    send_motordata_.leftPWM  = 0;
                    send_motordata_.rightPWM = 0;    
                }
              } catch (std::exception const &ex) {
                ROS_ERROR("An exception was thrown: %s", ex.what());
                }
                _lock.unlock();
            }else{
                print_count++;
                if (print_count > 50){
                    print_count = 0;
                    ROS_INFO("stm32 is updating now, stop polling.......");
                }
            }

            //如果开启自动使能，则自动使能
            if(autoReInitMotor_ && (lastLeftAlarm_ !=0 || lastRightAlarm_ != 0)){
                if (ros::Time::now() - last_enable_motor_time_ > ros::Duration(5.0)){
                    // 自动使能
                    reInitMotorStatus_ = 1 ;
                    last_enable_motor_time_ = ros::Time::now();
                    ROS_INFO("Auto reinit motor, lastLeftAlarm: %d, lastRightAlarm: %d", lastLeftAlarm_, lastRightAlarm_);
                }
            }
            r.sleep();
        }   
    }

    ///读取状态
    void Driver_ros::check_status(void){
        ros::NodeHandle n;
        uint8_t return_type = 0;
        bool check_driver_result = false;
        std_msgs::Int16 publishdata;
        
        boost::array<uint8_t, 40> response_bytes;
        last_time = ros::Time::now();

        ros::Rate imu_rate(1);
        
        while (n.ok()) {
                boost::recursive_mutex::scoped_lock _lock(run_mutex_);
                try {
                    // 获取状态值
                    check_driver_result = getdatafromdriver(MSG_ID_GET_MOTOR_STATUS, response_bytes);
                    if (check_driver_result){                    
                        // data:[0 1  2   3   4      5   6    7     8    9    10   11  12   13   14      15         16      17          18         19          20 21      22 23       24           25 26    27~36   37    3839]   
                        //       head len cmd result bat_temp batdianliu mode leftpwm rightpwm movedir androidcmd leftcode frontcode rightcode  sonar_switch left_scale right_scale motor_filter  version  reserve  check tail
                        return_type = response_bytes[4];

                        if (return_type == w_success) {
                            
                            // get batery temperature
                            driver_debuginfo_.batTemperature  = ((int16_t)response_bytes[5]<<8) + ((int16_t)response_bytes[6]);

                            // get batery current
                            driver_debuginfo_.BatDianliu  = ((int16_t)response_bytes[7]<<8) + ((int16_t)response_bytes[8]);

                            // get motor mode
                            driver_debuginfo_.DipanMode  = response_bytes[9];

                            // get left motor pwm
                            driver_debuginfo_.Left_PWM  = ((int16_t)response_bytes[10]<<8) + ((int16_t)response_bytes[11]);
                        
                            // get right motor pwm
                            driver_debuginfo_.Right_PWM  = ((int16_t)response_bytes[12]<<8) + ((int16_t)response_bytes[13]);

                            // get move dir
                            driver_debuginfo_.Move_DIR  = response_bytes[14];

                            // get android command
                            driver_debuginfo_.Android_Command  = response_bytes[15];

                            // get left code
                            driver_debuginfo_.Left_Code  = response_bytes[16];

                            // get front code
                            driver_debuginfo_.Front_Code  = response_bytes[17];

                            // get right code
                            driver_debuginfo_.Right_Code  = response_bytes[18];

                            // get sonar switch
                            driver_debuginfo_.Dipan_Sonarswitch  = response_bytes[19];

                            // get left scale
                            driver_debuginfo_.Dipan_Leftscale  = ((int16_t)response_bytes[20]<<8) + ((int16_t)response_bytes[21]);

                            // get right scale
                            driver_debuginfo_.Dipan_Rightscale  = ((int16_t)response_bytes[22]<<8) + ((int16_t)response_bytes[23]);

                            // get motor filter
                            driver_debuginfo_.Dipan_Motorfilter  = response_bytes[24];

                            // get motor version
                            driver_debuginfo_.Dipan_Version  = ((int16_t)response_bytes[25]<<8) + ((int16_t)response_bytes[26]);


                            driverdebuginfo_pub_.publish(driver_debuginfo_);  
                        }    
                    }
                } catch (std::exception const &ex) {
                    ROS_ERROR("An exception was thrown: %s", ex.what());
                }   
                _lock.unlock();
                imu_rate.sleep();
        }        
    }

    // 平滑函数
    std::pair<int16_t, int16_t> smooth_pwm_auto(int16_t target_left_pwm,
                                                int16_t target_right_pwm,
                                                float alpha_base = 0.8f,
                                                float max_accel_pwm_per_sec = 400.0f,
                                                float dt_limit = 0.05f,
                                                float max_target_delta = 400.0f)
    {
        // 静态变量保存上次状态
        static float last_left_pwm = 0;
        static float last_right_pwm = 0;
        static ros::Time last_time = ros::Time::now();

        ros::Time now = ros::Time::now();
        float dt = (now - last_time).toSec();
        last_time = now;

        if (dt > dt_limit) dt = dt_limit;
        float max_delta = max_accel_pwm_per_sec * dt;

        //---
        float delta_target_left = static_cast<float>(target_left_pwm) - last_left_pwm;
        float alpha_left = alpha_base + (fabs(delta_target_left)/max_target_delta) * (1.0f - alpha_base);
        if (alpha_left > 1.0f) alpha_left = 1.0f;
        float desired_left = last_left_pwm + alpha_left * delta_target_left;

        float delta_left = desired_left - last_left_pwm;
        if (delta_left > max_delta) delta_left = max_delta;
        if (delta_left < -max_delta) delta_left = -max_delta;
        last_left_pwm += delta_left;
        int16_t corrected_left_pwm = static_cast<int16_t>(last_left_pwm);

        // - ---
        float delta_target_right = static_cast<float>(target_right_pwm) - last_right_pwm;
        float alpha_right = alpha_base + (fabs(delta_target_right)/max_target_delta) * (1.0f - alpha_base);
        if (alpha_right > 1.0f) alpha_right = 1.0f;
        float desired_right = last_right_pwm + alpha_right * delta_target_right;

        float delta_right = desired_right - last_right_pwm;
        if (delta_right > max_delta) delta_right = max_delta;
        if (delta_right < -max_delta) delta_right = -max_delta;
        last_right_pwm += delta_right;
        int16_t corrected_right_pwm = static_cast<int16_t>(last_right_pwm);

        return {corrected_left_pwm, corrected_right_pwm};
    }

    //与电机驱动进行通信，发送请求并接收响应
    bool Driver_ros::getdatafromdriver(msg_id_t msg_ID, boost::array<uint8_t, 40> &_origin) {
        boost::array<uint8_t, 40> response_bytes;
        bool read_driver_result = false;
        uint8_t total_count = 0;
        uint8_t android_cmd = 0;

        boost::array<uint8_t, 40> _send_motorbaud = get_baud; 
        boost::array<uint8_t, 40> _send_motorstatus = get_motor_status;
        boost::array<uint8_t, 40> _send_motordata = get_motor_data;

        switch (msg_ID) {
            case MSG_ID_GET_BAUD:
                // calculate CHKS byte              
                for (int i = 0; i < 37; i++) {
                    total_count = total_count + _send_motorbaud[i];
                }
                total_count = (~total_count)+0x01;
                _send_motorbaud[37] = total_count;

                boost::asio::write(serial_, boost::asio::buffer(_send_motorbaud));
                read_driver_result = read_buffer_data(MSG_ID_GET_BAUD, 40, response_bytes);
                _origin = response_bytes;
                break;
            case MSG_ID_GET_MOTOR_STATUS:
                // fill the send data
                _send_motorstatus[4] = send_motorstatus_.motor_reset; 
                send_motorstatus_.motor_reset = 0;
                _send_motorstatus[5] = send_motorstatus_.get_ver;
                send_motorstatus_.get_ver = 0;
                _send_motorstatus[6] = send_motorstatus_.clear_motorcode;
                send_motorstatus_.clear_motorcode = 0;
                _send_motorstatus[7] = send_motorstatus_.clear_motoralarm;
                send_motorstatus_.clear_motoralarm = 0;

                // calculate CHKS byte              
                for (int i = 0; i < 37; i++) {
                    total_count = total_count + _send_motorstatus[i];
                }
                total_count = (~total_count)+0x01;
                _send_motorstatus[37] = total_count;

                boost::asio::write(serial_, boost::asio::buffer(_send_motorstatus));
                read_driver_result = read_buffer_data(MSG_ID_GET_MOTOR_STATUS, 40, response_bytes);
                _origin = response_bytes;
                break; 
            case MSG_ID_GET_MOTOR_DATA:
                boost::array<uint8_t, 2> tmep_arr;
                boost::array<uint8_t, 4> tmep_arr4;

                // fill the send data
                if (send_motordata_.move_cmd != old_send_motordata_.move_cmd) {
                    _send_motordata[4] = send_motordata_.move_cmd;
                    send_motordata_.move_cmd = 255;
                    tmep_arr = rchelper_->convert_data_array_(send_motordata_.move_distance);
                    _send_motordata[5] = tmep_arr[0];
                    _send_motordata[6] = tmep_arr[1];
                    android_cmd = 1;
                }
                if (send_motordata_.mode != 0){
                    _send_motordata[7] = send_motordata_.mode;
                    send_motordata_.mode = 0;
                }
                _send_motordata[8] = send_motordata_.sonar_switch;

                _send_motordata[9] = reInitMotorStatus_;

                if (android_cmd == 0){
                    auto pwm_pair = smooth_pwm_auto(send_motordata_.leftPWM, send_motordata_.rightPWM);

                    boost::array<uint8_t, 2> tmep_arr = rchelper_->convert_speed_array_(pwm_pair.first);
                    _send_motordata[10] = tmep_arr[0];
                    _send_motordata[11] = tmep_arr[1];
                    tmep_arr = rchelper_->convert_speed_array_(send_motordata_.rightPWM);
                    _send_motordata[12] = tmep_arr[0];
                    _send_motordata[13] = tmep_arr[1];

                }else{
                    _send_motordata[10] = 0;
                    _send_motordata[11] = 0;
                    _send_motordata[12] = 0;
                    _send_motordata[13] = 0;
                }

                if ((send_motordata_.leftPWM == 0) && (send_motordata_.rightPWM ==0)){
                    // stop 
                    if ((driver_data_.LeftEncoder !=0) || (driver_data_.RightEncoder !=0)){
                        motor_stop_count_++;
                        // 跟odom频率有关系
                        if (motor_stop_count_ > 100)
                        {
                            motor_stop_alm_ = 1;
                        }
                    }else{
                        motor_stop_count_ = 0;
                        motor_stop_alm_ = 0;
                    }
                }else{
                    // move
                    // 跟odom频率有关系
                    if ((driver_data_.LeftEncoder ==0) && (driver_data_.RightEncoder ==0)){
                        motor_move_count_++;
                        if (motor_move_count_ > 100)
                        {
                            motor_move_alm_ = 1;
                        }
                    }else{
                        motor_move_count_ = 0;
                        motor_move_alm_ = 0;
                    }  
                }

                tmep_arr = rchelper_->convert_data_array_(send_motordata_.leftmotor_scale);
                _send_motordata[14] = tmep_arr[0];
                _send_motordata[15] = tmep_arr[1];
                tmep_arr = rchelper_->convert_data_array_(send_motordata_.rightmotor_scale);
                _send_motordata[16] = tmep_arr[0];
                _send_motordata[17] = tmep_arr[1];

                // update motor chip
                if (send_motordata_.updatemotor == 1){
                    _send_motordata[18] = send_motordata_.updatemotor;
                    send_motordata_.updatemotor = 0;
                }

                tmep_arr = rchelper_->convert_speed_array_(send_motordata_.motor_filter);
                _send_motordata[19] = tmep_arr[0];
                _send_motordata[20] = tmep_arr[1];

                _send_motordata[21] = send_motordata_.alarmled_switch;
				
				_send_motordata[22] = send_motordata_.cliff_switch;

                _send_motordata[23] = cliff_data_[0];
                _send_motordata[24] = cliff_data_[1];
                _send_motordata[25] = cliff_data_[2];
				
                _send_motordata[26] = send_motordata_.wheel_diameter;
               
                tmep_arr4 = rchelper_->convert_4data_array_(send_motordata_.gear_reduction);
                _send_motordata[27] = tmep_arr4[2];
                _send_motordata[28] = tmep_arr4[3];
                _send_motordata[34] = tmep_arr4[0];
                _send_motordata[35] = tmep_arr4[1];
                
                tmep_arr = rchelper_->convert_speed_array_(send_motordata_.wheel_track);
                _send_motordata[29] = tmep_arr[0];
                _send_motordata[30] = tmep_arr[1];

                _send_motordata[31] = send_motordata_.tick_meter_ratio;

                // calculate CHKS byte              
                for (int i = 0; i < 37; i++) {
                    total_count = total_count + _send_motordata[i];
                }
                total_count = (~total_count)+0x01;
                _send_motordata[37] = total_count;
                old_send_motordata_ = send_motordata_;
                try{
                    boost::asio::write(serial_, boost::asio::buffer(_send_motordata));
                }catch (std::exception const &ex) {
                    ROS_ERROR("write 111 An exception was thrown: %s", ex.what());
                }
                read_driver_result = read_buffer_data(MSG_ID_GET_MOTOR_DATA, 40, response_bytes);
                _origin = response_bytes;
                break;             
            default:
                break;
        }
        return read_driver_result;
    }

    ///设置速度0
    void Driver_ros::setZeroVelocity(void){
        Set_MotorPWM(0, 0);
    }

    ///串口数据CHECKSUM检查
    bool Driver_ros::_CHKS_check(uint8_t len, boost::array<uint8_t, 40> _origin){
        uint8_t total_count = 0;
        if (len > 40)
            return false;

        for (int i = 0; i < (len - 3); i++) {
            total_count = total_count + _origin[i];
        }
        total_count = (~total_count)+0x01;
        if (_origin[len - 3] == total_count) {
            return true;
        } else {
            ROS_INFO("_CHKS_check fail, total_count: %d, _origin[%d - 3]: %d", total_count, len, _origin[len - 3]);
            return false;
        }
    }

    ///读取数据并返回数据
    bool Driver_ros::read_buffer_data(msg_id_t msg_ID, uint8_t len,  boost::array<uint8_t, 40> &_origin){
        boost::array<uint8_t, 40> response_bytes;
        boost::system::error_code err;
        boost::asio::streambuf response;
        uint8_t ret;
        for (int i=0; i< 40; i++)
            response_bytes[i] = 0;
        if (len > 40) {
            ROS_INFO("read_buffer_data error, len is more than 40!!!");
            return false;
        }

        // ret = serial_.read_some(boost::asio::buffer(&receive_bytes[0], 64), err);
        ret = boost::asio::read_until(serial_, response, "\r\n", err);
        int size = response.size();
        if (size > 40)
        {
            ROS_WARN("size is more than 40, size: %d, ERROR", size);
            return false;
        }
        char buffer[size + 1];
        response.sgetn(buffer, size);
        for (int i=0; i<size; i++)
            response_bytes[i] = buffer[i];
            
        if (err) {
            ROS_INFO("read_buffer_data, error!!!");
            return true;
        }
        if (_CHKS_check(len, response_bytes)) {
            if (response_bytes[3] == msg_ID) {
                uint8_t return_type = response_bytes[4];
                switch(return_type){
                    case w_success:
                        //ROS_INFO("read_buffer_data_success: Read a command(0x%x) callback successfully. command(%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x)",
                        //                response_bytes[2],response_bytes[0],response_bytes[1],response_bytes[2],response_bytes[3], response_bytes[4],response_bytes[5],response_bytes[6], response_bytes[7],response_bytes[8],
                        //                response_bytes[9], response_bytes[10], response_bytes[11],response_bytes[12],response_bytes[13],response_bytes[14],
                        //                response_bytes[15],response_bytes[16],response_bytes[17],response_bytes[18],response_bytes[19],response_bytes[20],
                        //                response_bytes[21],response_bytes[22],response_bytes[23],response_bytes[24],response_bytes[25],response_bytes[26],
                        //                response_bytes[27],response_bytes[28],response_bytes[29],response_bytes[30],response_bytes[31],response_bytes[32],
                        //                response_bytes[33],response_bytes[34],response_bytes[35],response_bytes[36],response_bytes[37],response_bytes[38],
                        //                response_bytes[39]);
                        _origin = response_bytes;
                        return true;
                    case wr_error:
                        ROS_INFO("read_buffer_data_error, An exception was thrown after call 0x%x with command(%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x)",
                                        response_bytes[2],response_bytes[0],response_bytes[1],response_bytes[2],response_bytes[3], response_bytes[4],response_bytes[5],response_bytes[6], response_bytes[7],response_bytes[8],
                                        response_bytes[9], response_bytes[10], response_bytes[11],response_bytes[12],response_bytes[13],response_bytes[14],
                                        response_bytes[15],response_bytes[16],response_bytes[17],response_bytes[18],response_bytes[19],response_bytes[20],
                                        response_bytes[21],response_bytes[22],response_bytes[23],response_bytes[24],response_bytes[25],response_bytes[26],
                                        response_bytes[27],response_bytes[28],response_bytes[29],response_bytes[30],response_bytes[31],response_bytes[32],
                                        response_bytes[33],response_bytes[34],response_bytes[35],response_bytes[36],response_bytes[37],response_bytes[38],
                                        response_bytes[39]);
                        break;
                    default:
                        ROS_INFO("read_buffer_data_default, An exception was thrown after call 0x%x with command(%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x)",
                                        response_bytes[2],response_bytes[0],response_bytes[1],response_bytes[2],response_bytes[3], response_bytes[4],response_bytes[5],response_bytes[6], response_bytes[7],response_bytes[8],
                                        response_bytes[9], response_bytes[10], response_bytes[11],response_bytes[12],response_bytes[13],response_bytes[14],
                                        response_bytes[15],response_bytes[16],response_bytes[17],response_bytes[18],response_bytes[19],response_bytes[20],
                                        response_bytes[21],response_bytes[22],response_bytes[23],response_bytes[24],response_bytes[25],response_bytes[26],
                                        response_bytes[27],response_bytes[28],response_bytes[29],response_bytes[30],response_bytes[31],response_bytes[32],
                                        response_bytes[33],response_bytes[34],response_bytes[35],response_bytes[36],response_bytes[37],response_bytes[38],
                                        response_bytes[39]);
                        return true;
                }
            } else {

                ROS_INFO("check_ID_error, read_buffer_data_error An exception was thrown after ID call 0x%x. command(%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x)",
                                        response_bytes[2],response_bytes[0],response_bytes[1],response_bytes[2],response_bytes[3], response_bytes[4],response_bytes[5],response_bytes[6], response_bytes[7],response_bytes[8],
                                        response_bytes[9], response_bytes[10], response_bytes[11],response_bytes[12],response_bytes[13],response_bytes[14],
                                        response_bytes[15],response_bytes[16],response_bytes[17],response_bytes[18],response_bytes[19],response_bytes[20],
                                        response_bytes[21],response_bytes[22],response_bytes[23],response_bytes[24],response_bytes[25],response_bytes[26],
                                        response_bytes[27],response_bytes[28],response_bytes[29],response_bytes[30],response_bytes[31],response_bytes[32],
                                        response_bytes[33],response_bytes[34],response_bytes[35],response_bytes[36],response_bytes[37],response_bytes[38],
                                        response_bytes[39]);

            }
        } else {

            ROS_INFO("check_CHKS_error, read_buffer_data_error An exception was thrown after CHKS call 0x%x. command(%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x)",
                                        response_bytes[2],response_bytes[0],response_bytes[1],response_bytes[2],response_bytes[3], response_bytes[4],response_bytes[5],response_bytes[6], response_bytes[7],response_bytes[8],
                                        response_bytes[9], response_bytes[10], response_bytes[11],response_bytes[12],response_bytes[13],response_bytes[14],
                                        response_bytes[15],response_bytes[16],response_bytes[17],response_bytes[18],response_bytes[19],response_bytes[20],
                                        response_bytes[21],response_bytes[22],response_bytes[23],response_bytes[24],response_bytes[25],response_bytes[26],
                                        response_bytes[27],response_bytes[28],response_bytes[29],response_bytes[30],response_bytes[31],response_bytes[32],
                                        response_bytes[33],response_bytes[34],response_bytes[35],response_bytes[36],response_bytes[37],response_bytes[38],
                                        response_bytes[39]);
        }
        return false;
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle priv_nh("~");

    std::string port = "/dev/rikibase";
    int baud_rate = 115200;

    boost::asio::io_service io;
    try {
        ROS_INFO("robot_driver start initializing!!!");
        rc_driver_ros::Driver_ros driver(port, baud_rate, io);
        ROS_INFO("*****************robot_driver initialize finished!**********");    
        ros::spin();
    }
    catch (boost::system::system_error ex) {
        ROS_ERROR("An exception was thrown: %s", ex.what());
        return -1;
    }
    
}


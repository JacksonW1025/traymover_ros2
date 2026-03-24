#ifndef ROS_rc_VALUES_H_
#define ROS_rc_VALUES_H_
/** Provides a mapping for often used rc values */
namespace rc_driver_ros
{ 
    typedef enum
    {
        MSG_ID_NULL             = 0,
        MSG_ID_GET_BAUD         = 1,
        MSG_ID_GET_MOTOR_DATA   = 2,
        MSG_ID_GET_MOTOR_STATUS = 3,

        MSG_ID_SET_SYSSTATUS    = 305,
        MSG_ID_SET_SYSSTRING    = 306,
        MSG_ID_SET_SYSNUMBER    = 307,
        MSG_ID_SET_MOTOR_SPEED  = 311,
        MSG_ID_SET_MOTOR_DIR    = 320,
        MSG_ID_SET_ALARM_LED    = 324,

        MSG_ID_SET_SUB_RESET           = 322,      
        MSG_ID_SET_SUB_UPDATE_DATA     = 1202,
        MSG_ID_SET_SUB_JUMP_TO_APP     = 1203,
        MSG_ID_SET_SUB_UPDATE_START    = 1205,
        MSG_ID_SET_SUB_UPDATE_END      = 1206,

        MSG_ID_SET_MCU_UPDATE_START    = 1225,
        MSG_ID_SET_MCU_UPDATE_DATA     = 1222,
        MSG_ID_SET_MCU_UPDATE_END      = 1226,

        MSG_ID_MAX
    } msg_id_t;

    typedef enum
    {
        MSG_ID_CLEAN_NULL             = 0,
        MSG_ID_GET_CLEAN_BAUD         = 1,
        MSG_ID_GET_CLEAN_DATA         = 2,
        MSG_ID_CLEAN_MAX
    } msg_id_clean;

    typedef	enum{
	    REQUEST_TYPE_NULL			= 0,
	    REQUEST_TYPE_ASK            = 1,
	    REQUEST_TYPE_SET            = 2,
	    REQUEST_TYPE_ACK_ASK        = 3,
	    REQUEST_TYPE_REPORT         = 4,
	    REQUEST_TYPE_ACK_SET        = 5,
    }REQUEST_TYPE;  

    typedef struct sonars_data
    {
        int    sonar_switch;
        double min_range;
        double max_range;
        double sonar_range;
        double sonar_offset_yaw;
        double sonar_offset_x;
        double sonar_offset_y;
    }_sonars_data;

    typedef struct send_motordata
    {
        uint8_t  move_cmd;
        uint16_t move_distance;
        uint8_t  mode;
        uint8_t  sonar_switch;
		uint8_t  cliff_switch;
        uint16_t  motor_filter;
        int16_t  leftPWM;
        int16_t  rightPWM;
        uint16_t leftmotor_scale;
        uint16_t rightmotor_scale;
        uint8_t  updatemotor;
        uint8_t  alarmled_switch;
        uint32_t gear_reduction;
        uint8_t  wheel_diameter;
        uint16_t wheel_track;
        uint16_t tick_meter_ratio;
    }_send_motordata;

    typedef struct send_motorstatus
    {
        uint8_t motor_reset;
        uint8_t get_ver;
        uint8_t clear_motorcode;
        uint8_t clear_motoralarm;
    }_send_motorstatus;   
    
    static const float pi = 3.1415926;
   
    /// 命令下发
    // 命令格式： head(2byte) + len(1byte) + serialid(1byte) + cmd(2byte) + data(4byte) + tail(2byte) + check(1byte)
    //           0x7F 0x7F       xx           xx                xxxx        xxxxxxxx       0x5F 0x5F     xx

    // 返回消息格式： head(2byte) + len(1byte) + ack_report(1byte) + serialid(1byte) + cmd(2byte) + result (1byte) + data(4byte) + tail(2byte) + check(1byte)
    //                0x7F 0x7F    xx              xx                xx                  xxxx           xx               xxxxxxx       0x5F 0x5F      xx

    /// 波特率
    static const boost::array<uint8_t, 40> get_baud        = {0x7F,0x7F,0x28,0x01,0x00,0x00,0x00,0x00,0x00,0x00,
                                                              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                              0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0d,0x0a};    
 
    /// 获取底盘实时数据：上报数据（）
    /// 下发数据：(data: 14byte) 
    /// move_cmd(1byte) move_distance(2byte)  Mode(1byte) Sonar_switch(1byte)  motor_filter(1byte) left_PWM(2byte) right_PWM(2byte) set_leftmotorenconder(2byte) set_rightmotorenconder(2byte) 
    static const boost::array<uint8_t, 40> get_motor_data = {0x7F,0x7F,0x28,0x02,0x00,0x00,0x00,0x00,0x00,0x00,
                                                             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0d,0x0a};

    static const boost::array<uint8_t, 40> get_clean_data = {0x7F,0x7F,0x28,0x02,0x00,0x00,0x00,0x00,0x00,0x00,
                                                             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0d,0x0a};	

    /// 获取底盘状态：上报数据（）
    /// 下发数据：(data: 11byte) 
    /// motor_reset(1byte) get_ver(1byte) clear_motorcode(1byte) clear_motoralarm(1byte) 
    static const boost::array<uint8_t, 40> get_motor_status = {0x7F,0x7F,0x28,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
                                                               0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                               0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                                               0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0d,0x0a};


    ///返回的CODE 类型
    static const uint8_t w_success = 0x60;
    static const uint8_t wr_error = 0x80;


    //在底盘运动时，取第一组数据
    static const boost::array<double, 36> ODOM_POSE_COVARIANCE =
            {1e6, 0, 0, 0, 0, 0,
            0, 1e6, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e6};
    //在底盘静止时，取第二组数据
    static const boost::array<double, 36> ODOM_POSE_COVARIANCE2 =
            {1e6, 0, 0, 0, 0, 0,
            0, 1e6, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e-9};
    //在底盘运动时，取第一组数据
    static const boost::array<double, 36> ODOM_TWIST_COVARIANCE =
            {1e-3, 0, 0, 0, 0, 0,
            0, 1e6, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e-6};
    //在底盘静止时，取第二组数据
    static const boost::array<double, 36> ODOM_TWIST_COVARIANCE2 =
            {1e-6, 0, 0, 0, 0, 0,
            0, 1e6, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e-6};
}
#endif

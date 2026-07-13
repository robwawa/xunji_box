
#include <string>
#include <cmath>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Vector3.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/Bool.h"

#include "lpsensor/LpmsIG1I.h"
#include "lpsensor/SensorDataI.h"
#include "lpsensor/LpmsIG1Registers.h"

struct IG1Command
{
    short command;
    union Data {
        uint32_t i[64];
        float f[64];
        unsigned char c[256];
    } data;
    int dataLength;
};

class LpIG1Proxy
{
public:
    // Node handler
    ros::NodeHandle nh, private_nh;
    ros::Timer updateTimer;

    // Publisher
    ros::Publisher imu_pub;
    ros::Publisher mag_pub;
    ros::Publisher yaw_pub;
    ros::Publisher autocalibration_status_pub;

    // Service
    ros::ServiceServer autocalibration_serv;
    ros::ServiceServer autoReconnect_serv;
    ros::ServiceServer gyrocalibration_serv;
    ros::ServiceServer resetHeading_serv;
    ros::ServiceServer getImuData_serv;
    ros::ServiceServer setStreamingMode_serv;
    ros::ServiceServer setCommandMode_serv;

    sensor_msgs::Imu imu_msg;
    sensor_msgs::MagneticField mag_msg;

    // Parameters
    std::string comportNo;
    int baudrate;
    bool autoReconnect;
    std::string frame_id;
    int rate;
    int imu_offset_samples_;

    // IMU 零偏校准
    bool imu_offset_calibrated_ = false;
    int imu_offset_count_ = 0;
    double calib_w_sum_ = 0, calib_x_sum_ = 0, calib_y_sum_ = 0, calib_z_sum_ = 0;
    double offset_conj_w_ = 1, offset_conj_x_ = 0, offset_conj_y_ = 0, offset_conj_z_ = 0;

    LpIG1Proxy(ros::NodeHandle h) : 
        nh(h),
        private_nh("~")
    {
        // Get node parameters
        private_nh.param<std::string>("port", comportNo, "/dev/ttyUSB0");
        private_nh.param("baudrate", baudrate, 921600);
        private_nh.param("autoreconnect", autoReconnect, true);
        private_nh.param<std::string>("frame_id", frame_id, "imu");
        private_nh.param("rate", rate, 200);
        private_nh.param("imu_offset_samples", imu_offset_samples_, 100);

        // Create LpmsIG1 object 
        sensor1 = IG1Factory();
        sensor1->setVerbose(VERBOSE_INFO);
        sensor1->setAutoReconnectStatus(autoReconnect);

        ROS_INFO("Settings");
        ROS_INFO("Port: %s", comportNo.c_str());
        ROS_INFO("Baudrate: %d", baudrate);
        ROS_INFO("Auto reconnect: %s", autoReconnect? "Enabled":"Disabled");
        
        imu_pub = nh.advertise<sensor_msgs::Imu>("data",1);
        mag_pub = nh.advertise<sensor_msgs::MagneticField>("mag",1);
        yaw_pub = nh.advertise<geometry_msgs::Vector3>("imu_yaw", 1);
        autocalibration_status_pub = nh.advertise<std_msgs::Bool>("is_autocalibration_active", 1, true);

        autocalibration_serv = nh.advertiseService("enable_gyro_autocalibration", &LpIG1Proxy::setAutocalibration, this);
        autoReconnect_serv = nh.advertiseService("enable_auto_reconnect", &LpIG1Proxy::setAutoReconnect, this);
        gyrocalibration_serv = nh.advertiseService("calibrate_gyroscope", &LpIG1Proxy::calibrateGyroscope, this);
        resetHeading_serv = nh.advertiseService("reset_heading", &LpIG1Proxy::resetHeading, this);
        getImuData_serv = nh.advertiseService("get_imu_data", &LpIG1Proxy::getImuData, this);
        setStreamingMode_serv = nh.advertiseService("set_streaming_mode", &LpIG1Proxy::setStreamingMode, this);
        setCommandMode_serv = nh.advertiseService("set_command_mode", &LpIG1Proxy::setCommandMode, this);
        
        // Connects to sensor
        if (!sensor1->connect(comportNo, baudrate))
        {
            ROS_ERROR("Error connecting to sensor\n");
            sensor1->release();
            ros::Duration(3).sleep(); // sleep 3 s
        }
        
        do
        {
            ROS_INFO("Waiting for sensor to connect %d", sensor1->getStatus());
            ros::Duration(1).sleep();
        } while(
            ros::ok() &&
            (
                !(sensor1->getStatus() == STATUS_CONNECTED) && 
                !(sensor1->getStatus() == STATUS_CONNECTION_ERROR)
            )
        );

        if (sensor1->getStatus() == STATUS_CONNECTED)
        {
            ROS_INFO("Sensor connected");
            ros::Duration(1).sleep();

            // 使能 Gyro 数据输出
            // 传感器默认 TDR 未使能角速度相关 bit。
            // 同时启用 bit6 (GYR0_ALIGN_CALIBRATED) 和 bit10 (ANGULAR_VELOCITY)
            // 优先使用 sd.angularVelocity 字段。
            {
              uint32_t gyro_config =
                  TDR_ACC_CALIBRATED_OUTPUT_ENABLED
                | TDR_GYR0_ALIGN_CALIBRATED_OUTPUT_ENABLED
                | TDR_ANGULAR_VELOCITY_OUTPUT_ENABLED
                | TDR_MAG_RAW_OUTPUT_ENABLED
                | TDR_QUAT_OUTPUT_ENABLED
                | TDR_LINACC_OUTPUT_ENABLED;
              sensor1->commandSetTransmitData(gyro_config);
              ros::Duration(0.2).sleep();
              sensor1->commandSaveParameters();
              ros::Duration(0.5).sleep();
              ROS_INFO("IMU gyro data enabled & saved (config=0x%04X)", gyro_config);
            }

            sensor1->commandGotoStreamingMode();
        }
        else 
        {
            ROS_INFO("Sensor connection error: %d.", sensor1->getStatus());
            ros::shutdown();
        }
    }

    ~LpIG1Proxy(void)
    {
        sensor1->release();
    }

    void update(const ros::TimerEvent& te)
    {
        static bool runOnce = false;

        if (sensor1->getStatus() == STATUS_CONNECTED &&
                sensor1->hasImuData())
        {
            if (!runOnce)
            {
                publishIsAutocalibrationActive();
                runOnce = true;
            }
            IG1ImuDataI sd;
            sensor1->getImuData(sd);

            // ---- IMU 零偏校准：上电后取前 N 组四元数均值作为偏移 ----
            if (!imu_offset_calibrated_)
            {
                calib_w_sum_ += sd.quaternion.data[0];
                calib_x_sum_ += sd.quaternion.data[1];
                calib_y_sum_ += sd.quaternion.data[2];
                calib_z_sum_ += sd.quaternion.data[3];
                imu_offset_count_++;
                if (imu_offset_count_ >= imu_offset_samples_)
                {
                    double n = imu_offset_count_;
                    double qw = calib_w_sum_ / n, qx = calib_x_sum_ / n;
                    double qy = calib_y_sum_ / n, qz = calib_z_sum_ / n;
                    // 归一化平均四元数
                    double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
                    if (norm > 1e-9) { qw /= norm; qx /= norm; qy /= norm; qz /= norm; }
                    // 存共轭（逆旋转），用于抵消初始偏置
                    offset_conj_w_ = qw;
                    offset_conj_x_ = -qx;
                    offset_conj_y_ = -qy;
                    offset_conj_z_ = -qz;
                    imu_offset_calibrated_ = true;
                    ROS_INFO("[LpIG1] IMU offset calibrated (%d samples)", imu_offset_count_);
                }
            }
            // 应用零偏：q_corrected = offset_conj * q_raw
            double qw, qx, qy, qz;
            if (imu_offset_calibrated_)
            {
                double aw = offset_conj_w_, ax = offset_conj_x_, ay = offset_conj_y_, az = offset_conj_z_;
                double bw = sd.quaternion.data[0], bx = sd.quaternion.data[1];
                double by = sd.quaternion.data[2], bz = sd.quaternion.data[3];
                qw = aw*bw - ax*bx - ay*by - az*bz;
                qx = aw*bx + ax*bw + ay*bz - az*by;
                qy = aw*by - ax*bz + ay*bw + az*bx;
                qz = aw*bz + ax*by - ay*bx + az*bw;
            }
            else { qw=sd.quaternion.data[0]; qx=sd.quaternion.data[1]; qy=sd.quaternion.data[2]; qz=sd.quaternion.data[3]; }

            /* Fill the IMU message */

            // Fill the header
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = frame_id;

            // Fill orientation quaternion
            imu_msg.orientation.w = qw;
            imu_msg.orientation.x = -qx;
            imu_msg.orientation.y = -qy;
            // Yaw 方向：传感器原始为 CW+，恢复后取反转为 ROS 标准 CCW+
            imu_msg.orientation.z = qz;

            // Fill angular velocity data
            // 优先使用 sd.angularVelocity (TDR bit10), 若为0则回退到
            // sd.gyroIAlignmentCalibrated (TDR bit6)
            // 传感器输出单位为 deg/s，需转换为 rad/s
            // 传感器坐标系 X前-Y右-Z下，转换为 ROS X前-Y左-Z上：
            //   ωx→ωx, ωy→-ωy, ωz→-ωz (绕X轴180°旋转)
            {
              double avx = sd.angularVelocity.data[0];
              double avy = sd.angularVelocity.data[1];
              double avz = sd.angularVelocity.data[2];
              if (std::abs(avx) < 1e-9 && std::abs(avy) < 1e-9 && std::abs(avz) < 1e-9)
              {
                avx = sd.gyroIAlignmentCalibrated.data[0];
                avy = sd.gyroIAlignmentCalibrated.data[1];
                avz = sd.gyroIAlignmentCalibrated.data[2];
              }
              const double DEG2RAD = M_PI / 180.0;
              // sd.angularVelocity 已是传感器处理后的角速度，坐标系已正确
              // 仅需 deg/s→rad/s 转换，不需要取反
              imu_msg.angular_velocity.x =  avx * DEG2RAD;
              imu_msg.angular_velocity.y =  avy * DEG2RAD;
              imu_msg.angular_velocity.z =  avz * DEG2RAD;
            }

            // Fill linear acceleration data
            imu_msg.linear_acceleration.x = -sd.accCalibrated.data[0]*9.81;
            imu_msg.linear_acceleration.y = -sd.accCalibrated.data[1]*9.81;
            // Z 取反移除：与 orientation.z 保持一致（传感器 z-down → ROS z-up）
            imu_msg.linear_acceleration.z = sd.accCalibrated.data[2]*9.81;

            /* Fill the magnetometer message */
            mag_msg.header.stamp = imu_msg.header.stamp;
            mag_msg.header.frame_id = frame_id;

            // Units are microTesla in the LPMS library, Tesla in ROS.
            mag_msg.magnetic_field.x = sd.magRaw.data[0]*1e-6;
            mag_msg.magnetic_field.y = sd.magRaw.data[1]*1e-6;
            mag_msg.magnetic_field.z = sd.magRaw.data[2]*1e-6;

            // Publish IMU yaw (RPY) for arm_board chassis control
            {
              double x = imu_msg.orientation.x;
              double y = imu_msg.orientation.y;
              double z = imu_msg.orientation.z;
              double w = imu_msg.orientation.w;
              double siny = 2.0 * (w * z + x * y);
              double cosy = 1.0 - 2.0 * (y * y + z * z);
              double sinr = 2.0 * (w * x + y * z);
              double cosr = 1.0 - 2.0 * (x * x + y * y);
              double sinp = 2.0 * (w * y - z * x);

              geometry_msgs::Vector3 yaw_msg;
              yaw_msg.x = std::atan2(sinr, cosr);
              yaw_msg.y = std::asin(std::max(-1.0, std::min(1.0, sinp)));
              yaw_msg.z = std::atan2(siny, cosy);
              yaw_pub.publish(yaw_msg);
            }

            // Publish the messages
            imu_pub.publish(imu_msg);
            mag_pub.publish(mag_msg);
        }
    }

    void run(void)
    {
        // The timer ensures periodic data publishing
        updateTimer = ros::Timer(nh.createTimer(ros::Duration(1.0f/rate),
                                                &LpIG1Proxy::update,
                                                this));
    }

    void publishIsAutocalibrationActive()
    {
        std_msgs::Bool msg;
        IG1SettingsI settings;
        sensor1->getSettings(settings);
        msg.data = settings.enableGyroAutocalibration;
        autocalibration_status_pub.publish(msg);
    }

    ///////////////////////////////////////////////////
    // Service Callbacks
    ///////////////////////////////////////////////////
    bool setAutocalibration (std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        ROS_INFO("set_autocalibration");

        // clear current settings
        IG1SettingsI settings;
        sensor1->getSettings(settings);

        // Send command
        cmdSetEnableAutocalibration(req.data);
        ros::Duration(0.2).sleep();
        cmdGetEnableAutocalibration();
        ros::Duration(0.1).sleep();

        double retryElapsedTime = 0;
        int retryCount = 0;
        while (!sensor1->hasSettings()) 
        {
            ros::Duration(0.1).sleep();
            ROS_INFO("set_autocalibration wait");
            
            retryElapsedTime += 0.1;
            if (retryElapsedTime > 2.0)
            {
                retryElapsedTime = 0;
                cmdGetEnableAutocalibration();
                retryCount++;
            }

            if (retryCount > 5)
                break;
        }
        ROS_INFO("set_autocalibration done");

        // Get settings
        sensor1->getSettings(settings);

        std::string msg;
        if (settings.enableGyroAutocalibration == req.data) 
        {
            res.success = true;
            msg.append(std::string("[Success] autocalibration status set to: ") + (settings.enableGyroAutocalibration?"True":"False"));
        }
        else 
        {
            res.success = false;
            msg.append(std::string("[Failed] current autocalibration status set to: ") + (settings.enableGyroAutocalibration?"True":"False"));
        }

        ROS_INFO("%s", msg.c_str());
        res.message = msg;

        publishIsAutocalibrationActive();
        return res.success;
    }

    // Auto reconnect
    bool setAutoReconnect (std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        ROS_INFO("set_auto_reconnect");

        sensor1->setAutoReconnectStatus(req.data);
        
        res.success = true;
        std::string msg;
        msg.append(std::string("[Success] auto reconnection status set to: ") + (sensor1->getAutoReconnectStatus()?"True":"False"));
    
        ROS_INFO("%s", msg.c_str());
        res.message = msg;

        return res.success;
    }
    
    // reset heading
    bool resetHeading (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        ROS_INFO("reset_heading");
        
        // Send command
        cmdResetHeading();

        res.success = true;
        res.message = "[Success] Heading reset";
        return true;
    }


    bool calibrateGyroscope (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        ROS_INFO("calibrate_gyroscope: Please make sure the sensor is stationary for 4 seconds");

        cmdCalibrateGyroscope();

        ros::Duration(4).sleep();
        res.success = true;
        res.message = "[Success] Gyroscope calibration procedure completed";
        ROS_INFO("calibrate_gyroscope: Gyroscope calibration procedure completed");
        return true;
    }
    
    bool getImuData (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        cmdGotoCommandMode();
        ros::Duration(0.1).sleep();
        cmdGetImuData();
        res.success = true;
        res.message = "[Success] Get imu data";
        return true;
    }

    bool setStreamingMode (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        cmdGotoStreamingMode();
        res.success = true;
        res.message = "[Success] Set streaming mode";
        return true;
    }

    bool setCommandMode (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        cmdGotoCommandMode();
        res.success = true;
        res.message = "[Success] Set command mode";
        return true;
    }


    ///////////////////////////////////////////////////
    // Helpers
    ///////////////////////////////////////////////////

    void cmdGotoCommandMode ()
    {
        IG1Command cmd;
        cmd.command = GOTO_COMMAND_MODE;
        cmd.dataLength = 0;
        sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);
    }

    void cmdGotoStreamingMode ()
    {
        IG1Command cmd;
        cmd.command = GOTO_STREAM_MODE;
        cmd.dataLength = 0;
        sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);
    }

    void cmdGetImuData()
    {
        IG1Command cmd;
        cmd.command = GET_IMU_DATA;
        cmd.dataLength = 0;
        sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);
    }

    void cmdCalibrateGyroscope()
    {
        IG1Command cmd;
        cmd.command = START_GYR_CALIBRATION;
        cmd.dataLength = 0;
        sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);
    }

    void cmdResetHeading()
    {
        IG1Command cmd;
        cmd.command = SET_ORIENTATION_OFFSET;
        cmd.dataLength = 4;
        cmd.data.i[0] = LPMS_OFFSET_MODE_HEADING;
        sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);
    }

    void cmdSetEnableAutocalibration(int status)
    {
        IG1Command cmd;
        cmd.command = SET_ENABLE_GYR_AUTOCALIBRATION;
        cmd.dataLength = 4;
        cmd.data.i[0] = status;
        sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);
    }

    void cmdGetEnableAutocalibration()
    {
        IG1Command cmd;
        cmd.command = GET_ENABLE_GYR_AUTOCALIBRATION;
        cmd.dataLength = 0;
        sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);
    }

 private:

    // Access to LPMS data
    IG1I* sensor1;
};

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "lpms_ig1_node");
    ros::NodeHandle nh("imu");

    ros::AsyncSpinner spinner(0);
    spinner.start();

    LpIG1Proxy lpIG1(nh);

    lpIG1.run();
    ros::waitForShutdown();

    return 0;
}

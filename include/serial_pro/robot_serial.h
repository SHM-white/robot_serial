//
// Created by mijiao on 23-11-20.
//

#ifndef ROBOT_SERIAL_ROBOT_SERIAL_H
#define ROBOT_SERIAL_ROBOT_SERIAL_H

#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "robot_serial/msg/aim.hpp"
#include "robot_serial/msg/armor.hpp"
#include "robot_serial/msg/dial_switch.hpp"
#include "robot_serial/msg/gimbal.hpp"
#include "robot_serial/msg/spinning_control.hpp"
#include "robot_serial/msg/mode.hpp"
#include "robot_serial/msg/chassis.hpp"


#include "robot_message.h"
#include "robot_comm.h"

class RobotSerial : public rclcpp::Node {
private:
    robot::RobotSerial serial;

    rclcpp::Clock rosClock;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
    rclcpp::Publisher<robot_serial::msg::Gimbal>::SharedPtr gimbalPublisher;
    rclcpp::Publisher<robot_serial::msg::DialSwitch>::SharedPtr dialSwitchPublisher;
    rclcpp::Publisher<robot_serial::msg::Mode>::SharedPtr modePublisher;
    rclcpp::Publisher<robot_serial::msg::Chassis>::SharedPtr chassisPublisher;

    rclcpp::Subscription<robot_serial::msg::Aim>::SharedPtr autoAimSubscription;

    void aimCallback(const robot_serial::msg::Aim::SharedPtr msg) {
        if (std::isnan(msg->yaw) || std::isnan(msg->pitch) || std::isnan(msg->w_yaw) || std::isnan(msg->w_pitch) ||
            std::isnan(msg->target_rate) ||
            std::isnan(msg->target_number) ||
            std::isinf(msg->yaw) || std::isinf(msg->pitch) || std::isinf(msg->w_yaw) || std::isinf(msg->w_pitch) ||
            std::isinf(msg->target_rate) ||
            std::isinf(msg->target_number)) {
            return;
        }
        Aim aim{
                (float) (msg->yaw),
                (float) (msg->pitch),
                (float) (msg->w_yaw),
                (float) (msg->w_pitch),
                (uint8_t) (msg->target_rate),
                (uint8_t) (msg->target_number),
        };
        serial.write(0x81, aim);
    }

//    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;(仅限哨兵)

//    ros::Subscriber chassis_con = nh.subscribe("/cmd_vel", 1, chassisCallback);(仅限哨兵)
//    ros::Subscriber chassis_key_RC = nh.subscribe("/cmd_vel_RC", 1, Key_RC_Callback);(仅限哨兵)

//    ros::Subscriber logic_pitch = nh.subscribe("/robot/logic_recommend_angle", 1, logic_pitch_callback);(仅限哨兵)
//    ros::Subscriber spinning_speeder = nh.subscribe("/robot/spnning_speed", 1, spin_speed_callback);(仅限哨兵)
//    ros::Subscriber exchange_sub = nh.subscribe("/robot/exchange", 1, exchangecallback);(仅限哨兵)

public:
    explicit RobotSerial() : Node("robot_serial_node") {
        declare_parameter("/serial_name", "/dev/ttyUSB0");
        serial = std::move(robot::RobotSerial(get_parameter("/serial_name").as_string(), 115200));

        //日志记录
        serial.registerErrorHandle([this](int label, const std::string& text) {
            robot::RobotSerial::error _label;
            _label = (robot::RobotSerial::error) label;
            std::stringstream _str;
            for (auto c: text) {
                _str << std::setw(2) << std::hex << (int) *(uint8_t*) &c << " ";
            }
            _str << std::endl;
            switch (_label) {
                case robot::RobotSerial::lengthNotMatch:
                    RCLCPP_ERROR_STREAM(get_logger(), "lengthNotMatch");
                    RCLCPP_ERROR_STREAM(get_logger(), _str.str());
                case robot::RobotSerial::rxLessThanLength:
                    RCLCPP_ERROR_STREAM(get_logger(), "rxLessThanLength");
                    break;
                case robot::RobotSerial::crcError:
                    RCLCPP_ERROR_STREAM(get_logger(), "crcError");
                    RCLCPP_ERROR_STREAM(get_logger(), _str.str());
                    break;
                default:
                    return;
            }
        });

        //ImuCallback
        serial.registerCallback(0x11, [this](const Imu& msg) {
            sensor_msgs::msg::Imu _imu;
            _imu.angular_velocity.x = msg.wx;
            _imu.angular_velocity.y = msg.wy;
            _imu.angular_velocity.z = msg.wz;
            _imu.linear_acceleration.x = msg.ax;
            _imu.linear_acceleration.y = msg.ay;
            _imu.linear_acceleration.z = msg.az;
            _imu.orientation.x = msg.qx;
            _imu.orientation.y = msg.qy;
            _imu.orientation.z = msg.qz;
            _imu.orientation.w = msg.qw;
            imuPublisher->publish(_imu);
        });

//        //odomCallback(仅限哨兵)
//        serial.registerCallback(0x12, [this](const Odometry& msg) {
//            nav_msgs::msg::Odometry _odometry;
//            _odometry.header.stamp = rosClock.now();
//            _odometry.header.frame_id = "_odometry";
//            _odometry.pose.pose.position.x = msg.x / 1000.;
//            _odometry.pose.pose.position.y = msg.y / 1000.;
//            _odometry.child_frame_id = "base_link";
//            _odometry.twist.twist.linear.x = msg.vx / 1000.;
//            _odometry.twist.twist.linear.y = msg.vy / 1000.;
//            _odometry.twist.twist.angular.z = msg.wz / 1000.;
//            odometryPublisher->publish(_odometry);
//        });

        //gimbalCallback
        serial.registerCallback(0x14, [this](const Kalman& msg) {
            robot_serial::msg::Gimbal _gimbal;
            _gimbal.pitch = msg.pitch;
            _gimbal.roll = msg.roll;
            _gimbal.yaw = msg.yaw;
            _gimbal.bullet = msg.bullet;
            gimbalPublisher->publish(_gimbal);
        });

        //modeCallback
        serial.registerCallback(0x15, [this](const uint8_t& msg) {
            robot_serial::msg::Mode mode;
            switch (msg) {
                case 0:
                    mode.mode = 0;
                    break;
                case 1:
                    mode.mode = 1;
                    mode.config.push_back(0);
                    break;
                case 2:
                    mode.mode = 1;
                    mode.config.push_back(1);
                    break;
                case 3:
                    mode.mode = 2;
                    break;
            }
            mode.mode = msg;
            modePublisher->publish(mode);
        });

        //chassis_angle_callback
        serial.registerCallback(0x16, [this](const float& msg) {
            robot_serial::msg::Chassis _chassis;
            _chassis.angle = msg;
            chassisPublisher->publish(_chassis);
        });

        //switchCallback
        serial.registerCallback(0x17, [this](const DialSwitch& msg) {
            robot_serial::msg::DialSwitch _switch;
            if (msg.enable) {
                _switch.is3big = msg.is3big;
                _switch.is4big = msg.is4big;
                _switch.is5big = msg.is5big;
                dialSwitchPublisher->publish(_switch);
            }
        });

//        odometryPublisher = create_publisher<nav_msgs::msg::Odometry>("/robot/odom", 5);
        imuPublisher = create_publisher<sensor_msgs::msg::Imu>("/robot/imu", 5);
        modePublisher = create_publisher<robot_serial::msg::Mode>("/robot/mode", 5);
        chassisPublisher = create_publisher<robot_serial::msg::Chassis>("/robot/chassis", 5);
        gimbalPublisher = create_publisher<robot_serial::msg::Gimbal>("/robot/gimbal", 5);
        dialSwitchPublisher = create_publisher<robot_serial::msg::DialSwitch>("/robot/dial_switch", 5);

        autoAimSubscription = create_subscription<robot_serial::msg::Aim>("/robot/auto_aim", 1,
                                                                          std::bind(&RobotSerial::aimCallback, this,
                                                                                    std::placeholders::_1));

        serial.spin(true);
    }
};

#endif //ROBOT_SERIAL_ROBOT_SERIAL_H

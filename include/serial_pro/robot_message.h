//
// Created by mijiao on 23-11-20.
//

#ifndef ROBOT_SERIAL_ROBOT_MESSAGE_H
#define ROBOT_SERIAL_ROBOT_MESSAGE_H

#include "msg_serialize.h"

message_data Aim {
    float yaw, pitch, w_yaw, w_pitch;
    uint8_t target_rate, target_number;
};

message_data SpinSpeed {
    int16_t spinning_speed;
};

message_data ExchangeStation {
    float x, y, z, q_x, q_y, q_z, q_w;
};

message_data Imu {
    float ax, ay, az;
    float wx, wy, wz;
    float qx, qy, qz, qw;
};

message_data Chassis {
    int16_t vx, vy, wz;
};

message_data Odometry {
    int x, y, yaw;
    int vx, vy, wz;
};

message_data Kalman {
    float pitch, roll, yaw;
    uint8_t bullet;
};

message_data DialSwitch {
    uint8_t enable, is3big, is4big, is5big;
};
#endif //ROBOT_SERIAL_ROBOT_MESSAGE_H

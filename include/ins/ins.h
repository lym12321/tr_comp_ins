//
// Created by fish on 2025/9/30.
//

#pragma once

#include <bsp/imu.h>
#include <math/matrix.h>
#include <math/quaternion.h>

namespace ins {
    struct data_t {
        math::quaternion q;
        math::matrix <3, 1> accel, gyro;
        float roll, pitch, yaw;
        float yaw_total_angle;
        bsp_imu_data_t raw;
    };
    void init(const math::matrix<3, 3> &trans = math::matrix<3, 3>::eye());
    bool ready();
    data_t state();
    data_t *data();
}

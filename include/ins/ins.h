//
// Created by fish on 2025/9/30.
//

#pragma once

#include <cstdint>
#include <bsp/imu.h>
#include <math/matrix.h>

namespace ins {
    struct data_t {
        float q[4], accel[3], gyro[3];
        float roll, pitch, yaw;
        float yaw_total_angle;
        bsp_imu_data_t raw;
    };
    inline bool inited = false;
    void init(const math::matrix<3, 3> &trans = math::matrix<3, 3>::eye());
    data_t *data();
}
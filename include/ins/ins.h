//
// Created by fish on 2025/9/30.
//

#pragma once

#include <cstdint>
#include <bsp/imu.h>

namespace ins {
    struct data_t {
        float q[4];
        float roll, pitch, yaw;
        float yaw_total_angle;
        uint32_t timestamp;
        bsp_imu_data_t raw;
    };
    inline bool inited = false;
    void init();
    data_t *data();
}
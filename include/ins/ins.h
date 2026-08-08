//
// Created by fish on 2025/9/30.
//

#pragma once

#include <bsp/imu.h>
#include <math/matrix.h>
#include <math/quaternion.h>

#include <array>
#include <cstdint>

namespace ins {
    inline constexpr uint32_t calibration_duration_ms = 30000;

    enum class calibration_state_e : uint8_t {
        IDLE,
        RUNNING,
        SAVING,
        SUCCEEDED,
        FAILED,
        CANCELLED,
    };

    enum class calibration_error_e : uint8_t {
        NONE,
        IMU_READ,
        FLASH_WRITE,
    };

    enum class calibration_result_e : uint8_t {
        OK,
        NOT_READY,
        BUSY,
    };

    struct calibration_status_t {
        calibration_state_e state = calibration_state_e::IDLE;
        calibration_error_e error = calibration_error_e::NONE;
        uint32_t progress_ms = 0;
        uint32_t duration_ms = calibration_duration_ms;
        bool valid = false;
        bool persisted = false;
        std::array<float, 3> gyro_corr {};
    };

    struct data_t {
        math::quaternion q;
        math::matrix <3, 1> accel, gyro;
        float roll, pitch, yaw;
        float yaw_total_angle;
        bsp_imu_data_t raw;
        bool converged;
        bool stationary;
    };

    void init(const math::matrix<3, 3> &trans = math::matrix<3, 3>::eye());
    bool ready();
    data_t state();
    data_t *data();
    calibration_result_e start_calibration();
    bool cancel_calibration();
    calibration_status_t calibration_status();
}

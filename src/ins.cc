// Created by fish on 2025/9/30.

#include "ins/ins.h"

#include "bsp/time.h"
#include "bsp/imu.h"
#include "bsp/flash.h"
#include "utils/logger.h"

#include "cmsis_os2.h"
#include "task.h"

#include "quaternion_ekf.h"

#define FLASH_VALID_KEY 998244353

using namespace ins;

namespace ins {
    data_t _data;
    static struct {
        float gyro_corr[3] = { 0, 0, 0 };
        uint32_t valid = 0;
    } _cali_data;
}

static TaskHandle_t task_handle;

[[noreturn]] static void task(void *args) {
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);
    logger::info("module inited");

    osDelay(50);

    bsp_flash_read("ins", &_cali_data, sizeof _cali_data);
    if (_cali_data.valid == FLASH_VALID_KEY) {
        logger::info("loaded calibration data: [%f, %f, %f]", _cali_data.gyro_corr[0], _cali_data.gyro_corr[1], _cali_data.gyro_corr[2]);
    } else {
        logger::info("calibrating");
        uint32_t _lst_wakeup_time = bsp_time_get_ms();
        for (int i = 0; i < 5000; i++) {
            _data.raw = bsp_imu_read();
            _cali_data.gyro_corr[0] += _data.raw.gyro[0];
            _cali_data.gyro_corr[1] += _data.raw.gyro[1];
            _cali_data.gyro_corr[2] += _data.raw.gyro[2];
            vTaskDelayUntil(&_lst_wakeup_time, pdMS_TO_TICKS(1));
        }
        _cali_data.gyro_corr[0] /= 5000, _cali_data.gyro_corr[1] /= 5000, _cali_data.gyro_corr[2] /= 5000;
        logger::info("calibration data: [%f, %f, %f]", _cali_data.gyro_corr[0], _cali_data.gyro_corr[1], _cali_data.gyro_corr[2]);
        // _cali_data.valid = 998244353;
        // bsp_flash_write("ins", &_cali_data, sizeof _cali_data);
    }

    inited = true;
    logger::info("done");

    uint32_t lst_wakeup_time = bsp_time_get_ms();

    for (;;) {
        _data.raw = bsp_imu_read();
        _data.raw.gyro[0] -= _cali_data.gyro_corr[0], _data.raw.gyro[1] -= _cali_data.gyro_corr[1], _data.raw.gyro[2] -= _cali_data.gyro_corr[2];
        IMU_QuaternionEKF_Update(_data.raw.gyro[0], _data.raw.gyro[1], _data.raw.gyro[2], _data.raw.accel[0], _data.raw.accel[1], _data.raw.accel[2], 0.001);
        _data.roll = QEKF_INS.Roll, _data.pitch = QEKF_INS.Pitch, _data.yaw = QEKF_INS.Yaw, _data.yaw_total_angle = QEKF_INS.YawTotalAngle;
        _data.roll *= M_PI / 180.f;
        _data.pitch *= M_PI / 180.f;
        _data.yaw *= M_PI / 180.f;
        _data.yaw_total_angle *= M_PI / 180.f;
        memcpy(_data.q, QEKF_INS.q, sizeof _data.q);
        vTaskDelayUntil(&lst_wakeup_time, pdMS_TO_TICKS(1));
    }
}

void ins::init() {
    xTaskCreate(task, "ins", 128, nullptr, osPriorityRealtime, &task_handle);
}

data_t *ins::data() {
    return &_data;
}
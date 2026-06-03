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
    math::matrix<3, 3> _trans;
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
    logger::info("transform matrix: [%f, %f, %f; %f, %f, %f; %f, %f, %f]",
        _trans(0, 0), _trans(0, 1), _trans(0, 2),
        _trans(1, 0), _trans(1, 1), _trans(1, 2),
        _trans(2, 0), _trans(2, 1), _trans(2, 2)
    );

    osDelay(50);

    bsp_flash_read("ins", &_cali_data, sizeof _cali_data);
    if (_cali_data.valid == FLASH_VALID_KEY) {
        logger::info("loaded calibration data: [%f, %f, %f]", _cali_data.gyro_corr[0], _cali_data.gyro_corr[1], _cali_data.gyro_corr[2]);
    } else {
        logger::info("calibrating");
        _cali_data = {};
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
    math::matrix<3, 1> t_gyro, t_accel;

    for (;;) {
        _data.raw = bsp_imu_read();
        memcpy(_data.gyro, _data.raw.gyro, sizeof _data.gyro);
        memcpy(_data.accel, _data.raw.accel, sizeof _data.accel);
        _data.gyro[0] -= _cali_data.gyro_corr[0], _data.gyro[1] -= _cali_data.gyro_corr[1], _data.gyro[2] -= _cali_data.gyro_corr[2];

        t_gyro.load(_data.gyro);
        t_accel.load(_data.accel);
        (_trans * t_gyro).save(_data.gyro);
        (_trans * t_accel).save(_data.accel);

        IMU_QuaternionEKF_Update(_data.gyro[0], _data.gyro[1], _data.gyro[2], _data.accel[0], _data.accel[1], _data.accel[2], 0.001);
        _data.roll = QEKF_INS.Roll, _data.pitch = QEKF_INS.Pitch, _data.yaw = QEKF_INS.Yaw, _data.yaw_total_angle = QEKF_INS.YawTotalAngle;
        _data.roll *= M_PI / 180.f;
        _data.pitch *= M_PI / 180.f;
        _data.yaw *= M_PI / 180.f;
        _data.yaw_total_angle *= M_PI / 180.f;
        memcpy(_data.q, QEKF_INS.q, sizeof _data.q);
        vTaskDelayUntil(&lst_wakeup_time, pdMS_TO_TICKS(1));
    }
}

void ins::init(const math::matrix<3, 3> &trans) {
    _trans = trans;
    const BaseType_t ok = xTaskCreate(task, "ins", 256, nullptr, osPriorityRealtime, &task_handle);
    BSP_ASSERT(ok == pdPASS);
}

data_t *ins::data() {
    return &_data;
}

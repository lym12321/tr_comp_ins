// Created by fish on 2025/9/30.

#include "ins/ins.h"

#include "bsp/time.h"
#include "bsp/imu.h"
#include "bsp/flash.h"
#include "utils/logger.h"
#include "utils/os.h"
#include "utils/terminal.h"

#include "quaternion_ekf.h"
#include "utils/crc.h"

using namespace ins;

namespace ins {
    struct calibration_t {
        std::array <float, 3> gyro_corr { 0, 0, 0 };
        uint8_t crc = 0;

        [[nodiscard]] uint8_t calc_crc() const {
            return crc8::calc(
                reinterpret_cast<const uint8_t *>(gyro_corr.data()),
                gyro_corr.size() * sizeof(float),
                0xff
            );
        }

        [[nodiscard]] bool valid() const { return crc == calc_crc(); }
    };

    math::matrix<3, 3> _trans;
    data_t _data;
    static calibration_t _cali_data;
}

static TaskHandle_t task_handle;
static uint32_t last_wakeup_time;
static constexpr uint32_t default_calibration_ms = 5000;
static constexpr uint32_t active_calibration_ms = 30000;
static constexpr float deg_to_rad = 0.01745329252f;

static bool run_calibration(const uint32_t duration_ms, bool save_to_flash = false) {
    calibration_t candidate;

    uint32_t wakeup_time = bsp_time_get_ms();
    for (uint32_t sample = 0; sample < duration_ms; sample++) {
        const auto data = bsp_imu_read();
        for (int axis = 0; axis < 3; axis++) {
            candidate.gyro_corr[axis] += data.gyro[axis];
        }
        vTaskDelayUntil(&wakeup_time, pdMS_TO_TICKS(1));
    }

    for (float &correction : candidate.gyro_corr) {
        correction /= static_cast<float>(duration_ms);
    }
    candidate.crc = candidate.calc_crc();
    _cali_data = candidate;

    if (save_to_flash and !bsp_flash_write("ins/cali", &candidate, sizeof candidate)) {
        logger::error("failed to save calibration data");
        return false;
    }
    return true;
}

static bool request_calibration() {
    vTaskSuspend(task_handle);
    const bool ok = run_calibration(active_calibration_ms, true);
    last_wakeup_time = bsp_time_get_ms();
    vTaskResume(task_handle);
    return ok;
}

static void register_terminal_command() {
    terminal::register_cmd("imu", [](const std::vector<std::string> &args) {
        if (args.size() == 1) {
            terminal::info(
                "IMU calibration: %s, gyro correction: [%f, %f, %f] rad/s\r\n",
                _cali_data.valid() ? "ready" : "uncalibrated",
                _cali_data.gyro_corr[0],
                _cali_data.gyro_corr[1],
                _cali_data.gyro_corr[2]
            );
            return;
        }

        if (args.size() == 2 && args[1] == "calibrate") {
            if (!inited) {
                terminal::info("IMU is not ready.\r\n");
                return;
            }

            terminal::info("Keep the robot stationary for 30 seconds...\r\n");
            terminal::info(request_calibration() ?
                "Calibration saved.\r\n" :
                "Calibration applied, but saving to flash failed.\r\n");
            return;
        }

        if (args.size() == 2 && args[1] == "watch") {
            if (!inited) {
                terminal::info("IMU is not ready.\r\n");
                return;
            }
            while (terminal::running()) {
                terminal::info("[roll, pitch, yaw] = [%f, %f, %f]\r\n", _data.roll, _data.pitch, _data.yaw);
                os::task::sleep(1);
            }
            return;
        }

        terminal::info("Usage: imu [calibrate]\r\n");
    }, "Show IMU status or calibrate gyro");
}

[[noreturn]] static void task(void *args) {
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);
    logger::info("module inited");
    const auto &trans = _trans;
    logger::info("transform matrix: [%f, %f, %f; %f, %f, %f; %f, %f, %f]",
        trans(0, 0), trans(0, 1), trans(0, 2),
        trans(1, 0), trans(1, 1), trans(1, 2),
        trans(2, 0), trans(2, 1), trans(2, 2)
    );

    osDelay(50);

    bsp_flash_read("ins/cali", &_cali_data, sizeof _cali_data);
    if (_cali_data.valid()) {
        logger::info("loaded calibration data: [%f, %f, %f]", _cali_data.gyro_corr[0], _cali_data.gyro_corr[1], _cali_data.gyro_corr[2]);
    } else {
        logger::warn("calibration data invalid, calibrating for 5 seconds");
        run_calibration(default_calibration_ms);
    }

    inited = true;
    logger::info("done");

    last_wakeup_time = bsp_time_get_ms();

    for (;;) {
        _data.raw = bsp_imu_read();
        _data.gyro.load(_data.raw.gyro);
        _data.accel.load(_data.raw.accel);
        _data.gyro = _trans * (_data.gyro - math::matrix<3, 1>(_cali_data.gyro_corr));
        _data.accel = _trans * _data.accel;

        IMU_QuaternionEKF_Update(_data.gyro[0], _data.gyro[1], _data.gyro[2], _data.accel[0], _data.accel[1], _data.accel[2], 0.001);

        // 这里调换一下 pitch 和 roll, 因为玺佬的 pitch 是绕 x 轴的
        _data.roll = QEKF_INS.Pitch, _data.pitch = QEKF_INS.Roll, _data.yaw = QEKF_INS.Yaw, _data.yaw_total_angle = QEKF_INS.YawTotalAngle;
        _data.roll *= deg_to_rad;
        _data.pitch *= deg_to_rad;
        _data.yaw *= deg_to_rad;
        _data.yaw_total_angle *= deg_to_rad;
        _data.q.load(QEKF_INS.q);

        vTaskDelayUntil(&last_wakeup_time, pdMS_TO_TICKS(1));
    }
}

void ins::init(const math::matrix<3, 3> &trans) {
    if (task_handle != nullptr) return;
    register_terminal_command();
    _trans = trans;
    const BaseType_t ok = xTaskCreate(task, "ins", 256, nullptr, osPriorityRealtime, &task_handle);
    BSP_ASSERT(ok == pdPASS);
}

data_t *ins::data() {
    return &_data;
}

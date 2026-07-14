// Created by fish on 2025/9/30.

#include "ins/ins.h"

#include "bsp/time.h"
#include "bsp/imu.h"
#include "bsp/flash.h"
#include "bsp/sys.h"
#include "utils/logger.h"
#include "utils/os.h"
#include "utils/terminal.h"

#include <cstdint>
#include "attitude_ekf.h"
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
    static bool inited = false;
}

static TaskHandle_t task_handle;
static uint32_t last_wakeup_time;
static constexpr uint32_t default_calibration_ms = 5000;
static constexpr uint32_t active_calibration_ms = 30000;
static constexpr uint32_t initial_stable_ms = 500;

static attitude_ekf ekf;

static bool run_calibration(const uint32_t duration_ms, bool save_to_flash = false) {
    calibration_t candidate;

    uint32_t wakeup_time = bsp_time_get_ms();
    for (uint32_t sample = 0; sample < duration_ms; sample++) {
        bsp_imu_data_t data;
        if (bsp_imu_read(&data) != BSP_STATUS_OK) return false;
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
            if (!ready()) {
                terminal::info("IMU is not ready.\r\n");
                return;
            }

            terminal::info("Keep the robot stationary for 30 seconds...\r\n");
            terminal::info(request_calibration() ?
                "Calibration saved.\r\n" :
                "Calibration failed; check IMU and flash.\r\n");
            return;
        }

        if (args.size() == 2 && args[1] == "watch") {
            if (!ready()) {
                terminal::info("IMU is not ready.\r\n");
                return;
            }
            while (terminal::running()) {
                const auto current = state();
                terminal::info("[roll, pitch, yaw] = [%f, %f, %f]\r\n", current.roll, current.pitch, current.yaw);
                os::task::sleep(1);
            }
            return;
        }

        terminal::info("Usage: imu [calibrate | watch]\r\n");
    }, "Show IMU status or calibrate gyro");
}

[[noreturn]] static void task(void *args) {
    ekf.init({});
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
        while (!run_calibration(default_calibration_ms)) {
            logger::error("IMU read failed during calibration, retrying");
            os::task::sleep(100);
        }
    }

    logger::info("waiting for stationary...");
    math::matrix<3, 1> accel_sum(0.0f);
    uint32_t stable_samples = 0;
    for (uint32_t wait = 0; ; ++wait) {
        bsp_imu_data_t first;
        if (bsp_imu_read(&first) != BSP_STATUS_OK) {
            stable_samples = 0;
            accel_sum = math::matrix<3, 1>(0.0f);
            os::task::sleep(1);
            continue;
        }

        math::matrix<3, 1> gyro, accel;
        gyro.load(first.gyro);
        accel.load(first.accel);
        gyro = _trans * (gyro - math::matrix<3, 1>(_cali_data.gyro_corr));
        accel = _trans * accel;

        const float g2 = gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2];
        const float a2 = accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2];

        if (g2 < 0.09f && a2 > 86.49f && a2 < 106.09f) {
            accel_sum += accel;
            if (++stable_samples >= initial_stable_ms) {
                ekf.init_attitude(accel_sum * (1.0f / static_cast<float>(stable_samples)));
                logger::info("attitude initialized (waited %lu ms)", wait);
                break;
            }
        } else {
            stable_samples = 0;
            accel_sum = math::matrix<3, 1>(0.0f);
        }
        os::task::sleep(1);
    }

    auto state = bsp_sys_enter_critical();
    inited = true;
    bsp_sys_exit_critical(state);
    last_wakeup_time = bsp_time_get_ms();

    for (;;) {
        data_t next;
        if (bsp_imu_read(&next.raw) != BSP_STATUS_OK) {
            vTaskDelayUntil(&last_wakeup_time, pdMS_TO_TICKS(1));
            continue;
        }
        next.gyro.load(next.raw.gyro);
        next.accel.load(next.raw.accel);
        next.gyro = _trans * (next.gyro - math::matrix<3, 1>(_cali_data.gyro_corr));
        next.accel = _trans * next.accel;

        ekf.update(next.gyro, next.accel, 0.001f, next.raw.temp);

        // euler() 返回 [yaw, pitch, roll] (rad)，与原 QEKF_INS 的 Roll/Pitch 定义互换
        const auto euler = ekf.euler();
        next.roll = euler[2];
        next.pitch = euler[1];
        next.yaw = euler[0];
        next.yaw_total_angle = ekf.yaw_total();
        next.q = ekf.q();
        state = bsp_sys_enter_critical();
        _data = next;
        bsp_sys_exit_critical(state);

        vTaskDelayUntil(&last_wakeup_time, pdMS_TO_TICKS(1));
    }
}

void ins::init(const math::matrix<3, 3> &trans) {
    if (task_handle != nullptr) return;
    register_terminal_command();
    _trans = trans;
    const BaseType_t ok = xTaskCreate(task, "ins", 1024, nullptr, osPriorityRealtime, &task_handle);
    BSP_ASSERT(ok == pdPASS);
}

bool ins::ready() {
    auto state = bsp_sys_enter_critical();
    const bool result = inited;
    bsp_sys_exit_critical(state);
    return result;
}

data_t ins::state() {
    const unsigned long state = bsp_sys_enter_critical();
    data_t copy = _data;
    bsp_sys_exit_critical(state);
    return copy;
}

data_t *ins::data() {
    return &_data;
}

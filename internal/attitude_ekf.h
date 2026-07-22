#pragma once

#include "math/matrix.h"
#include "math/quaternion.h"

namespace ins {

class attitude_ekf {
public:
    struct config {
        float q_var    = 10.0f;
        float bias_var = 0.0001f;
        float meas_var = 5e7f;
        float lambda   = 1.0f;
        float acc_lpf  = 0.003f;
        float gyro_lpf = 0.001f;
        float acc_fs   = 3.0f;
        float bias_learn_rate = 0.0001f;
        float stationary_gyro_threshold = 0.01f;
        float stationary_accel_tolerance = 0.35f;
        float stationary_direction_threshold = 0.05f;
        float stationary_confirm_time = 0.5f;
        float rotation_measurement_scale = 0.25f;
        int   meas_rate_div   = 5;
        float temp_slope = 0.0f;
        float temp_ref   = 25.0f;
        float notch_freq = 0.0f;       // 陷波频率 (Hz), 0=关闭
        float notch_bw   = 10.0f;      // 陷波带宽 (Hz)
    };
    void init(const config& cfg);
    void init_attitude(const math::matrix<3, 1>& accel);
    void update(const math::matrix<3, 1>& gyro, const math::matrix<3, 1>& accel,
                float dt, float temp = 0.0f);

    const math::quaternion& q()           const { return _q; }
    math::matrix<3, 1>      euler()       const { return _q.get_euler_angle(); }
    const math::matrix<2, 1>& gyro_bias()    const { return _gyro_bias; }
    float yaw_total()       const { return _yaw_total; }
    bool  converged()       const { return _converged; }
    bool  stable_confirmed() const { return _stable; }

private:
    math::quaternion   _q{1, 0, 0, 0};
    math::matrix<2, 1> _gyro_bias{};
    math::matrix<2, 1> _gyro_bias_lt{};
    math::matrix<3, 1> _accel_lpf{};
    math::matrix<3, 1> _gyro_lpf{};
    math::matrix<3, 1> _gyro_corr_prev{};

    math::matrix<6, 6> _cov{};

    math::matrix<6, 6> _F{}, _Q{};
    math::matrix<6, 6> _cov_pred{};
    math::matrix<3, 6> _H{};
    math::matrix<3, 3> _R{};

    float _q_var    = 10.0f;
    float _bias_var = 0.0001f;
    float _meas_var = 5e7f;
    float _lambda   = 1.0f;
    float _acc_lpf_tau  = 0.003f;
    float _gyro_lpf_tau = 0.001f;
    float _acc_fs_g     = 3.0f;
    float _bias_learn_rate = 0.0001f;
    float _stationary_gyro_threshold = 0.01f;
    float _stationary_accel_tolerance = 0.35f;
    float _stationary_direction_threshold = 0.05f;
    float _stationary_confirm_time = 0.5f;
    float _rotation_measurement_scale = 0.25f;
    int   _meas_rate_div    = 5;
    int   _meas_skip_counter = 0;
    float _temp_slope = 0.0f;
    float _temp_ref   = 25.0f;

    float _notch_freq = 0.0f, _notch_bw = 10.0f;
    float _notch_b0 = 1, _notch_b1 = 0, _notch_b2 = 0, _notch_a1 = 0, _notch_a2 = 0;
    float _notch_x1[3]{}, _notch_x2[3]{}, _notch_y1[3]{}, _notch_y2[3]{};

    bool     _initialized = false;
    bool     _converged = false;
    bool     _stable = false;
    uint32_t _update_count = 0;

    float _stable_time = 0.0f;
    math::matrix<3, 1> _bias_accum{};
    float _bias_accum_time = 0.0f;
    bool     _gyro_corr_prev_valid = false;

    float _chi_threshold = 1e-8f;
    float _adaptive_gain = 1.0f;

    float   _yaw_total = 0.0f;
    float   _yaw_last = 0.0f;
    int16_t _yaw_rounds = 0;

    static math::matrix<6, 1>  pack_state(const math::quaternion& q, const math::matrix<2, 1>& bias);
    static void                unpack_state(math::quaternion& q, math::matrix<2, 1>& bias, const math::matrix<6, 1>& x);
    static math::matrix<3, 1>  predict_gravity(const math::quaternion& q);
    static void                normalize(math::quaternion& q);
    static math::quaternion    rotation_delta(const math::matrix<3, 1>& gyro, float dt);
    static math::quaternion    propagate(const math::quaternion& q,
                                         const math::matrix<3, 1>& gyro, float dt);

    void reset_state();
    void build_F_gyro(const math::matrix<3, 1>& gyro, float dt);
    void build_F_quat(const math::quaternion& q_pred, float dt);
    void build_H(const math::quaternion& q_pred);
};

}

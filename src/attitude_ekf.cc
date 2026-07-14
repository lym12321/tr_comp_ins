#include "attitude_ekf.h"
#include <cmath>

namespace ins {

math::matrix<6, 1> attitude_ekf::pack_state(const math::quaternion& q, const math::matrix<2, 1>& bias) {
    math::matrix<6, 1> x;
    x[0] = q[0];  x[1] = q[1];  x[2] = q[2];  x[3] = q[3];
    x[4] = bias[0];  x[5] = bias[1];
    return x;
}

void attitude_ekf::unpack_state(math::quaternion& q, math::matrix<2, 1>& bias, const math::matrix<6, 1>& x) {
    q = math::quaternion(x[0], x[1], x[2], x[3]);
    bias[0] = x[4];  bias[1] = x[5];
}

math::matrix<3, 1> attitude_ekf::predict_gravity(const math::quaternion& q) {
    const float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    return math::matrix<3, 1>({
        2.0f * (q1 * q3 - q0 * q2),
        2.0f * (q0 * q1 + q2 * q3),
        q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3
    });
}

void attitude_ekf::normalize(math::quaternion& q) {
    const float n2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (n2 <= 1e-12f) { q = math::quaternion(1, 0, 0, 0); return; }
    const float inv = 1.0f / std::sqrt(n2);
    q = math::quaternion(q[0]*inv, q[1]*inv, q[2]*inv, q[3]*inv);
}

void attitude_ekf::build_F_gyro(const math::matrix<3, 1>& gyro, const float dt) {
    const float a = 0.5f * gyro[0] * dt;
    const float b = 0.5f * gyro[1] * dt;
    const float c = 0.5f * gyro[2] * dt;

    _F(0, 0) = 1;   _F(0, 1) = -a;  _F(0, 2) = -b;  _F(0, 3) = -c;  _F(0, 4) = 0;  _F(0, 5) = 0;
    _F(1, 0) = a;   _F(1, 1) = 1;   _F(1, 2) = c;   _F(1, 3) = -b;  _F(1, 4) = 0;  _F(1, 5) = 0;
    _F(2, 0) = b;   _F(2, 1) = -c;  _F(2, 2) = 1;   _F(2, 3) = a;   _F(2, 4) = 0;  _F(2, 5) = 0;
    _F(3, 0) = c;   _F(3, 1) = b;   _F(3, 2) = -a;  _F(3, 3) = 1;   _F(3, 4) = 0;  _F(3, 5) = 0;
    _F(4, 0) = 0;   _F(4, 1) = 0;   _F(4, 2) = 0;   _F(4, 3) = 0;   _F(4, 4) = 1;  _F(4, 5) = 0;
    _F(5, 0) = 0;   _F(5, 1) = 0;   _F(5, 2) = 0;   _F(5, 3) = 0;   _F(5, 4) = 0;  _F(5, 5) = 1;
}

void attitude_ekf::build_F_quat(const math::quaternion& q_pred, const float dt) {
    const float hdt = 0.5f * dt;
    const float q0 = q_pred[0], q1 = q_pred[1], q2 = q_pred[2], q3 = q_pred[3];
    _F(0, 4) =  q1 * hdt;   _F(0, 5) =  q2 * hdt;
    _F(1, 4) = -q0 * hdt;   _F(1, 5) =  q3 * hdt;
    _F(2, 4) = -q3 * hdt;   _F(2, 5) = -q0 * hdt;
    _F(3, 4) =  q2 * hdt;   _F(3, 5) = -q1 * hdt;
}

void attitude_ekf::build_H(const math::quaternion& q_pred) {
    const float q0 = q_pred[0], q1 = q_pred[1], q2 = q_pred[2], q3 = q_pred[3];
    _H = math::matrix<3, 6>(0.0f);
    _H(0, 0) = -2.0f * q2;   _H(0, 1) =  2.0f * q3;   _H(0, 2) = -2.0f * q0;   _H(0, 3) =  2.0f * q1;
    _H(1, 0) =  2.0f * q1;   _H(1, 1) =  2.0f * q0;   _H(1, 2) =  2.0f * q3;   _H(1, 3) =  2.0f * q2;
    _H(2, 0) =  2.0f * q0;   _H(2, 1) = -2.0f * q1;   _H(2, 2) = -2.0f * q2;   _H(2, 3) =  2.0f * q3;
}

void attitude_ekf::reset_state() {
    _q = math::quaternion(1, 0, 0, 0);
    _gyro_bias = math::matrix<2, 1>(0.0f);
    _gyro_bias_lt = math::matrix<2, 1>(0.0f);
    _accel_lpf = math::matrix<3, 1>(0.0f);
    _gyro_lpf = math::matrix<3, 1>(0.0f);
    for (int i = 0; i < 3; ++i)
        _notch_x1[i] = _notch_x2[i] = _notch_y1[i] = _notch_y2[i] = 0.0f;

    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c)
            _cov(r, c) = 0.1f;
    for (int i = 0; i < 4; ++i) _cov(i, i) = 100000.0f;
    _cov(4, 4) = _cov(5, 5) = 100.0f;

    _converged = false;
    _stable = false;
    _error_count = 0;
    _update_count = 0;
    _adaptive_gain = 1.0f;
    _stable_hold = 0;
    _bias_accum = math::matrix<3, 1>(0.0f);
    _bias_accum_count = 0;
    _stable_prev = false;
    _converge_boost = 0;
    _meas_skip_counter = 0;
    _yaw_total = 0.0f;
    _yaw_last = 0.0f;
    _yaw_rounds = 0;
}

void attitude_ekf::init(const config& cfg) {
    _q_var        = cfg.q_var;
    _bias_var     = cfg.bias_var;
    _meas_var     = cfg.meas_var;
    _lambda       = cfg.lambda > 1.0f ? 1.0f : cfg.lambda;
    _acc_lpf_tau  = cfg.acc_lpf;
    _gyro_lpf_tau = cfg.gyro_lpf;
    _acc_fs_g         = cfg.acc_fs;
    _bias_learn_rate  = cfg.bias_learn_rate;
    _meas_rate_div    = cfg.meas_rate_div > 0 ? cfg.meas_rate_div : 1;
    _meas_skip_counter = 0;
    _temp_slope = cfg.temp_slope;
    _temp_ref   = cfg.temp_ref;
    _notch_freq = cfg.notch_freq;
    _notch_bw   = cfg.notch_bw;
    _chi_threshold = 1e-8f;

    // 预计算陷波系数 (假设 fs=1000Hz)
    if (_notch_freq > 0.0f) {
        const float w = 6.283185307f * _notch_freq * 0.001f;
        const float r = 1.0f - 3.141592654f * _notch_bw * 0.001f;
        _notch_b0 = 1.0f;
        _notch_b1 = -2.0f * std::cos(w);
        _notch_b2 = 1.0f;
        _notch_a1 = -2.0f * r * std::cos(w);
        _notch_a2 = r * r;
    }
    reset_state();
    _initialized = true;
}

void attitude_ekf::init_attitude(const math::matrix<3, 1>& accel) {
    const float ax = accel[0], ay = accel[1], az = accel[2];
    const float n2 = ax*ax + ay*ay + az*az;
    if (n2 < 1e-12f) return;
    const float inv_n = 1.0f / std::sqrt(n2);
    const float az_u = az * inv_n;

    const float k = std::sqrt(2.0f * (1.0f + az_u));
    if (k > 1e-6f) {
        _q = math::quaternion(0.5f * k, ay * inv_n / k, -ax * inv_n / k, 0.0f);
    } else {
        _q = math::quaternion(0.0f, 1.0f, 0.0f, 0.0f);
    }
}

void attitude_ekf::update(const math::matrix<3, 1>& gyro,
                          const math::matrix<3, 1>& accel,
                          const float dt, const float temp) {
    if (!_initialized) return;

    const float state_n2 = _q[0]*_q[0] + _q[1]*_q[1] + _q[2]*_q[2] + _q[3]*_q[3]
                         + _gyro_bias[0]*_gyro_bias[0] + _gyro_bias[1]*_gyro_bias[1]
                         + _gyro_bias_lt[0]*_gyro_bias_lt[0] + _gyro_bias_lt[1]*_gyro_bias_lt[1];
    if (!(state_n2 > 1e-12f) || state_n2 > 4.0f) {
        reset_state();
        _converge_boost = 200;
    }
    const float temp_bias = _temp_slope * (temp - _temp_ref);

    const math::matrix<3, 1> gyro_f = (_update_count == 0 || _gyro_lpf_tau <= 0.0f)
        ? gyro
        : _gyro_lpf + (gyro - _gyro_lpf) * (dt / (dt + _gyro_lpf_tau));
    _gyro_lpf = gyro_f;

    math::matrix<3, 1> gyro_notch = gyro_f;
    if (_notch_freq > 0.0f) {
        for (int i = 0; i < 3; ++i) {
            const float y = _notch_b0 * gyro_f[i] + _notch_b1 * _notch_x1[i] + _notch_b2 * _notch_x2[i]
                          - _notch_a1 * _notch_y1[i] - _notch_a2 * _notch_y2[i];
            _notch_x2[i] = _notch_x1[i];  _notch_x1[i] = gyro_f[i];
            _notch_y2[i] = _notch_y1[i];  _notch_y1[i] = y;
            gyro_notch[i] = y;
        }
    }

    const math::matrix<3, 1> gyro_corr{
        gyro_notch[0] - _gyro_bias[0] - _gyro_bias_lt[0] - temp_bias,
        gyro_notch[1] - _gyro_bias[1] - _gyro_bias_lt[1] - temp_bias,
        gyro_notch[2] - temp_bias
    };

    build_F_gyro(gyro_corr, dt);

    const auto x_cur = pack_state(_q, _gyro_bias);
    const auto x_pred_raw = _F * x_cur;

    math::quaternion q_pred;
    math::matrix<2, 1> bias_pred;
    unpack_state(q_pred, bias_pred, x_pred_raw);
    normalize(q_pred);

    build_F_quat(q_pred, dt);

    _cov(4, 4) = std::fmin(_cov(4, 4) / _lambda, 10000.0f);
    _cov(5, 5) = std::fmin(_cov(5, 5) / _lambda, 10000.0f);

    _Q = math::matrix<6, 6>(0.0f);
    _Q(0, 0) = _Q(1, 1) = _Q(2, 2) = _Q(3, 3) = _q_var   * dt;
    _Q(4, 4) = _Q(5, 5) = _bias_var * dt;
    _cov_pred = _F * _cov * _F.T() + _Q;

    const float raw_acc_n2 = accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2];
    const float fs_thr = _acc_fs_g * 9.80665f * 0.95f;
    const bool raw_accel_valid = raw_acc_n2 > 1e-12f && raw_acc_n2 < 1e6f
                              && std::fabs(accel[0]) <= fs_thr
                              && std::fabs(accel[1]) <= fs_thr
                              && std::fabs(accel[2]) <= fs_thr;
    if (raw_accel_valid) {
        _accel_lpf = (_update_count == 0 || _acc_lpf_tau <= 0.0f)
            ? accel
            : _accel_lpf + (accel - _accel_lpf) * (dt / (dt + _acc_lpf_tau));
    }

    const float acc_n2 = _accel_lpf[0]*_accel_lpf[0]
                       + _accel_lpf[1]*_accel_lpf[1]
                       + _accel_lpf[2]*_accel_lpf[2];
    const bool accel_valid = raw_accel_valid && acc_n2 > 1e-12f && acc_n2 < 1e6f;
    const float acc_n = accel_valid ? std::sqrt(acc_n2) : 0.0f;
    const auto h_pred = predict_gravity(q_pred);
    auto z = h_pred;
    if (accel_valid) z = _accel_lpf * (1.0f / acc_n);

    _stable = accel_valid
           && (gyro_corr[0]*gyro_corr[0] + gyro_corr[1]*gyro_corr[1] + gyro_corr[2]*gyro_corr[2] < 0.09f)
           && (acc_n2 > 86.49f && acc_n2 < 106.09f);

    if (_stable && !_stable_prev) _converge_boost = 200;
    _stable_prev = _stable;

    if (_stable) {
        ++_stable_hold;
        if (_stable_hold >= 500) {
            _bias_accum[0] += gyro_corr[0];
            _bias_accum[1] += gyro_corr[1];
            ++_bias_accum_count;
        }
    } else {
        _stable_hold = 0;
        _bias_accum = math::matrix<3, 1>(0.0f);
        _bias_accum_count = 0;
    }

    build_H(q_pred);
    const auto innov = z - h_pred;

    const float acc_dev = accel_valid ? std::fabs(acc_n - 9.80665f) : 9.80665f;
    float r_scale = 1.0f + 10.0f * (acc_dev * acc_dev);

    if (_converge_boost > 0) r_scale *= 0.001f;

    if (!accel_valid) r_scale *= 1e6f;

    auto innov_clamped = innov;
    for (int i = 0; i < 3; ++i)
        innov_clamped[i] = std::fmax(-0.5f, std::fmin(0.5f, innov[i]));

    _R = math::matrix<3, 3>(_meas_var * r_scale);
    const auto S   = _H * _cov_pred * _H.T() + _R;
    const auto invS = S.inv();
    const auto invS_nu = invS * innov_clamped;
    float chi_sq = 0.0f;
    for (int i = 0; i < 3; ++i) chi_sq += innov_clamped[i] * invS_nu[i];

    float chi_thr = _chi_threshold;
    if (_converge_boost > 0) { chi_thr *= 1000.0f; --_converge_boost; }

    if (accel_valid && chi_sq < 0.5f * chi_thr) _converged = true;

    bool do_update = accel_valid;

    if (!accel_valid) {
        _q = q_pred;
        _gyro_bias = bias_pred;
    } else if (chi_sq > chi_thr && _converged) {
        _stable ? ++_error_count : (_error_count = 0);

        if (_error_count > 50) {
            _converged = false;
        } else {
            _q = q_pred;
            _gyro_bias = bias_pred;
            do_update = false;
        }
    } else if (!_stable && _meas_skip_counter != 0) {
        _q = q_pred;
        _gyro_bias = bias_pred;
        do_update = false;
    }

    if (do_update) {
        const float adaptive_gain = (chi_sq > 0.1f * chi_thr && _converged)
            ? (chi_thr - chi_sq) / (0.9f * chi_thr)
            : 1.0f;
        _adaptive_gain = std::fmax(0.0f, std::fmin(1.0f, adaptive_gain));
        _error_count = 0;

        auto K = _cov_pred * _H.T() * invS;

        float ori_cos[2];
        for (int i = 0; i < 2; ++i)
            ori_cos[i] = std::acos(std::fmin(1.0f, std::fabs(h_pred[i])));

        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 3; ++j)
                K(i, j) *= _adaptive_gain;
        for (int i = 4; i < 6; ++i)
            for (int j = 0; j < 3; ++j)
                K(i, j) *= _adaptive_gain * ori_cos[i - 4] / 1.5707963f;

        auto dx = K * innov_clamped;
        if (_converged) {
            for (int i = 4; i < 6; ++i)
                dx[i] = std::fmax(-1e-2f * dt, std::fmin(1e-2f * dt, dx[i]));
        }
        const auto x_new = x_pred_raw + dx;
        unpack_state(_q, _gyro_bias, x_new);
        // 绝对零偏限幅: ±0.1 rad/s
        for (int i = 0; i < 2; ++i)
            _gyro_bias[i] = std::fmax(-0.1f, std::fmin(0.1f, _gyro_bias[i]));
        normalize(_q);
        _cov = _cov_pred - K * _H * _cov_pred;
    } else {
        _cov = _cov_pred;
    }

    {
        constexpr float tau = 6.283185307179586f;
        const float yaw = _q.get_euler_angle()[0];

        if (yaw - _yaw_last >  tau * 0.5f) --_yaw_rounds;
        if (yaw - _yaw_last < -tau * 0.5f) ++_yaw_rounds;
        _yaw_total = tau * static_cast<float>(_yaw_rounds) + yaw;
        _yaw_last = yaw;
    }

    // 静止 0.5s 后，每累积 1s 数据更新长期零偏
    if (_bias_accum_count >= 1000) {
        const float inv_n = 1.0f / static_cast<float>(_bias_accum_count);
        const float avg_x = _bias_accum[0] * inv_n;
        const float avg_y = _bias_accum[1] * inv_n;
        _gyro_bias_lt[0] += _bias_learn_rate * avg_x;
        _gyro_bias_lt[1] += _bias_learn_rate * avg_y;
        _bias_accum = math::matrix<3, 1>(0.0f);
        _bias_accum_count = 0;
    }

    if (!_stable) {
        if (++_meas_skip_counter >= _meas_rate_div) _meas_skip_counter = 0;
    } else {
        _meas_skip_counter = 0;
    }

    ++_update_count;
}

}

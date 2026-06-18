#include "imu_preintegration.h"

#define _USE_MATH_DEFINES
#include <cmath>

ImuPreintegration::ImuPreintegration(const Eigen::Vector3d& ba, const Eigen::Vector3d& bg,
                                     double sigma_acc, double sigma_gyro,
                                     double sigma_ba_walk, double sigma_bg_walk)
    : sigma_acc_(sigma_acc),
      sigma_gyro_(sigma_gyro),
      sigma_ba_walk_(sigma_ba_walk),
      sigma_bg_walk_(sigma_bg_walk) {
    Reset(ba, bg);
}

void ImuPreintegration::Reset(const Eigen::Vector3d& ba, const Eigen::Vector3d& bg) {
    linearized_ba = ba;
    linearized_bg = bg;

    delta_p.setZero();
    delta_v.setZero();
    delta_q = Eigen::Quaterniond::Identity();

    dp_dba.setZero();
    dp_dbg.setZero();
    dv_dba.setZero();
    dv_dbg.setZero();
    dq_dbg.setZero();

    cov.setZero();
    dt_sum = 0.0;
    samples.clear();
}

static Eigen::Matrix3d Skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0.0, -v.z(), v.y(),
         v.z(), 0.0,  -v.x(),
        -v.y(), v.x(), 0.0;
    return m;
}

// Small-angle quaternion from rotation vector.
static Eigen::Quaterniond ExpSO3(const Eigen::Vector3d& theta) {
    double angle = theta.norm();
    if (angle < 1e-8) {
        return Eigen::Quaterniond(1.0, theta.x() * 0.5, theta.y() * 0.5, theta.z() * 0.5).normalized();
    }
    Eigen::Vector3d axis = theta / angle;
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
}

void ImuPreintegration::Integrate(double dt, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro) {
    samples.push_back({dt, acc, gyro});

    Eigen::Vector3d acc_hat  = acc  - linearized_ba;
    Eigen::Vector3d gyro_hat = gyro - linearized_bg;

    Eigen::Matrix3d R = delta_q.toRotationMatrix();

    // ---- Bias Jacobian updates (chain rule, forward propagation) ----
    // d(delta_p)/d(ba):  dp_dba_new = dp_dba + dv_dba*dt - 0.5*R*dt^2
    // d(delta_p)/d(bg):  dp_dbg_new = dp_dbg + dv_dbg*dt - 0.5*R*Skew(acc_hat)*dq_dbg*dt^2
    dp_dba += dv_dba * dt - 0.5 * R * dt * dt;
    dp_dbg += dv_dbg * dt - 0.5 * R * Skew(acc_hat) * dq_dbg * dt * dt;

    // d(delta_v)/d(ba):  dv_dba_new = dv_dba - R*dt
    // d(delta_v)/d(bg):  dv_dbg_new = dv_dbg - R*Skew(acc_hat)*dq_dbg*dt
    dv_dba -= R * dt;
    dv_dbg -= R * Skew(acc_hat) * dq_dbg * dt;

    // d(Log(delta_R))/d(bg):  dq_dbg_new = (I-Skew(gyro_hat*dt))*dq_dbg - I*dt
    Eigen::Matrix3d dR_step = Eigen::Matrix3d::Identity() - Skew(gyro_hat * dt);
    dq_dbg = dR_step * dq_dbg - Eigen::Matrix3d::Identity() * dt;

    // ---- Noise covariance propagation ----
    // Discrete error-state transition: A (9x9), B (9x6)
    Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Identity();
    A.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    A.block<3, 3>(0, 6) = -0.5 * R * Skew(acc_hat) * dt * dt;
    A.block<3, 3>(3, 6) = -R * Skew(acc_hat) * dt;
    A.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - Skew(gyro_hat * dt);

    Eigen::Matrix<double, 9, 6> B = Eigen::Matrix<double, 9, 6>::Zero();
    B.block<3, 3>(0, 0) = 0.5 * R * dt * dt;
    B.block<3, 3>(3, 0) = R * dt;
    B.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dt;

    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (sigma_acc_ * sigma_acc_);
    Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (sigma_gyro_ * sigma_gyro_);

    cov = A * cov * A.transpose() + B * Q * B.transpose();

    // ---- State update (zero-order hold Euler) ----
    delta_p += delta_v * dt + 0.5 * R * acc_hat * dt * dt;
    delta_v += R * acc_hat * dt;
    delta_q = (delta_q * ExpSO3(gyro_hat * dt)).normalized();
    dt_sum += dt;
}

void ImuPreintegration::Repropagate(const Eigen::Vector3d& new_ba, const Eigen::Vector3d& new_bg) {
    std::vector<ImuSample> saved = samples;
    Reset(new_ba, new_bg);
    for (const auto& s : saved) {
        Integrate(s.dt, s.acc, s.gyro);
    }
}

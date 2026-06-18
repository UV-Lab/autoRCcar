#ifndef AUTORCCAR_INS_GNSS__IMU_PREINTEGRATION_H
#define AUTORCCAR_INS_GNSS__IMU_PREINTEGRATION_H

#include <eigen3/Eigen/Dense>
#include <vector>

struct ImuSample {
    double dt;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
};

// IMU pre-integration between two keyframes following Forster et al. (2015).
// Accumulates delta_p, delta_v, delta_q in the body frame at the start of
// integration, along with first-order bias Jacobians for online correction.
class ImuPreintegration {
   public:
    ImuPreintegration(const Eigen::Vector3d& ba, const Eigen::Vector3d& bg,
                      double sigma_acc, double sigma_gyro,
                      double sigma_ba_walk, double sigma_bg_walk);

    // Accumulate one IMU sample (midpoint Euler integration).
    void Integrate(double dt, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro);

    // Re-propagate from stored raw samples with updated bias estimates.
    void Repropagate(const Eigen::Vector3d& new_ba, const Eigen::Vector3d& new_bg);

    // Reset to initial state (clears all accumulated data).
    void Reset(const Eigen::Vector3d& ba, const Eigen::Vector3d& bg);

    // Pre-integrated deltas (body frame at start of integration window).
    Eigen::Vector3d delta_p;
    Eigen::Vector3d delta_v;
    Eigen::Quaterniond delta_q;

    // First-order Jacobians w.r.t. biases (for online bias correction).
    Eigen::Matrix3d dp_dba, dp_dbg;
    Eigen::Matrix3d dv_dba, dv_dbg;
    Eigen::Matrix3d dq_dbg;  // d(Log(delta_R))/d(bg)

    // Noise covariance of the pre-integrated measurement [9x9: p, v, phi].
    Eigen::Matrix<double, 9, 9> cov;

    double dt_sum;
    Eigen::Vector3d linearized_ba, linearized_bg;

    std::vector<ImuSample> samples;  // raw samples stored for re-propagation

   private:
    double sigma_acc_, sigma_gyro_, sigma_ba_walk_, sigma_bg_walk_;
};

#endif

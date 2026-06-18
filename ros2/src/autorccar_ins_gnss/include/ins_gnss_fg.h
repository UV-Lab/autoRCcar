#ifndef AUTORCCAR_INS_GNSS__INS_GNSS_FG_H
#define AUTORCCAR_INS_GNSS__INS_GNSS_FG_H

#include <deque>
#include <memory>
#include <string>

#include <eigen3/Eigen/Dense>

#include "ekf.h"              // NavSol reuse
#include "imu_preintegration.h"

// One node in the sliding-window factor graph.
struct Keyframe {
    double time;

    // Navigation state (NED frame)
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Quaterniond quat;  // body → NED

    // IMU biases
    Eigen::Vector3d ba;
    Eigen::Vector3d bg;

    // GNSS measurement associated with this keyframe
    Eigen::Vector3d gnss_pos;  // NED
    Eigen::Vector3d gnss_vel;  // NED
    bool has_gnss;

    // IMU pre-integration from the *previous* keyframe to this one.
    // Null for the very first keyframe.
    std::shared_ptr<ImuPreintegration> preint;
};

// Sliding-window factor graph for loosely-coupled INS/GNSS fusion.
//
// At each GNSS epoch a new keyframe is created.  The graph contains:
//   - IMU pre-integration factors between consecutive keyframes
//   - GNSS position / velocity factors at each keyframe
//   - Bias random-walk factors between consecutive keyframes
//   - A strong prior pinning the oldest keyframe in the window
//
// One Gauss-Newton step is performed per GNSS epoch (real-time capable).
// Between epochs the state is propagated via the current pre-integrator to
// deliver NavState at IMU rate.
class InsGnssFg {
   public:
    InsGnssFg(const std::string& config_file);

    bool is_time_set;
    bool is_initialized;

    std::string GetNavFrame() const { return nav_frame_; }

    void SetTimeInit(double time);
    void SetYawInit(float yaw_deg);

    // Call once per IMU measurement.  Returns the current propagated NavSol
    // (non-null only after initialization).
    NavSol* UpdateImu(double time, const Eigen::Vector3d& acc,
                      const Eigen::Vector3d& gyro);

    // Call once per GNSS measurement.  Triggers a keyframe + optimization.
    // Returns the corrected NavSol at the keyframe epoch.
    NavSol* UpdateGnss(double time, const Eigen::Vector3d& pos_ecef,
                       const Eigen::Vector3d& vel_ecef);

   private:
    // ---- Configuration ----------------------------------------
    std::string nav_frame_;
    double alignment_time_;
    double yaw_init_deg_;
    int    max_window_size_;

    double sigma_acc_, sigma_gyro_;
    double sigma_ba_walk_, sigma_bg_walk_;
    double sigma_gnss_pos_, sigma_gnss_vel_;

    // ---- Coordinate frame origin (set at first GNSS fix) ------
    Eigen::Vector3d pos_origin_ecef_;
    Eigen::Matrix3d Ce2n_;            // ECEF → NED rotation at origin
    Eigen::Vector3d g_ned_;           // gravity in NED [0, 0, 9.80665]

    // ---- Alignment state --------------------------------------
    double time_init_;
    int    alignment_cnt_;
    Eigen::Vector3d sum_acc_, sum_gyro_;
    bool   gnss_received_during_align_;
    Eigen::Vector3d first_gnss_pos_ecef_;
    Eigen::Vector3d first_gnss_vel_ecef_;

    // ---- Sliding window ---------------------------------------
    std::deque<Keyframe> window_;

    // Current pre-integrator (from last keyframe up to now)
    std::shared_ptr<ImuPreintegration> current_preint_;

    // Propagated navigation solution (output at IMU rate)
    NavSol nav_sol_;

    // Last raw IMU for live propagation
    double imu_time_prev_;
    Eigen::Vector3d imu_acc_prev_;
    Eigen::Vector3d imu_gyro_prev_;

    // ---- Internal helpers -------------------------------------
    std::string config_file_;  // stored for SetYawInit re-use

    void LoadSettings(const std::string& config_file);
    void Reset();

    // Initialize the first keyframe once alignment is complete.
    void InitFirstKeyframe();

    // Add a new keyframe (called on each GNSS update after init).
    void AddKeyframe(double time, const Eigen::Vector3d& gnss_pos_ned,
                     const Eigen::Vector3d& gnss_vel_ned);

    // Gauss-Newton optimization over the current window.
    void Optimize();

    // Propagate state from a keyframe using supplied pre-integration.
    void PropagateFromKeyframe(const Keyframe& kf,
                               const ImuPreintegration& preint,
                               Eigen::Vector3d& pos_out,
                               Eigen::Vector3d& vel_out,
                               Eigen::Quaterniond& quat_out) const;

    // Convert GNSS ECEF measurements to NED.
    void Ecef2Ned(const Eigen::Vector3d& pos_ecef,
                  const Eigen::Vector3d& vel_ecef,
                  Eigen::Vector3d& pos_ned,
                  Eigen::Vector3d& vel_ned) const;

    // Update nav_sol_ from the last keyframe + current_preint_.
    void UpdateNavSol(double time, const Eigen::Vector3d& acc,
                      const Eigen::Vector3d& gyro);
};

#endif

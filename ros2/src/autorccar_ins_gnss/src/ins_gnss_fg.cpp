#include "ins_gnss_fg.h"

#include <fstream>
#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>

#include <yaml-cpp/yaml.h>

#include "ins_toolbox.h"

// ---------------------------------------------------------------------------
// Lie algebra helpers (SO3)
// ---------------------------------------------------------------------------

static Eigen::Matrix3d Skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m <<  0.0,  -v.z(),  v.y(),
         v.z(),   0.0,  -v.x(),
        -v.y(),  v.x(),   0.0;
    return m;
}

static Eigen::Quaterniond ExpSO3(const Eigen::Vector3d& theta) {
    double angle = theta.norm();
    if (angle < 1e-8) {
        return Eigen::Quaterniond(1.0, theta.x() * 0.5,
                                       theta.y() * 0.5,
                                       theta.z() * 0.5).normalized();
    }
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, theta / angle));
}

static Eigen::Vector3d LogSO3(const Eigen::Quaterniond& q) {
    Eigen::Quaterniond qn = q.normalized();
    if (qn.w() < 0.0) qn.coeffs() = -qn.coeffs();
    double vec_norm = qn.vec().norm();
    if (vec_norm < 1e-8) {
        return 2.0 * qn.vec();
    }
    double angle = 2.0 * std::atan2(vec_norm, qn.w());
    return angle * qn.vec() / vec_norm;
}

// ---------------------------------------------------------------------------
// Constructor / Reset / LoadSettings
// ---------------------------------------------------------------------------

InsGnssFg::InsGnssFg(const std::string& config_file)
    : is_time_set(false), is_initialized(false), config_file_(config_file) {
    LoadSettings(config_file);
    Reset();
}

void InsGnssFg::LoadSettings(const std::string& config_file) {
    std::ifstream fin(config_file);
    if (!fin) {
        std::cerr << "[InsGnssFg] Failed to open config: " << config_file << std::endl;
        exit(-1);
    }
    YAML::Node cfg = YAML::LoadFile(config_file);

    nav_frame_      = cfg["navigation_frame"].as<std::string>();
    alignment_time_ = cfg["alignment_time_sec"].as<double>();
    yaw_init_deg_   = cfg["initial_yaw_deg"].as<double>();

    max_window_size_ = cfg["fg.max_window_size"].as<int>(5);

    sigma_acc_      = cfg["fg.sigma_acc"].as<double>(0.1);
    sigma_gyro_     = cfg["fg.sigma_gyro"].as<double>(0.01);
    sigma_ba_walk_  = cfg["fg.sigma_ba_walk"].as<double>(1e-4);
    sigma_bg_walk_  = cfg["fg.sigma_bg_walk"].as<double>(1e-5);
    sigma_gnss_pos_ = cfg["fg.sigma_gnss_pos"].as<double>(1.0);
    sigma_gnss_vel_ = cfg["fg.sigma_gnss_vel"].as<double>(0.1);

    std::cout << "[InsGnssFg] Config loaded: frame=" << nav_frame_
              << "  window=" << max_window_size_ << std::endl;
}

void InsGnssFg::Reset() {
    is_time_set     = false;
    is_initialized  = false;
    time_init_      = 0.0;
    alignment_cnt_  = 0;
    sum_acc_.setZero();
    sum_gyro_.setZero();
    gnss_received_during_align_ = false;
    first_gnss_pos_ecef_.setZero();
    first_gnss_vel_ecef_.setZero();

    pos_origin_ecef_.setZero();
    Ce2n_.setZero();
    g_ned_ << 0.0, 0.0, 9.80665;   // NED: gravity points downward (+Z)

    window_.clear();
    current_preint_.reset();
    imu_time_prev_  = 0.0;
    imu_acc_prev_.setZero();
    imu_gyro_prev_.setZero();
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void InsGnssFg::SetTimeInit(double time) {
    time_init_  = time;
    is_time_set = true;
    std::cout.precision(15);
    std::cout << "[InsGnssFg] Start time: " << time_init_ << " s" << std::endl;
}

void InsGnssFg::SetYawInit(float yaw_deg) {
    LoadSettings(config_file_);  // reload all params first
    yaw_init_deg_ = static_cast<double>(yaw_deg);
    Reset();
}

NavSol* InsGnssFg::UpdateImu(double time, const Eigen::Vector3d& acc,
                              const Eigen::Vector3d& gyro) {
    // --- Alignment phase ---
    if (!is_initialized) {
        if (imu_time_prev_ > 0.0) {
            double dt = time - imu_time_prev_;
            if (dt > 0.0 && dt < 1.0) {
                sum_acc_  += acc;
                sum_gyro_ += gyro;
                alignment_cnt_++;

                if (current_preint_) {
                    current_preint_->Integrate(dt, acc, gyro);
                }
            }
        }

        imu_time_prev_ = time;
        imu_acc_prev_  = acc;
        imu_gyro_prev_ = gyro;

        // Check if alignment is complete
        if (is_time_set && gnss_received_during_align_ &&
            (time - time_init_) >= alignment_time_) {
            InitFirstKeyframe();
        }
        return nullptr;
    }

    // --- Normal operation: pre-integrate and propagate ---
    if (imu_time_prev_ > 0.0) {
        double dt = time - imu_time_prev_;
        if (dt > 0.0 && dt < 1.0 && current_preint_) {
            current_preint_->Integrate(dt, acc, gyro);
        }
    }
    imu_time_prev_ = time;
    imu_acc_prev_  = acc;
    imu_gyro_prev_ = gyro;

    UpdateNavSol(time, acc, gyro);
    return &nav_sol_;
}

NavSol* InsGnssFg::UpdateGnss(double time, const Eigen::Vector3d& pos_ecef,
                               const Eigen::Vector3d& vel_ecef) {
    // Store first GNSS fix for alignment
    if (!gnss_received_during_align_) {
        first_gnss_pos_ecef_ = pos_ecef;
        first_gnss_vel_ecef_ = vel_ecef;
        gnss_received_during_align_ = true;

        // Begin pre-integrating from now toward the first keyframe
        Eigen::Vector3d ba_init = Eigen::Vector3d::Zero();
        Eigen::Vector3d bg_init = Eigen::Vector3d::Zero();
        current_preint_ = std::make_shared<ImuPreintegration>(
            ba_init, bg_init, sigma_acc_, sigma_gyro_,
            sigma_ba_walk_, sigma_bg_walk_);
    }

    if (!is_initialized) return nullptr;

    // Convert GNSS to NED
    Eigen::Vector3d gnss_pos_ned, gnss_vel_ned;
    Ecef2Ned(pos_ecef, vel_ecef, gnss_pos_ned, gnss_vel_ned);

    // Add keyframe and optimize
    AddKeyframe(time, gnss_pos_ned, gnss_vel_ned);
    Optimize();

    // Start fresh pre-integration from the new keyframe
    const Keyframe& kf = window_.back();
    current_preint_ = std::make_shared<ImuPreintegration>(
        kf.ba, kf.bg, sigma_acc_, sigma_gyro_,
        sigma_ba_walk_, sigma_bg_walk_);
    imu_time_prev_ = time;

    // Publish corrected state at GNSS epoch
    UpdateNavSol(time, imu_acc_prev_, imu_gyro_prev_);
    return &nav_sol_;
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

void InsGnssFg::InitFirstKeyframe() {
    // ---- Estimate gyro bias and gravity direction ----
    Eigen::Vector3d bg_init = sum_gyro_ / alignment_cnt_;
    Eigen::Vector3d g_body  = sum_acc_ / alignment_cnt_;
    g_body.normalize();

    // Estimate roll/pitch from gravity
    Eigen::Vector3d g_nav(0.0, 0.0, 1.0);  // NED: gravity = +Z
    Eigen::Vector3d g_mid = (g_body + g_nav).normalized();
    Eigen::Quaterniond att_q;
    att_q.w()   = g_body.dot(g_mid);
    att_q.vec() = g_body.cross(g_mid);
    att_q.normalize();

    // Apply initial yaw
    Eigen::Vector3d yaw_vec(0.0, 0.0, yaw_init_deg_ * M_PI / 180.0);
    att_q = (Euler2Quat(yaw_vec) * att_q).normalized();

    // ---- Set ECEF origin ----
    pos_origin_ecef_ = first_gnss_pos_ecef_;

    Eigen::Vector3d LLH = ECEF2LLH(pos_origin_ecef_);
    double lat = LLH.x();
    double lon = LLH.y();

    Ce2n_ << -std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon),  std::cos(lat),
              -std::sin(lon),                  std::cos(lon),                   0.0,
              -std::cos(lat) * std::cos(lon), -std::cos(lat) * std::sin(lon), -std::sin(lat);

    Eigen::Vector3d pos_ned = Ce2n_ * (first_gnss_pos_ecef_ - pos_origin_ecef_);
    Eigen::Vector3d vel_ned = Ce2n_ * first_gnss_vel_ecef_;

    // ---- Create first keyframe ----
    Keyframe kf;
    kf.time     = time_init_ + alignment_time_;
    kf.pos      = pos_ned;
    kf.vel      = vel_ned;
    kf.quat     = att_q;
    kf.ba       = Eigen::Vector3d::Zero();
    kf.bg       = bg_init;
    kf.gnss_pos = pos_ned;
    kf.gnss_vel = vel_ned;
    kf.has_gnss = true;
    kf.preint   = nullptr;
    window_.push_back(kf);

    // Fresh pre-integrator from here
    current_preint_ = std::make_shared<ImuPreintegration>(
        kf.ba, kf.bg, sigma_acc_, sigma_gyro_,
        sigma_ba_walk_, sigma_bg_walk_);
    imu_time_prev_ = kf.time;

    is_initialized = true;

    Eigen::Vector3d eul = Quat2Euler(att_q);
    std::cout << "[InsGnssFg] Initialized. "
              << "pos_ned=[" << pos_ned.transpose() << "] "
              << "att_deg=[" << eul.transpose() * 180.0 / M_PI << "]" << std::endl;
}

// ---------------------------------------------------------------------------
// Keyframe management
// ---------------------------------------------------------------------------

void InsGnssFg::AddKeyframe(double time,
                             const Eigen::Vector3d& gnss_pos_ned,
                             const Eigen::Vector3d& gnss_vel_ned) {
    const Keyframe& prev = window_.back();

    // Propagate state from previous keyframe using pre-integration
    Eigen::Vector3d pos_prop, vel_prop;
    Eigen::Quaterniond quat_prop;
    PropagateFromKeyframe(prev, *current_preint_, pos_prop, vel_prop, quat_prop);

    Keyframe kf;
    kf.time     = time;
    kf.pos      = pos_prop;
    kf.vel      = vel_prop;
    kf.quat     = quat_prop;
    kf.ba       = prev.ba;
    kf.bg       = prev.bg;
    kf.gnss_pos = gnss_pos_ned;
    kf.gnss_vel = gnss_vel_ned;
    kf.has_gnss = true;
    kf.preint   = current_preint_;  // hand off

    window_.push_back(kf);

    // Slide window
    while (static_cast<int>(window_.size()) > max_window_size_) {
        window_.pop_front();
    }
}

// ---------------------------------------------------------------------------
// State propagation
// ---------------------------------------------------------------------------

void InsGnssFg::PropagateFromKeyframe(const Keyframe& kf,
                                       const ImuPreintegration& preint,
                                       Eigen::Vector3d& pos_out,
                                       Eigen::Vector3d& vel_out,
                                       Eigen::Quaterniond& quat_out) const {
    const Eigen::Matrix3d Ri = kf.quat.toRotationMatrix();
    double T = preint.dt_sum;

    // Bias correction to pre-integrated deltas
    Eigen::Vector3d dba = kf.ba - preint.linearized_ba;
    Eigen::Vector3d dbg = kf.bg - preint.linearized_bg;

    Eigen::Vector3d dp = preint.delta_p + preint.dp_dba * dba + preint.dp_dbg * dbg;
    Eigen::Vector3d dv = preint.delta_v + preint.dv_dba * dba + preint.dv_dbg * dbg;
    Eigen::Quaterniond dq_corr = (preint.delta_q * ExpSO3(preint.dq_dbg * dbg)).normalized();

    // pos_j = pos_i + vel_i*T + 0.5*g*T^2 + R_i * dp
    pos_out  = kf.pos + kf.vel * T + 0.5 * g_ned_ * T * T + Ri * dp;
    // vel_j = vel_i + g*T + R_i * dv
    vel_out  = kf.vel + g_ned_ * T + Ri * dv;
    // quat_j = quat_i * delta_q_corrected
    quat_out = (kf.quat * dq_corr).normalized();
}

void InsGnssFg::UpdateNavSol(double time, const Eigen::Vector3d& acc,
                              const Eigen::Vector3d& gyro) {
    if (window_.empty()) return;

    const Keyframe& kf = window_.back();
    Eigen::Vector3d pos, vel;
    Eigen::Quaterniond quat;
    PropagateFromKeyframe(kf, *current_preint_, pos, vel, quat);

    Eigen::Vector3d origin = pos_origin_ecef_;
    Eigen::Vector3d ba     = kf.ba;
    Eigen::Vector3d bg     = kf.bg;
    nav_sol_.SetNavSol(time, origin, pos, vel, quat, ba, bg);
    Eigen::Vector3d acc_raw  = acc;
    Eigen::Vector3d gyro_raw = gyro;
    nav_sol_.SetImu(acc_raw, gyro_raw);
}

// ---------------------------------------------------------------------------
// ECEF → NED
// ---------------------------------------------------------------------------

void InsGnssFg::Ecef2Ned(const Eigen::Vector3d& pos_ecef,
                          const Eigen::Vector3d& vel_ecef,
                          Eigen::Vector3d& pos_ned,
                          Eigen::Vector3d& vel_ned) const {
    pos_ned = Ce2n_ * (pos_ecef - pos_origin_ecef_);
    vel_ned = Ce2n_ * vel_ecef;
}

// ---------------------------------------------------------------------------
// Factor graph optimization (one Gauss-Newton step)
// ---------------------------------------------------------------------------

void InsGnssFg::Optimize() {
    int N = static_cast<int>(window_.size());
    if (N < 2) return;

    // State layout per keyframe: [δp(3), δv(3), δφ(3), δba(3), δbg(3)] = 15
    const int kStateDim = 15;
    int total_dim = kStateDim * N;

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(total_dim, total_dim);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(total_dim);

    // Information weights
    double w_gnss_pos  = 1.0 / (sigma_gnss_pos_ * sigma_gnss_pos_);
    double w_gnss_vel  = 1.0 / (sigma_gnss_vel_ * sigma_gnss_vel_);
    double w_ba_walk   = 1.0 / (sigma_ba_walk_  * sigma_ba_walk_);
    double w_bg_walk   = 1.0 / (sigma_bg_walk_  * sigma_bg_walk_);

    // Strong prior on the oldest keyframe (anchor)
    {
        double w_prior = 1e6;
        H.block(0, 0, kStateDim, kStateDim) +=
            w_prior * Eigen::MatrixXd::Identity(kStateDim, kStateDim);
        // residual = 0 → b contribution = 0
    }

    // ---- IMU pre-integration factors ----
    for (int i = 0; i < N - 1; i++) {
        int j = i + 1;
        const Keyframe& kfi = window_[i];
        const Keyframe& kfj = window_[j];
        const ImuPreintegration& pi = *kfj.preint;

        const Eigen::Matrix3d Ri = kfi.quat.toRotationMatrix();
        const Eigen::Matrix3d Rj = kfj.quat.toRotationMatrix();
        double T = pi.dt_sum;

        // Bias corrections at linearization point
        Eigen::Vector3d dba = kfi.ba - pi.linearized_ba;
        Eigen::Vector3d dbg = kfi.bg - pi.linearized_bg;

        Eigen::Vector3d dp_meas = pi.delta_p + pi.dp_dba * dba + pi.dp_dbg * dbg;
        Eigen::Vector3d dv_meas = pi.delta_v + pi.dv_dba * dba + pi.dv_dbg * dbg;
        Eigen::Quaterniond dq_meas = (pi.delta_q * ExpSO3(pi.dq_dbg * dbg)).normalized();

        // Residuals
        Eigen::Vector3d r_p = Ri.transpose() * (kfj.pos - kfi.pos - kfi.vel * T
                                                  - 0.5 * g_ned_ * T * T) - dp_meas;
        Eigen::Vector3d r_v = Ri.transpose() * (kfj.vel - kfi.vel - g_ned_ * T) - dv_meas;
        Eigen::Quaterniond dq_err = dq_meas.inverse() * kfi.quat.inverse() * kfj.quat;
        Eigen::Vector3d r_R = LogSO3(dq_err);

        // Information matrix from pre-integration covariance
        Eigen::Matrix<double, 9, 9> info_imu;
        // Regularize for numerical safety
        Eigen::Matrix<double, 9, 9> cov_reg = pi.cov;
        cov_reg.diagonal() = cov_reg.diagonal().cwiseMax(1e-8);
        info_imu = cov_reg.inverse();

        // Residual vector [r_p; r_v; r_R]
        Eigen::Matrix<double, 9, 1> r_imu;
        r_imu.segment<3>(0) = r_p;
        r_imu.segment<3>(3) = r_v;
        r_imu.segment<3>(6) = r_R;

        // Jacobians: J_i (9×15), J_j (9×15)
        Eigen::MatrixXd Ji = Eigen::MatrixXd::Zero(9, kStateDim);
        Eigen::MatrixXd Jj = Eigen::MatrixXd::Zero(9, kStateDim);

        // Relative position between keyframes (in world frame)
        Eigen::Vector3d dp_world = kfj.pos - kfi.pos - kfi.vel * T - 0.5 * g_ned_ * T * T;
        Eigen::Vector3d dv_world = kfj.vel - kfi.vel - g_ned_ * T;

        // d(r_p)/d(δp_i) = -Ri^T
        Ji.block<3, 3>(0, 0) = -Ri.transpose();
        // d(r_p)/d(δv_i) = -Ri^T * T
        Ji.block<3, 3>(0, 3) = -Ri.transpose() * T;
        // d(r_p)/d(δφ_i) = Skew(Ri^T * dp_world)  [right perturbation]
        Ji.block<3, 3>(0, 6) = Skew(Ri.transpose() * dp_world);
        // d(r_p)/d(δba_i) = -dp_dba
        Ji.block<3, 3>(0, 9)  = -pi.dp_dba;
        // d(r_p)/d(δbg_i) = -dp_dbg
        Ji.block<3, 3>(0, 12) = -pi.dp_dbg;

        // d(r_v)/d(δv_i) = -Ri^T
        Ji.block<3, 3>(3, 3) = -Ri.transpose();
        // d(r_v)/d(δφ_i) = Skew(Ri^T * dv_world)
        Ji.block<3, 3>(3, 6) = Skew(Ri.transpose() * dv_world);
        // d(r_v)/d(δba_i) = -dv_dba
        Ji.block<3, 3>(3, 9)  = -pi.dv_dba;
        // d(r_v)/d(δbg_i) = -dv_dbg
        Ji.block<3, 3>(3, 12) = -pi.dv_dbg;

        // d(r_R)/d(δφ_i) ≈ -(dq_meas^{-1} * Rj^T * Ri) as matrix
        Ji.block<3, 3>(6, 6)  = -(dq_meas.inverse() * kfj.quat.inverse() * kfi.quat).toRotationMatrix();
        // d(r_R)/d(δbg_i) ≈ -dq_dbg
        Ji.block<3, 3>(6, 12) = -pi.dq_dbg;

        // d(r_p)/d(δp_j) = Ri^T
        Jj.block<3, 3>(0, 0) = Ri.transpose();
        // d(r_v)/d(δv_j) = Ri^T
        Jj.block<3, 3>(3, 3) = Ri.transpose();
        // d(r_R)/d(δφ_j) ≈ I
        Jj.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();

        // Accumulate into H and b
        int oi = i * kStateDim;
        int oj = j * kStateDim;

        H.block(oi, oi, kStateDim, kStateDim) += Ji.transpose() * info_imu * Ji;
        H.block(oi, oj, kStateDim, kStateDim) += Ji.transpose() * info_imu * Jj;
        H.block(oj, oi, kStateDim, kStateDim) += Jj.transpose() * info_imu * Ji;
        H.block(oj, oj, kStateDim, kStateDim) += Jj.transpose() * info_imu * Jj;

        b.segment(oi, kStateDim) -= Ji.transpose() * info_imu * r_imu;
        b.segment(oj, kStateDim) -= Jj.transpose() * info_imu * r_imu;
    }

    // ---- GNSS position / velocity factors ----
    for (int i = 0; i < N; i++) {
        const Keyframe& kf = window_[i];
        if (!kf.has_gnss) continue;

        int oi = i * kStateDim;

        // Position residual
        Eigen::Vector3d r_pos = kf.pos - kf.gnss_pos;
        H.block<3, 3>(oi, oi)         += w_gnss_pos * Eigen::Matrix3d::Identity();
        b.segment<3>(oi)              -= w_gnss_pos * r_pos;

        // Velocity residual
        Eigen::Vector3d r_vel = kf.vel - kf.gnss_vel;
        H.block<3, 3>(oi + 3, oi + 3) += w_gnss_vel * Eigen::Matrix3d::Identity();
        b.segment<3>(oi + 3)          -= w_gnss_vel * r_vel;
    }

    // ---- Bias random-walk factors ----
    for (int i = 0; i < N - 1; i++) {
        int j = i + 1;
        const Keyframe& kfi = window_[i];
        const Keyframe& kfj = window_[j];
        int oi = i * kStateDim;
        int oj = j * kStateDim;

        // ba residual
        Eigen::Vector3d r_ba = kfj.ba - kfi.ba;
        H.block<3, 3>(oi + 9,  oi + 9)  += w_ba_walk * Eigen::Matrix3d::Identity();
        H.block<3, 3>(oi + 9,  oj + 9)  -= w_ba_walk * Eigen::Matrix3d::Identity();
        H.block<3, 3>(oj + 9,  oi + 9)  -= w_ba_walk * Eigen::Matrix3d::Identity();
        H.block<3, 3>(oj + 9,  oj + 9)  += w_ba_walk * Eigen::Matrix3d::Identity();
        b.segment<3>(oi + 9)             -= w_ba_walk * (-r_ba);
        b.segment<3>(oj + 9)             -= w_ba_walk * ( r_ba);

        // bg residual
        Eigen::Vector3d r_bg = kfj.bg - kfi.bg;
        H.block<3, 3>(oi + 12, oi + 12) += w_bg_walk * Eigen::Matrix3d::Identity();
        H.block<3, 3>(oi + 12, oj + 12) -= w_bg_walk * Eigen::Matrix3d::Identity();
        H.block<3, 3>(oj + 12, oi + 12) -= w_bg_walk * Eigen::Matrix3d::Identity();
        H.block<3, 3>(oj + 12, oj + 12) += w_bg_walk * Eigen::Matrix3d::Identity();
        b.segment<3>(oi + 12)            -= w_bg_walk * (-r_bg);
        b.segment<3>(oj + 12)            -= w_bg_walk * ( r_bg);
    }

    // ---- Solve: H * δx = b ----
    Eigen::VectorXd dx = H.ldlt().solve(b);

    // ---- Apply state corrections ----
    for (int i = 0; i < N; i++) {
        Keyframe& kf = window_[i];
        int oi = i * kStateDim;

        kf.pos += dx.segment<3>(oi);
        kf.vel += dx.segment<3>(oi + 3);

        Eigen::Vector3d dphi = dx.segment<3>(oi + 6);
        kf.quat = (kf.quat * ExpSO3(dphi)).normalized();  // right perturbation

        kf.ba += dx.segment<3>(oi + 9);
        kf.bg += dx.segment<3>(oi + 12);
    }

    // Re-propagate biases in pre-integrators after bias update
    for (int i = 1; i < N; i++) {
        Keyframe& kf = window_[i];
        if (kf.preint) {
            kf.preint->Repropagate(window_[i - 1].ba, window_[i - 1].bg);
        }
    }
}

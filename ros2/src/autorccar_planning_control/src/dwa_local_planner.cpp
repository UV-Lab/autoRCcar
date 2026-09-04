#include "dwa_local_planner.h"

#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <limits>

namespace {

// Same bicycle-model forward as simulator.cpp's UpdateBicycleModel, but
// evaluated purely in software (no actuator lag term) since this is a
// planning-time rollout, not the real vehicle model.
double GetHeading(const Eigen::Quaterniond& quat) {
    Eigen::Vector3d direction = quat * Eigen::Vector3d::UnitX();
    return std::atan2(direction.y(), direction.x());
}

}  // namespace

namespace autorccar {
namespace planning_control {
namespace dwa_local_planner {

DwaLocalPlanner::DwaLocalPlanner(const Parameters& parameters) : parameters_(parameters) {
    std::cout << "dwa_local_planner parameters:" << std::endl;
    std::cout << "wheelbase: " << parameters_.wheelbase << std::endl;
    std::cout << "max_steering_angle: " << parameters_.max_steering_angle << std::endl;
    std::cout << "max_speed: " << parameters_.max_speed << std::endl;
    std::cout << "min_speed: " << parameters_.min_speed << std::endl;
    std::cout << "max_accel: " << parameters_.max_accel << std::endl;
    std::cout << "max_steering_rate: " << parameters_.max_steering_rate << std::endl;
    std::cout << "speed_samples: " << parameters_.speed_samples << std::endl;
    std::cout << "steering_samples: " << parameters_.steering_samples << std::endl;
    std::cout << "predict_time: " << parameters_.predict_time << std::endl;
    std::cout << "dt: " << parameters_.dt << std::endl;
    std::cout << "look_ahead_distance: " << parameters_.look_ahead_distance << std::endl;
    std::cout << "target_speed: " << parameters_.target_speed << std::endl;
    std::cout << "robot_radius: " << parameters_.robot_radius << std::endl;
    std::cout << "k_obstacle: " << parameters_.k_obstacle << std::endl;
    std::cout << "k_heading: " << parameters_.k_heading << std::endl;
    std::cout << "k_distance: " << parameters_.k_distance << std::endl;
    std::cout << "k_velocity: " << parameters_.k_velocity << std::endl;
    std::cout << "k_smoothness: " << parameters_.k_smoothness << std::endl;
    std::cout << "k_path: " << parameters_.k_path << std::endl;
}

void DwaLocalPlanner::Planning(const std::unique_ptr<CubicSplinePath>& global_path, const State& current_state) {
    if (global_path == nullptr || !global_path->IsPathGenerated()) {
        current_path_ = {};
        return;
    }

    const Point current_pos{current_state.pos.x(), current_state.pos.y()};
    const double current_yaw = GetHeading(current_state.quat);

    // Local goal: a point further along the global path from the closest
    // reference point, similar in spirit to the pure-pursuit look-ahead.
    const Reference path_ref = global_path->ReferencePoint(current_pos);
    const Reference goal_ref = global_path->ReferencePoint(path_ref.distance + parameters_.look_ahead_distance);
    const Point goal_point = goal_ref.point;

    // dynamic window: reachable (v, steering) range from the last commanded values.
    const double v_min = std::max(parameters_.min_speed, last_speed_ - parameters_.max_accel * parameters_.dt);
    const double v_max = std::min(parameters_.max_speed, last_speed_ + parameters_.max_accel * parameters_.dt);
    const double steer_min =
        std::max(-parameters_.max_steering_angle, last_steering_angle_ - parameters_.max_steering_rate * parameters_.dt);
    const double steer_max =
        std::min(parameters_.max_steering_angle, last_steering_angle_ + parameters_.max_steering_rate * parameters_.dt);

    bool found = false;
    double best_cost = std::numeric_limits<double>::lowest();
    DwaPath best_path{};

    for (int i = 0; i < parameters_.speed_samples; i++) {
        const double v = (parameters_.speed_samples <= 1)
                             ? v_max
                             : v_min + (v_max - v_min) * static_cast<double>(i) / (parameters_.speed_samples - 1);

        for (int j = 0; j < parameters_.steering_samples; j++) {
            const double steering =
                (parameters_.steering_samples <= 1)
                    ? 0.0
                    : steer_min +
                          (steer_max - steer_min) * static_cast<double>(j) / (parameters_.steering_samples - 1);

            std::vector<double> yaws;
            Path trajectory = SimulateTrajectory(v, steering, current_pos, current_yaw, &yaws);
            if (trajectory.size() < 2) continue;
            if ((trajectory.back() - trajectory.front()).norm() < 1e-6) continue;  // v≈0 퇴화 궤적

            const double obstacle_cost = CalcObstacleCost(trajectory);
            if (obstacle_cost < 0.0) continue;  // trajectory collides, discard

            const double heading_cost = CalcHeadingCost(trajectory.back(), yaws.back(), goal_point);
            const double distance_cost = CalcDistanceCost(trajectory.back(), goal_point);
            const double velocity_cost =
                std::fabs(parameters_.target_speed - v) / std::max(parameters_.max_speed, 1e-6);
            const double smoothness_cost =
                std::fabs(steering - last_steering_angle_) / std::max(parameters_.max_steering_angle, 1e-6);
            const double path_cost = CalcPathFollowingCost(trajectory, *global_path);

            const double cost = -(parameters_.k_obstacle * obstacle_cost + parameters_.k_heading * heading_cost +
                                  parameters_.k_distance * distance_cost + parameters_.k_velocity * velocity_cost +
                                  parameters_.k_smoothness * smoothness_cost + parameters_.k_path * path_cost);

            if (cost > best_cost) {
                best_cost = cost;
                best_path.path = trajectory;
                best_path.yaw = yaws;
                best_path.target_speed = v;
                best_path.steering_angle = steering;
                best_path.cost = cost;
                found = true;
            }
        }
    }

    if (!found) {
        std::cout << "dwa_local_planner: no collision-free trajectory found." << std::endl;
        current_path_ = {};
        return;
    }

    last_speed_ = best_path.target_speed;
    last_steering_angle_ = best_path.steering_angle;
    current_path_ = best_path;
}

Path DwaLocalPlanner::SimulateTrajectory(double v, double steering, const Point& start_pos, double start_yaw,
                                         std::vector<double>* yaw_out) const {
    Path trajectory;
    std::vector<double> yaws;

    const int steps = std::max(1, static_cast<int>(parameters_.predict_time / parameters_.dt));
    trajectory.reserve(steps);
    yaws.reserve(steps);

    double x = start_pos.x();
    double y = start_pos.y();
    double yaw = start_yaw;

    for (int k = 0; k < steps; k++) {
        x += v * std::cos(yaw) * parameters_.dt;
        y += v * std::sin(yaw) * parameters_.dt;
        yaw += (v / parameters_.wheelbase) * std::tan(steering) * parameters_.dt;
        trajectory.emplace_back(x, y);
        yaws.push_back(yaw);
    }

    if (yaw_out != nullptr) *yaw_out = std::move(yaws);
    return trajectory;
}

double DwaLocalPlanner::CalcObstacleCost(const Path& path) const {
    if (bounding_boxes_.empty()) return 0.0;  // no obstacle -> zero cost, plan proceeds normally

    for (const auto& point : path) {
        for (const auto& bbox : bounding_boxes_) {
            const double x_min = bbox.x_min - parameters_.robot_radius;
            const double x_max = bbox.x_max + parameters_.robot_radius;
            const double y_min = bbox.y_min - parameters_.robot_radius;
            const double y_max = bbox.y_max + parameters_.robot_radius;

            if (point.x() >= x_min && point.x() <= x_max && point.y() >= y_min && point.y() <= y_max) {
                return -1.0;  // collision -> caller discards this trajectory
            }
        }
    }
    return 0.0;
}

double DwaLocalPlanner::CalcPathFollowingCost(const Path& trajectory, CubicSplinePath& global_path) const {
    if (trajectory.empty()) {
        return std::numeric_limits<double>::max();
    }

    double cost = 0.0;
    for (const auto& point : trajectory) {
        const Reference reference = global_path.ReferencePoint(point);

        cost += (point - reference.point).squaredNorm();
    }
    return cost / static_cast<double>(trajectory.size());
}

double DwaLocalPlanner::CalcHeadingCost(const Point& end_pos, double end_yaw, const Point& goal) const {
    const Point delta = goal - end_pos;
    const double target_yaw = std::atan2(delta.y(), delta.x());
    const double diff = std::atan2(std::sin(target_yaw - end_yaw), std::cos(target_yaw - end_yaw));
    return std::fabs(diff) / M_PI;
}

double DwaLocalPlanner::CalcDistanceCost(const Point& end_pos, const Point& goal) const {
    return (goal - end_pos).norm() / std::max(parameters_.look_ahead_distance, 1e-6);
}

void DwaLocalPlanner::SetBoundingBoxes(std::vector<BoundingBox>&& bounding_boxes) {
    bounding_boxes_ = std::move(bounding_boxes);
}

bool DwaLocalPlanner::IsPathGenerated() {
    return current_path_.path.size() >= 2 && (current_path_.path.back() - current_path_.path.front()).norm() > 1e-6;
}

DwaPath DwaLocalPlanner::GetCurrentPath() const {
    if (current_path_.path.size() < 2) return {};
    return current_path_;
}

}  // namespace dwa_local_planner
}  // namespace planning_control
}  // namespace autorccar

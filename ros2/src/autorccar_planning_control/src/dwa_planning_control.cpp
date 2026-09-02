#include "dwa_planning_control.h"

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <limits>
#include <vector>

namespace {

double GetCurrentHeading(const autorccar::planning_control::dwa_planning_control::State& state) {
    Eigen::Vector3d direction = state.quat * Eigen::Vector3d::UnitX();
    return atan2(direction.y(), direction.x());
}

}  // namespace

namespace autorccar {
namespace planning_control {
namespace dwa_planning_control {

DwaPlanningControl::DwaPlanningControl(const Parameters& parameters)
    : parameters_(parameters),
      look_ahead_distance_squared_(parameters.control.pure_pursuit.look_ahead_distance *
                                   parameters.control.pure_pursuit.look_ahead_distance),
      goal_reach_threshold_squared_(parameters.control.goal_reach_threshold * parameters.control.goal_reach_threshold) {
    std::cout << "dwa_planning_control parameters:" << std::endl;
    std::cout << "wheelbase: " << parameters_.wheelbase << std::endl;
    std::cout << "max_steering_angle: " << parameters_.max_steering_angle << std::endl;
    std::cout << "goal_reach_threshold: " << parameters_.control.goal_reach_threshold << std::endl;
    std::cout << "target_speed: " << parameters_.target_speed << std::endl;
    std::cout << "accel: " << parameters_.control.accel << std::endl;
    std::cout << "decel: " << parameters_.control.decel << std::endl;
    std::cout << "control_dt: " << parameters_.control.control_dt << std::endl;
    std::cout << "look_ahead_distance: " << parameters_.control.pure_pursuit.look_ahead_distance << std::endl;
    std::cout << "min_look_ahead_distance: " << parameters_.control.pure_pursuit.min_look_ahead_distance << std::endl;

    // propagate shared vehicle/target-speed parameters into the DWA planner
    // so they only need to be configured once (rccar_config / controller.target_speed).
    parameters_.dwa.wheelbase = parameters_.wheelbase;
    parameters_.dwa.max_steering_angle = parameters_.max_steering_angle;
    parameters_.dwa.target_speed = parameters_.target_speed;
    dwa_local_planner_ = std::make_unique<DwaLocalPlanner>(parameters_.dwa);
}

void DwaPlanningControl::SetGlobalPath(std::vector<Point>&& global_path, std::vector<double>&& /*speeds*/) {
    goal_ = global_path.back();
    global_path_.reset();
    got_global_path_ = false;
    global_path_ = std::make_unique<CubicSplinePath>(std::move(global_path));
    got_global_path_ = global_path_->IsPathGenerated();
}

void DwaPlanningControl::SetCurrentTargetSpeed(const double speed) { current_target_speed_ = speed; }

void DwaPlanningControl::SetCurrentState(const State& state) { current_state_ = state; }

void DwaPlanningControl::SetBoundingBoxes(std::vector<BoundingBox>&& bounding_boxes) {
    dwa_local_planner_->SetBoundingBoxes(std::move(bounding_boxes));
}

void DwaPlanningControl::PlanOnce() {
    if (!got_global_path_) return;
    dwa_local_planner_->Planning(global_path_, current_state_);
}

bool DwaPlanningControl::GoalReached(const State& state) const {
    Point current_pos(state.pos.x(), state.pos.y());
    return (current_pos - goal_).squaredNorm() < goal_reach_threshold_squared_;
}

ControlCommand DwaPlanningControl::GenerateMotionCommand() {
    if (!got_global_path_) {
        std::cout << "Have not received global path yet." << std::endl;
        return {0.0, 0.0};
    }

    if (GoalReached(current_state_)) {
        std::cout << "Final Goal Reached!!" << std::endl;
        return {0.0, 0.0};
    }

    current_dwa_path_ = dwa_local_planner_->GetCurrentPath();
    if (!dwa_local_planner_->IsPathGenerated()) {
        std::cout << "dwa local path is not generated." << std::endl;
        return {0.0, 0.0};
    }
    Path local_trajectory = current_dwa_path_.path;
    local_path_.reset();
    local_path_ = std::make_unique<CubicSplinePath>(std::move(local_trajectory));

    if (!local_path_->IsPathGenerated()) {
        std::cout << "local path is not generated." << std::endl;
        return {0.0, 0.0};
    }

    double target_speed = current_dwa_path_.target_speed;
    double speed = CalcSpeedCommand(current_state_, target_speed);

    if (speed == 0.0) return {0.0, 0.0};

    auto [is_steering_valid, steering_angle] = CalculateSteeringCommand(current_state_);

    if (!is_steering_valid) {
        std::cout << "Steering command is invalid." << std::endl;
        return {0.0, 0.0};
    }

    return {speed, steering_angle};
}

Point DwaPlanningControl::GetLookAheadPoint() { return look_ahead_point_; }

Path DwaPlanningControl::GetCurrentLocalPath() { return current_dwa_path_.path; }

double DwaPlanningControl::CalcSpeedCommand(const State& state, const double target_speed) {
    Point position(state.pos.x(), state.pos.y());
    Reference global_reference = global_path_->ReferencePoint(position);
    double remain_distance = std::max(0.0, global_reference.remain_distance);

    double stop_distance = (target_speed * target_speed) / (2 * parameters_.control.decel);

    if (remain_distance < stop_distance) {
        current_target_speed_ = sqrt(2 * parameters_.control.decel * remain_distance);
    } else {
        if (current_target_speed_ < target_speed) {
            current_target_speed_ += parameters_.control.accel * parameters_.control.control_dt;
            if (current_target_speed_ > target_speed) {
                current_target_speed_ = target_speed;
            }
        } else if (current_target_speed_ > target_speed) {
            current_target_speed_ -= parameters_.control.decel * parameters_.control.control_dt;
            if (current_target_speed_ < 0.0) {
                current_target_speed_ = 0.0;
            }
        }
    }

    return current_target_speed_;
}

std::pair<bool, double> DwaPlanningControl::CalculateSteeringCommand(const State& state) {
    if (!FindLookAheadPoint(state)) {
        std::cout << "Cannot find look ahead point.";
        return {false, 0.0};
    }
    double heading_error = CalcHeadingError(state);
    double steering_angle = CalcSteeringAngle(heading_error);
    return {true, steering_angle};
}

bool DwaPlanningControl::FindLookAheadPoint(const State& state) {
    Point position(state.pos.x(), state.pos.y());
    Reference reference = local_path_->ReferencePoint(position);

    Point ahead_point = reference.point + Point(cos(reference.heading), sin(reference.heading)) *
                                              parameters_.control.pure_pursuit.look_ahead_distance;
    Reference reference_ahead_point = local_path_->ReferencePoint(ahead_point);
    look_ahead_point_ = reference_ahead_point.point;
    return true;
}

double DwaPlanningControl::CalcHeadingError(const State& state) const {
    double heading = GetCurrentHeading(state);
    Point target_vec = look_ahead_point_ - Point(state.pos.x(), state.pos.y());
    return atan2(target_vec.y(), target_vec.x()) - heading;
}

double DwaPlanningControl::CalcSteeringAngle(double heading_error) const {
    double steering_angle =
        atan2(2.0 * parameters_.wheelbase * sin(heading_error), parameters_.control.pure_pursuit.look_ahead_distance);
    return std::clamp(steering_angle, -parameters_.max_steering_angle, parameters_.max_steering_angle);
}

}  // namespace dwa_planning_control
}  // namespace planning_control
}  // namespace autorccar

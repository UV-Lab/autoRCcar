#ifndef AUTORCCAR_PLANNING_CONTROL_DWA_PLANNING_CONTROL_H_
#define AUTORCCAR_PLANNING_CONTROL_DWA_PLANNING_CONTROL_H_

#include <eigen3/Eigen/Dense>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common.h"
#include "cubic_spline_path.h"
#include "dwa_local_planner.h"

namespace autorccar {
namespace planning_control {
namespace dwa_planning_control {

// Aliases for convenience.
using Point = Eigen::Vector2d;
using common::BoundingBox;
using common::ControlCommand;
using common::State;
using dwa_local_planner::DwaLocalPlanner;
using dwa_local_planner::DwaPath;

struct PurePursuitParameters {
    double min_look_ahead_distance = 0.3;
    double look_ahead_distance = 0.3;
};

struct ControlParameters {
    double goal_reach_threshold = 0.3;
    double accel = 0.0;
    double decel = 0.0;
    double control_dt = 0.0;
    PurePursuitParameters pure_pursuit;
};

struct Parameters {
    double wheelbase = 0.0;
    double max_steering_angle = 0.0;
    double target_speed = 0.0;
    ControlParameters control;
    dwa_local_planner::Parameters dwa;
};

// Mirrors PlanningControl (Frenet version): DwaLocalPlanner is only
// responsible for producing an obstacle-aware local path; speed/steering
// command generation reuses the same pure-pursuit + accel/decel-limited
// speed profile logic as the Frenet pipeline.
class DwaPlanningControl {
   public:
    DwaPlanningControl(const DwaPlanningControl&) = delete;
    DwaPlanningControl& operator=(const DwaPlanningControl&) = delete;
    DwaPlanningControl(DwaPlanningControl&&) = delete;
    DwaPlanningControl& operator=(DwaPlanningControl&&) = delete;

    explicit DwaPlanningControl(const Parameters& parameters);

    void SetGlobalPath(std::vector<Point>&& global_path, std::vector<double>&& speeds);
    void SetCurrentTargetSpeed(const double speed);
    void SetCurrentState(const State& state);
    void SetBoundingBoxes(std::vector<BoundingBox>&& bounding_boxes);
    void PlanOnce();
    ControlCommand GenerateMotionCommand();
    Point GetLookAheadPoint();
    Path GetCurrentLocalPath();

   private:
    double CalcSpeedCommand(const State& state, double target_speed);
    std::pair<bool, double> CalculateSteeringCommand(const State& state);
    double CalcHeadingError(const State& state) const;
    double CalcSteeringAngle(double deviation_angle) const;
    bool FindLookAheadPoint(const State& state);
    bool GoalReached(const State& state) const;

    Parameters parameters_;
    double look_ahead_distance_squared_ = 0.0;
    double goal_reach_threshold_squared_ = 0.0;
    double current_target_speed_ = 0.0;
    Point goal_;
    Point look_ahead_point_{0.0, 0.0};
    bool got_global_path_ = false;
    State current_state_;
    std::unique_ptr<CubicSplinePath> global_path_;
    std::unique_ptr<CubicSplinePath> local_path_;
    std::unique_ptr<DwaLocalPlanner> dwa_local_planner_;
    DwaPath current_dwa_path_{};
};

}  // namespace dwa_planning_control
}  // namespace planning_control
}  // namespace autorccar

#endif  // AUTORCCAR_PLANNING_CONTROL_DWA_PLANNING_CONTROL_H_

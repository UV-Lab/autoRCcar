#ifndef AUTORCCAR_PLANNING_CONTROL_DWA_LOCAL_PLANNER_H_
#define AUTORCCAR_PLANNING_CONTROL_DWA_LOCAL_PLANNER_H_

#include <eigen3/Eigen/Dense>
#include <memory>
#include <utility>
#include <vector>

#include "common.h"
#include "cubic_spline_path.h"

namespace autorccar {
namespace planning_control {
namespace dwa_local_planner {

// aliases for convenience.
using Path = std::vector<Point>;
using Point = Eigen::Vector2d;
using common::BoundingBox;
using common::State;

// Best trajectory selected in the current planning cycle.
// Plays the same role as FrenetPath: PlanningControl-side code turns
// `path` into a CubicSplinePath and reuses the existing pure-pursuit
// speed/steering logic, so `target_speed` / `steering_angle` here are
// only the DWA-side open-loop choice (used as the initial/target values).
struct DwaPath {
    Path path;
    std::vector<double> yaw;
    double target_speed = 0.0;
    double steering_angle = 0.0;
    double cost = 0.0;
};

struct Parameters {
    // vehicle model (filled in from Parameters::wheelbase / max_steering_angle by the caller)
    double wheelbase = 0.0;
    double max_steering_angle = 0.0;

    // dynamic window
    double max_speed = 0.0;
    double min_speed = 0.0;
    double max_accel = 0.0;
    double max_steering_rate = 0.0;  // [rad/s]

    // sampling
    int speed_samples = 7;
    int steering_samples = 15;
    double predict_time = 2.5;  // [s] forward simulation horizon
    double dt = 0.1;            // [s] simulation step

    // goal selection along the global path
    double look_ahead_distance = 2.0;  // [m]
    double target_speed = 0.0;         // filled in from Parameters::target_speed by the caller

    // safety
    double robot_radius = 0.3;  // [m] collision margin added around bounding boxes

    // cost weights
    double k_obstacle = 0.0;
    double k_heading = 0.0;
    double k_distance = 0.0;
    double k_velocity = 0.0;
    double k_smoothness = 0.0;
};

class DwaLocalPlanner {
   public:
    DwaLocalPlanner(const DwaLocalPlanner&) = delete;
    DwaLocalPlanner& operator=(const DwaLocalPlanner&) = delete;
    DwaLocalPlanner(DwaLocalPlanner&&) = delete;
    DwaLocalPlanner& operator=(DwaLocalPlanner&&) = delete;

    explicit DwaLocalPlanner(const Parameters& parameters);

    // Runs regardless of whether any bounding box currently exists.
    void Planning(const std::unique_ptr<CubicSplinePath>& global_path, const State& current_state);
    void SetBoundingBoxes(std::vector<BoundingBox>&& bounding_boxes);
    bool IsPathGenerated();
    DwaPath GetCurrentPath() const;

   private:
    Path SimulateTrajectory(double v, double steering, const Point& start_pos, double start_yaw,
                            std::vector<double>* yaw_out) const;
    double CalcObstacleCost(const Path& path) const;
    double CalcHeadingCost(const Point& end_pos, double end_yaw, const Point& goal) const;
    double CalcDistanceCost(const Point& end_pos, const Point& goal) const;

    Parameters parameters_;
    std::vector<BoundingBox> bounding_boxes_;
    DwaPath current_path_{};
    double last_speed_ = 0.0;
    double last_steering_angle_ = 0.0;
};

}  // namespace dwa_local_planner
}  // namespace planning_control
}  // namespace autorccar

#endif  // AUTORCCAR_PLANNING_CONTROL_DWA_LOCAL_PLANNER_H_

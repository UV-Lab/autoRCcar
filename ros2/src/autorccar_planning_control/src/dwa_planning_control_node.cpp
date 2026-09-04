#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <utility>
#include <vector>

#include "autorccar_interfaces/msg/bounding_boxes.hpp"
#include "autorccar_interfaces/msg/control_command.hpp"
#include "autorccar_interfaces/msg/nav_state.hpp"
#include "autorccar_interfaces/msg/path.hpp"
#include "common.h"
#include "dwa_planning_control.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace {

using Point = Eigen::Vector2d;

}  // namespace

// NOTE: this node publishes to the same topic names as planning_control_node
// (planning_control/control_command, rviz/*) so it can act as a drop-in
// replacement for the Frenet planner. Do not run both nodes at the same time
// unless you remap topics (e.g. via launch-file `remappings=`) to compare them.
class DwaPlanningControlNode : public rclcpp::Node {
   public:
    explicit DwaPlanningControlNode(const rclcpp::NodeOptions& options) : Node("dwa_planning_control", options) {
        // read parameters
        ReadParameters();

        // initialize controller
        dwa_planning_controller_ =
            std::make_unique<autorccar::planning_control::dwa_planning_control::DwaPlanningControl>(parameters_);

        // publisher
        control_command_publisher_ = create_publisher<autorccar_interfaces::msg::ControlCommand>(
            "planning_control/control_command",
            rclcpp::SystemDefaultsQoS().deadline(std::chrono::milliseconds(control_command_deadline_)));
        global_path_marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "rviz/global_path_marker", rclcpp::SystemDefaultsQoS());
        current_pos_marker_publisher_ =
            create_publisher<visualization_msgs::msg::Marker>("rviz/current_pos_marker", rclcpp::SystemDefaultsQoS());
        look_ahead_point_marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
            "rviz/look_ahead_point_marker", rclcpp::SystemDefaultsQoS());
        current_heading_odometry_publisher_ =
            create_publisher<nav_msgs::msg::Odometry>("rviz/current_heading", rclcpp::SystemDefaultsQoS());
        local_path_marker_publisher_ =
            create_publisher<nav_msgs::msg::Path>("rviz/local_path", rclcpp::SystemDefaultsQoS());

        // subscriber
        auto nav_state_callback = [this](autorccar_interfaces::msg::NavState::UniquePtr msg) {
            this->NavStateCallback(*msg);
        };
        nav_state_subscriber_ =
            create_subscription<autorccar_interfaces::msg::NavState>("nav_topic", 10, nav_state_callback);

        auto bounding_boxes_callback = [this](autorccar_interfaces::msg::BoundingBoxes::UniquePtr msg) {
            this->BoundingBoxesCallback(*msg);
        };
        bounding_boxes_subscriber_ = create_subscription<autorccar_interfaces::msg::BoundingBoxes>(
            "bounding_boxes", 10, bounding_boxes_callback);

        auto global_path_callback = [this](autorccar_interfaces::msg::Path::UniquePtr msg) {
            this->GlobalPathCallback(*msg);
        };
        global_path_subscriber_ = create_subscription<autorccar_interfaces::msg::Path>(
            "gcs/global_path", rclcpp::SystemDefaultsQoS(), global_path_callback);
    }

   private:
    void ReadParameters() {
        get_parameter_or<double>("rccar_config.wheelbase", parameters_.wheelbase, parameters_.wheelbase);
        get_parameter_or<double>("rccar_config.max_steering_angle", parameters_.max_steering_angle,
                                 parameters_.max_steering_angle);
        get_parameter_or<int>("hardware_control.control_command_deadline", control_command_deadline_,
                              control_command_deadline_);
        get_parameter_or<double>("controller.goal_reach_threshold", parameters_.control.goal_reach_threshold,
                                 parameters_.control.goal_reach_threshold);
        get_parameter_or<double>("controller.target_speed", parameters_.target_speed, parameters_.target_speed);
        get_parameter_or<int>("controller.nav_hz", nav_hz_, nav_hz_);
        get_parameter_or<int>("controller.control_hz", control_hz_, control_hz_);
        nav_sampling_period_ = nav_hz_ / control_hz_;
        parameters_.control.control_dt = 1.0 / control_hz_;
        get_parameter_or<double>("controller.accel", parameters_.control.accel, parameters_.control.accel);
        get_parameter_or<double>("controller.decel", parameters_.control.decel, parameters_.control.decel);
        get_parameter_or<double>("controller.pure_pursuiter.look_ahead_distance",
                                 parameters_.control.pure_pursuit.look_ahead_distance,
                                 parameters_.control.pure_pursuit.look_ahead_distance);

        get_parameter_or<double>("dwa_optimal.max_speed", parameters_.dwa.max_speed, parameters_.dwa.max_speed);
        get_parameter_or<double>("dwa_optimal.min_speed", parameters_.dwa.min_speed, parameters_.dwa.min_speed);
        get_parameter_or<double>("dwa_optimal.max_accel", parameters_.dwa.max_accel, parameters_.dwa.max_accel);
        get_parameter_or<double>("dwa_optimal.max_steering_rate", parameters_.dwa.max_steering_rate,
                                 parameters_.dwa.max_steering_rate);
        get_parameter_or<int>("dwa_optimal.speed_samples", parameters_.dwa.speed_samples,
                              parameters_.dwa.speed_samples);
        get_parameter_or<int>("dwa_optimal.steering_samples", parameters_.dwa.steering_samples,
                              parameters_.dwa.steering_samples);
        get_parameter_or<double>("dwa_optimal.predict_time", parameters_.dwa.predict_time,
                                 parameters_.dwa.predict_time);
        get_parameter_or<double>("dwa_optimal.dt", parameters_.dwa.dt, parameters_.dwa.dt);
        get_parameter_or<double>("dwa_optimal.look_ahead_distance", parameters_.dwa.look_ahead_distance,
                                 parameters_.dwa.look_ahead_distance);
        get_parameter_or<double>("dwa_optimal.robot_radius", parameters_.dwa.robot_radius,
                                 parameters_.dwa.robot_radius);
        get_parameter_or<double>("dwa_optimal.k_obstacle", parameters_.dwa.k_obstacle, parameters_.dwa.k_obstacle);
        get_parameter_or<double>("dwa_optimal.k_heading", parameters_.dwa.k_heading, parameters_.dwa.k_heading);
        get_parameter_or<double>("dwa_optimal.k_distance", parameters_.dwa.k_distance, parameters_.dwa.k_distance);
        get_parameter_or<double>("dwa_optimal.k_velocity", parameters_.dwa.k_velocity, parameters_.dwa.k_velocity);
        get_parameter_or<double>("dwa_optimal.k_smoothness", parameters_.dwa.k_smoothness,
                                 parameters_.dwa.k_smoothness);
        get_parameter_or<double>("dwa_optimal.k_path", parameters_.dwa.k_path, parameters_.dwa.k_path);
    }

    void NavStateCallback(const autorccar_interfaces::msg::NavState& msg) {
        if (++nav_sample_count_ > nav_sampling_period_) return;
        nav_sample_count_ = 0;

        VisualizeNavState(msg);
        GenerateControlCommand(msg);
        VisualizeLocalPath(dwa_planning_controller_->GetCurrentLocalPath());
    }

    void BoundingBoxesCallback(const autorccar_interfaces::msg::BoundingBoxes& msg) {
        std::vector<autorccar::planning_control::common::BoundingBox> bboxes;
        bboxes.reserve(msg.bounding_boxes.size());
        for (const auto& bbox : msg.bounding_boxes) {
            autorccar::planning_control::common::BoundingBox bounding_box;
            bounding_box.x_min = bbox.center.position.x - (bbox.size_x / 2.0);
            bounding_box.x_max = bbox.center.position.x + (bbox.size_x / 2.0);
            bounding_box.y_min = bbox.center.position.y - (bbox.size_y / 2.0);
            bounding_box.y_max = bbox.center.position.y + (bbox.size_y / 2.0);
            bboxes.push_back(bounding_box);
        }
        dwa_planning_controller_->SetBoundingBoxes(std::move(bboxes));
        dwa_planning_controller_->PlanOnce();
    }

    void GenerateControlCommand(const autorccar_interfaces::msg::NavState& msg) const {
        autorccar::planning_control::common::State state;

        state.timestamp = msg.timestamp.sec + msg.timestamp.nanosec * 1e-9;
        state.pos << msg.position.x, msg.position.y, msg.position.z;
        state.vel << msg.velocity.x, msg.velocity.y, msg.velocity.z;
        state.quat.w() = msg.quaternion.w;
        state.quat.x() = msg.quaternion.x;
        state.quat.y() = msg.quaternion.y;
        state.quat.z() = msg.quaternion.z;
        state.accel << msg.acceleration.x, msg.acceleration.y, msg.acceleration.z;
        state.ang_vel << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;
        dwa_planning_controller_->SetCurrentState(state);

        autorccar::planning_control::dwa_planning_control::ControlCommand control_command;
        control_command = dwa_planning_controller_->GenerateMotionCommand();

        autorccar_interfaces::msg::ControlCommand control_command_msg;
        control_command_msg.speed = control_command.speed;
        control_command_msg.steering_angle = control_command.steering_angle;

        control_command_publisher_->publish(control_command_msg);
    }

    void GlobalPathCallback(const autorccar_interfaces::msg::Path& msg) const {
        std::vector<Point> global_path;
        std::vector<double> speeds;
        for (const auto& path_point : msg.path_points) {
            Point point;
            double speed;
            point.x() = path_point.x;
            point.y() = path_point.y;
            speed = path_point.speed;
            global_path.push_back(point);
            speeds.push_back(speed);
        }
        VisualizeGlobalPath(global_path);
        dwa_planning_controller_->SetGlobalPath(std::move(global_path), std::move(speeds));
    }

    void VisualizeNavState(const autorccar_interfaces::msg::NavState& msg) const {
        // visualize current position
        visualization_msgs::msg::Marker current_pos_msg;
        current_pos_msg.header.frame_id = "map";
        current_pos_msg.header.stamp = this->now();
        current_pos_msg.action = visualization_msgs::msg::Marker::ADD;
        current_pos_msg.type = visualization_msgs::msg::Marker::SPHERE;
        current_pos_msg.ns = "current_pos";
        current_pos_msg.id = 0;
        current_pos_msg.pose.orientation.w = 1;
        current_pos_msg.scale.x = 0.5;
        current_pos_msg.scale.y = 0.5;
        current_pos_msg.scale.z = 0.5;
        current_pos_msg.color.r = 1.0;
        current_pos_msg.color.g = 1.0;
        current_pos_msg.color.b = 0.0;
        current_pos_msg.color.a = 1.0;

        geometry_msgs::msg::Point point;

        point.x = msg.position.x;
        point.y = msg.position.y;
        point.z = 0.0;

        current_pos_msg.pose.position.x = msg.position.x;
        current_pos_msg.pose.position.y = msg.position.y;
        current_pos_msg.pose.position.z = 0.0;

        current_pos_msg.points.push_back(point);
        current_pos_marker_publisher_->publish(current_pos_msg);

        // visualize look ahead point
        visualization_msgs::msg::Marker look_ahead_point_msg;
        look_ahead_point_msg.header.frame_id = "map";
        look_ahead_point_msg.header.stamp = this->now();
        look_ahead_point_msg.action = visualization_msgs::msg::Marker::ADD;
        look_ahead_point_msg.type = visualization_msgs::msg::Marker::SPHERE;
        look_ahead_point_msg.ns = "current_pos";
        look_ahead_point_msg.id = 0;
        look_ahead_point_msg.pose.orientation.w = 1;
        look_ahead_point_msg.scale.x = 0.5;
        look_ahead_point_msg.scale.y = 0.5;
        look_ahead_point_msg.scale.z = 0.5;
        look_ahead_point_msg.color.r = 0.0;
        look_ahead_point_msg.color.g = 1.0;
        look_ahead_point_msg.color.b = 1.0;
        look_ahead_point_msg.color.a = 1.0;

        Point look_ahead_point = dwa_planning_controller_->GetLookAheadPoint();
        point.x = look_ahead_point.x();
        point.y = look_ahead_point.y();
        point.z = 0.0;

        look_ahead_point_msg.pose.position.x = look_ahead_point.x();
        look_ahead_point_msg.pose.position.y = look_ahead_point.y();
        look_ahead_point_msg.pose.position.z = 0.0;

        look_ahead_point_msg.points.push_back(point);
        look_ahead_point_marker_publisher_->publish(look_ahead_point_msg);

        // visualize current heading
        nav_msgs::msg::Odometry current_heading_msg;
        current_heading_msg.header.frame_id = "map";
        current_heading_msg.header.stamp = this->now();

        current_heading_msg.pose.pose.position.x = msg.position.x;
        current_heading_msg.pose.pose.position.y = msg.position.y;
        current_heading_msg.pose.pose.position.z = 0.0;

        current_heading_msg.pose.pose.orientation.w = msg.quaternion.w;
        current_heading_msg.pose.pose.orientation.x = msg.quaternion.x;
        current_heading_msg.pose.pose.orientation.y = msg.quaternion.y;
        current_heading_msg.pose.pose.orientation.z = msg.quaternion.z;

        current_heading_odometry_publisher_->publish(current_heading_msg);
    }

    void VisualizeGlobalPath(const std::vector<Point>& global_path) const {
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker node;
        node.header.frame_id = "map";
        node.header.stamp = this->now();
        node.action = visualization_msgs::msg::Marker::ADD;
        node.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        node.ns = "global_path_node";
        node.id = 0;
        node.pose.orientation.w = 1;
        node.scale.x = 0.5;
        node.scale.y = 0.5;
        node.scale.z = 0.5;
        node.color.r = 1.0;
        node.color.g = 1.0;
        node.color.b = 1.0;
        node.color.a = 1.0;

        visualization_msgs::msg::Marker edge;
        edge.header.frame_id = "map";
        edge.header.stamp = this->now();
        edge.action = visualization_msgs::msg::Marker::ADD;
        edge.type = visualization_msgs::msg::Marker::LINE_LIST;
        edge.ns = "global_path_edge";
        edge.id = 1;
        edge.pose.orientation.w = 1;
        edge.scale.x = 0.1;
        edge.color.r = 1.0;
        edge.color.g = 1.0;
        edge.color.b = 1.0;
        edge.color.a = 1.0;

        for (auto it = global_path.begin(); it != global_path.end(); it++) {
            geometry_msgs::msg::Point point;
            if ((it + 1) != global_path.end()) {
                point.x = it->x();
                point.y = it->y();
                point.z = 0.0;
                node.points.push_back(point);
                edge.points.push_back(point);

                point.x = (it + 1)->x();
                point.y = (it + 1)->y();
                point.z = 0.0;
                edge.points.push_back(point);
            } else {
                point.x = it->x();
                point.y = it->y();
                point.z = 0.0;
                node.points.push_back(point);
            }
        }

        marker_array.markers.push_back(node);
        marker_array.markers.push_back(edge);

        global_path_marker_publisher_->publish(marker_array);
    }

    void VisualizeLocalPath(const std::vector<Point>& local_path) const {
        nav_msgs::msg::Path path_marker;
        path_marker.header.set__frame_id("map").set__stamp(now());
        for (const auto& point : local_path) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.set__frame_id("map").set__stamp(path_marker.header.stamp);
            pose_stamped.pose.position.set__x(point.x()).set__y(point.y()).set__z(0.0);
            pose_stamped.pose.orientation.set__x(0.0).set__y(0.0).set__z(0.0).set__w(1.0);
            path_marker.poses.push_back(pose_stamped);
        }
        local_path_marker_publisher_->publish(path_marker);
    }

    std::unique_ptr<autorccar::planning_control::dwa_planning_control::DwaPlanningControl> dwa_planning_controller_;
    autorccar::planning_control::dwa_planning_control::Parameters parameters_;

    // publisher
    rclcpp::Publisher<autorccar_interfaces::msg::ControlCommand>::SharedPtr control_command_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_path_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr current_pos_marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr look_ahead_point_marker_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr current_heading_odometry_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_marker_publisher_;

    // subscriber
    rclcpp::Subscription<autorccar_interfaces::msg::NavState>::SharedPtr nav_state_subscriber_;
    rclcpp::Subscription<autorccar_interfaces::msg::BoundingBoxes>::SharedPtr bounding_boxes_subscriber_;
    rclcpp::Subscription<autorccar_interfaces::msg::Path>::SharedPtr global_path_subscriber_;

    // variables
    int nav_hz_{100};
    int control_hz_{50};
    int nav_sample_count_{0};
    int nav_sampling_period_{2};
    int control_command_deadline_{1000};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<DwaPlanningControlNode>(options));
    rclcpp::shutdown();

    return 0;
}

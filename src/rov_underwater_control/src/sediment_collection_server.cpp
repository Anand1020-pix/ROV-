#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rov_interfaces/action/collect_sediment.hpp"
#include "rov_interfaces/srv/remove_sediment.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SedimentCollectionServer final : public rclcpp_lifecycle::LifecycleNode
{
public:
  using Action = rov_interfaces::action::CollectSediment;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  using RemoveSediment = rov_interfaces::srv::RemoveSediment;
  using BoolPub = rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>;
  using Float = std_msgs::msg::Float64;
  using SteadyClock = std::chrono::steady_clock;

  explicit SedimentCollectionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : LifecycleNode("sediment_collection_server", options)
  {
    declare_parameter<double>("sensor_timeout_sec", 0.75);
    declare_parameter<double>("pose_timeout_sec", 0.5);
    declare_parameter<double>("maximum_vacuum_kpa", 55.0);
    declare_parameter<double>("minimum_flow_lpm", 0.1);
    declare_parameter<double>("vacuum_acquisition_timeout_sec", 6.0);
    declare_parameter<double>("maximum_nozzle_error_m", 0.10);
    declare_parameter<double>("maximum_collection_drift_m", 0.20);
  }

  ~SedimentCollectionServer() override
  {
    active_ = false;
    if (worker_.joinable()) {
      worker_.join();
    }
    safe_stop();
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    pump_pub_ = create_publisher<std_msgs::msg::Bool>("/rov/collection/pump_enable", 10);
    valve_pub_ = create_publisher<std_msgs::msg::Bool>("/rov/collection/valve_open", 10);
    wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("/rov/cmd_wrench", 10);
    arm_pub_ = create_publisher<std_msgs::msg::Bool>("/rov/arm", 10);
    collected_pub_ = create_publisher<std_msgs::msg::UInt32>(
      "/rov/collection/target_removed", 10);
    remove_client_ = create_client<RemoveSediment>("/rov/sim/remove_sediment");

    pressure_sub_ = create_subscription<Float>(
      "/rov/sensors/collection_vacuum_kpa", 10,
      [this](Float::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        vacuum_kpa_ = msg->data;
        pressure_time_ = SteadyClock::now();
        have_pressure_ = std::isfinite(msg->data);
      });
    flow_sub_ = create_subscription<Float>(
      "/rov/sensors/collection_flow_lpm", 10,
      [this](Float::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        flow_lpm_ = msg->data;
        flow_time_ = SteadyClock::now();
        have_flow_ = std::isfinite(msg->data);
      });
    leak_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/rov/sensors/leak_detected", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {leak_detected_ = msg->data;});
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/rov/emergency_stop", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {emergency_stop_ = msg->data;});
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/rov/pose", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        pose_ = *msg;
        pose_time_ = SteadyClock::now();
        have_pose_ = true;
      });
    controller_sub_ = create_subscription<std_msgs::msg::String>(
      "/rov/control_status", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        controller_status_ = msg->data;
        controller_status_time_ = SteadyClock::now();
      });

    action_server_ = rclcpp_action::create_server<Action>(
      this, "collect_sediment",
      std::bind(&SedimentCollectionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SedimentCollectionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&SedimentCollectionServer::handle_accepted, this, std::placeholders::_1));
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    pump_pub_->on_activate();
    valve_pub_->on_activate();
    active_ = true;
    safe_stop();
    RCLCPP_INFO(get_logger(), "Target-approach and simulated sediment collection action active");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    safe_stop();
    if (worker_.joinable()) {
      worker_.join();
    }
    pump_pub_->on_deactivate();
    valve_pub_->on_deactivate();
    RCLCPP_WARN(get_logger(), "Collection server deactivated; vehicle disarmed, pump OFF, valve CLOSED");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    action_server_.reset();
    pressure_sub_.reset();
    flow_sub_.reset();
    leak_sub_.reset();
    estop_sub_.reset();
    pose_sub_.reset();
    controller_sub_.reset();
    pump_pub_.reset();
    valve_pub_.reset();
    wrench_pub_.reset();
    arm_pub_.reset();
    collected_pub_.reset();
    remove_client_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  struct Target
  {
    double x;
    double y;
    double radius;
  };

  static constexpr std::array<Target, 8> targets_{{
      {1.6, -1.0, 0.09},
      {1.85, -0.85, 0.12},
      {2.1, -1.15, 0.08},
      {2.35, -0.9, 0.10},
      {2.7, 0.9, 0.11},
      {2.95, 1.15, 0.08},
      {3.2, 0.85, 0.13},
      {3.5, 1.0, 0.09}}};

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal)
  {
    if (!active_ || goal_running_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->target_id < 1 || goal->target_id > targets_.size() ||
        !std::isfinite(goal->target_vacuum_kpa) ||
        !std::isfinite(goal->collection_duration_sec) ||
        !std::isfinite(goal->max_duration_sec) ||
        !std::isfinite(goal->approach_timeout_sec) ||
        goal->target_vacuum_kpa <= 0.0 ||
        goal->target_vacuum_kpa > get_parameter("maximum_vacuum_kpa").as_double() ||
        goal->collection_duration_sec <= 0.0 ||
        goal->max_duration_sec < goal->collection_duration_sec ||
        goal->approach_timeout_sec < 5.0 ||
        goal->approach_timeout_sec > 120.0)
    {
      RCLCPP_WARN(get_logger(), "Rejected invalid target or collection limits");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (leak_detected_ || emergency_stop_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_WARN(get_logger(), "Collection cancellation requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) {
      worker_.join();
    }
    goal_running_ = true;
    worker_ = std::thread(&SedimentCollectionServer::execute, this, goal_handle);
  }

  bool sensors_fresh(double & vacuum, double & flow)
  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    const auto now = SteadyClock::now();
    const double timeout = get_parameter("sensor_timeout_sec").as_double();
    const bool pressure_fresh = have_pressure_ &&
      std::chrono::duration<double>(now - pressure_time_).count() <= timeout;
    const bool flow_fresh = have_flow_ &&
      std::chrono::duration<double>(now - flow_time_).count() <= timeout;
    vacuum = vacuum_kpa_;
    flow = flow_lpm_;
    return pressure_fresh && flow_fresh && std::isfinite(vacuum) && std::isfinite(flow);
  }

  bool pose_fresh(geometry_msgs::msg::PoseStamped & pose)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const double timeout = get_parameter("pose_timeout_sec").as_double();
    if (!have_pose_ ||
        std::chrono::duration<double>(SteadyClock::now() - pose_time_).count() > timeout)
    {
      return false;
    }
    pose = pose_;
    return true;
  }

  bool controller_ready()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool fresh = std::chrono::duration<double>(
      SteadyClock::now() - controller_status_time_).count() <= 1.0;
    return fresh && (controller_status_ == "ACTIVE_DISARMED" ||
      controller_status_ == "ARMED" || controller_status_ == "DISARMED");
  }

  static geometry_msgs::msg::Vector3 rotate_vector(
    const geometry_msgs::msg::Quaternion & q, const geometry_msgs::msg::Vector3 & v)
  {
    const double ux = q.x;
    const double uy = q.y;
    const double uz = q.z;
    const double scalar = q.w;
    const double dot = ux * v.x + uy * v.y + uz * v.z;
    const double uu = ux * ux + uy * uy + uz * uz;
    const double cross_x = uy * v.z - uz * v.y;
    const double cross_y = uz * v.x - ux * v.z;
    const double cross_z = ux * v.y - uy * v.x;
    geometry_msgs::msg::Vector3 result;
    result.x = 2.0 * dot * ux + (scalar * scalar - uu) * v.x + 2.0 * scalar * cross_x;
    result.y = 2.0 * dot * uy + (scalar * scalar - uu) * v.y + 2.0 * scalar * cross_y;
    result.z = 2.0 * dot * uz + (scalar * scalar - uu) * v.z + 2.0 * scalar * cross_z;
    return result;
  }

  static geometry_msgs::msg::Vector3 rotate_inverse(
    const geometry_msgs::msg::Quaternion & q, const geometry_msgs::msg::Vector3 & v)
  {
    auto inverse = q;
    inverse.x = -inverse.x;
    inverse.y = -inverse.y;
    inverse.z = -inverse.z;
    return rotate_vector(inverse, v);
  }

  static geometry_msgs::msg::Vector3 nozzle_offset_world(
    const geometry_msgs::msg::Quaternion & orientation)
  {
    geometry_msgs::msg::Vector3 nozzle_offset;
    nozzle_offset.x = 0.30;
    nozzle_offset.z = -0.385;
    return rotate_vector(orientation, nozzle_offset);
  }

  void set_pump(bool on)
  {
    if (pump_pub_ && pump_pub_->is_activated()) {
      std_msgs::msg::Bool message;
      message.data = on;
      pump_pub_->publish(message);
    }
  }

  void set_valve(bool open)
  {
    if (valve_pub_ && valve_pub_->is_activated()) {
      std_msgs::msg::Bool message;
      message.data = open;
      valve_pub_->publish(message);
    }
  }

  void set_armed(bool armed)
  {
    if (arm_pub_) {
      std_msgs::msg::Bool message;
      message.data = armed;
      arm_pub_->publish(message);
    }
  }

  void publish_motion_command(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Point & nozzle_goal,
    SteadyClock::time_point & last_arm_request,
    const geometry_msgs::msg::Vector3 & nozzle_velocity = geometry_msgs::msg::Vector3())
  {
    const auto offset = nozzle_offset_world(pose.pose.orientation);
    geometry_msgs::msg::Vector3 error_world;
    error_world.x = nozzle_goal.x -
      (pose.pose.position.x + offset.x);
    error_world.y = nozzle_goal.y -
      (pose.pose.position.y + offset.y);
    error_world.z = nozzle_goal.z -
      (pose.pose.position.z + offset.z);
    geometry_msgs::msg::Vector3 force_world;
    force_world.x = 4.0 * error_world.x - 8.0 * nozzle_velocity.x;
    force_world.y = 4.0 * error_world.y - 8.0 * nozzle_velocity.y;
    force_world.z = 8.0 * error_world.z - 10.0 * nozzle_velocity.z;
    const auto force_body = rotate_inverse(pose.pose.orientation, force_world);

    geometry_msgs::msg::WrenchStamped wrench;
    wrench.header.stamp = get_clock()->now();
    wrench.header.frame_id = "base_link";
    wrench.wrench.force.x = std::clamp(force_body.x, -4.0, 4.0);
    wrench.wrench.force.y = std::clamp(force_body.y, -4.0, 4.0);
    wrench.wrench.force.z = std::clamp(force_body.z, -6.0, 6.0);

    wrench_pub_->publish(wrench);

    const auto now = SteadyClock::now();
    if (!controller_is_armed() &&
        std::chrono::duration<double>(now - last_arm_request).count() >= 0.3)
    {
      set_armed(true);
      last_arm_request = now;
    }
  }

  bool controller_is_armed()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return controller_status_ == "ARMED" &&
      std::chrono::duration<double>(SteadyClock::now() - controller_status_time_).count() <= 1.0;
  }

  double nozzle_error(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Point & goal)
  {
    const auto offset = nozzle_offset_world(pose.pose.orientation);
    const double dx = goal.x - (pose.pose.position.x + offset.x);
    const double dy = goal.y - (pose.pose.position.y + offset.y);
    const double dz = goal.z - (pose.pose.position.z + offset.z);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  bool checks_ok(const std::shared_ptr<GoalHandle> & goal_handle, std::string & failure)
  {
    if (!active_) {
      failure = "Action server deactivated";
      return false;
    }
    if (goal_handle->is_canceling()) {
      failure = "Collection cancelled";
      return false;
    }
    if (emergency_stop_) {
      failure = "Vehicle E-stop active";
      return false;
    }
    if (leak_detected_) {
      failure = "Leak detected: emergency collection shutdown";
      return false;
    }
    return true;
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::string & state,
    double elapsed,
    double vacuum,
    double flow)
  {
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->state = state;
    feedback->elapsed_sec = static_cast<float>(elapsed);
    feedback->vacuum_kpa = static_cast<float>(vacuum);
    feedback->flow_lpm = static_cast<float>(flow);
    goal_handle->publish_feedback(feedback);
  }

  bool approach_target(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const Target & target,
    geometry_msgs::msg::Point & nozzle_goal,
    std::string & failure)
  {
    const auto approach_start = SteadyClock::now();
    const double timeout = goal_handle->get_goal()->approach_timeout_sec;
    nozzle_goal.x = target.x;
    nozzle_goal.y = target.y;
    nozzle_goal.z = -2.9 + 2.0 * target.radius + 0.045;
    auto last_arm_request = SteadyClock::time_point{};
    auto previous_nozzle_position = geometry_msgs::msg::Point{};
    auto previous_pose_time = SteadyClock::time_point{};
    bool have_previous_pose = false;
    double error = std::numeric_limits<double>::infinity();
    int settled_ticks = 0;

    while (true) {
      if (!checks_ok(goal_handle, failure)) {
        return false;
      }
      if (std::chrono::duration<double>(SteadyClock::now() - approach_start).count() >= timeout) {
        failure = "Approach timeout: nozzle remained " + std::to_string(error) +
          " m from the target";
        return false;
      }
      if (!controller_ready()) {
        failure = "Thruster controller is inactive or its status is stale";
        return false;
      }

      geometry_msgs::msg::PoseStamped pose;
      if (!pose_fresh(pose)) {
        failure = "Vehicle pose missing or stale during approach";
        return false;
      }

      const auto offset = nozzle_offset_world(pose.pose.orientation);
      geometry_msgs::msg::Point nozzle_position;
      nozzle_position.x = pose.pose.position.x + offset.x;
      nozzle_position.y = pose.pose.position.y + offset.y;
      nozzle_position.z = pose.pose.position.z + offset.z;
      const auto now = SteadyClock::now();
      geometry_msgs::msg::Vector3 nozzle_velocity;
      if (have_previous_pose) {
        const double dt = std::chrono::duration<double>(now - previous_pose_time).count();
        if (dt >= 0.02 && dt <= 0.5) {
          nozzle_velocity.x = (nozzle_position.x - previous_nozzle_position.x) / dt;
          nozzle_velocity.y = (nozzle_position.y - previous_nozzle_position.y) / dt;
          nozzle_velocity.z = (nozzle_position.z - previous_nozzle_position.z) / dt;
        }
      }
      previous_nozzle_position = nozzle_position;
      previous_pose_time = now;
      have_previous_pose = true;
      publish_motion_command(pose, nozzle_goal, last_arm_request, nozzle_velocity);

      error = nozzle_error(pose, nozzle_goal);
      if (error <= get_parameter("maximum_nozzle_error_m").as_double()) {
        ++settled_ticks;
        if (settled_ticks >= 5) {
          return true;
        }
      } else {
        settled_ticks = 0;
      }
      publish_feedback(goal_handle, "APPROACHING", std::chrono::duration<double>(
        SteadyClock::now() - approach_start).count(), 0.0, 0.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  bool move_to_base_position(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const geometry_msgs::msg::Point & base_goal,
    const std::string & state,
    double timeout,
    std::string & failure)
  {
    const auto move_start = SteadyClock::now();
    auto last_arm_request = SteadyClock::time_point{};
    auto previous_position = geometry_msgs::msg::Point{};
    auto previous_pose_time = SteadyClock::time_point{};
    bool have_previous_pose = false;
    double error = std::numeric_limits<double>::infinity();
    int settled_ticks = 0;

    while (true) {
      if (!checks_ok(goal_handle, failure)) {
        return false;
      }
      const double elapsed = std::chrono::duration<double>(
        SteadyClock::now() - move_start).count();
      if (elapsed >= timeout) {
        failure = state + " timeout: vehicle remained " + std::to_string(error) +
          " m from the requested position";
        return false;
      }
      if (!controller_ready()) {
        failure = "Thruster controller is inactive or its status is stale during " + state;
        return false;
      }

      geometry_msgs::msg::PoseStamped pose;
      if (!pose_fresh(pose)) {
        failure = "Vehicle pose missing or stale during " + state;
        return false;
      }

      const auto now = SteadyClock::now();
      geometry_msgs::msg::Vector3 base_velocity;
      if (have_previous_pose) {
        const double dt = std::chrono::duration<double>(now - previous_pose_time).count();
        if (dt >= 0.02 && dt <= 0.5) {
          base_velocity.x = (pose.pose.position.x - previous_position.x) / dt;
          base_velocity.y = (pose.pose.position.y - previous_position.y) / dt;
          base_velocity.z = (pose.pose.position.z - previous_position.z) / dt;
        }
      }
      previous_position = pose.pose.position;
      previous_pose_time = now;
      have_previous_pose = true;

      const auto offset = nozzle_offset_world(pose.pose.orientation);
      geometry_msgs::msg::Point nozzle_goal;
      nozzle_goal.x = base_goal.x + offset.x;
      nozzle_goal.y = base_goal.y + offset.y;
      nozzle_goal.z = base_goal.z + offset.z;
      publish_motion_command(pose, nozzle_goal, last_arm_request, base_velocity);

      const double dx = base_goal.x - pose.pose.position.x;
      const double dy = base_goal.y - pose.pose.position.y;
      const double dz = base_goal.z - pose.pose.position.z;
      error = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (error <= get_parameter("maximum_nozzle_error_m").as_double()) {
        ++settled_ticks;
        if (settled_ticks >= 5) {
          return true;
        }
      } else {
        settled_ticks = 0;
      }
      publish_feedback(goal_handle, state, elapsed, 0.0, 0.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  bool return_to_start_if_safe(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const geometry_msgs::msg::PoseStamped & starting_pose,
    double timeout,
    std::string & failure)
  {
    if (!active_ || goal_handle->is_canceling() || emergency_stop_ || leak_detected_) {
      failure = "Safety interlock prevents automatic return";
      return false;
    }
    if (!controller_ready()) {
      failure = "Controller unavailable; automatic return skipped";
      return false;
    }

    geometry_msgs::msg::PoseStamped current_pose;
    if (!pose_fresh(current_pose)) {
      failure = "Vehicle pose unavailable; automatic return skipped";
      return false;
    }

    const double dx = starting_pose.pose.position.x - current_pose.pose.position.x;
    const double dy = starting_pose.pose.position.y - current_pose.pose.position.y;
    const double dz = starting_pose.pose.position.z - current_pose.pose.position.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (distance <= get_parameter("maximum_nozzle_error_m").as_double()) {
      return true;
    }

    if (current_pose.pose.position.z < -1.5) {
      geometry_msgs::msg::Point retract_position;
      retract_position.x = current_pose.pose.position.x;
      retract_position.y = current_pose.pose.position.y;
      retract_position.z = current_pose.pose.position.z + 0.5;
      if (!move_to_base_position(
          goal_handle, retract_position, "FAILSAFE_RETRACTING", timeout, failure))
      {
        return false;
      }
    }

    geometry_msgs::msg::Point home_position;
    home_position.x = starting_pose.pose.position.x;
    home_position.y = starting_pose.pose.position.y;
    home_position.z = starting_pose.pose.position.z;
    return move_to_base_position(
      goal_handle, home_position, "FAILSAFE_RETURNING_HOME", timeout, failure);
  }

  bool remove_target(uint8_t target_id, std::string & failure)
  {
    if (!remove_client_->wait_for_service(std::chrono::seconds(1))) {
      failure = "Gazebo sediment-removal service unavailable";
      return false;
    }
    auto request = std::make_shared<RemoveSediment::Request>();
    request->target_id = target_id;
    auto future = remove_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      failure = "Timed out waiting for Gazebo to remove the sediment target";
      return false;
    }
    const auto response = future.get();
    failure = response->message;
    return response->success;
  }

  void safe_stop()
  {
    set_pump(false);
    set_valve(false);
    set_armed(false);
    if (wrench_pub_) {
      geometry_msgs::msg::WrenchStamped wrench;
      wrench.header.stamp = get_clock()->now();
      wrench.header.frame_id = "base_link";
      wrench_pub_->publish(wrench);
    }
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<Action::Result>();
    const Target target = targets_[goal->target_id - 1];
    double vacuum = 0.0;
    double flow = 0.0;
    double collected = 0.0;
    std::string failure;
    geometry_msgs::msg::PoseStamped starting_pose;
    bool have_starting_pose = false;
    bool return_attempted = false;

    auto finish = [&](bool success, const std::string & message) {
      safe_stop();
      std::string final_message = message;
      if (!success && have_starting_pose && !return_attempted) {
        geometry_msgs::msg::PoseStamped current_pose;
        if (pose_fresh(current_pose)) {
          const double dx = starting_pose.pose.position.x - current_pose.pose.position.x;
          const double dy = starting_pose.pose.position.y - current_pose.pose.position.y;
          const double dz = starting_pose.pose.position.z - current_pose.pose.position.z;
          const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
          if (distance > get_parameter("maximum_nozzle_error_m").as_double()) {
            return_attempted = true;
            std::string recovery_failure;
            if (return_to_start_if_safe(
                goal_handle, starting_pose,
                goal_handle->get_goal()->approach_timeout_sec, recovery_failure))
            {
              final_message += "; fail-safe return to the starting position completed";
            } else {
              final_message += "; fail-safe return was not completed: " + recovery_failure;
            }
          }
        }
      }
      safe_stop();
      result->success = success;
      result->collected_duration_sec = static_cast<float>(collected);
      result->final_vacuum_kpa = static_cast<float>(vacuum);
      result->message = final_message;
      goal_running_ = false;
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else if (success) {
        goal_handle->succeed(result);
      } else {
        goal_handle->abort(result);
      }
    };

    if (!checks_ok(goal_handle, failure)) {
      finish(false, failure);
      return;
    }
    if (!controller_ready()) {
      finish(false, "Thruster controller must be configured and active first");
      return;
    }
    if (!pose_fresh(starting_pose)) {
      finish(false, "Vehicle pose missing or stale before approach");
      return;
    }
    have_starting_pose = true;
    if (!sensors_fresh(vacuum, flow)) {
      finish(false, "Pressure or flow sensor missing/stale");
      return;
    }
    if (!remove_client_->wait_for_service(std::chrono::seconds(1))) {
      finish(false, "Gazebo sediment-removal service unavailable");
      return;
    }

    geometry_msgs::msg::Point nozzle_goal;
    publish_feedback(goal_handle, "PRECHECK", 0.0, vacuum, flow);
    if (!approach_target(goal_handle, target, nozzle_goal, failure)) {
      finish(false, failure);
      return;
    }

    set_valve(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    if (!checks_ok(goal_handle, failure)) {
      finish(false, failure);
      return;
    }
    set_pump(true);
    const auto collection_start = SteadyClock::now();
    const double acquisition_timeout =
      get_parameter("vacuum_acquisition_timeout_sec").as_double();
    auto last_arm_request = SteadyClock::time_point{};
    bool vacuum_acquired = false;
    double elapsed = 0.0;

    while (active_) {
      if (!checks_ok(goal_handle, failure)) {
        finish(false, failure);
        return;
      }
      if (!controller_ready()) {
        finish(false, "Thruster controller lost during collection");
        return;
      }
      geometry_msgs::msg::PoseStamped pose;
      if (!pose_fresh(pose)) {
        finish(false, "Vehicle pose missing or stale during collection");
        return;
      }
      publish_motion_command(pose, nozzle_goal, last_arm_request);
      if (nozzle_error(pose, nozzle_goal) >
          get_parameter("maximum_collection_drift_m").as_double())
      {
        finish(false, "Nozzle moved outside the safe collection envelope");
        return;
      }
      if (!sensors_fresh(vacuum, flow)) {
        finish(false, "Sensor watchdog: pressure or flow data stale");
        return;
      }
      if (vacuum > get_parameter("maximum_vacuum_kpa").as_double()) {
        finish(false, "Vacuum pressure exceeded safety limit");
        return;
      }

      elapsed = std::chrono::duration<double>(
        SteadyClock::now() - collection_start).count();
      if (!vacuum_acquired) {
        if (vacuum >= goal->target_vacuum_kpa) {
          vacuum_acquired = true;
        } else if (elapsed > acquisition_timeout) {
          finish(false, "Could not reach target vacuum before acquisition timeout");
          return;
        }
      } else {
        if (flow < get_parameter("minimum_flow_lpm").as_double()) {
          finish(false, "Flow below minimum threshold during collection");
          return;
        }
        collected += 0.1;
      }

      publish_feedback(
        goal_handle, vacuum_acquired ? "VACUUMING" : "BUILDING_VACUUM",
        elapsed, vacuum, flow);
      if (collected >= goal->collection_duration_sec) {
        break;
      }
      if (elapsed >= goal->max_duration_sec) {
        finish(false, "Maximum collection duration exceeded");
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!active_) {
      finish(false, "Lifecycle node deactivated during collection");
      return;
    }
    if (!checks_ok(goal_handle, failure)) {
      finish(false, failure);
      return;
    }

    set_pump(false);
    set_valve(false);

    return_attempted = true;
    geometry_msgs::msg::PoseStamped pose_after_collection;
    if (!pose_fresh(pose_after_collection)) {
      finish(false, "Vehicle pose missing or stale before retracting from target");
      return;
    }
    geometry_msgs::msg::Point retract_position;
    retract_position.x = pose_after_collection.pose.position.x;
    retract_position.y = pose_after_collection.pose.position.y;
    retract_position.z = pose_after_collection.pose.position.z + 0.5;
    if (!move_to_base_position(
        goal_handle, retract_position, "RETRACTING", goal->approach_timeout_sec, failure))
    {
      finish(false, failure);
      return;
    }

    geometry_msgs::msg::Point home_position;
    home_position.x = starting_pose.pose.position.x;
    home_position.y = starting_pose.pose.position.y;
    home_position.z = starting_pose.pose.position.z;
    if (!move_to_base_position(
        goal_handle, home_position, "RETURNING_HOME", goal->approach_timeout_sec, failure))
    {
      finish(false, failure);
      return;
    }

    set_armed(false);
    geometry_msgs::msg::WrenchStamped zero_wrench;
    zero_wrench.header.stamp = get_clock()->now();
    zero_wrench.header.frame_id = "base_link";
    wrench_pub_->publish(zero_wrench);
    publish_feedback(goal_handle, "REMOVING_SEDIMENT", elapsed, vacuum, flow);

    if (!remove_target(goal->target_id, failure)) {
      finish(false, failure);
      return;
    }
    std_msgs::msg::UInt32 removed;
    removed.data = goal->target_id;
    collected_pub_->publish(removed);
    finish(true, "Simulated vacuum completed; target removed from Gazebo and RViz");
  }

  rclcpp_action::Server<Action>::SharedPtr action_server_;
  BoolPub::SharedPtr pump_pub_, valve_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr collected_pub_;
  rclcpp::Client<RemoveSediment>::SharedPtr remove_client_;
  rclcpp::Subscription<Float>::SharedPtr pressure_sub_, flow_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr leak_sub_, estop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr controller_sub_;
  std::mutex sensor_mutex_, state_mutex_, worker_mutex_;
  std::thread worker_;
  std::atomic<bool> active_{false};
  std::atomic<bool> leak_detected_{false};
  std::atomic<bool> emergency_stop_{false};
  std::atomic<bool> goal_running_{false};
  double vacuum_kpa_{0.0}, flow_lpm_{0.0};
  bool have_pressure_{false}, have_flow_{false};
  SteadyClock::time_point pressure_time_{}, flow_time_{};
  geometry_msgs::msg::PoseStamped pose_;
  bool have_pose_{false};
  SteadyClock::time_point pose_time_{}, controller_status_time_{};
  std::string controller_status_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SedimentCollectionServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

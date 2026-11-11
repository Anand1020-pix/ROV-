#include <array>
#include <algorithm>
#include <chrono>
#include <functional>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RovThrusterController final : public rclcpp_lifecycle::LifecycleNode
{
public:
  using FloatPub = rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>;
  using SteadyClock = std::chrono::steady_clock;

  explicit RovThrusterController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : LifecycleNode("rov_thruster_controller", options)
  {
    declare_parameter<double>("command_timeout_sec", 0.30);
    declare_parameter<double>("max_horizontal_thrust_n", 20.0);
    declare_parameter<double>("max_vertical_thrust_n", 20.0);
    declare_parameter<double>("horizontal_arm_x_m", 0.22);
    declare_parameter<double>("horizontal_arm_y_m", 0.18);
    declare_parameter<double>("vertical_arm_x_m", 0.16);
    declare_parameter<double>("publish_rate_hz", 50.0);
  }

  ~RovThrusterController() override
  {
    if (timer_) timer_->cancel();
    publish_zero_if_active();
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    const std::array<std::string, 6> topics = {
      "/model/sediment_collector_rov/joint/front_left_thruster_propeller_joint/cmd_thrust",
      "/model/sediment_collector_rov/joint/front_right_thruster_propeller_joint/cmd_thrust",
      "/model/sediment_collector_rov/joint/rear_left_thruster_propeller_joint/cmd_thrust",
      "/model/sediment_collector_rov/joint/rear_right_thruster_propeller_joint/cmd_thrust",
      "/model/sediment_collector_rov/joint/vertical_front_thruster_propeller_joint/cmd_thrust",
      "/model/sediment_collector_rov/joint/vertical_rear_thruster_propeller_joint/cmd_thrust"
    };

    for (std::size_t i = 0; i < topics.size(); ++i) {
      thruster_pubs_[i] = create_publisher<std_msgs::msg::Float64>(topics[i], 10);
    }
    status_pub_ = create_publisher<std_msgs::msg::String>("/rov/control_status", 10);

    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/rov/cmd_wrench", rclcpp::QoS(10),
      [this](geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
        const auto & f = msg->wrench.force;
        const auto & t = msg->wrench.torque;
        if (!msg->header.frame_id.empty() && msg->header.frame_id != "base_link") {
          RCLCPP_ERROR(get_logger(), "Rejecting wrench: expected frame_id=base_link, got '%s'",
            msg->header.frame_id.c_str());
          return;
        }
        if (!finite(f.x) || !finite(f.y) || !finite(f.z) ||
            !finite(t.x) || !finite(t.y) || !finite(t.z)) {
          fault_latched_ = true;
          armed_ = false;
          RCLCPP_ERROR(get_logger(), "Non-finite wrench received: latching control fault");
          return;
        }
        desired_ = {f.x, f.y, f.z, t.x, t.y, t.z};
        have_command_ = true;
        last_command_ = SteadyClock::now();
      });

    arm_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/rov/arm", 10, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (!msg->data) {
          armed_ = false;
          publish_status("DISARMED");
          return;
        }
        if (estop_latched_ || estop_asserted_ || fault_latched_) {
          RCLCPP_ERROR(get_logger(), "Arm rejected: E-stop or fault is latched");
          return;
        }
        if (!command_is_fresh()) {
          RCLCPP_ERROR(get_logger(), "Arm rejected: send a fresh /rov/cmd_wrench first");
          return;
        }
        armed_ = true;
        RCLCPP_WARN(get_logger(), "ROV thruster controller ARMED");
        publish_status("ARMED");
      });

    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/rov/emergency_stop", 10, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        estop_asserted_ = msg->data;
        if (msg->data) {
          estop_latched_ = true;
          armed_ = false;
          RCLCPP_FATAL(get_logger(), "EMERGENCY STOP: all thruster commands forced to zero");
          publish_zero_if_active();
          publish_status("ESTOP_LATCHED");
        }
      });

    reset_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/rov/reset_estop", 10, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (msg->data && !estop_asserted_) {
          estop_latched_ = false;
          fault_latched_ = false;
          armed_ = false;  // Explicit re-arm is always required after reset.
          RCLCPP_WARN(get_logger(), "E-stop reset; controller remains DISARMED");
          publish_status("RESET_DISARMED");
        }
      });

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    for (auto & pub : thruster_pubs_) pub->on_activate();
    status_pub_->on_activate();
    active_ = true;

    const double hz = std::max(1.0, get_parameter("publish_rate_hz").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / hz),
      std::bind(&RovThrusterController::control_tick, this));

    publish_zero();
    publish_status("ACTIVE_DISARMED");
    RCLCPP_INFO(get_logger(), "Controller active. Send wrench, then publish /rov/arm true.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    armed_ = false;
    if (timer_) timer_->cancel();
    publish_zero();
    for (auto & pub : thruster_pubs_) pub->on_deactivate();
    status_pub_->on_deactivate();
    RCLCPP_WARN(get_logger(), "Controller deactivated , thrusters set to zero");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    wrench_sub_.reset();
    arm_sub_.reset();
    estop_sub_.reset();
    reset_sub_.reset();
    for (auto & pub : thruster_pubs_) pub.reset();
    status_pub_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  static bool finite(double x) { return std::isfinite(x); }

  bool command_is_fresh() const
  {
    if (!have_command_) return false;
    const double timeout = get_parameter("command_timeout_sec").as_double();
    return std::chrono::duration<double>(SteadyClock::now() - last_command_).count() <= timeout;
  }

  void publish_status(const std::string & state)
  {
    if (!status_pub_ || !status_pub_->is_activated()) return;
    std_msgs::msg::String msg;
    msg.data = state;
    status_pub_->publish(msg);
  }

  void publish_zero_if_active()
  {
    if (active_) publish_zero();
  }

  void publish_zero()
  {
    for (auto & pub : thruster_pubs_) {
      if (pub && pub->is_activated()) {
        std_msgs::msg::Float64 msg;
        msg.data = 0.0;
        pub->publish(msg);
      }
    }
  }

  void publish_forces(const std::array<double, 6> & forces)
  {
    for (std::size_t i = 0; i < forces.size(); ++i) {
      std_msgs::msg::Float64 msg;
      msg.data = forces[i];
      thruster_pubs_[i]->publish(msg);
    }
  }

  void control_tick()
  {
    if (!active_) return;

    if (estop_latched_ || estop_asserted_ || fault_latched_) {
      armed_ = false;
      publish_zero();
      return;
    }

    if (armed_ && !command_is_fresh()) {
      armed_ = false;  // A lost command stream requires explicit re-arming.
      publish_zero();
      publish_status("WATCHDOG_DISARMED");
      RCLCPP_ERROR(get_logger(), "Command watchdog expired , thrusters zeroed; re-arm required");
      return;
    }

    if (!armed_) {
      publish_zero();
      publish_status("ACTIVE_DISARMED");
      return;
    }

    const double fx = desired_[0];
    const double fy = desired_[1];
    const double fz = desired_[2];
    const double my = desired_[4];
    const double mz = desired_[5];

    // Four vectored horizontal thrusters: surge, sway and yaw.
    const double c = std::cos(M_PI / 4.0);
    const double a = get_parameter("horizontal_arm_x_m").as_double();
    const double b = get_parameter("horizontal_arm_y_m").as_double();
    const double yaw_lever = (a - b) * c;
    const double vertical_arm = get_parameter("vertical_arm_x_m").as_double();
    if (std::abs(yaw_lever) < 1e-4 || a <= 0.0 || b <= 0.0 || vertical_arm <= 1e-4) {
      fault_latched_ = true;
      armed_ = false;
      publish_zero();
      RCLCPP_FATAL(get_logger(), "Invalid thruster geometry parameters control fault latched");
      return;
    }

    const double surge = fx / (4.0 * c);
    const double sway = fy / (4.0 * c);
    const double yaw = mz / (4.0 * yaw_lever);

    std::array<double, 6> out = {
      surge + sway + yaw,  // front-left
      surge - sway - yaw,  // front-right
      surge - sway + yaw,  // rear-left
      surge + sway - yaw,  // rear-right
      0.5 * (fz - my / vertical_arm),
      0.5 * (fz + my / vertical_arm)
    };

    // This 4+2 configuration cannot independently command roll torque.
    if (std::abs(desired_[3]) > 0.05) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Requested roll torque is not controllable with this 6-thruster layout ignoring Mx");
    }

    const double hmax = get_parameter("max_horizontal_thrust_n").as_double();
    const double vmax = get_parameter("max_vertical_thrust_n").as_double();
    for (std::size_t i = 0; i < 4; ++i) out[i] = std::clamp(out[i], -hmax, hmax);
    for (std::size_t i = 4; i < 6; ++i) out[i] = std::clamp(out[i], -vmax, vmax);
    publish_forces(out);
    publish_status("ARMED");
  }

  std::array<FloatPub::SharedPtr, 6> thruster_pubs_{};
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_sub_, estop_sub_, reset_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<double, 6> desired_{0, 0, 0, 0, 0, 0};
  SteadyClock::time_point last_command_{};
  bool have_command_{false};
  bool active_{false};
  bool armed_{false};
  bool estop_asserted_{false};
  bool estop_latched_{false};
  bool fault_latched_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RovThrusterController>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

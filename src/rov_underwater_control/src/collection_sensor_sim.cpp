#include <algorithm>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

class CollectionSensorSim final : public rclcpp::Node
{
public:
  CollectionSensorSim() : Node("collection_sensor_sim")
  {
    pump_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/rov/collection/pump_enable", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) { pump_on_ = m->data; });
    valve_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/rov/collection/valve_open", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) { valve_open_ = m->data; });
    leak_inject_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/rov/sim/inject_leak", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        leak_injected_ = m->data;
        RCLCPP_WARN(get_logger(), "Simulated leak injection %s",
          leak_injected_ ? "ENABLED" : "cleared");
      });
    sensor_disable_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/rov/sim/disable_collection_sensors", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        sensors_disabled_ = m->data;
        RCLCPP_WARN(get_logger(), "Collection sensor publication %s",
          sensors_disabled_ ? "DISABLED" : "enabled");
      });

    vacuum_pub_ = create_publisher<std_msgs::msg::Float64>("/rov/sensors/collection_vacuum_kpa", 10);
    flow_pub_ = create_publisher<std_msgs::msg::Float64>("/rov/sensors/collection_flow_lpm", 10);
    leak_pub_ = create_publisher<std_msgs::msg::Bool>("/rov/sensors/leak_detected", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
      const bool collecting = pump_on_ && valve_open_;
      vacuum_kpa_ += collecting ? 1.8 : -2.5;
      vacuum_kpa_ = std::clamp(vacuum_kpa_, 0.0, 40.0);

      std_msgs::msg::Float64 vacuum; vacuum.data = vacuum_kpa_;
      std_msgs::msg::Float64 flow; flow.data = collecting ? 2.0 : 0.0;
      std_msgs::msg::Bool leak; leak.data = leak_injected_;
      if (!sensors_disabled_) {
        vacuum_pub_->publish(vacuum);
        flow_pub_->publish(flow);
        leak_pub_->publish(leak);
      }
    });
    RCLCPP_WARN(get_logger(),
      "SIMULATION ONLY: collection sensor model active; it does not simulate real sediment transport");
  }

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pump_sub_, valve_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr leak_inject_sub_, sensor_disable_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vacuum_pub_, flow_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr leak_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool pump_on_{false}, valve_open_{false}, leak_injected_{false}, sensors_disabled_{false};
  double vacuum_kpa_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollectionSensorSim>());
  rclcpp::shutdown();
  return 0;
}

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rov_interfaces/srv/remove_sediment.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class RovSimBridge final : public rclcpp::Node
{
public:
  RovSimBridge()
  : Node("rov_sim_bridge")
  {
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/rov/pose", 10);
    tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
    remove_service_ = create_service<rov_interfaces::srv::RemoveSediment>(
      "/rov/sim/remove_sediment",
      [this](
        const std::shared_ptr<rov_interfaces::srv::RemoveSediment::Request> request,
        std::shared_ptr<rov_interfaces::srv::RemoveSediment::Response> response)
      {
        if (request->target_id < 1 || request->target_id > 8) {
          response->success = false;
          response->message = "target_id must be between 1 and 8";
          return;
        }

        gz::msgs::Entity entity;
        entity.set_name(
          "sediment_0" + std::to_string(static_cast<unsigned int>(request->target_id)));
        entity.set_type(gz::msgs::Entity::MODEL);

        gz::msgs::Empty scene_request;
        gz::msgs::StringMsg scene_response;
        bool scene_executed = false;
        const bool scene_completed = gz_node_.Request(
          "/world/underwater_test_world/scene/graph",
          scene_request, 2000, scene_response, scene_executed);
        if (!scene_completed || !scene_executed) {
          response->success = false;
          response->message = "Unable to verify that sediment target " + entity.name() +
            " exists in Gazebo";
          return;
        }
        const std::string entity_label = "label=\"" + entity.name() + " (";
        if (scene_response.data().find(entity_label) == std::string::npos) {
          response->success = false;
          response->message = "Sediment target " + entity.name() +
            " is not present in Gazebo; restart the seabed scene to restore collected targets";
          return;
        }

        gz::msgs::Boolean result;
        bool executed = false;
        const bool completed = gz_node_.Request(
          "/world/underwater_test_world/remove", entity, 2000, result, executed);
        response->success = completed && executed && result.data();
        response->message = response->success ?
          "Sediment model removed from Gazebo" :
          "Gazebo did not remove the requested sediment model";
      });

    const bool subscribed = gz_node_.Subscribe(
      "/world/underwater_test_world/dynamic_pose/info",
      &RovSimBridge::on_pose_message, this);
    if (!subscribed) {
      throw std::runtime_error("Unable to subscribe to Gazebo dynamic pose information");
    }
    RCLCPP_INFO(get_logger(), "Publishing Gazebo base_link pose and TF");
  }

private:
  void on_pose_message(const gz::msgs::Pose_V & message)
  {
    for (const auto & pose : message.pose()) {
      if (pose.name() != "sediment_collector_rov") {
        continue;
      }

      const auto stamp = get_clock()->now();
      geometry_msgs::msg::PoseStamped pose_message;
      pose_message.header.stamp = stamp;
      pose_message.header.frame_id = "world";
      pose_message.pose.position.x = pose.position().x();
      pose_message.pose.position.y = pose.position().y();
      pose_message.pose.position.z = pose.position().z();
      pose_message.pose.orientation.x = pose.orientation().x();
      pose_message.pose.orientation.y = pose.orientation().y();
      pose_message.pose.orientation.z = pose.orientation().z();
      pose_message.pose.orientation.w = pose.orientation().w();
      pose_pub_->publish(pose_message);

      geometry_msgs::msg::TransformStamped transform;
      transform.header = pose_message.header;
      transform.child_frame_id = "base_link";
      transform.transform.translation.x = pose.position().x();
      transform.transform.translation.y = pose.position().y();
      transform.transform.translation.z = pose.position().z();
      transform.transform.rotation = pose_message.pose.orientation;
      tf2_msgs::msg::TFMessage tf_message;
      tf_message.transforms.push_back(transform);
      tf_pub_->publish(tf_message);
      return;
    }
  }

  gz::transport::Node gz_node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Service<rov_interfaces::srv::RemoveSediment>::SharedPtr remove_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RovSimBridge>());
  rclcpp::shutdown();
  return 0;
}

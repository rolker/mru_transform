#ifndef MRU_TRANSFORM_NODES_TIDE_COPIER_HPP
#define MRU_TRANSFORM_NODES_TIDE_COPIER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class TideCopier : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit TideCopier(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  :rclcpp_lifecycle::LifecycleNode("tide_copier", options)
  {
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state)
  {
    input_map_frame_ = declare_parameter<std::string>("input_map_frame", input_map_frame_);
    input_map_tide_frame_ = declare_parameter<std::string>("input_map_tide_frame", input_map_tide_frame_);
    output_map_frame_ = declare_parameter<std::string>("output_map_frame", output_map_frame_);
    output_map_tide_frame_ = declare_parameter<std::string>("output_map_tide_frame", output_map_tide_frame_);

    tf_publisher_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
    tf_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 10, std::bind(&TideCopier::tf_callback, this, std::placeholders::_1));
    return LifecycleNode::on_configure(state);
  }

  CallbackReturn  on_activate(const rclcpp_lifecycle::State & state)
  {
    return LifecycleNode::on_activate(state); 
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state)
  {
    return LifecycleNode::on_cleanup(state);
  }

  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    for (const auto &transform : msg->transforms)
    {
      if (transform.header.frame_id == input_map_frame_ && transform.child_frame_id == input_map_tide_frame_)
      {
        auto new_transform = transform;
        new_transform.header.frame_id = output_map_frame_;
        new_transform.child_frame_id = output_map_tide_frame_;
        tf2_msgs::msg::TFMessage output_msg;
        output_msg.transforms.push_back(new_transform);
        tf_publisher_->publish(output_msg);
        return;
      }
    }
  }

private:
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;

  std::string input_map_frame_ = "in/map";
  std::string input_map_tide_frame_ = "in/map_tide";
  std::string output_map_frame_ = "out/map";
  std::string output_map_tide_frame_ = "out/map_tide";
};

#endif  // MRU_TRANSFORM_NODES_TIDE_COPIER_HPP

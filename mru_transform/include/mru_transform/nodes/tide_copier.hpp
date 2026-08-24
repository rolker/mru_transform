#ifndef MRU_TRANSFORM_NODES_TIDE_COPIER_HPP
#define MRU_TRANSFORM_NODES_TIDE_COPIER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <string>

class TideCopier : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit TideCopier(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  :rclcpp_lifecycle::LifecycleNode("tide_copier", options)
  {
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override
  {
    // Guarded so a second configure does not throw
    // ParameterAlreadyDeclaredException, and deliberately not undeclared in
    // on_cleanup so a value an operator set with `ros2 param set` survives a
    // cleanup -> configure cycle. The declare_parameter<T>() return-value form
    // used here before has no guarded equivalent, hence the declare/get
    // pairs. (#34)
    declare_if_missing("input_map_frame", input_map_frame_);
    declare_if_missing("input_map_tide_frame", input_map_tide_frame_);
    declare_if_missing("output_map_frame", output_map_frame_);
    declare_if_missing("output_map_tide_frame", output_map_tide_frame_);

    tf_publisher_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
    tf_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 10, std::bind(&TideCopier::tf_callback, this, std::placeholders::_1));
    return LifecycleNode::on_configure(state);
  }

  CallbackReturn  on_activate(const rclcpp_lifecycle::State & state) override
  {
    return LifecycleNode::on_activate(state); 
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override
  {
    // Everything on_configure created. The subscription goes first so no
    // callback can be running against the publisher released below it -- and
    // because a cleaned-up node that keeps a live /tf subscription is how this
    // node went on copying map_tide after its lifecycle said it had stopped.
    // Ordering alone only suffices under a single-threaded executor:
    // shared_ptr::reset() provides no synchronization against a callback
    // already dispatched. This node's main() uses one.
    tf_subscription_.reset();
    tf_publisher_.reset();

    // Parameters stay declared on purpose: see on_configure.
    return LifecycleNode::on_cleanup(state);
  }

  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    // rclcpp::Publisher::publish() is a non-virtual template that
    // LifecyclePublisher only hides, so nothing but this check keeps a
    // deactivated node from copying. map_tide is the tide applied to every
    // sounding: it must follow this node's lifecycle state, not ignore it.
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }

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
  // Declare `name` only if it is not declared yet, then read the current value
  // into `value` -- on a re-configure that is the value the operator set.
  void declare_if_missing(const std::string & name, std::string & value)
  {
    if (!has_parameter(name)) {
      declare_parameter(name, value);
    }
    get_parameter(name, value);
  }

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;

  std::string input_map_frame_ = "in/map";
  std::string input_map_tide_frame_ = "in/map_tide";
  std::string output_map_frame_ = "out/map";
  std::string output_map_tide_frame_ = "out/map_tide";
};

#endif  // MRU_TRANSFORM_NODES_TIDE_COPIER_HPP

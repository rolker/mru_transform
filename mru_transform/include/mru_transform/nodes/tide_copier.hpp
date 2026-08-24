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
    release_everything_on_configure_created();
    // Parameters stay declared on purpose: see on_configure.
    return LifecycleNode::on_cleanup(state);
  }

  // `shutdown` is legal from `unconfigured`, `inactive` AND `active`, and it
  // runs on_shutdown ONLY -- on_deactivate and on_cleanup are both skipped.
  // Without this override nothing released what on_configure created, so a
  // FINALIZED node kept its endpoints up for the rest of the process's life.
  // The release helper is null-safe and idempotent, so one override is correct
  // from all three source states. (#34)
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override
  {
    release_everything_on_configure_created();
    return LifecycleNode::on_shutdown(state);
  }

  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    // Why this check exists, stated correctly.
    // rclcpp_lifecycle::LifecyclePublisher::publish() IS virtual (jazzy
    // lifecycle_publisher.hpp) and returns early unless is_activated(), so now
    // that tf_publisher_ is a LifecyclePublisher the inactive and cleaned-up
    // paths are gated by the publisher itself. This comment used to claim
    // publish() was a non-virtual hide that the call site bypassed: that was
    // true of the plain rclcpp::Publisher this node held on jazzy, and it went
    // stale when the member type changed here. It is NOT why the check is kept.
    //
    // What the publisher's gate does not cover is a transition that skips its
    // teardown: `shutdown` from `active` runs on_shutdown ONLY, so
    // LifecycleNode::on_deactivate never runs and every managed publisher stays
    // activated into `finalized`. on_shutdown below closes that at the cause;
    // this check is the second, publisher-type-independent gate behind it, and
    // it is the same shape the other three nodes use -- where it is not
    // redundant at all, because a plain tf2_ros::TransformBroadcaster has no
    // activation gate whatsoever (pinned by
    // SeaSurfaceEstimatorDoesNotBroadcastWhenInactive).
    //
    // map_tide is the tide applied to every sounding: it must follow this
    // node's lifecycle state, not ignore it.
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

  // An exception out of on_configure or on_activate routes the FSM through
  // `errorprocessing` to `unconfigured` WITHOUT running on_cleanup, and
  // `cleanup` is not a legal transition from `unconfigured` -- so without this
  // override nothing ever releases what the failed transition had already
  // allocated. The supported recovery is another configure, which overwrites
  // both pointers and strands the old endpoints: a /tf subscription still
  // dispatching into a half-configured node. Same shape as the sibling nodes,
  // same fix. (#34)
  CallbackReturn on_error(const rclcpp_lifecycle::State &state) override
  {
    RCLCPP_ERROR(
      get_logger(),
      "Transition failed with an exception; releasing everything the failed "
      "configure/activate had allocated. Correct the fault and configure again.");
    release_everything_on_configure_created();
    return LifecycleNode::on_error(state);
  }

private:
  // Shared by on_cleanup, on_shutdown and on_error. Everything on_configure
  // created. The
  // subscription goes first so no callback can be running against the
  // publisher released below it -- and because a node that keeps a live /tf
  // subscription is how this one went on copying map_tide after its lifecycle
  // said it had stopped. Ordering alone only suffices under a single-threaded
  // executor: shared_ptr::reset() provides no synchronization against a
  // callback already dispatched. This node's main() uses one. Every reset is
  // null-safe and re-nulls, so the helper is idempotent.
  void release_everything_on_configure_created()
  {
    tf_subscription_.reset();
    tf_publisher_.reset();
  }

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

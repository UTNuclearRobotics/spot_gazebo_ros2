// contact_adapter_node.cpp
//
// Bridges per-leg Gazebo contact sensor data (ros_gz_interfaces/msg/Contacts,
// one topic per foot) into the single champ_msgs/msg/ContactsStamped message
// that champ_base's state_estimation_node expects on /foot_contacts.
//
// IMPORTANT: Gazebo's contact sensor only publishes a message when there is
// an actual contact to report. During swing phase (foot in the air), no
// message arrives at all -- not even one with an empty `contacts` array.
// Naively latching "last received state" therefore gets stuck at `true`
// forever once a foot lifts off, since nothing ever arrives to clear it.
// To handle this, each leg tracks the time of its last received message and
// is treated as NOT in contact if nothing has arrived within
// kContactTimeout, regardless of what the last message said.
//
// Leg order MUST match what state_estimation_node / quadruped_controller
// expect internally (left_front, right_front, left_hind, right_hind, per
// links.yaml / joints.yaml). Verify this against champ_base source
// (base_.legs[i] indexing) before trusting it in production.
//
// Subscribes:
//   /spot/contact/front_left   (ros_gz_interfaces/msg/Contacts)
//   /spot/contact/front_right  (ros_gz_interfaces/msg/Contacts)
//   /spot/contact/rear_left    (ros_gz_interfaces/msg/Contacts)
//   /spot/contact/rear_right   (ros_gz_interfaces/msg/Contacts)
//
// Publishes:
//   /foot_contacts (champ_msgs/msg/ContactsStamped) at 50 Hz
//
// Build dependencies (package.xml / CMakeLists.txt):
//   rclcpp, champ_msgs, ros_gz_interfaces

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "champ_msgs/msg/contacts_stamped.hpp"
#include "ros_gz_interfaces/msg/contacts.hpp"

using namespace std::chrono_literals;

// How long to wait without a new message before assuming the foot lifted
// off and is no longer in contact. Should be comfortably longer than the
// gap between consecutive sensor updates while in stance (sensor runs at
// ~50 Hz when in contact -> ~20ms between messages), but shorter than the
// shortest expected swing-phase duration for your gait, so a genuine
// lift-off is detected quickly rather than staying latched `true`.
// Tune against your gait.yaml's stance_duration / swing_height if false
// "still in contact" readings persist into the early part of swing phase.
static constexpr auto kContactTimeout = 150ms;

// Debounce: require a contact reading to be consistent for this many
// consecutive received messages before it's allowed to flip the reported
// state. Gazebo's contact solver can report several rapidly-alternating
// contact/no-contact events during touchdown (multiple discrete collision
// events as the foot settles), and champ_base's orientation estimator
// recomputes the body plane from scratch every cycle using whichever feet
// are *currently* marked "in contact" -- so unfiltered chatter here
// directly shows up as visible orientation jitter downstream (lidar cloud
// appearing to tilt/wobble). This does not need to be large; it only needs
// to absorb solver-level bounce, not real gait timing.
static constexpr int kDebounceCount = 3;

class ContactAdapter : public rclcpp::Node
{
public:
  ContactAdapter()
  : Node("contact_adapter")
  {
    // Index order: 0=left_front, 1=right_front, 2=left_hind, 3=right_hind
    contact_state_.fill(false);
    pending_state_.fill(false);
    pending_count_.fill(0);

    const std::array<std::string, 4> leg_topics = {
      "/spot/contact/front_left",
      "/spot/contact/front_right",
      "/spot/contact/rear_left",
      "/spot/contact/rear_right"
    };

    const auto now = this->get_clock()->now();
    last_msg_time_.fill(now);

    for (size_t i = 0; i < leg_topics.size(); ++i)
    {
      auto callback =
        [this, i](const ros_gz_interfaces::msg::Contacts::SharedPtr msg) {
          this->contactCallback(msg, i);
        };

      subscriptions_[i] = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
        leg_topics[i], rclcpp::SensorDataQoS(), callback);
    }

    contacts_publisher_ =
      this->create_publisher<champ_msgs::msg::ContactsStamped>("/foot_contacts", 10);

    timer_ = this->create_wall_timer(
      20ms, std::bind(&ContactAdapter::publishContactState, this));  // 50 Hz

    RCLCPP_INFO(this->get_logger(),
      "contact_adapter started, publishing /foot_contacts at 50 Hz "
      "(per-leg contact timeout: %ld ms)",
      static_cast<long>(kContactTimeout.count()));
  }

private:
  void contactCallback(
    const ros_gz_interfaces::msg::Contacts::SharedPtr msg,
    size_t leg_index)
  {
    const bool reading = !msg->contacts.empty();
    last_msg_time_[leg_index] = this->get_clock()->now();

    if (reading == contact_state_[leg_index])
    {
      // Consistent with current accepted state; nothing pending to confirm.
      pending_count_[leg_index] = 0;
      pending_state_[leg_index] = reading;
      return;
    }

    if (reading == pending_state_[leg_index])
    {
      // Another reading agreeing with the opposite-of-current state.
      pending_count_[leg_index]++;
    }
    else
    {
      // First reading of a potential new state change; start counting.
      pending_state_[leg_index] = reading;
      pending_count_[leg_index] = 1;
    }

    if (pending_count_[leg_index] >= kDebounceCount)
    {
      contact_state_[leg_index] = reading;
      pending_count_[leg_index] = 0;
    }
  }

  void publishContactState()
  {
    const auto now = this->get_clock()->now();

    champ_msgs::msg::ContactsStamped out_msg;
    out_msg.header.stamp = now;
    out_msg.contacts.resize(contact_state_.size());

    for (size_t i = 0; i < contact_state_.size(); ++i)
    {
      const auto time_since_last_msg = now - last_msg_time_[i];
      const bool timed_out = time_since_last_msg > rclcpp::Duration(kContactTimeout);

      // No recent message (e.g. foot is mid-swing, sensor has nothing to
      // report) -> treat as not in contact, even if the last received
      // value was `true`.
      out_msg.contacts[i] = timed_out ? false : contact_state_[i];
    }

    contacts_publisher_->publish(out_msg);
  }

  std::array<bool, 4> contact_state_;
  std::array<bool, 4> pending_state_;
  std::array<int, 4> pending_count_;
  std::array<rclcpp::Time, 4> last_msg_time_;
  std::array<rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr, 4> subscriptions_;
  rclcpp::Publisher<champ_msgs::msg::ContactsStamped>::SharedPtr contacts_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ContactAdapter>());
  rclcpp::shutdown();
  return 0;
}
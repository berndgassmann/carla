// Copyright (c) 2022 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/ros2/publishers/PublisherBase.h"
#include "rosgraph_msgs/msg/ClockPubSubTypes.h"

namespace carla {
namespace ros2 {

using ClockPublisherImpl = DdsPublisherImpl<rosgraph_msgs::msg::Clock, rosgraph_msgs::msg::ClockPubSubType>;

class ClockPublisher : public PublisherBase {
public:
  ClockPublisher();
  virtual ~ClockPublisher() = default;

  /**
   * Implements ROS2NameRecord::Init() interface
   */
  bool Init(std::shared_ptr<DdsDomainParticipantImpl> domain_participant) override;

  /**
   * Implement PublisherInterface::Publish interface
   */
  bool Publish() override;

  /**
   * Implement PublisherInterface::SubscribersConnected() interface
   */
  bool SubscribersConnected() const override;

  /**
   * UpdateData()
   */
  void UpdateData(const builtin_interfaces::msg::Time &stamp);

private:
  std::shared_ptr<ClockPublisherImpl> _impl;
};
}  // namespace ros2
}  // namespace carla

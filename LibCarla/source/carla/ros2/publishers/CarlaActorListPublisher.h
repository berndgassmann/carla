// Copyright (c) 2022 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/ros2/publishers/PublisherBase.h"
#include "carla_msgs/msg/CarlaActorListPubSubTypes.h"

namespace carla {
namespace ros2 {

using CarlaActorListPublisherImpl =
    DdsPublisherImpl<carla_msgs::msg::CarlaActorList, carla_msgs::msg::CarlaActorListPubSubType>;

class CarlaActorListPublisher : public PublisherBase {
public:
  CarlaActorListPublisher(std::string const &role_name);
  virtual ~CarlaActorListPublisher() = default;

  /**
   * Implements ROS2NameRecord::Init() interface
   */
  bool Init(std::shared_ptr<DdsDomainParticipantImpl> domain_participant) override;

  /**
   * Implement PublisherInterface::Publish interface
   */
  bool Publish() override;
  /**
   * Implement PublisherInterface::SubscribersConnected interface
   */
  bool SubscribersConnected() const override;

  void UpdateCarlaActorList(const carla_msgs::msg::CarlaActorList& actor_list);

private:
  std::shared_ptr<CarlaActorListPublisherImpl> _impl;
};
}  // namespace ros2
}  // namespace carla

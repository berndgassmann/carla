// Copyright (c) 2022 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "TrafficLightPublisher.h"

#include "carla/ros2/impl/DdsPublisherImpl.h"

namespace carla {
namespace ros2 {

TrafficLightPublisher::TrafficLightPublisher(
    std::shared_ptr<carla::ros2::types::TrafficLightActorDefinition> traffic_light_actor_definition,
    std::shared_ptr<ObjectsPublisher> objects_publisher,
    std::shared_ptr<ObjectsWithCovariancePublisher> objects_with_covariance_publisher,
    std::shared_ptr<TrafficLightsPublisher> traffic_lights_publisher)
  : PublisherBaseSensor(
        std::static_pointer_cast<carla::ros2::types::ActorNameDefinition>(traffic_light_actor_definition)),
    _traffic_light_info(std::make_shared<TrafficLightInfoPublisherImpl>()),
    _traffic_light_status(std::make_shared<TrafficLightStatusPublisherImpl>()),
    _traffic_light_object_publisher(std::make_shared<ObjectPublisher>(*this, objects_publisher)),
    _traffic_light_object_with_covariance_publisher(
        std::make_shared<ObjectWithCovariancePublisher>(*this, objects_with_covariance_publisher)),
    _traffic_lights_publisher(traffic_lights_publisher) {
  // prefill some traffic_light info data
  _traffic_light_info->Message().id(traffic_light_actor_definition->id);
  // TODO: add respective data to actor definitions
  //         _traffic_light_info->Message().trigger_volume(??);
}

bool TrafficLightPublisher::Init(std::shared_ptr<DdsDomainParticipantImpl> domain_participant) {
  return _traffic_light_info->Init(domain_participant, get_topic_name("traffic_light_info"),
                                   PublisherBase::get_topic_qos()) &&
         _traffic_light_status->Init(domain_participant, get_topic_name("traffic_light_status"),
                                     PublisherBase::get_topic_qos()) &&
         _traffic_light_object_publisher->Init(domain_participant) &&
         _traffic_light_object_with_covariance_publisher->Init(domain_participant);
}

bool TrafficLightPublisher::Publish() {
  if (_traffic_light_info_initialized && (!_traffic_light_info_published)) {
    _traffic_light_info_published = _traffic_light_info->Publish();
  }
  bool success = _traffic_light_info_published;
  success &= _traffic_light_status->Publish();
  success &= _traffic_light_object_publisher->Publish();
  success &= _traffic_light_object_with_covariance_publisher->Publish();
  return success;
}

bool TrafficLightPublisher::SubscribersConnected() const {
  return _traffic_light_info->SubscribersConnected() || _traffic_light_status->SubscribersConnected() ||
         _traffic_light_object_publisher->SubscribersConnected() ||
         _traffic_light_object_with_covariance_publisher->SubscribersConnected();
}

void TrafficLightPublisher::UpdateTrafficLight(std::shared_ptr<carla::ros2::types::Object> &object,
                                               carla::sensor::data::ActorDynamicState const &actor_dynamic_state) {
  if (!_traffic_light_info_initialized) {
    _traffic_light_info_initialized = true;
    _traffic_light_info->Message().transform(object->Transform().pose());
    _traffic_light_info->SetMessageUpdated();
    _traffic_lights_publisher->UpdateTrafficLightInfo(_traffic_light_info->Message());
  }

  if (_traffic_light_status->Message().state() != carla::ros2::types::GetTrafficLightState(actor_dynamic_state)) {
    _traffic_light_status->SetMessageHeader(object->Timestamp().time(), "map");
    _traffic_light_status->Message().id(_traffic_light_info->Message().id());
    _traffic_light_status->Message().state(carla::ros2::types::GetTrafficLightState(actor_dynamic_state));
  }

  _traffic_light_object_publisher->UpdateObject(object);
  _traffic_light_object_with_covariance_publisher->UpdateObject(object);
  _traffic_lights_publisher->UpdateTrafficLightStatus(_traffic_light_status->Message());
}

}  // namespace ros2
}  // namespace carla

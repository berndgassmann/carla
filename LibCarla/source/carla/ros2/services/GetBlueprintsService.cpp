// Copyright (c) 2022 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/ros2/services/GetBlueprintsService.h"

#include <algorithm>

#include "carla/actors/BlueprintLibrary.h"
#include "carla/ros2/impl/DdsServiceImpl.h"

namespace carla {
namespace ros2 {

GetBlueprintsService::GetBlueprintsService(
    carla::rpc::RpcServerInterface &carla_server,
    std::shared_ptr<carla::ros2::types::ActorNameDefinition> actor_name_definition)
  : ServiceBase(carla_server, actor_name_definition), _impl(std::make_shared<GetBlueprintsServiceImpl>()) {}

bool GetBlueprintsService::Init(std::shared_ptr<DdsDomainParticipantImpl> domain_participant) {
  _impl->SetServiceCallback(std::bind(&GetBlueprintsService::GetBlueprints, this, std::placeholders::_1));
  return _impl->Init(domain_participant, get_topic_name());
}

void GetBlueprintsService::CheckRequest() {
  _impl->CheckRequest();
}

carla_msgs::srv::GetBlueprints_Response GetBlueprintsService::GetBlueprints(
    carla_msgs::srv::GetBlueprints_Request const &request) {
  carla_msgs::srv::GetBlueprints_Response response;

  auto filter = request.filter();
  if (filter == "") {
    filter = "*";
  }
  auto blueprints = carla::actors::BlueprintLibrary(_carla_server.call_get_actor_definitions().Get()).Filter(filter);
  response.blueprints().reserve(blueprints->size());
  for (auto const &blueprint : *blueprints) {
    carla_msgs::msg::CarlaActorBlueprint ros_blueprint;
    ros_blueprint.id(blueprint.GetId());
    for (auto const &tag: blueprint.GetTags()) {
      ros_blueprint.tags().push_back(tag);
    }
    for (auto const &attribute: blueprint) {
      diagnostic_msgs::msg::KeyValue key_value;
      key_value.key(attribute.GetId());
      key_value.value(attribute.GetValue());
      ros_blueprint.attributes().push_back(key_value);
    }
    response.blueprints().push_back(ros_blueprint);
  }
  return response;
}

}  // namespace ros2
}  // namespace carla

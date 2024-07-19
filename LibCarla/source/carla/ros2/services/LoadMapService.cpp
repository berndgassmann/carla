// Copyright (c) 2022 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/ros2/services/LoadMapService.h"

#include <algorithm>

#include "carla/actors/BlueprintLibrary.h"
#include "carla/ros2/impl/DdsServiceImpl.h"

namespace carla {
namespace ros2 {

LoadMapService::LoadMapService(
    carla::rpc::RpcServerInterface &carla_server,
    std::shared_ptr<carla::ros2::types::ActorNameDefinition> actor_name_definition)
  : ServiceBase(carla_server, actor_name_definition), _impl(std::make_shared<LoadMapServiceImpl>()) {}

bool LoadMapService::Init(std::shared_ptr<DdsDomainParticipantImpl> domain_participant) {
  _impl->SetServiceCallback(std::bind(&LoadMapService::LoadMap, this, std::placeholders::_1));
  return _impl->Init(domain_participant, get_topic_name());
}

void LoadMapService::CheckRequest() {
  _impl->CheckRequest();
}

carla_msgs::srv::LoadMap_Response LoadMapService::LoadMap(
    carla_msgs::srv::LoadMap_Request const &request) {
  carla_msgs::srv::LoadMap_Response response;

  auto current_map_name = _carla_server.call_get_map_info().Get().name;
  std::string map_name_prefix = "Carla/Maps/";
  std::string map_name_without_prefix = request.mapname();
  if (map_name_without_prefix.starts_with(map_name_prefix)) {
    map_name_without_prefix.erase(0, map_name_prefix.length());
  }
  std::string map_name_with_prefix = map_name_prefix + map_name_without_prefix;
  if( request.force_reload() || 
      (!(map_name_without_prefix == current_map_name) && !(map_name_with_prefix == current_map_name))) {
    auto call_response = _carla_server.call_load_new_episode(map_name_without_prefix, request.reset_episode_settings(), static_cast<rpc::MapLayer>(request.map_layers()));
    response.success(!call_response.HasError());
  }
  else {
    response.success(false);
  }
  return response;
}

}  // namespace ros2
}  // namespace carla

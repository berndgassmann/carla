// Copyright (c) 2022 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/sensor/s11n/GBufferUint8Serializer.h"

#include "carla/sensor/data/Image.h"

namespace carla {
namespace sensor {
namespace s11n {

  SharedPtr<SensorData> GBufferUint8Serializer::Deserialize(RawData DESERIALIZE_DECL_DATA(data)) {
    return SharedPtr<data::Image>(new data::Image{DESERIALIZE_MOVE_DATA(data)});
  }

} // namespace s11n
} // namespace sensor
} // namespace carla

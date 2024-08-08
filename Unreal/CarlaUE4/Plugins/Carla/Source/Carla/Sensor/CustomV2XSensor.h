// Copyright (c) 2024 Institut fuer Technik der Informationsverarbeitung (ITIV) at the 
// Karlsruhe Institute of Technology
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Sensor/Sensor.h"
#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Actor/ActorDescription.h"
#include <carla/sensor/data/V2XData.h>
#include "V2X/PathLossModel.h"
#include <list>
#include <map>
#include "CustomV2XSensor.generated.h"


UCLASS()
class CARLA_API ACustomV2XSensor : public ASensor
{
    GENERATED_BODY()

    using FV2XData = carla::sensor::data::CustomV2XDataS;
    using V2XDataList = std::vector<carla::sensor::data::CustomV2XData>;
    struct DataToSend {
        std::string ChannelId;
        carla::sensor::data::CustomV2XData Message;
    };
    using ActorV2XDataMap = std::map<AActor *, DataToSend>;

public:
    ACustomV2XSensor(const FObjectInitializer &ObjectInitializer);

    static FActorDefinition GetSensorDefinition();

    void Set(const FActorDescription &ActorDescription) override;
    void SetPropagationParams(const float TransmitPower,
                                      const float ReceiverSensitivity,
                                      const float Frequency,
                                      const float combined_antenna_gain,
                                      const float path_loss_exponent,
                                      const float reference_distance_fspl,
                                      const float filter_distance,
                                      const bool use_etsi_fading,
                                      const float custom_fading_stddev);
    void SetScenario(EScenario scenario);
    void SetPathLossModel(const EPathLossModel path_loss_model);
    
    virtual void PrePhysTick(float DeltaSeconds) override;
    virtual void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime) override;
    void SetOwner(AActor *Owner) override;

    void Send(const carla::rpc::CustomV2XBytes &data);

private:
    // global data
    static ActorV2XDataMap gActorV2XDataMap;

    PathLossModel *PathLossModelObj;
    FV2XData mV2XData;

    //write
    void WriteMessageToV2XData(const V2XDataList &msg_received_power_list);

    const long mProtocolVersion = 2;
    const long mMessageId = ITSContainer::messageID_custom;
    long mStationId;
    std::string mChannelId;
    carla::rpc::CustomV2XBytes mMessageData;
    bool mMessageDataChanged = false;
};

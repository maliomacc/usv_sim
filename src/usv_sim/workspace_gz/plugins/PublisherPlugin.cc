/*
 * Copyright (C) 2023 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <google/protobuf/message.h>

#include <chrono>
#include <string>
#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <sdf/sdf.hh>

#include "PublisherPlugin.hh"

using namespace ignition;
namespace sim = ignition::gazebo;
using namespace vrx;

/// \brief Private PublisherPlugin data class.
class vrx::PublisherPlugin::Implementation
{
  /// \brief Class that holds necessary bits for each specified message.
  public: struct MessageInfo
  {
    /// \brief Message type
    std::string msgType;

    /// \brief Message topic
    std::string topic;

    /// \brief Protobuf message parsed from the human-readable string specified
    /// in the <message> element of the plugin's configuration.
    transport::ProtoMsgPtr msgData;

    /// \brief Simulation time to start publishing messages (secs).
    double at = 0;

    /// \brief Re-publish the message every seconds.
    double every = 0;

    /// \brief Transport publisher.
    transport::Node::Publisher pub;
  };

  /// \brief A transport node.
  public: transport::Node node;

  /// \brief List of messages.
  public: std::vector<MessageInfo> messages;
};

//////////////////////////////////////////////////
PublisherPlugin::PublisherPlugin()
  : System(), dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void PublisherPlugin::Configure(const sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    sim::EntityComponentManager &_ecm,
    sim::EventManager &/*_eventMgr*/)
{
  sdf::ElementPtr sdfClone = _sdf->Clone();

  if (sdfClone->HasElement("message"))
  {
    for (auto messageElem = sdfClone->GetElement("message"); messageElem;
         messageElem = messageElem->GetNextElement("message"))
    {
      Implementation::MessageInfo info;

      // Process the message type.
      info.msgType = messageElem->Get<std::string>("type");
      if (info.msgType.empty())
      {
        ignerr << "Message type cannot be empty\n";
        continue;
      }

      // Process the topic name.
      auto topic = messageElem->Get<std::string>("topic");
      info.topic = transport::TopicUtils::AsValidTopic(topic);
      if (info.topic.empty())
      {
        ignerr << "Invalid topic [" << topic << "]" << std::endl;
        continue;
      }

      // Process the <at> element.
      if (messageElem->HasAttribute("at"))
      {
        info.at = messageElem->Get<double>("at");
        if (info.at < 0)
        {
          ignerr << "Invalid <at> [" << info.at << "]" << std::endl;
          continue;
        }
      }

      // Process the <every> element.
      if (messageElem->HasAttribute("every"))
      {
        info.every = messageElem->Get<double>("every");
        if (info.every < 0)
        {
          ignerr << "Invalid <every> [" << info.every << "]" << std::endl;
          continue;
        }
      }

      const std::string msgStr = messageElem->Get<std::string>();
      info.msgData = msgs::Factory::New(info.msgType, msgStr);
      if (info.msgData)
      {
        info.pub = this->dataPtr->node.Advertise(
          info.topic, info.msgData->GetTypeName());
        if (info.pub.Valid())
        {
          this->dataPtr->messages.push_back(std::move(info));
        }
        else
        {
          ignerr << "Message publisher could not be created for topic ["
                << info.topic << "] with message type [" << info.msgType
                << "]" << std::endl;
        }
      }
      else
      {
        ignerr << "Unable to create message of type [" << info.msgType
              << "] with data [" << msgStr << "] when creating message"
              << " publisher on topic " << info.topic << ".\n";
      }
    }
  }
}

//////////////////////////////////////////////////
void PublisherPlugin::PreUpdate(const sim::UpdateInfo &_info,
    sim::EntityComponentManager &_ecm)
{
  IGN_PROFILE("Publisher::PreUpdate");

  if (_info.paused)
    return;

  // Iterate through the messages and publish when needed.
  auto iter = this->dataPtr->messages.begin();
  while (iter != this->dataPtr->messages.end())
  {
    // Time to publish a new message.
    if (std::chrono::duration<double>(_info.simTime).count() >= iter->at)
    {
      iter->pub.Publish(*iter->msgData);

      // Single message.
      if (iter->every == 0)
        iter = this->dataPtr->messages.erase(iter);
      // Repeated message.
      else
      {
        iter->at += iter->every;
        ++iter;
      }
    }
    else
      ++iter;
  }
}

IGNITION_ADD_PLUGIN(PublisherPlugin,
              sim::System,
              PublisherPlugin::ISystemConfigure,
              PublisherPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(vrx::PublisherPlugin,
                    "vrx::PublisherPlugin")

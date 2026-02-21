/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <mutex>
#include <string>

#include <ignition/math/Matrix4.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/PoseCmd.hh>

#include <ignition/transport/Node.hh>

#include "BallShooterPlugin.hh"

using namespace ignition;
namespace sim = ignition::gazebo;
using namespace vrx;

/// \brief Private BallShooterPlugin data class.
class BallShooterPlugin::Implementation
{
  /// \brief Callback function called when receiving a new fire message.
  /// \param[in] _msg Unused.
  public: void OnFire(const msgs::Boolean &_msg);

  /// \brief Protect some member variables used in the callback.
  public: std::mutex mutex;

  /// \brief Gz transport node
  public: ignition::transport::Node node;

  /// \brief Number of shots allowed.
  public: unsigned int remainingShots = math::MAX_UI32;

  /// \brief The force (N) to be applied to the projectile.
  public: double shotForce = 250;

  /// \brief Projectile model entity.
  public: ignition::gazebo::Model projectileModel;

  /// \brief Projectile link entity.
  public: ignition::gazebo::Link projectileLink;

  /// \brief Link/model entity that the projectile pose uses as its frame of
  /// reference.
  public: ignition::gazebo::Entity frame;

  /// \brief Pose in which the projectile should be placed before launching it.
  public: ignition::math::Pose3d pose = ignition::math::Pose3d::Zero;

  /// \brief Ready to shoot a ball when true.
  public: bool shotReady = false;

  /// \brief True to reset ball velocities
  public: bool resetVel = false;
};

//////////////////////////////////////////////////
BallShooterPlugin::BallShooterPlugin()
  : System(), dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void BallShooterPlugin::Configure(const sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  sim::EntityComponentManager &_ecm, sim::EventManager &_eventMgr)
{
  auto sdf = _sdf->Clone();

  // Parse the required <projectile> element.
  if (!sdf->HasElement("projectile"))
  {
    ignerr << "BallShooterPlugin: Missing <projectile> element" << std::endl;
    return;
  }

  sdf::ElementPtr projectileElem = sdf->GetElement("projectile");

  // Parse the required <projectile><model_name> used as projectile.
  if (!projectileElem->HasElement("model_name"))
  {
    ignerr << "BallShooterPlugin: Missing <projectile><model_name> element\n";
    return;
  }

  std::string projectileName =
    projectileElem->GetElement("model_name")->Get<std::string>();

  sim::Entity projectileModelEntity = _ecm.EntityByComponents(
      sim::components::Model(), sim::components::Name(projectileName));

  if (projectileModelEntity == sim::kNullEntity)
  {
    ignerr << "BallShooterPlugin: The model '" << projectileName
          << "' does not exist" << std::endl;
    return;
  }
  this->dataPtr->projectileModel = sim::Model(projectileModelEntity);

  // Parse the required <projectile><link_name>
  if (!projectileElem->HasElement("link_name"))
  {
    ignerr << "BallShooterPlugin: Missing <projectile><link_name> element\n";
    return;
  }

  std::string projectileLinkName =
    projectileElem->GetElement("link_name")->Get<std::string>();

  sim::Entity projectileLinkEntity = _ecm.EntityByComponents(
      sim::components::Link(), sim::components::Name(projectileLinkName),
      sim::components::ParentEntity(this->dataPtr->projectileModel.Entity()));

  if (projectileLinkEntity == sim::kNullEntity)
  {
    ignerr << "BallShooterPlugin: The link '" << projectileLinkName
          << "' does not exist within '" << projectileName << "'" << std::endl;
    return;
  }
  this->dataPtr->projectileLink = sim::Link(projectileLinkEntity);

  // Parse <frame> if available.
  std::string frameName;
  if (projectileElem->HasElement("frame"))
  {
    frameName = projectileElem->Get<std::string>("frame");

    this->dataPtr->frame = _ecm.EntityByComponents(
        sim::components::Name(frameName));

    if (this->dataPtr->frame == sim::kNullEntity)
    {
      ignerr << "The frame '" << frameName << "' does not exist" << std::endl;
      frameName = "";
    }
    else
    {
      auto isModel = _ecm.Component<sim::components::Model>(
          this->dataPtr->frame);
      auto isLink = _ecm.Component<sim::components::Link>(this->dataPtr->frame);
      if (isLink)
      {
        // create world pose comp to be populated by physics if it does not
        // exist yet
        auto worldPoseComp = _ecm.Component<sim::components::WorldPose>(
            this->dataPtr->frame);
        if (!worldPoseComp)
        {
          _ecm.CreateComponent(
              this->dataPtr->frame,
              sim::components::WorldPose());
        }
      }
      else if (!isModel)
      {
        this->dataPtr->frame = sim::kNullEntity;
        frameName = "";
        ignerr << "<frame> tag must list the name of a link or model" << std::endl;
      }
    }
  }

  // Parse <pose> if available.
  if (projectileElem->HasElement("pose"))
    this->dataPtr->pose = projectileElem->Get<math::Pose3d>("pose");

  // Parse <topic> if available.
  std::string topic = "/ball_shooter/fire";
  if (sdf->HasElement("topic"))
    topic = sdf->GetElement("topic")->Get<std::string>();

  // Parse <num_shots> if available.
  if (sdf->HasElement("num_shots"))
  {
    this->dataPtr->remainingShots =
        sdf->GetElement("num_shots")->Get<unsigned int>();
  }

  // Parse <shot_force> if available.
  if (sdf->HasElement("shot_force"))
    this->dataPtr->shotForce = sdf->GetElement("shot_force")->Get<double>();

  this->dataPtr->node.Subscribe(topic,
    &BallShooterPlugin::Implementation::OnFire, this->dataPtr.get());

  // Debug output.
  igndbg << "<projectile><model_name>: " << projectileName << std::endl;
  igndbg << "<projectile><link_name>: " << projectileLinkName << std::endl;
  igndbg << "<frame>: " << frameName << std::endl;
  igndbg << "<num_shots>: " << this->dataPtr->remainingShots << std::endl;
  igndbg << "<pose>: " << this->dataPtr->pose.Pos() << " "
                       << this->dataPtr->pose.Rot().Euler() << std::endl;
  igndbg << "<shot_force>: " << this->dataPtr->shotForce << std::endl;
  igndbg << "<topic>: " << topic << std::endl;
}

//////////////////////////////////////////////////
void BallShooterPlugin::PreUpdate(const sim::UpdateInfo &,
    sim::EntityComponentManager &_ecm)

{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!this->dataPtr->shotReady ||
      this->dataPtr->projectileModel.Entity() == sim::kNullEntity ||
      this->dataPtr->projectileLink.Entity() == sim::kNullEntity)
    return;

  // reset so the ball does not accumulate vel
  // Do this for one physics iteration first. Then fire the shooter in the next
  // iteration.
  if (!this->dataPtr->resetVel)
  {
    this->dataPtr->projectileLink.SetLinearVelocity(_ecm, math::Vector3d::Zero);
    this->dataPtr->projectileLink.SetAngularVelocity(_ecm, math::Vector3d::Zero);
    this->dataPtr->resetVel = true;
    return;
  }
  else
  {
    // make sure to remove the comp otherwise the physics system will keep
    // setting zero vel to the link
    _ecm.RemoveComponent<sim::components::AngularVelocityCmd>(
        this->dataPtr->projectileLink.Entity());
    _ecm.RemoveComponent<sim::components::LinearVelocityCmd>(
        this->dataPtr->projectileLink.Entity());
  }

  // Set the new pose of the projectile based on the frame.
  math::Pose3d projectilePose = this->dataPtr->pose;
  if (this->dataPtr->frame != sim::kNullEntity)
  {
    // get frame entity's world pose
    math::Pose3d framePose;
    if (_ecm.Component<sim::components::Model>(this->dataPtr->frame))
    {
      framePose = _ecm.Component<sim::components::Pose>(
          this->dataPtr->frame)->Data();
    }
    else if (_ecm.Component<sim::components::Link>(this->dataPtr->frame))
    {
      framePose = _ecm.Component<sim::components::WorldPose>(
          this->dataPtr->frame)->Data();
    }

    math::Matrix4d transMat(framePose);
    math::Matrix4d poseLocal(this->dataPtr->pose);
    projectilePose = (transMat * poseLocal).Pose();
  }

  // Teleport the projectile to the ball shooter.
  this->dataPtr->projectileModel.SetWorldPoseCmd(_ecm, projectilePose);

  // Ignition!
  const auto worldPose = this->dataPtr->projectileLink.WorldPose(_ecm);
  auto force = worldPose->Rot().RotateVector(
      math::Vector3d(this->dataPtr->shotForce, 0, 0));
  this->dataPtr->projectileLink.AddWorldForce(_ecm, force);

  --this->dataPtr->remainingShots;
  this->dataPtr->shotReady = false;
  this->dataPtr->resetVel = false;
}

//////////////////////////////////////////////////
void BallShooterPlugin::Implementation::OnFire(const msgs::Boolean &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  if (this->remainingShots <= 0)
  {
    igndbg << "BallShooterPlugin: Maximum number of shots already reached. "
          << "Request ignored" << std::endl;
    return;
  }

  this->shotReady = true;
}

IGNITION_ADD_PLUGIN(BallShooterPlugin,
              sim::System,
              BallShooterPlugin::ISystemConfigure,
              BallShooterPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(vrx::BallShooterPlugin,
                    "vrx::BallShooterPlugin")

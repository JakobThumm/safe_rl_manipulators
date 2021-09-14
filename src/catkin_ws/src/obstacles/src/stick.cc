/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <functional>

#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "obstacles/stick.h"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(StickPlugin)

#define FOLDING_ANIMATION "fold"

/////////////////////////////////////////////////
StickPlugin::StickPlugin()
{
}

/////////////////////////////////////////////////
void StickPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&StickPlugin::OnUpdate, this, std::placeholders::_1)));

  // Initialize ros, if it has not already bee initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client",
        ros::init_options::NoSigintHandler);
  }
}

/////////////////////////////////////////////////
void StickPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  ignition::math::Pose3d pose = this->actor->WorldPose();
  ROS_INFO_STREAM("Actor world Pose: " << pose.Pos().X() << ", " << pose.Pos().Y() << ", " << pose.Pos().Z());
  auto skelAnims = this->actor->SkeletonAnimations();
  auto folding_animation = skelAnims.find(FOLDING_ANIMATION);
  if (folding_animation == skelAnims.end())
  {
    ROS_ERROR_STREAM("Skeleton animation " << FOLDING_ANIMATION << " not found.");
  }
  else
  {
    std::map<std::string, ignition::math::Matrix4d> node_pose = folding_animation->second->PoseAt(this->actor->ScriptTime(), true);
    for (auto const& [key, val] : node_pose)
    {
      ROS_INFO_STREAM("Pose of node " << key << " is \n" <<
            val(0,0) << " " << val(0,1) << " " <<  val(0,2) << " " <<  val(0,3) << "\n" << 
            val(1,0) << " " << val(1,1) << " " <<  val(1,2) << " " <<  val(1,3) << "\n" << 
            val(2,0) << " " << val(2,1) << " " <<  val(2,2) << " " <<  val(2,3) << "\n" << 
            val(3,0) << " " << val(3,1) << " " <<  val(3,2) << " " <<  val(3,3));
    }
  }
}

/////////////////////////////////////////////////
void StickPlugin::Reset()
{

}

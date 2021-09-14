#include "verify_iso/advanced_verify_iso.h"

namespace verify_iso {


bool AdvancedVerifyISO::robotHumanCollision(const custom_robot_msgs::StartGoalCapsuleArrayConstPtr& robot_capsules, 
    const custom_robot_msgs::CapsuleArrayConstPtr& human_capsules) {
  // key: Robot capsule id, values: Human capsules that are in collision with this robot capsule
  std::map<int, std::vector<int>> robot_human_collisions;
  bool is_safe = true;
  // Check position capsules
  for (int i = 0; i < robot_capsules->capsules.size(); i++) {
    for(int j = 0; j < human_capsules->capsules.size(); j++) {
      // If there is a collision, safe it to map
      if (capsuleCollisionCheck(robot_capsules->capsules[i], human_capsules->capsules[j])) {
        if (robot_human_collisions.count(i) == 0) {
          robot_human_collisions.insert(std::pair<int, std::vector<int>> (i, std::vector<int>()));
        }
        robot_human_collisions.at(i).push_back(j);
        is_safe = false;
      }
    }
  }
  // If no collision occured, return safe
  if (is_safe) {
    return false;
  }
  // First check every collision if the movement would lead to a safer position
  for (auto& collision_pair : robot_human_collisions) {
    int i = collision_pair.first;
    for (int j : collision_pair.second) {
      bool movement_safe = startGoalSafetyCheck(robot_capsules->starts[i], robot_capsules->goals[i], human_capsules->capsules[j].segment, 0.0, 2*robot_capsules->capsules[i].radius);
      // If one movement would not lead to a safer state, return unsafe.
      if (!movement_safe) {
        return true;
      }
    }
  }
  // If all movements would be safe, check if there are any potential clamps
  // We assume this is not neccessary right now, since when every robot elements moves away from the potential human collision,
  // there cannot be any clamps.

  // return safe
  return false;
}


void AdvancedVerifyISO::verify_human_reach(const custom_robot_msgs::StartGoalCapsuleArrayConstPtr& robot_capsules, 
    const custom_robot_msgs::CapsuleArrayConstPtr& human_reach_capsules_P, 
    const custom_robot_msgs::CapsuleArrayConstPtr& human_reach_capsules_V, 
    const custom_robot_msgs::CapsuleArrayConstPtr& human_reach_capsules_A) {
  custom_robot_msgs::BoolHeaderedPtr is_safe(new custom_robot_msgs::BoolHeadered());
  is_safe->data = false;
  is_safe->header.stamp = robot_capsules->header.stamp;
  // If no collision occured, we are safe and don't have to check the rest.
  if(!robotHumanCollision(robot_capsules, human_reach_capsules_P)) {
    is_safe->data = true;
    pub_safe_.publish(is_safe);
    return;
  }
  // If no collision occured, we are safe and don't have to check the rest.
  if(!robotHumanCollision(robot_capsules, human_reach_capsules_V)) {
    is_safe->data = true;
    pub_safe_.publish(is_safe);
    return;
  }
  // If no collision occured, we are safe and don't have to check the rest.
  if(!robotHumanCollision(robot_capsules, human_reach_capsules_A)) {
    is_safe->data = true;
    pub_safe_.publish(is_safe);
    return;
  }
  // We are not safe and publish that.
  pub_safe_.publish(is_safe);
}
} // namespace verify_iso
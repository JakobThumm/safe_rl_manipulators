#include "verify_iso/verify_iso.h"

namespace verify_iso {

double VerifyISO::segmentsDistance(const custom_robot_msgs::Segment &seg1, const custom_robot_msgs::Segment &seg2) {
  geometry_msgs::Point d1 = geometry_helpers::fromSegmentToVector(seg1.p,seg1.q);
  geometry_msgs::Point d2 = geometry_helpers::fromSegmentToVector(seg2.p,seg2.q);
  geometry_msgs::Point r = geometry_helpers::fromSegmentToVector(seg2.p,seg1.p);
  double a = geometry_helpers::scalarProduct(d1,d1);
  double e = geometry_helpers::scalarProduct(d2,d2);
  double f = geometry_helpers::scalarProduct(d2,r);
  double epsilon = 1e-6;

  double t,s;
  if(a <= epsilon && e <= epsilon) { //[p1,q1] is actually a point && [p2,q2] is actually a point
    return geometry_helpers::scalarProduct(r,r); // distance = norm(p2-p1)
  }
  else{
    double c = geometry_helpers::scalarProduct(d1,r);
    if (a <= epsilon) {
      s = 0;
      t = std::clamp(f/e, 0.0, 1.0);
    } else if(e < epsilon) {
      t = 0;
      s = std::clamp(-c/a, 0.0, 1.0);
    } else{
      double b = geometry_helpers::scalarProduct(d1,d2);
      double denom = a*e - b*b;
      if(denom != 0) {
        s = std::clamp((b*f - c*e)/denom, 0.0, 1.0);
      }
      else{
        s = 0;
      }
      t = (b*s + f)/e;
      if(t < 0) {
        t = 0;
        s = std::clamp(-c/a, 0.0, 1.0);
      }
      else if(t > 1) {
        t = 1;
        s = std::clamp((b - c)/a, 0.0, 1.0);
      }
    }
  
    geometry_msgs::Point closest_point_1 = geometry_helpers::getPointFromSegment(seg1,s);
    geometry_msgs::Point closest_point_2 = geometry_helpers::getPointFromSegment(seg2,t);
    return pointPointDistance(closest_point_1, closest_point_2);
  }
}


bool VerifyISO::robotHumanCollision(const custom_robot_msgs::CapsuleArrayConstPtr& robot_capsules, 
    const custom_robot_msgs::CapsuleArrayConstPtr& human_capsules) {
  // Check position capsules
  for(auto& human_capsule : human_capsules->capsules) {
    for (auto& robot_capsule : robot_capsules->capsules) {
      // If there is a collision, return true
      if (capsuleCollisionCheck(robot_capsule, human_capsule)) {
        return true;
      }
    }
  }
  return false;
}


void VerifyISO::verify(const custom_robot_msgs::CapsuleArrayConstPtr& robot_ra, 
    const custom_robot_msgs::PolycapsuleArrayConstPtr& human_ra) { 
  custom_robot_msgs::BoolHeaderedPtr is_safe(new custom_robot_msgs::BoolHeadered());
  // check if there is a risk of collision
  for (auto robot_capsule = robot_ra->capsules.begin(); robot_capsule != robot_ra->capsules.end(); robot_capsule++) {
    for(auto human_polycapsule = human_ra->polycapsules.begin(); human_polycapsule != human_ra->polycapsules.end(); human_polycapsule++) {
      if (isIn(robot_capsule->segment.p, human_polycapsule->polygon)) {
        is_safe->data = false;
        is_safe->header.stamp = robot_ra->header.stamp;
        pub_safe_.publish(is_safe);
        return;
      }
      for(auto segment = human_polycapsule->polygon.begin(); segment != human_polycapsule->polygon.end(); segment++) {
        double distance = segmentsDistance(robot_capsule->segment, *segment);
        if ((distance - robot_capsule->radius - human_polycapsule->radius) <= 0) {
          is_safe->data = false;
          is_safe->header.stamp = robot_ra->header.stamp;
          pub_safe_.publish(is_safe); 
          return;
        }
      }
    }
  }
  is_safe->data = true;
  is_safe->header.stamp = robot_ra->header.stamp;
  pub_safe_.publish(is_safe);
}


void VerifyISO::verify_human_reach(const custom_robot_msgs::CapsuleArrayConstPtr& robot_capsules, 
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
#include <nodelet/nodelet.h>
#include <dummy_obstacle.hpp>
#include "geometry_msgs/Pose.h"
#include <geometry_msgs/Point.h>

namespace dummy_obstacle
{
  class DummyObstacleNodelet : public nodelet::Nodelet
  {
    public:

      ros::Subscriber sub_tbrake;
      DummyObstacle obstacle;

    private:
      virtual void onInit();
  };
}

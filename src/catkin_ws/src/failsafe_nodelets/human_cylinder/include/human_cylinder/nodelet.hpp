#include <nodelet/nodelet.h>
#include <human_cylinder.hpp>

namespace human_cylinder
{
  class HumanCylinderNodelet : public nodelet::Nodelet
  {
    public:

      ros::Subscriber sub_tbrake;
      ros::Subscriber sub_curtains;
      ros::Subscriber sub_lasers;
      ros::Subscriber sub_presence_sensors;
      HumanCylinderComputation human_computation;

    private:
      virtual void onInit();
  };
}

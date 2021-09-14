#include "human_cylinder.hpp"
#include <global_library/global_library.h>

/////////////////////////////Sensor/////////////////////////////////////////////////

Sensor::Sensor(){}

////////////////////////////////////////////////////////////////////////////////////

Sensor::Sensor(int id, Sensors type, double delay, double pos_uncertainty, double human_maximum_thickness) :
  id(id),
  type(type),
  delay(delay),
  pos_uncertainty(pos_uncertainty),
  human_maximum_thickness(human_maximum_thickness)
{}

////////////////////////////////////////////////////////////////////////////////////

Sensor::~Sensor() {}

////////////////////////////////////////////////////////////////////////////////////

int Sensor::getId() const
{
  return id;
}

////////////////////////////////////////////////////////////////////////////////////

Sensors Sensor::getType() const
{
  return type;
}

////////////////////////////////////////////////////////////////////////////////////

double Sensor::getDelay() const
{
  return delay;
}

////////////////////////////////////////////////////////////////////////////////////

double Sensor::getPosUncertainty() const 
{
  return pos_uncertainty;
}

////////////////////////////////////////////////////////////////////////////////////

double Sensor::getHumanMaxThickness() const
{
  return human_maximum_thickness;
}

////////////////////////////////////////////////////////////////////////////////////

custom_robot_msgs::Polycapsule Sensor::getHumanInitPos() const
{
  return init_pos;
}

////////////////////////////////////////////////////////////////////////////////////

void Sensor::setHumanInitPos(const custom_robot_msgs::Polycapsule &new_pos)
{
  init_pos = new_pos;
}

////////////////////////////////////////////////////////////////////////////////////

void Sensor::reset()
{
  init_pos.polygon.clear();
  init_pos.radius = 0;
}


///////////////////////////////////Curtain//////////////////////////////////////////

Curtain::Curtain(){}

////////////////////////////////////////////////////////////////////////////////////

Curtain::Curtain(int id, double delay, double pos_uncertainty, double human_maximum_thickness, geometry_msgs::Point& p1, geometry_msgs::Point& p2) :
  Sensor(id, Sensors(CURTAIN), delay, pos_uncertainty, human_maximum_thickness),
  someone_cross(false),
  someone_detected(false)
{
  custom_robot_msgs::Segment seg;
  seg.p = p1;
  seg.q = p2;
  init_pos.polygon.push_back(seg);
  init_pos.radius = 0;
}

////////////////////////////////////////////////////////////////////////////////////

Curtain::~Curtain() {}

////////////////////////////////////////////////////////////////////////////////////

bool Curtain::someoneDetected() const
{
  return someone_detected;
}

////////////////////////////////////////////////////////////////////////////////////

void Curtain::detected()
{
    someone_detected = true;
}

////////////////////////////////////////////////////////////////////////////////////

bool Curtain::someoneCross()
{
  bool answer = someone_cross;
  someone_cross = false;
  return answer;
}

////////////////////////////////////////////////////////////////////////////////////

void Curtain::setCurtainPos(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
  init_pos.polygon[0].p = p1;
  init_pos.polygon[0].q = p2;
  init_pos.radius = 0;
}

////////////////////////////////////////////////////////////////////////////////////

void Curtain::callback()
{
  someone_cross = true;
}

////////////////////////////////////////////////////////////////////////////////////

void Curtain::reset()
{
  init_pos.radius = 0;
  someone_detected = false;
}

////////////////////////////////LaserScanner////////////////////////////////////////

LaserScanner::LaserScanner(int id, double delay, double pos_uncertainty, double human_maximum_thickness, const HalfCircle &hc) :
  Sensor(id, Sensors(LASER), delay, pos_uncertainty, human_maximum_thickness),
  hc(hc)
{}

////////////////////////////////////////////////////////////////////////////////////

LaserScanner::~LaserScanner() {}

////////////////////////////////////////////////////////////////////////////////////

void LaserScanner::callback(const custom_robot_msgs::PositionsConstPtr& data)
{
  reset();
  for (auto pt = data->data.begin(); pt != data->data.end(); pt++) {
    custom_robot_msgs::Segment seg;
    seg.p = *pt;
    seg.q = *pt;
    init_pos.polygon.push_back(seg);
  }
}

/////////////////////////////PresenseSensor/////////////////////////////////////////

PresenceSensor::PresenceSensor(){}

////////////////////////////////////////////////////////////////////////////////////

PresenceSensor::PresenceSensor(int id):
  Sensor(id, Sensors(PRESENCE), 0, 0, 0)
{}

////////////////////////////////////////////////////////////////////////////////////

PresenceSensor::~PresenceSensor() {}

////////////////////////////////////////////////////////////////////////////////////

bool PresenceSensor::someoneDetected() const
{
    return is_someone_in;
}

////////////////////////////////////////////////////////////////////////////////////

void PresenceSensor::callback(const std_msgs::BoolConstPtr &data)
{
  is_someone_in = data->data;
}

/////////////////////////////MultiSensorsAnalysis///////////////////////////////////

MultiSensorsAnalysis::MultiSensorsAnalysis(){}

////////////////////////////////////////////////////////////////////////////////////

MultiSensorsAnalysis::MultiSensorsAnalysis(const HumanCylinder& human, double sample_time) :
  human(human),
  sample_time(sample_time)
{}

////////////////////////////////////////////////////////////////////////////////////

MultiSensorsAnalysis::~MultiSensorsAnalysis() {}

////////////////////////////////////////////////////////////////////////////////////

std::vector<custom_robot_msgs::Polycapsule> MultiSensorsAnalysis::getHumanInitPos()
{
  std::vector<custom_robot_msgs::Polycapsule> init_pos;
  return init_pos;
}

////////////////////////////////////////////////////////////////////////////////////

void MultiSensorsAnalysis::checkLayout() const {}

/////////////////////////////CurtainsAnalysis///////////////////////////////////////

CurtainsAnalysis::CurtainsAnalysis(const HumanCylinder& human, double sample_time, const std::vector<Curtain>& curtains) :
  MultiSensorsAnalysis(human, sample_time),
  curtains(curtains),
  someone_is_in(false)
{}

////////////////////////////////////////////////////////////////////////////////////

CurtainsAnalysis::~CurtainsAnalysis(){};

////////////////////////////////////////////////////////////////////////////////////

void CurtainsAnalysis::isSomeoneIn() 
{
  auto it = curtains.begin();
  while (it != curtains.end() && !it->someoneCross()) {
    it++;
  }
  if (it != curtains.end()) { // someone crosses one curtain ie leaves or enters
    if (!someone_is_in) { // the person enters
      it->detected();
    }
    someone_is_in = !someone_is_in;
  }
}

////////////////////////////////////////////////////////////////////////////////////

std::vector<custom_robot_msgs::Polycapsule> CurtainsAnalysis::getHumanInitPos()
{
  std::vector<custom_robot_msgs::Polycapsule> init_pos_array;
  custom_robot_msgs::Polycapsule init_pos;
  isSomeoneIn();
  for (auto curtain = curtains.begin(); curtain != curtains.end(); curtain++) {
    init_pos = curtain->getHumanInitPos();
    if (init_pos.radius == 0) {
      init_pos.radius = curtain->getPosUncertainty() + curtain->getHumanMaxThickness();
      human.reach(init_pos, curtain->getDelay());
    }
    if (curtain->someoneDetected()) { // the initial polycapsule should grow
      if (someone_is_in) {
        human.reach(init_pos, sample_time);
      }
      else {
        curtain->reset();
        init_pos.radius = curtain->getPosUncertainty() + curtain->getHumanMaxThickness();
        human.reach(init_pos, curtain->getDelay());
      }
    }
    init_pos_array.push_back(init_pos);
    curtain->setHumanInitPos(init_pos);
  }
  return init_pos_array;
}

////////////////////////////////////////////////////////////////////////////////////

void CurtainsAnalysis::callback(const std_msgs::Int8ConstPtr& data) { 
  auto it = curtains.begin();
  while(it->getId() != data->data && it != curtains.end()){
    it++;
  }
  if(it != curtains.end()){
    it->callback();
  }
}

////////////////////////////////////////////////////////////////////////////////////

void CurtainsAnalysis::checkLayout() const 
{
  int n = curtains.size();
  if (n == 1) {
    ROS_WARN("There is only 1 curtain. Check if nobody can enter without crossing the curtains.");
    return;
  }
  
  double distances[2*n][2*n];
  geometry_msgs::Point p11, p12, p21, p22;
  auto curtain = curtains.begin(), curtain2 = curtains.begin()+1;
  for (int i = 0; i < n - 1; curtain++, i++) {
    p11 = curtain->getHumanInitPos().polygon.begin()->p;
    p12 = curtain->getHumanInitPos().polygon.begin()->q;
    curtain2 = curtain + 1;
    for (int j = i + 1; j < n; j++, curtain2++) {
      p21 = curtain2->getHumanInitPos().polygon.begin()->p;
      p22 = curtain2->getHumanInitPos().polygon.begin()->q;
      distances[2*i][2*j] = geometry_helpers::distance2(p11, p21);
      distances[2*i + 1][2*j] = geometry_helpers::distance2(p12, p21);
      distances[2*i][2*j + 1] = geometry_helpers::distance2(p11, p22);
      distances[2 * i + 1][2 * j + 1] = geometry_helpers::distance2(p12, p22);
    }
  }

  bool ok[2*n];
  for (int i = 0; i < 2*n; i++) {
    ok[i] = false;
  }
  int j;
  for (int i = 0; i < 2*n; i++) {
    if (!ok[i]) {
      j = i % 2 ? i + 1 : i + 2;
      while ((distances[i][j] > 0.0025 || distances[i][j] < 0) && j < 2 * n) {
        j += 1;
      }
      if (j < 2*n) {
        ok[i] = true;
        ok[j] = true;
      }
    }
  }
  double min_dist = -1;
  int min, q;
  for (int i = 0; i < 2*n; i++) {
    if (!ok[i]) {
      min_dist = -1;
      min = -1;
      q = i / 2;
      for (int j = i % 2 ? i + 1 : i + 2; j < 2*n; j++) {
        if (j != 2*q && j != 2*q + 1 && !ok[j]) {
          if (min_dist > distances[i][j] || min_dist == -1) {
            min_dist = distances[i][j];
            min = j;
          }
        }
      }
      if (min >= 0) {
        if (min % 2) {
          ROS_WARN("The second point of the curtain %d is too far from the second point of the curtain %d. Check if nobody can enter without crossing the curtains.", i / 2, min / 2);
        }
        ok[2*i] = true;
        ok[min] = true;
      }
    }
  }
}

/////////////////////////CurtainsWithPresenceSensorsAnalysis////////////////////////

CurtainsWithPresenceSensorsAnalysis::CurtainsWithPresenceSensorsAnalysis(const HumanCylinder& human, const PresenceSensor& presence_sensor, double sample_time, const std::vector<Curtain>& curtains) :
  CurtainsAnalysis(human, sample_time, curtains),
  presence_sensor(presence_sensor) 
{}

////////////////////////////////////////////////////////////////////////////////////

CurtainsWithPresenceSensorsAnalysis::~CurtainsWithPresenceSensorsAnalysis(){};

////////////////////////////////////////////////////////////////////////////////////

void CurtainsWithPresenceSensorsAnalysis::isSomeoneIn() 
{
  someone_is_in = presence_sensor.someoneDetected();
}

////////////////////////////////////////////////////////////////////////////////////

void CurtainsWithPresenceSensorsAnalysis::presence_callback(const std_msgs::BoolConstPtr& data){
  presence_sensor.callback(data);
}

/////////////////////////////LaserScannerAnalysis///////////////////////////////////

LaserScannerAnalysis::LaserScannerAnalysis(const std::vector<LaserScanner> laser_scanners, const HumanCylinder& human, double sample_time):
  MultiSensorsAnalysis(human, sample_time),
  laser_scanners(laser_scanners)
{}

////////////////////////////////////////////////////////////////////////////////////

LaserScannerAnalysis::~LaserScannerAnalysis(){};

////////////////////////////////////////////////////////////////////////////////////

std::vector<custom_robot_msgs::Polycapsule> LaserScannerAnalysis::getHumanInitPos()
{
  std::vector<custom_robot_msgs::Polycapsule> init_pos_array;
  custom_robot_msgs::Polycapsule init_pos;
  for (auto laser_scanner = laser_scanners.begin(); laser_scanner != laser_scanners.end(); laser_scanner++) {
    init_pos = laser_scanner->getHumanInitPos();
    if (init_pos.radius == 0) {
      init_pos.radius = laser_scanner->getPosUncertainty() + laser_scanner->getHumanMaxThickness();
      human.reach(init_pos, laser_scanner->getDelay());
    }
    human.reach(init_pos, sample_time);
    init_pos_array.push_back(init_pos);
  }
  return init_pos_array;
}

////////////////////////////////////////////////////////////////////////////////////

void LaserScannerAnalysis::callback(const custom_robot_msgs::PositionsConstPtr& data) { 
  auto it = laser_scanners.begin();
  while(it->getId() != data->id && it != laser_scanners.end()){
    it++;
  }
  if(it != laser_scanners.end()){
    it->callback(data);
  }
}

///////////////////////CurtainsWithLaserScannerAnalysis/////////////////////////////

CurtainsWithLaserScannerAnalysis::CurtainsWithLaserScannerAnalysis(const CurtainsAnalysis& curtains, const LaserScannerAnalysis& laser_scanner):
  curtains(curtains), 
  laser_scanners(laser_scanner)
{}

////////////////////////////////////////////////////////////////////////////////////

CurtainsWithLaserScannerAnalysis::~CurtainsWithLaserScannerAnalysis() {}

////////////////////////////////////////////////////////////////////////////////////

std::vector<custom_robot_msgs::Polycapsule> CurtainsWithLaserScannerAnalysis::getHumanInitPos()
{
  std::vector<custom_robot_msgs::Polycapsule> laser_reachable_set = laser_scanners.getHumanInitPos();
  std::vector<custom_robot_msgs::Polycapsule> curtains_reachable_set = curtains.getHumanInitPos();
  if (laser_reachable_set.front().polygon.size() > 0) {
    return laser_reachable_set;
  }
  return curtains_reachable_set;
}

////////////////////////////////////////////////////////////////////////////////////

void CurtainsWithLaserScannerAnalysis::laser_callback(const custom_robot_msgs::PositionsConstPtr& data) {
  laser_scanners.callback(data);
}

////////////////////////////////////////////////////////////////////////////////////

void CurtainsWithLaserScannerAnalysis::curtain_callback(const std_msgs::Int8ConstPtr& data) {
  curtains.callback(data);
}

///////////////////////////////////HumanCylinder////////////////////////////////////

HumanCylinder::HumanCylinder() {}

////////////////////////////////////////////////////////////////////////////////////

HumanCylinder::HumanCylinder(double max_speed):
  max_speed(max_speed)
{}

////////////////////////////////////////////////////////////////////////////////////

HumanCylinder::~HumanCylinder(){}

////////////////////////////////////////////////////////////////////////////////////

void HumanCylinder::reach(custom_robot_msgs::Polycapsule& init_pos, double t_end)
{
  init_pos.radius += t_end * max_speed;
}

////////////////////////////HumanCylinderComputation////////////////////////////////

HumanCylinderComputation::HumanCylinderComputation()
{}

////////////////////////////////////////////////////////////////////////////////////

HumanCylinderComputation::HumanCylinderComputation(const HumanCylinder& human, MultiSensorsAnalysis* analyzer, ros::Publisher pub) :
  human(human),
  analyzer(analyzer),
  pub(pub)
{}

////////////////////////////////////////////////////////////////////////////////////

HumanCylinderComputation::~HumanCylinderComputation() {}

////////////////////////////////////////////////////////////////////////////////////

void HumanCylinderComputation::humanReachabilityAnalysis(const custom_robot_msgs::DoubleHeaderedConstPtr& t_brake)
{
  custom_robot_msgs::PolycapsuleArrayPtr human_pos(new custom_robot_msgs::PolycapsuleArray());
  std::vector<custom_robot_msgs::Polycapsule> init_pos_array = analyzer->getHumanInitPos();
  for (auto init_pos = init_pos_array.begin(); init_pos != init_pos_array.end(); init_pos++) {
    human.reach(*init_pos, t_brake->data);
  }
  human_pos->header.stamp = t_brake->header.stamp;
  human_pos->polycapsules = init_pos_array;
  pub.publish(human_pos);
}

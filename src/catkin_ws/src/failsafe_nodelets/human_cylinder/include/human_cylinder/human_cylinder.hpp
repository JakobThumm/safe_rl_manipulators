// -*- lsst-c++ -*-
/**
 * @file simulation_helpers.hpp
 * @brief defined all the structure used in the human reachability analysis nodes
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <ros/ros.h>
#include <custom_robot_msgs/PolycapsuleArray.h>
#include <custom_robot_msgs/Positions.h>
#include <custom_robot_msgs/DoubleHeadered.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <Helpers.hpp>


#pragma once

#ifndef Human_Cylinder_H
#define Human_Cylinder_H

enum Sensors
{
    CURTAIN, LASER, PRESENCE
};

/// Class that represents a sensor
class Sensor {
protected:
  int id;                                   ///< the sensor's ID
  Sensors type;                             ///< the sensor's type 
  double delay;                             ///< the sensor's delay
  double pos_uncertainty;                   ///< the sensor's position uncertainty
  double human_maximum_thickness;           ///< the maximum extention of the human body
  custom_robot_msgs::Polycapsule init_pos; ///< the human polycapsule set associated to this sensor

public:
  /**
   * An empty sensor contructor
   */
   Sensor();
  /**
   * A basic sensor contructor
   */
  Sensor(int id, Sensors type, double delay, double pos_uncertainty, double human_maximum_thickness);

  /**
   * A sensor destructor
   */
  virtual ~Sensor();

  /**
   * Returns the ID of the sensor
   *
   * @return the sensor's ID
   */
  virtual int getId() const;

  /**
   * Returns the type of the sensor
   *
   * @return the sensor's type
   */
  virtual Sensors getType() const;

  /**
   * Returns the delay of the sensor
   * 
   * @return the sensor's delay
   */
  virtual double getDelay() const;

  /**
   * Returns the position uncertainty of the sensor
   *
   * @return the sensor's position uncertainty
   */
  virtual double getPosUncertainty() const;

  /**
   * Returns the maximum human thickness of the sensor
   *
   * @return the sensor's maximum human thickness
   */
  virtual double getHumanMaxThickness() const;

  /**
   * Returns the initial position(s) of the human(s)
   *
   * @return the initial human(s) position(s)
   */
  virtual custom_robot_msgs::Polycapsule getHumanInitPos() const;

  /**
   * Modify the initial position(s) of the human(s)
   *
   * @param new_pos the new initial position(s) of the human(s)
   *
   */
  virtual void setHumanInitPos(const custom_robot_msgs::Polycapsule &new_pos);

  /**
   * Reset the reachable set
   */
  virtual void reset();
};

/// Class that represents a curtain with its position, if someone crosses it and the time from witch someone crosses it (if this person is still inside)
class Curtain : public Sensor {
private:
  bool someone_cross;    ///< stores the curtain's crossing information
  bool someone_detected; ///< true iif someone is inside the cage and entered using this curtain
public:
  /** 
   * An empty curtain contructor
   */
  Curtain();

  /** 
   * A basic curtain contructor
   */
  Curtain(int id, double delay, double pos_uncertainty, double human_maximum_thickness, geometry_msgs::Point& p1, geometry_msgs::Point& p2);

  /**
   * A curtain destructor
   */
  ~Curtain();

  /**
   * Returns true iif someone is inside the cage and entered using this curtain
   *
   * @return true iif someone is inside the cage and entered using this curtain
   */
  bool someoneDetected() const;

  /**
   * Stores that someone is inside the cage and entered using this curtain
   */
  void detected();

  /**
   * Get the situation of the curtain since the last check and reset the situation
   * 
   * @return true iif someone crosses the curtain since the last check
   */
  bool someoneCross();

  /**
   * Modify the curtain's position
   *
   * @param p1 the position of the transmitter (resp. receiver)
   * @param p2 the position of the receiver (resp. transmitter)
   *
   */
  void setCurtainPos(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2);

  /**
   * Advertise that someone crosses the curtain
   */
  void callback();

  /**
   * Override the father's methode: reset the reachable set
   */
  void reset();

};

class LaserScanner : public Sensor {
private:
  HalfCircle hc; ///< the scanner visibility area
public:
  /**
   * A laser basic contructor 
   */
  LaserScanner(int id, double delay, double pos_uncertainty, double human_maximum_thickness, const HalfCircle &hc);

  /**
   * A laser destructor
   */
  ~LaserScanner();

  /**
   * Receive the positions of the detected people and store them
   */
  void callback(const custom_robot_msgs::PositionsConstPtr& data);
};

class PresenceSensor : public Sensor {
private:
  bool is_someone_in; ///< true iif the presence_sensor detects someone inside the cage
public:
  /**
   * An empty presence sensor constructor
   */
  PresenceSensor();

  /**
   * A presence sensor basic constructor
   */
  PresenceSensor(int id);

  /**
   * A presence sensor destructor 
   */
  ~PresenceSensor();

  /**
   * Returns true iif the presence sensor detects someone inside
   */
  bool someoneDetected() const;

  /**
   * Receive and store the data from the presence sensor
   */
  void callback(const std_msgs::BoolConstPtr &data);
};

/// Class that, considering a human as a cylinder, computes and sends the reachability analysis
class HumanCylinder{
private:
  double max_speed; ///< the human maximum speed

public:
  /**
   * An empty HumanCapsule constructor 
   */
  HumanCylinder();

  /**
   * A basic HumanCapsule constructor 
   */
  HumanCylinder(double max_speed);

  /**
   * A HumanCapsule destructor
   */
  ~HumanCylinder();

  /// Computes the reachability analysis starting from a given initial pos and until a given time
  /**
   * @param t_end              the time until which the reachability analysis has to be computed
   * @param[in,out] init_pos   the initial set that shall be updated
   */
  void reach(custom_robot_msgs::Polycapsule& init_pos, double t_end);
};

class MultiSensorsAnalysis {
protected:
  double sample_time;   ///< the sampling time
  HumanCylinder human;   ///< human reachability analyser to update the reachable set
public:
  /**
   * An empty MultiSensorsAnalysis constructor
   */
  MultiSensorsAnalysis();

  /**
   * A basic MultiSensorsAnalysis constructor
   */
  MultiSensorsAnalysis(const HumanCylinder &human, double sample_time);

  /**
   * A MultiSensorsAnalysis destructor
   */
  ~MultiSensorsAnalysis();

  /**
   * Computes and returns the Humans initial set
   *
   * @return the initial reachable set
   */
  virtual std::vector<custom_robot_msgs::Polycapsule> getHumanInitPos();

  /**
   * Checks that there are no safety failure in the sensors' layout
   */
  virtual void checkLayout() const;
};

class CurtainsAnalysis : public MultiSensorsAnalysis {
protected:
  std::vector<Curtain> curtains; ///< the list of curtains
  bool someone_is_in;            ///< true iif there is someone in the cage at time t
public:
  /**
   * A basic CurtainsAnalysis constructor
   */
  CurtainsAnalysis(const HumanCylinder& human, double sample_time, const std::vector<Curtain>& curtains);

  /**
   * A CurtainsAnalysis destructor
   */
  ~CurtainsAnalysis();

  /**
   * Analyzes the curtains' situation to determine if someone is inside
   */
  virtual void isSomeoneIn();

  /**
   * Overrides the father's method: computes and returns the initial position
   */
  std::vector<custom_robot_msgs::Polycapsule> getHumanInitPos();

  /**
   * Reads the cross curtain's id in the associated buffer and call the corresponding callback function
   * 
   * @param data the data stored in the buffer
   */
  void callback(const std_msgs::Int8ConstPtr& data);

  /**
   * Overrides the father's method: checks that there are no safety failure in the sensors' layout
   */
  virtual void checkLayout() const;
};

class CurtainsWithPresenceSensorsAnalysis : public CurtainsAnalysis {
private:
  PresenceSensor presence_sensor; ///< the presence sensor
public:
  /**
   * A basic CurtainsWithPresenceSensorsAnalysis constructor
   */
  CurtainsWithPresenceSensorsAnalysis(const HumanCylinder& human, const PresenceSensor &presence_sensor, double sample_time, const std::vector<Curtain>& curtains);

  /**
   * A CurtainsWithPresenceSensorsAnalysis destructor
   */
  ~CurtainsWithPresenceSensorsAnalysis();

  /**
   * Overrides the father's method
   */
  void isSomeoneIn();
  
  /**
   * Reads the presence sensor's data in the associated buffer and call the corresponding callback function
   * 
   * @param data the data stored in the buffer
   */
  void presence_callback(const std_msgs::BoolConstPtr& data);
};

class LaserScannerAnalysis : public MultiSensorsAnalysis {
private:
    std::vector<LaserScanner> laser_scanners;
public:
    /**
     * A basic LaserScannerAnalysis constructor
     */
    LaserScannerAnalysis(const std::vector<LaserScanner> laser_scanners, const HumanCylinder& human, double sample_time);
    
    /**
     * A LaserScannerAnalysis destructor
     */
    ~LaserScannerAnalysis();

    /**
     * Overrides the father's method: computes and returns the initial position
     */
    virtual std::vector<custom_robot_msgs::Polycapsule> getHumanInitPos();

    /**
     * Reads the laser scanner's information in the associated buffer and call the corresponding callback function
     * 
     * @param data the data stored in the buffer
     */
     void callback(const custom_robot_msgs::PositionsConstPtr& data);
};

class CurtainsWithLaserScannerAnalysis : public MultiSensorsAnalysis {
private:
    CurtainsAnalysis curtains;            ///< the light curtain(s)
    LaserScannerAnalysis laser_scanners;  ///< the laser scanner(s)
public:
    /**
     * A basic CurtainsWithLaserScannerAnalysis constructor
     */
    CurtainsWithLaserScannerAnalysis(const CurtainsAnalysis &curtains, const LaserScannerAnalysis &laser_scanners);
    
    /**
     * A CurtainsWithLaserScannerAnalysis destructor
     */
    ~CurtainsWithLaserScannerAnalysis();

    /**
     * Overrides the father's method: computes and returns the initial set
     */
    std::vector<custom_robot_msgs::Polycapsule> getHumanInitPos();

    /**
     * Reads the laser scanner's information in the associated buffer and call the corresponding callback function
     * 
     * @param data the data stored in the buffer
     */
     void laser_callback(const custom_robot_msgs::PositionsConstPtr& data);

    /**
     * Reads the cross curtain's id in the associated buffer and call the corresponding callback function
     * 
     * @param data the data stored in the buffer
     */
     void curtain_callback(const std_msgs::Int8ConstPtr& data);
};

class HumanCylinderComputation {
private:
  HumanCylinder human;             ///> human model for reachability computation
  MultiSensorsAnalysis* analyzer;  ///> the sensors' analyzer to get the initial position
  ros::Publisher pub;              ///> the polycapsules' publisher

public:
  /**
   * An empty HumanCylinderComputation constructor
   */
  HumanCylinderComputation();
  /**
   * A basic HumanReachabilityAnalysisComputation constructor
   */
  HumanCylinderComputation(const HumanCylinder& human, MultiSensorsAnalysis* analyzer, ros::Publisher pub);

  /**
   * A HumanReachabilityAnalysisComputation destructor
   */
  ~HumanCylinderComputation();

  /**
   * Reads the new potential buffer in the associated buffer and computes and sends the occupancy capsules of the robot
   */
  /**
   * Reads the ending time of the reachability analysis in the associated buffer and computes and sends the occupancy polycapsules of the humans
   *
   * @param data the message's data (contains t_end)
   */
    void humanReachabilityAnalysis(const custom_robot_msgs::DoubleHeaderedConstPtr& t_brake);
};

#endif

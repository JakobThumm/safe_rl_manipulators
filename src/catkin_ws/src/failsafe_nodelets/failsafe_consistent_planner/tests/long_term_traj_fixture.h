// -*- lsst-c++ -*/
/**
 * @file long_term_traj_fixture.h
 * @brief Defines the test fixture for verify long term trajectory class
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "failsafe_consistent_planner/motion.h"
#include "failsafe_consistent_planner/long_term_traj.h"

#ifndef LONG_TERM_TRAJ_FIXTURE_H
#define LONG_TERM_TRAJ_FIXTURE_H

namespace online_verification {

/**
 * @brief Test fixture for verify long term trajectory class
 */
class LongTermTrajTest : public ::testing::Test {
 protected:
  /**
   * @brief The LTT object
   */
  LongTermTraj long_term_trajectory_;

  /**
   * @brief Create the LTT object
   */
  void SetUp() override {
      long_term_trajectory_ = LongTermTraj();
  }
};
} // namespace online_verification

#endif // LONG_TERM_TRAJ_FIXTURE_H
// -*- lsst-c++ -*/
/**
 * @file verify_iso_fixture.h
 * @brief Defines the test fixture for verify ISO class
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "verify_iso/verify_iso.h"

#ifndef VERFIY_ISO_FIXTURE_H
#define VERFIY_ISO_FIXTURE_H

namespace verify_iso {

/**
 * @brief Test fixture for verify ISO class
 */
class VerifyIsoTest : public ::testing::Test {
 protected:
  /**
   * @brief The verify iso object
   */
  VerifyISO verify_iso_;

  /**
   * @brief Create the verify iso object
   */
  void SetUp() override {
      verify_iso_ = VerifyISO();
  }
};
} // namespace verify_iso

#endif // VERFIY_ISO_FIXTURE_H
#include <gtest/gtest.h>

// Std lib
#include <stdio.h>
#include <chrono>
#include <fstream>
#include <iostream>

// RMP
#include <rmp/rmp.hpp>
// GTSAM
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>

#include <vector>
#include <boost/any.hpp>

using namespace std;
using namespace gtsam;
using namespace rmp;

TEST(UnitTests, RmpConstructorTest) {
  Rmp2 rmp(Vector2(1, 2), Matrix2::Identity() * 2.0);
  ASSERT_EQ(1, rmp.acceleration()(0));
  ASSERT_EQ(2, rmp.acceleration()(1));
  ASSERT_NEAR(2.0, rmp.metric()(0, 0), 1e-6);
  ASSERT_NEAR(2.0, rmp.metric()(1, 1), 1e-6);
}

TEST(UnitTests, GoalRmpTest) {
  // Create RMP problem
  rmp::Problem problem;

  // Create variable
  Symbol x_ddot('x');
  Vector3_ acc(x_ddot);

  // State
  Pose2 state(0, 0, 0);
  Vector3 velocity(0, 0, 0);

  // Create RMP
  Pose2 goal(1, 0, 0);
  problem.addRmp(acc, Rmp3(MotionPolicy::makeGoalPosePolicy(state, goal, 1.0), Metric3::makeConstantMetric()));

  // Create initial values
  Values initial_acc;
  initial_acc.insert(x_ddot, Vector3(0.0, 0.0, 0.0));

  // Solve
  Values accelerations = problem.solve(initial_acc, false);
  Vector3 opt_acc = accelerations.at<Vector3>(x_ddot);

  ASSERT_NEAR(opt_acc(0), 1.0, 1e-3);
  ASSERT_NEAR(opt_acc(1), 0.0, 1e-3);
  ASSERT_NEAR(opt_acc(2), 0.0, 1e-3);
}

TEST(UnitTests, GoalRmpGainTest) {
  // Create RMP problem
  rmp::Problem problem;

  // Create variable
  Symbol x_ddot('x');
  Vector3_ acc(x_ddot);

  // State
  Pose2 state(0, 0, 0);
  Vector3 velocity(0, 0, 0);

  // Create RMP
  Pose2 goal(1, 0, 0);
  problem.addRmp(acc, Rmp3(MotionPolicy::makeGoalPosePolicy(state, goal, 10.0), Metric3::makeConstantMetric()));

  // Create initial values
  Values initial_acc;
  initial_acc.insert(x_ddot, Vector3(0.0, 0.0, 0.0));

  // Solve
  Values accelerations = problem.solve(initial_acc, false);
  Vector3 opt_acc = accelerations.at<Vector3>(x_ddot);

  ASSERT_NEAR(opt_acc(0), 10.0, 1e-3);
  ASSERT_NEAR(opt_acc(1), 0.0, 1e-3);
  ASSERT_NEAR(opt_acc(2), 0.0, 1e-3);
}
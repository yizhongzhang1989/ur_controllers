#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "simple_joint_impedance_controller/math.hpp"

using simple_joint_impedance_controller::auto_fill_critical_damping;
using simple_joint_impedance_controller::clamp_to_limits;
using simple_joint_impedance_controller::compute_pd_torque;
using simple_joint_impedance_controller::saturate_torque_abs;
using simple_joint_impedance_controller::saturate_torque_rate;
using simple_joint_impedance_controller::validate_target_joint_state;

namespace
{
constexpr double kEps = 1e-12;
}

TEST(AutoFillCriticalDamping, ReplacesNegativeEntries)
{
  std::vector<double> k{100.0, 400.0, 0.0};
  std::vector<double> d{-1.0, -0.5, -1.0};
  ASSERT_TRUE(auto_fill_critical_damping(k, d));
  EXPECT_NEAR(d[0], 2.0 * std::sqrt(100.0), kEps);
  EXPECT_NEAR(d[1], 2.0 * std::sqrt(400.0), kEps);
  EXPECT_NEAR(d[2], 0.0, kEps);
}

TEST(AutoFillCriticalDamping, LeavesNonNegativeEntriesAlone)
{
  std::vector<double> k{100.0, 400.0};
  std::vector<double> d{5.0, 0.0};
  ASSERT_TRUE(auto_fill_critical_damping(k, d));
  EXPECT_NEAR(d[0], 5.0, kEps);
  EXPECT_NEAR(d[1], 0.0, kEps);
}

TEST(AutoFillCriticalDamping, RejectsLengthMismatch)
{
  std::vector<double> k{100.0, 400.0};
  std::vector<double> d{-1.0};
  EXPECT_FALSE(auto_fill_critical_damping(k, d));
}

TEST(AutoFillCriticalDamping, TreatsNegativeKAsZero)
{
  std::vector<double> k{-10.0};
  std::vector<double> d{-1.0};
  ASSERT_TRUE(auto_fill_critical_damping(k, d));
  EXPECT_NEAR(d[0], 0.0, kEps);
}

TEST(ClampToLimits, ClampsElementwise)
{
  std::vector<double> q_d{-2.0, 0.5, 10.0};
  const std::vector<double> lo{-1.0, 0.0, 0.0};
  const std::vector<double> hi{1.0, 1.0, 2.0};
  ASSERT_TRUE(clamp_to_limits(q_d, lo, hi));
  EXPECT_NEAR(q_d[0], -1.0, kEps);
  EXPECT_NEAR(q_d[1], 0.5, kEps);
  EXPECT_NEAR(q_d[2], 2.0, kEps);
}

TEST(ClampToLimits, RejectsInvertedBounds)
{
  std::vector<double> q_d{0.0};
  const std::vector<double> lo{1.0};
  const std::vector<double> hi{-1.0};
  EXPECT_FALSE(clamp_to_limits(q_d, lo, hi));
}

TEST(ComputePdTorque, ScalarCaseMatchesHandCalc)
{
  const std::vector<double> q{0.0, 0.0};
  const std::vector<double> q_d{0.1, -0.2};
  const std::vector<double> qdot{0.05, 0.0};
  const std::vector<double> qdot_d{};  // zero FF
  const std::vector<double> k{100.0, 200.0};
  const std::vector<double> d{10.0, 5.0};
  std::vector<double> tau;
  ASSERT_TRUE(compute_pd_torque(q, q_d, qdot, qdot_d, k, d, tau));
  // tau[0] = 100 * 0.1 - 10 * 0.05 = 9.5
  // tau[1] = 200 * -0.2 - 5 * 0   = -40
  EXPECT_NEAR(tau[0], 9.5, kEps);
  EXPECT_NEAR(tau[1], -40.0, kEps);
}

TEST(ComputePdTorque, VelocityFeedforwardAppliesWhenSameLength)
{
  const std::vector<double> q{0.0};
  const std::vector<double> q_d{0.0};
  const std::vector<double> qdot{0.1};
  const std::vector<double> qdot_d{0.3};
  const std::vector<double> k{0.0};
  const std::vector<double> d{10.0};
  std::vector<double> tau;
  ASSERT_TRUE(compute_pd_torque(q, q_d, qdot, qdot_d, k, d, tau));
  // tau = 10 * (0.3 - 0.1) = 2.0
  EXPECT_NEAR(tau[0], 2.0, kEps);
}

TEST(ComputePdTorque, RejectsLengthMismatch)
{
  const std::vector<double> q{0.0};
  const std::vector<double> q_d{0.0, 0.0};
  const std::vector<double> qdot{0.0};
  const std::vector<double> qdot_d{};
  const std::vector<double> k{100.0};
  const std::vector<double> d{10.0};
  std::vector<double> tau;
  EXPECT_FALSE(compute_pd_torque(q, q_d, qdot, qdot_d, k, d, tau));
}

TEST(SaturateTorqueRate, ClampsPositiveAndNegativeDelta)
{
  const std::vector<double> tau{5.0, -5.0, 0.3};
  const std::vector<double> tau_prev{0.0, 0.0, 0.0};
  const std::vector<double> max_delta{1.0, 2.0, 1.0};
  std::vector<double> out;
  ASSERT_TRUE(saturate_torque_rate(tau, tau_prev, max_delta, out));
  EXPECT_NEAR(out[0], 1.0, kEps);    // +5 clamped to +1
  EXPECT_NEAR(out[1], -2.0, kEps);   // -5 clamped to -2
  EXPECT_NEAR(out[2], 0.3, kEps);    // within band
}

TEST(SaturateTorqueRate, PreservesPrevWhenDeltaZero)
{
  const std::vector<double> tau{3.0};
  const std::vector<double> tau_prev{3.0};
  const std::vector<double> max_delta{0.1};
  std::vector<double> out;
  ASSERT_TRUE(saturate_torque_rate(tau, tau_prev, max_delta, out));
  EXPECT_NEAR(out[0], 3.0, kEps);
}

TEST(SaturateTorqueRate, RejectsNegativeMaxDelta)
{
  const std::vector<double> tau{0.0};
  const std::vector<double> tau_prev{0.0};
  const std::vector<double> max_delta{-1.0};
  std::vector<double> out;
  EXPECT_FALSE(saturate_torque_rate(tau, tau_prev, max_delta, out));
}

TEST(SaturateTorqueAbs, ClampsIntoSymmetricBand)
{
  std::vector<double> tau{100.0, -100.0, 5.0};
  const std::vector<double> tau_max{50.0, 60.0, 50.0};
  ASSERT_TRUE(saturate_torque_abs(tau, tau_max));
  EXPECT_NEAR(tau[0], 50.0, kEps);
  EXPECT_NEAR(tau[1], -60.0, kEps);
  EXPECT_NEAR(tau[2], 5.0, kEps);
}

TEST(SaturateTorqueAbs, RejectsNegativeLimit)
{
  std::vector<double> tau{0.0};
  const std::vector<double> tau_max{-1.0};
  EXPECT_FALSE(saturate_torque_abs(tau, tau_max));
}

TEST(ValidateTargetJointState, AcceptsPositionOnlyWithoutNames)
{
  const std::vector<std::string> names;
  const std::vector<double> pos{0.1, -0.2, 0.3};
  const std::vector<double> vel;
  const std::vector<std::string> expected{"a", "b", "c"};
  std::vector<double> q_d, qdot_d;
  ASSERT_TRUE(validate_target_joint_state(names, pos, vel, expected, q_d, qdot_d));
  EXPECT_EQ(q_d, pos);
  EXPECT_TRUE(qdot_d.empty());
}

TEST(ValidateTargetJointState, AcceptsPositionAndVelocity)
{
  const std::vector<std::string> names{"a", "b"};
  const std::vector<double> pos{0.0, 1.0};
  const std::vector<double> vel{0.5, -0.5};
  const std::vector<std::string> expected{"a", "b"};
  std::vector<double> q_d, qdot_d;
  ASSERT_TRUE(validate_target_joint_state(names, pos, vel, expected, q_d, qdot_d));
  EXPECT_EQ(q_d, pos);
  EXPECT_EQ(qdot_d, vel);
}

TEST(ValidateTargetJointState, RejectsNameReorder)
{
  const std::vector<std::string> names{"b", "a"};
  const std::vector<double> pos{0.0, 1.0};
  const std::vector<double> vel;
  const std::vector<std::string> expected{"a", "b"};
  std::vector<double> q_d, qdot_d;
  EXPECT_FALSE(validate_target_joint_state(names, pos, vel, expected, q_d, qdot_d));
}

TEST(ValidateTargetJointState, RejectsPositionLengthMismatch)
{
  const std::vector<std::string> names;
  const std::vector<double> pos{0.0};
  const std::vector<double> vel;
  const std::vector<std::string> expected{"a", "b"};
  std::vector<double> q_d, qdot_d;
  EXPECT_FALSE(validate_target_joint_state(names, pos, vel, expected, q_d, qdot_d));
}

TEST(ValidateTargetJointState, RejectsVelocityLengthMismatch)
{
  const std::vector<std::string> names;
  const std::vector<double> pos{0.0, 0.0};
  const std::vector<double> vel{0.1};
  const std::vector<std::string> expected{"a", "b"};
  std::vector<double> q_d, qdot_d;
  EXPECT_FALSE(validate_target_joint_state(names, pos, vel, expected, q_d, qdot_d));
}

TEST(ValidateTargetJointState, RejectsNonFinitePosition)
{
  const std::vector<std::string> names;
  const std::vector<double> pos{0.0, std::numeric_limits<double>::quiet_NaN()};
  const std::vector<double> vel;
  const std::vector<std::string> expected{"a", "b"};
  std::vector<double> q_d, qdot_d;
  EXPECT_FALSE(validate_target_joint_state(names, pos, vel, expected, q_d, qdot_d));
}

TEST(ValidateTargetJointState, RejectsNonFiniteVelocity)
{
  const std::vector<std::string> names;
  const std::vector<double> pos{0.0, 0.0};
  const std::vector<double> vel{0.0, std::numeric_limits<double>::infinity()};
  const std::vector<std::string> expected{"a", "b"};
  std::vector<double> q_d, qdot_d;
  EXPECT_FALSE(validate_target_joint_state(names, pos, vel, expected, q_d, qdot_d));
}

TEST(ValidateTargetJointState, RejectsEmptyExpectedJoints)
{
  const std::vector<std::string> names;
  const std::vector<double> pos;
  const std::vector<double> vel;
  const std::vector<std::string> expected;
  std::vector<double> q_d, qdot_d;
  EXPECT_FALSE(validate_target_joint_state(names, pos, vel, expected, q_d, qdot_d));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

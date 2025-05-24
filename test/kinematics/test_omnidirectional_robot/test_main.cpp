#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <unity.h>

#include <cmath>
#include <stdexcept>

#include "kinematics/omnidirectional_robot.cpp"  // Include the implementation file for testing
#include "kinematics/omnidirectional_robot.h"

namespace config
{
namespace test
{
constexpr double RADIUS = 0.1;
constexpr double WHEEL_DISTANCE = 0.2;
constexpr double EPSILON = 1e-6;
const double SQRT2_2 = std::sqrt(2) / 2;
constexpr double ROTATION_FACTOR = -WHEEL_DISTANCE / RADIUS;
const double TRANSLATION_FACTOR = SQRT2_2 / RADIUS;
}  // namespace test
}  // namespace config

// Helper functions
void assert_vector_equal(const Eigen::VectorXd & expected,
                         const Eigen::VectorXd & actual, const char * context,
                         double epsilon = config::test::EPSILON)
{
    TEST_ASSERT_EQUAL_MESSAGE(expected.size(), actual.size(),
                              "Vector size mismatch");
    for (int i = 0; i < expected.size(); ++i)
    {
        char msg[128];
        snprintf(msg, sizeof(msg), "%s index %d (%.4f vs %.4f)", context, i,
                 expected[i], actual[i]);

        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(epsilon, expected[i], actual[i], msg);
    }
}

bool throws_invalid_argument(double r, double d)
{
    try
    {
        OmnidirectionalRobot robot(r, d);
        return false;
    }
    catch (const std::invalid_argument &)
    {
        return true;
    }
    catch (...)
    {
        return false;
    }
}

// Test cases
void test_constructor_invalid_params()
{
    TEST_ASSERT_TRUE_MESSAGE(throws_invalid_argument(0, 0.1),
                             "Zero radius should throw");
    TEST_ASSERT_TRUE_MESSAGE(throws_invalid_argument(-0.1, 0.1),
                             "Negative radius should throw");
    TEST_ASSERT_TRUE_MESSAGE(throws_invalid_argument(0.1, -0.1),
                             "Negative distance should throw");
    TEST_ASSERT_FALSE_MESSAGE(throws_invalid_argument(0.1, 0.1),
                              "Valid params shouldn't throw");
}

void test_pure_rotation()
{
    OmnidirectionalRobot robot(config::test::RADIUS,
                               config::test::WHEEL_DISTANCE);

    const Eigen::Vector4d expected =
      Eigen::Vector4d::Constant(config::test::ROTATION_FACTOR);

    const Eigen::Vector4d actual = robot.computeWheelVelocities(1.0, 0, 0);

    assert_vector_equal(expected, actual, "Pure rotation");
}

void test_pure_translation_x()
{
    OmnidirectionalRobot robot(config::test::RADIUS,
                               config::test::WHEEL_DISTANCE);

    const Eigen::Vector4d expected(
      config::test::TRANSLATION_FACTOR, -config::test::TRANSLATION_FACTOR,
      -config::test::TRANSLATION_FACTOR, config::test::TRANSLATION_FACTOR);

    const Eigen::Vector4d actual = robot.computeWheelVelocities(0, 1.0, 0);

    assert_vector_equal(expected, actual, "X translation");
}

void test_pure_translation_y()
{
    OmnidirectionalRobot robot(config::test::RADIUS,
                               config::test::WHEEL_DISTANCE);

    const Eigen::Vector4d expected(
      config::test::TRANSLATION_FACTOR, config::test::TRANSLATION_FACTOR,
      -config::test::TRANSLATION_FACTOR, -config::test::TRANSLATION_FACTOR);

    const Eigen::Vector4d actual = robot.computeWheelVelocities(0, 0, 1.0);

    assert_vector_equal(expected, actual, "Y translation");
}

void test_kinematics_round_trip()
{
    OmnidirectionalRobot robot(config::test::RADIUS,
                               config::test::WHEEL_DISTANCE);

    const Eigen::Vector3d original(0.5, 1.2, -0.8);

    const Eigen::Vector4d wheel_vel =
      robot.computeWheelVelocities(original[0], original[1], original[2]);

    const Eigen::Vector3d reconstructed =
      robot.computeBodyVelocities(wheel_vel);

    assert_vector_equal(original, reconstructed, "Round trip");
}

void test_zero_input_zero_output()
{
    OmnidirectionalRobot robot(config::test::RADIUS,
                               config::test::WHEEL_DISTANCE);

    const Eigen::Vector4d wheel_zeros = robot.computeWheelVelocities(0, 0, 0);

    assert_vector_equal(Eigen::Vector4d::Zero(), wheel_zeros, "Zero wheels");

    const Eigen::Vector3d body_zeros =
      robot.computeBodyVelocities(Eigen::Vector4d::Zero());

    assert_vector_equal(Eigen::Vector3d::Zero(), body_zeros, "Zero body");
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_constructor_invalid_params);
    RUN_TEST(test_pure_rotation);
    RUN_TEST(test_pure_translation_x);
    RUN_TEST(test_pure_translation_y);
    RUN_TEST(test_kinematics_round_trip);
    RUN_TEST(test_zero_input_zero_output);
    UNITY_END();
}

void loop() {}

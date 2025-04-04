#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <unity.h>

#include <cmath>
#include <stdexcept>

#include "kinematics/omnidirectional_robot.cpp"  // Include implementation for testing
#include "kinematics/omnidirectional_robot.h"

constexpr double EPSILON = 1e-6;  // Tolerance for floating-point comparisons

// Helper function for exception testing
bool throws_invalid_argument(double r, double d) {
    try {
        OmnidirectionalRobot robot(r, d);
        return false;
    }
    catch(const std::invalid_argument&) {
        return true;
    }
    catch(...) {
        return false;
    }
}

void test_constructor_validation() {
    TEST_ASSERT_TRUE(throws_invalid_argument(0, 0.1));    // Zero radius
    TEST_ASSERT_TRUE(throws_invalid_argument(-0.1, 0.1)); // Negative radius
    TEST_ASSERT_FALSE(throws_invalid_argument(0.1, 0.1)); // Valid parameters
}


void test_pure_rotation()
{
    OmnidirectionalRobot robot(0.1, 0.2);  // r=0.1m, d=0.2m
    const double omega_z = 1.0;            // 1 rad/s rotation

    Eigen::Vector4d wheel_vel = robot.computeWheelVelocities(omega_z, 0, 0);
    Eigen::Vector4d expected(-0.2 * 1.0 / 0.1, -0.2 * 1.0 / 0.1,
                             -0.2 * 1.0 / 0.1, -0.2 * 1.0 / 0.1);

    for (int i = 0; i < 4; i++)
    {
        TEST_ASSERT_FLOAT_WITHIN(EPSILON, expected[i], wheel_vel[i]);
    }
}

void test_pure_translation_x()
{
    OmnidirectionalRobot robot(0.1, 0.2);
    const double vx = 1.0;  // 1 m/s in x

    Eigen::Vector4d wheel_vel = robot.computeWheelVelocities(0, vx, 0);
    Eigen::Vector4d expected(sqrt(2) / 2 / 0.1,   // 45° wheel
                             -sqrt(2) / 2 / 0.1,  // 135° wheel
                             -sqrt(2) / 2 / 0.1,  // 225° wheel
                             sqrt(2) / 2 / 0.1    // 315° wheel
    );

    for (int i = 0; i < 4; i++)
    {
        TEST_ASSERT_FLOAT_WITHIN(EPSILON, expected[i], wheel_vel[i]);
    }
}

void test_pure_translation_y()
{
    OmnidirectionalRobot robot(0.1, 0.2);
    const double vy = 1.0;  // 1 m/s in y

    Eigen::Vector4d wheel_vel = robot.computeWheelVelocities(0, 0, vy);
    Eigen::Vector4d expected(sqrt(2) / 2 / 0.1,   // 45° wheel
                             sqrt(2) / 2 / 0.1,   // 135° wheel
                             -sqrt(2) / 2 / 0.1,  // 225° wheel
                             -sqrt(2) / 2 / 0.1   // 315° wheel
    );

    for (int i = 0; i < 4; i++)
    {
        TEST_ASSERT_FLOAT_WITHIN(EPSILON, expected[i], wheel_vel[i]);
    }
}

void test_inverse_kinematics()
{
    OmnidirectionalRobot robot(0.1, 0.2);

    // Test forward + inverse kinematics round trip
    Eigen::Vector3d original(0.5, 1.2, -0.8);  // ω, vx, vy
    Eigen::Vector4d wheel_vel =
      robot.computeWheelVelocities(original[0], original[1], original[2]);
    Eigen::Vector3d reconstructed = robot.computeBodyVelocities(wheel_vel);

    for (int i = 0; i < 3; i++)
    {
        TEST_ASSERT_FLOAT_WITHIN(EPSILON, original[i], reconstructed[i]);
    }
}

void test_zero_motion()
{
    OmnidirectionalRobot robot(0.1, 0.2);

    Eigen::Vector4d wheel_vel = robot.computeWheelVelocities(0, 0, 0);
    for (int i = 0; i < 4; i++)
    {
        TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.0, wheel_vel[i]);
    }

    Eigen::Vector3d body_vel =
      robot.computeBodyVelocities(Eigen::Vector4d::Zero());
    for (int i = 0; i < 3; i++)
    {
        TEST_ASSERT_FLOAT_WITHIN(EPSILON, 0.0, body_vel[i]);
    }
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_constructor_validation);
    RUN_TEST(test_pure_rotation);
    RUN_TEST(test_pure_translation_x);
    RUN_TEST(test_pure_translation_y);
    RUN_TEST(test_inverse_kinematics);
    RUN_TEST(test_zero_motion);
    UNITY_END();
}

void loop()
{
    // Not used in PlatformIO unit testing
}

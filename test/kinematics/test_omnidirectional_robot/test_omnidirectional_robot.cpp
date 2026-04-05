#include <unity.h>

#include <cmath>

#include "omni_robot.h"

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
template <typename T, size_t N>
void assert_vector_equal(const vt::generic_vector<T, N> & expected, const vt::generic_vector<T, N> & actual,
                         const char * context, double epsilon = config::test::EPSILON)
{
    TEST_ASSERT_EQUAL_MESSAGE(expected.size(), actual.size(), "Vector size mismatch");
    for (int i = 0; i < expected.size(); ++i)
    {
        char msg[128];
        snprintf(msg, sizeof(msg), "%s index %d (%.4f vs %.4f)", context, i, expected[i], actual[i]);

        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(epsilon, expected[i], actual[i], msg);
    }
}

void test_pure_rotation()
{
    OmnidirectionalRobot robot(config::test::RADIUS, config::test::WHEEL_DISTANCE);

    const vt::numeric_vector<4> expected = vt::numeric_vector<4>(config::test::ROTATION_FACTOR);

    const vt::numeric_vector<4> actual = robot.compute_wheel_velocities(vt::make_numeric_vector({1.0, 0.0, 0.0}));

    assert_vector_equal(expected, actual, "Pure rotation");
}

void test_pure_translation_x()
{
    OmnidirectionalRobot robot(config::test::RADIUS, config::test::WHEEL_DISTANCE);

    const vt::numeric_vector<4> expected =
      vt::make_numeric_vector<4>({config::test::TRANSLATION_FACTOR, -config::test::TRANSLATION_FACTOR,
                                  -config::test::TRANSLATION_FACTOR, config::test::TRANSLATION_FACTOR});

    const vt::numeric_vector<4> actual = robot.compute_wheel_velocities(vt::make_numeric_vector({0.0, 1.0, 0.0}));

    assert_vector_equal(expected, actual, "X translation");
}

void test_pure_translation_y()
{
    OmnidirectionalRobot robot(config::test::RADIUS, config::test::WHEEL_DISTANCE);

    const vt::numeric_vector<4> expected =
      vt::make_numeric_vector<4>({config::test::TRANSLATION_FACTOR, config::test::TRANSLATION_FACTOR,
                                  -config::test::TRANSLATION_FACTOR, -config::test::TRANSLATION_FACTOR});

    const vt::numeric_vector<4> actual = robot.compute_wheel_velocities(vt::make_numeric_vector({0.0, 0.0, 1.0}));

    assert_vector_equal(expected, actual, "Y translation");
}

void test_kinematics_round_trip()
{
    OmnidirectionalRobot robot(config::test::RADIUS, config::test::WHEEL_DISTANCE);

    const vt::numeric_vector<3> original = vt::make_numeric_vector<3>({0.5, 1.2, -0.8});

    const vt::numeric_vector<4> wheel_vel =
      robot.compute_wheel_velocities(vt::make_numeric_vector<3>({original[0], original[1], original[2]}));

    const vt::numeric_vector<3> reconstructed = robot.compute_body_velocities(wheel_vel);

    assert_vector_equal(original, reconstructed, "Round trip");
}

void test_zero_input_zero_output()
{
    OmnidirectionalRobot robot(config::test::RADIUS, config::test::WHEEL_DISTANCE);

    const vt::numeric_vector<4> wheel_zeros = robot.compute_wheel_velocities(vt::numeric_vector<3>::zeros());

    assert_vector_equal(vt::numeric_vector<4>::zeros(), wheel_zeros, "Zero wheels");

    const vt::numeric_vector<3> body_zeros = robot.compute_body_velocities(vt::numeric_vector<4>::zeros());

    assert_vector_equal(vt::numeric_vector<3>::zeros(), body_zeros, "Zero body");
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_pure_rotation);
    RUN_TEST(test_pure_translation_x);
    RUN_TEST(test_pure_translation_y);
    RUN_TEST(test_kinematics_round_trip);
    RUN_TEST(test_zero_input_zero_output);
    UNITY_END();
}

extern "C" void app_main(void)
{
    setup();
}

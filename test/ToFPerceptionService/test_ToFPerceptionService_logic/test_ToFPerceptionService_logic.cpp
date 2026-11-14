#include <unity.h>

#include "ToFPerceptionService.h"

void test_tof_frame_projects_to_local_perception_state()
{
    ToFPerceptionFrame frame = {};
    frame.valid = true;
    frame.timestamp_us = 99;
    frame.distance_mm[3] = 400;
    frame.target_status[3] = 5;
    frame.distance_mm[63] = 120;
    frame.target_status[63] = 5;

    const LocalPerceptionState state = ToFPerceptionService::project_frame(frame, 150);
    TEST_ASSERT_TRUE(state.valid);
    TEST_ASSERT_EQUAL_UINT8(VL53L5CX_RESOLUTION_8X8, state.resolution);
    TEST_ASSERT_EQUAL_INT16(120, state.nearest_distance_mm);
    TEST_ASSERT_EQUAL_UINT8(63, state.nearest_zone);
    TEST_ASSERT_TRUE(state.obstacle_close);
    TEST_ASSERT_TRUE(state.timestamp_us == 99);
}

void test_tof_rejects_failed_and_uncertain_target_statuses()
{
    ToFPerceptionFrame frame = {};
    frame.valid = true;
    frame.distance_mm[0] = 100;
    frame.distance_mm[3] = 400;
    frame.target_status[3] = 5;

    for (uint16_t status = 0; status <= 255; ++status)
    {
        frame.target_status[0] = static_cast<uint8_t>(status);
        const LocalPerceptionState state = ToFPerceptionService::project_frame(frame, 150);
        const bool fully_valid = status == 5;
        TEST_ASSERT_EQUAL(fully_valid, frame.has_valid_target(0));
        TEST_ASSERT_EQUAL_INT16(fully_valid ? 100 : 0, state.range_mm[0]);
        TEST_ASSERT_EQUAL_UINT8(status, state.target_status[0]);
        TEST_ASSERT_EQUAL_INT16(fully_valid ? 100 : 400, state.nearest_distance_mm);
        TEST_ASSERT_EQUAL(fully_valid, state.obstacle_close);
    }
}

void test_tof_rejects_nonpositive_distances_and_missing_frames()
{
    ToFPerceptionFrame frame = {};
    frame.valid = true;
    frame.target_status[0] = 5;
    TEST_ASSERT_FALSE(frame.has_valid_target(0));
    frame.distance_mm[0] = -1;
    TEST_ASSERT_FALSE(frame.has_valid_target(0));
    frame.distance_mm[0] = 100;
    frame.valid = false;
    TEST_ASSERT_FALSE(frame.has_valid_target(0));

    const LocalPerceptionState state = ToFPerceptionService::project_frame(frame, 150);
    TEST_ASSERT_FALSE(state.valid);
    TEST_ASSERT_EQUAL_INT16(0, state.range_mm[0]);
    TEST_ASSERT_EQUAL_INT16(-1, state.nearest_distance_mm);
    TEST_ASSERT_FALSE(state.obstacle_close);

    frame.valid = true;
    TEST_ASSERT_FALSE(frame.has_valid_target(ToFPerceptionFrame::MAX_TARGETS));
    frame.target_status[0] = 255;
    const LocalPerceptionState empty_state = ToFPerceptionService::project_frame(frame, 150);
    TEST_ASSERT_TRUE(empty_state.valid);
    TEST_ASSERT_EQUAL_INT16(-1, empty_state.nearest_distance_mm);
    TEST_ASSERT_FALSE(empty_state.obstacle_close);
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_tof_frame_projects_to_local_perception_state);
    RUN_TEST(test_tof_rejects_failed_and_uncertain_target_statuses);
    RUN_TEST(test_tof_rejects_nonpositive_distances_and_missing_frames);
    UNITY_END();
}

extern "C" void app_main(void)
{
    setup();
}

#include <unity.h>

#include "ToFPerceptionService.h"

void test_tof_frame_projects_to_local_perception_state()
{
    ToFPerceptionFrame frame = {};
    frame.valid = true;
    frame.resolution = VL53L5CX_RESOLUTION_4X4;
    frame.timestamp_us = 99;
    frame.distance_mm[3] = 400;
    frame.target_status[3] = 5;
    frame.distance_mm[7] = 120;
    frame.target_status[7] = 5;

    const LocalPerceptionState state = ToFPerceptionService::project_frame(frame, 150);
    TEST_ASSERT_TRUE(state.valid);
    TEST_ASSERT_EQUAL_INT16(120, state.nearest_distance_mm);
    TEST_ASSERT_EQUAL_UINT8(7, state.nearest_zone);
    TEST_ASSERT_TRUE(state.obstacle_close);
    TEST_ASSERT_TRUE(state.timestamp_us == 99);
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_tof_frame_projects_to_local_perception_state);
    UNITY_END();
}

extern "C" void app_main(void)
{
    setup();
}

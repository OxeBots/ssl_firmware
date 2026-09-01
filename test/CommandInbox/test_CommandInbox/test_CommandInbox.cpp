#include <unity.h>

#include "CommandInbox.h"

static RobotCommand make_command(int16_t x_mm, int16_t y_mm, uint8_t kick_velocity = 0)
{
    RobotCommand command = {};
    command.target_pose.x = x_mm;
    command.target_pose.y = y_mm;
    command.target_pose.angle = 9000;
    command.target_pose.x_v = 1000;
    command.target_pose.y_v = 0;
    command.target_pose.angular_vel = 0;
    command.kick_velocity = kick_velocity;
    return command;
}

void test_command_inbox_preserves_fifo_and_timestamps()
{
    CommandInbox inbox;
    TEST_ASSERT_TRUE(inbox.init());

    RobotCommand first = make_command(100, 0);
    RobotCommand second = make_command(200, 0);
    TEST_ASSERT_TRUE(inbox.push(first, 1000));
    TEST_ASSERT_TRUE(inbox.push(second, 2000));
    TEST_ASSERT_EQUAL_UINT8(2, inbox.size());

    CommandEnvelope envelope = {};
    TEST_ASSERT_TRUE(inbox.pop(&envelope));
    TEST_ASSERT_EQUAL_INT16(100, envelope.command.target_pose.x);
    TEST_ASSERT_EQUAL_UINT32(1, envelope.sequence);
    TEST_ASSERT_TRUE(envelope.received_at_us == 1000);
    TEST_ASSERT_TRUE(inbox.pop(&envelope));
    TEST_ASSERT_EQUAL_INT16(200, envelope.command.target_pose.x);
    TEST_ASSERT_EQUAL_UINT32(2, envelope.sequence);
    TEST_ASSERT_FALSE(inbox.pop(&envelope));
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_command_inbox_preserves_fifo_and_timestamps);
    UNITY_END();
}

extern "C" void app_main(void)
{
    setup();
}

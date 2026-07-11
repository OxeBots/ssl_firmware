#include <BL48250.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>
#include <stdio.h>
#include <unity.h>

#include <array>

namespace config
{
namespace test
{
constexpr std::array<gpio_num_t, 4> MOTOR_PWM_PINS = {(gpio_num_t)CONFIG_MOTOR_FL_PWM_GPIO,
                                                      (gpio_num_t)CONFIG_MOTOR_BL_PWM_GPIO,
                                                      (gpio_num_t)CONFIG_MOTOR_BR_PWM_GPIO,
                                                      (gpio_num_t)CONFIG_MOTOR_FR_PWM_GPIO};

constexpr std::array<gpio_num_t, 4> MOTOR_DIR_PINS = {(gpio_num_t)CONFIG_MOTOR_FL_DIR_GPIO,
                                                      (gpio_num_t)CONFIG_MOTOR_BL_DIR_GPIO,
                                                      (gpio_num_t)CONFIG_MOTOR_BR_DIR_GPIO,
                                                      (gpio_num_t)CONFIG_MOTOR_FR_DIR_GPIO};

constexpr std::array<ledc_channel_t, 4> MOTOR_CHANNELS = {
  LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3, LEDC_CHANNEL_4};

constexpr ledc_timer_bit_t DUTY_RESOLUTION = LEDC_TIMER_10_BIT;
constexpr uint32_t PWM_FREQ = 5000;
constexpr uint32_t MAX_DUTY = (1 << static_cast<int>(DUTY_RESOLUTION)) - 1;
}  // namespace test
}  // namespace config

void setUp(void)
{
    config::driver::MotorDriverConfig motor_cfg;
    motor_cfg.timer = LEDC_TIMER_0;
    motor_cfg.speed_mode = LEDC_LOW_SPEED_MODE;
    motor_cfg.duty_resolution = config::test::DUTY_RESOLUTION;
    motor_cfg.pwm_freq = config::test::PWM_FREQ;
    motor_cfg.motor_pwm_pins = config::test::MOTOR_PWM_PINS;
    motor_cfg.motor_dir_pins = config::test::MOTOR_DIR_PINS;
    motor_cfg.motor_channels = config::test::MOTOR_CHANNELS;
    BL48250::get_instance().configure(motor_cfg);
}

void tearDown(void)
{
    BL48250::get_instance().deinit();
}

void init_uart()
{
    // Configure UART parameters if needed and install the driver
    // In many cases, UART0 is already configured by the console, but the driver might not be
    // installed. We install it to use uart_read_bytes safely.
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
}

char get_serial_char()
{
    uint8_t c = 0;
    while (true)
    {
        int len = uart_read_bytes(UART_NUM_0, &c, 1, 20 / portTICK_PERIOD_MS);
        if (len > 0)
        {
            return (char)c;
        }
    }
}

void flush_serial()
{
    uint8_t c;
    while (uart_read_bytes(UART_NUM_0, &c, 1, 10 / portTICK_PERIOD_MS) > 0)
    {
        // Discard
    }
}

char wait_for_user_input()
{
    while (true)
    {
        char c = get_serial_char();
        if (c == 'y' || c == 'Y' || c == 'n' || c == 'N')
        {
            flush_serial();
            return c;
        }
    }
}

void wait_for_enter()
{
    while (true)
    {
        char c = get_serial_char();
        if (c == '\n' || c == '\r')
        {
            flush_serial();
            return;
        }
    }
}

void test_interactive_motor(size_t motor_idx)
{
    const char * motor_names[] = {
      "Front Left (M1)", "Back Left (M2)", "Back Right (M3)", "Front Right (M4)"};
    const uint32_t test_duty = config::test::MAX_DUTY * 0.2;  // 20% speed

    printf("\n=== Testing %s ===\n", motor_names[motor_idx]);

    // Test CW
    printf("Press ENTER to start moving CW...\n");
    wait_for_enter();

    std::array<uint32_t, 4> duties = {0, 0, 0, 0};
    std::array<uint8_t, 4> dirs = {config::driver::MOTOR_CW,
                                   config::driver::MOTOR_CW,
                                   config::driver::MOTOR_CW,
                                   config::driver::MOTOR_CW};

    duties[motor_idx] = test_duty;
    dirs[motor_idx] = config::driver::MOTOR_CW;
    BL48250::get_instance().set_duties(duties, dirs);

    printf("Is the motor moving CW? (y/n)\n");
    char response = wait_for_user_input();

    // Stop motor
    BL48250::get_instance().set_duties({0, 0, 0, 0}, dirs);

    TEST_ASSERT_MESSAGE(response == 'y' || response == 'Y', "User reported CW movement failed");

    vTaskDelay(500 / portTICK_PERIOD_MS);  // Wait before next movement

    // Test CCW
    printf("Press ENTER to start moving CCW...\n");
    wait_for_enter();

    dirs[motor_idx] = config::driver::MOTOR_CCW;
    BL48250::get_instance().set_duties(duties, dirs);

    printf("Is the motor moving CCW? (y/n)\n");
    response = wait_for_user_input();

    // Stop motor
    BL48250::get_instance().set_duties({0, 0, 0, 0}, dirs);

    TEST_ASSERT_MESSAGE(response == 'y' || response == 'Y', "User reported CCW movement failed");

    printf("Motor %s test passed!\n", motor_names[motor_idx]);
}

void test_motor_0()
{
    test_interactive_motor(0);
}
void test_motor_1()
{
    test_interactive_motor(1);
}
void test_motor_2()
{
    test_interactive_motor(2);
}
void test_motor_3()
{
    test_interactive_motor(3);
}

void test_all_motors_cw()
{
    const uint32_t test_duty = config::test::MAX_DUTY * 0.01;  // 15% speed for safety with 4 motors
    printf("\n=== Testing ALL Motors CW ===\n");
    printf("Press ENTER to start moving ALL motors CW...\n");
    wait_for_enter();

    std::array<uint32_t, 4> duties = {test_duty, test_duty, test_duty, test_duty};
    std::array<uint8_t, 4> dirs = {config::driver::MOTOR_CW,
                                   config::driver::MOTOR_CW,
                                   config::driver::MOTOR_CW,
                                   config::driver::MOTOR_CW};

    BL48250::get_instance().set_duties(duties, dirs);
    printf("Are ALL motors moving CW? (y/n)\n");
    char response = wait_for_user_input();
    BL48250::get_instance().set_duties({0, 0, 0, 0}, dirs);
    TEST_ASSERT_MESSAGE(response == 'y' || response == 'Y',
                        "User reported ALL motors CW movement failed");
}

void test_all_motors_ccw()
{
    const uint32_t test_duty = config::test::MAX_DUTY * 0.15;  // 15% speed
    printf("\n=== Testing ALL Motors CCW ===\n");
    printf("Press ENTER to start moving ALL motors CCW...\n");
    wait_for_enter();

    std::array<uint32_t, 4> duties = {test_duty, test_duty, test_duty, test_duty};
    std::array<uint8_t, 4> dirs = {config::driver::MOTOR_CCW,
                                   config::driver::MOTOR_CCW,
                                   config::driver::MOTOR_CCW,
                                   config::driver::MOTOR_CCW};

    BL48250::get_instance().set_duties(duties, dirs);
    printf("Are ALL motors moving CCW? (y/n)\n");
    char response = wait_for_user_input();
    BL48250::get_instance().set_duties({0, 0, 0, 0}, dirs);
    TEST_ASSERT_MESSAGE(response == 'y' || response == 'Y',
                        "User reported ALL motors CCW movement failed");
}

extern "C" void app_main(void)
{
    // Initialize UART to read from serial
    init_uart();

    // A small delay to allow serial monitor to connect
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    printf("\n\n=======================================================\n");
    printf("    BL48250 Interactive Motor Hardware Test\n");
    printf("=======================================================\n");
    printf("WARNING: This test will move the motors.\n");
    printf("Please make sure the robot is elevated or safe to move.\n");
    printf("Press ENTER to begin tests...\n");

    flush_serial();
    wait_for_enter();

    UNITY_BEGIN();
    RUN_TEST(test_motor_0);
    RUN_TEST(test_motor_1);
    RUN_TEST(test_motor_2);
    RUN_TEST(test_motor_3);
    RUN_TEST(test_all_motors_cw);
    RUN_TEST(test_all_motors_ccw);
    UNITY_END();
}

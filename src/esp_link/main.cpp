#include <driver/uart.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <sdkconfig.h>

#include <cstring>

#include "ESP_NOW_Dev.h"
#include "SerialDebugCodec.h"
#include "SslRadioProtocol.h"

static constexpr uart_port_t HOST_UART = UART_NUM_0;
static constexpr uint8_t SERIAL_FRAME_TYPE = 0xA5;
static constexpr uint8_t LOOPBACK_FRAME_TYPE = 0xA6;
static constexpr uint8_t BROADCAST_FRAME_TYPE = 0xA7;
static constexpr uint64_t PEER_TIMEOUT_US = 1000000;
static constexpr uint16_t IDLE_RX_INTERVAL_MS = 100;
static constexpr uint16_t IDLE_RX_WINDOW_MS = 20;
static constexpr int64_t RADIO_ACTIVE_HOLD_US = 150000;
static constexpr char READY_NOTICE[] = "\0ESP_LINK ready\n";

struct RobotPeer
{
    uint8_t address[ESP_NOW_ETH_ALEN];
    uint64_t seen_at_us;
};

static RobotPeer robot_peers[ROBOT_ID_MAX + 1] = {};
static portMUX_TYPE peer_lock = portMUX_INITIALIZER_UNLOCKED;

static void write_telemetry(uint8_t frame_type, const uint8_t * payload)
{
    // Leading delimiter lets the host discard any boot output before this frame.
    uint8_t frame[1 + BYTES_LENGTH_ROBOT_TELEMETRY + 4] = {};
    const size_t written = serial_debug::encode_frame(
      frame_type, payload, BYTES_LENGTH_ROBOT_TELEMETRY, frame + 1, sizeof(frame) - 1);
    if (written != 0)
        uart_write_bytes(HOST_UART, frame, written + 1);
}

static void forward_telemetry(const uint8_t * payload,
                              uint8_t length,
                              const uint8_t * source,
                              uint64_t received_at_us)
{
    if (length != BYTES_LENGTH_ROBOT_TELEMETRY || (payload[0] & 0x0F) != MSG_TYPE_TELEMETRY)
        return;
    const uint8_t robot_id = payload[0] >> 4;
    if (robot_id > ROBOT_ID_MAX)
        return;

    portENTER_CRITICAL(&peer_lock);
    RobotPeer & peer = robot_peers[robot_id];
    memcpy(peer.address, source, sizeof(peer.address));
    peer.seen_at_us = received_at_us;
    portEXIT_CRITICAL(&peer_lock);
    write_telemetry(SERIAL_FRAME_TYPE, payload);
}

static void forward_downlink(uint8_t frame_type, const uint8_t * payload, size_t length)
{
    auto & radio = ESP_NOW_Dev::get_instance();
    const uint8_t robot_id = payload[0] >> 4;
    const uint8_t message_type = payload[0] & 0x0F;
    if (frame_type == SERIAL_FRAME_TYPE && message_type != MSG_TYPE_TEAM_VEL_CMD &&
        robot_id <= ROBOT_ID_MAX)
    {
        portENTER_CRITICAL(&peer_lock);
        const RobotPeer peer = robot_peers[robot_id];
        portEXIT_CRITICAL(&peer_lock);
        const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
        if (peer.seen_at_us && now - peer.seen_at_us < PEER_TIMEOUT_US &&
            radio.send(peer.address, payload, length) == ESP_OK)
            return;
    }
    radio.send(ESP_NOW_Dev::BROADCAST, payload, length);
}

static void handle_serial_frame(const serial_debug::Decoder & decoder,
                                int64_t & radio_awake_until_us)
{
    const uint8_t frame_type = decoder.message_type();
    const uint8_t * payload = decoder.message();
    const size_t length = decoder.message_length();
    if (!ssl_radio::valid_downlink(payload, length))
        return;

    if (frame_type == SERIAL_FRAME_TYPE || frame_type == BROADCAST_FRAME_TYPE)
    {
        if (radio_awake_until_us == 0)
            ESP_ERROR_CHECK(esp_wifi_force_wakeup_acquire());
        forward_downlink(frame_type, payload, length);
        radio_awake_until_us = esp_timer_get_time() + RADIO_ACTIVE_HOLD_US;
    }
    else if (frame_type == LOOPBACK_FRAME_TYPE && (payload[0] & 0x0F) == MSG_TYPE_TELEMETRY_REQ)
    {
        // Match request and response sizes used by the radio path.
        uint8_t response[BYTES_LENGTH_ROBOT_TELEMETRY] = {};
        memcpy(response, payload, BYTES_LENGTH_HEADER);
        response[0] = (response[0] & 0xF0) | MSG_TYPE_TELEMETRY;
        write_telemetry(LOOPBACK_FRAME_TYPE, response);
    }
}

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    auto & radio = ESP_NOW_Dev::get_instance();
    ESP_ERROR_CHECK(radio.init(CONFIG_ESP_NOW_CHANNEL));

    uart_config_t uart = {};
    uart.baud_rate = BAUD_RATE;
    uart.data_bits = UART_DATA_8_BITS;
    uart.parity = UART_PARITY_DISABLE;
    uart.stop_bits = UART_STOP_BITS_1;
    uart.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart.source_clk = UART_SCLK_DEFAULT;
    ESP_ERROR_CHECK(uart_param_config(HOST_UART, &uart));
    ESP_ERROR_CHECK(uart_driver_install(HOST_UART, 1024, 0, 0, nullptr, 0));
    // Release short host frames after two idle symbols instead of the default ten.
    ESP_ERROR_CHECK(uart_set_rx_timeout(HOST_UART, 2));
    ESP_LOGI("ESP_LINK", "Ready on channel %d at %d baud", CONFIG_ESP_NOW_CHANNEL, BAUD_RATE);
    esp_log_level_set("*", ESP_LOG_NONE);
    ESP_ERROR_CHECK(radio.start(false, forward_telemetry));
    ESP_ERROR_CHECK(esp_now_set_wake_window(IDLE_RX_WINDOW_MS));
    ESP_ERROR_CHECK(esp_wifi_connectionless_module_set_wake_interval(IDLE_RX_INTERVAL_MS));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
    // The ROM starts at a fixed baud rate. Report readiness at the link rate.
    uart_write_bytes(HOST_UART, READY_NOTICE, sizeof(READY_NOTICE) - 1);

    serial_debug::Decoder decoder;
    uint8_t buffer[128];
    int64_t radio_awake_until_us = 0;
    while (true)
    {
        TickType_t timeout = portMAX_DELAY;
        if (radio_awake_until_us != 0)
        {
            const int64_t remaining_us = radio_awake_until_us - esp_timer_get_time();
            if (remaining_us <= 0)
            {
                ESP_ERROR_CHECK(esp_wifi_force_wakeup_release());
                radio_awake_until_us = 0;
            }
            else
                timeout = pdMS_TO_TICKS((remaining_us + 999) / 1000) + 1;
        }

        if (uart_read_bytes(HOST_UART, buffer, 1, timeout) != 1)
            continue;
        const int count = 1 + uart_read_bytes(HOST_UART, buffer + 1, sizeof(buffer) - 1, 0);
        for (int i = 0; i < count; ++i)
        {
            decoder.feed(buffer[i]);
            if (!decoder.has_frame())
                continue;
            handle_serial_frame(decoder, radio_awake_until_us);
            decoder.pop();
        }
    }
}

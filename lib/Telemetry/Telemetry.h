#ifndef _TELEMETRY_H_
#define _TELEMETRY_H_

#include <esp_err.h>
#include <esp_log.h>
#include <stdint.h>

class ESP_NOW_Dev;

class Telemetry
{
   public:
    static Telemetry & get_instance();

    void init(ESP_NOW_Dev * radio);

    void send(uint32_t echo_timestamp, const uint8_t * destination);

   private:
    Telemetry() = default;
    ~Telemetry() = default;
    Telemetry(const Telemetry &) = delete;
    Telemetry & operator=(const Telemetry &) = delete;

    ESP_NOW_Dev * m_radio = nullptr;

    // TODO: Replace the battery stub when battery percentage sensing is available.
    static constexpr uint8_t STUB_BATTERY_PCT = 95;
};

#endif  // _TELEMETRY_H_

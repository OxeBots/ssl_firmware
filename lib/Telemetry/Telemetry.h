#ifndef TELEMETRY_SERVICE_H
#define TELEMETRY_SERVICE_H

#include <esp_err.h>
#include <esp_log.h>
#include <stdint.h>

#include "NRF24L01.h"

class Telemetry
{
   public:
    static Telemetry & get_instance();

    void init(NRF24L01 * radio);

    void send(uint32_t echo_timestamp);

   private:
    Telemetry() = default;
    ~Telemetry() = default;
    Telemetry(const Telemetry &) = delete;
    Telemetry & operator=(const Telemetry &) = delete;

    NRF24L01 * m_radio = nullptr;

    // TODO: Stub values — replace when real battery/kicker sensing is wired.
    static constexpr uint8_t STUB_BATTERY_PCT = 95;
    static constexpr uint16_t STUB_KICKER_VOLTAGE = 1650;
};

#endif  // TELEMETRY_SERVICE_H

#ifndef TELEMETRY_SERVICE_H
#define TELEMETRY_SERVICE_H

#include <esp_err.h>
#include <esp_log.h>
#include <stdint.h>

#include "NRF24L01.h"

class TelemetryService
{
   public:
    static TelemetryService & get_instance();

    void init(NRF24L01 * radio);

    void send(uint32_t echo_timestamp);

   private:
    TelemetryService() = default;
    ~TelemetryService() = default;
    TelemetryService(const TelemetryService &) = delete;
    TelemetryService & operator=(const TelemetryService &) = delete;

    NRF24L01 * m_radio = nullptr;

    // TODO: Stub values — replace when real battery/kicker sensing is wired.
    static constexpr uint8_t STUB_BATTERY_PCT = 95;
    static constexpr uint16_t STUB_KICKER_VOLTAGE = 1650;
};

#endif  // TELEMETRY_SERVICE_H

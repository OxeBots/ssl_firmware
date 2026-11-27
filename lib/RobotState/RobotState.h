#ifndef _ROBOT_STATE_H_
#define _ROBOT_STATE_H_

#include <esp_err.h>
#include <esp_log.h>
#include <stdint.h>

class RobotState
{
   public:
    static RobotState & get_instance();

    esp_err_t init();

    uint8_t get_id() const;
    void set_id(uint8_t id);

    uint8_t get_radio_channel() const;
    uint8_t get_radio_tx_channel() const;
    void set_radio_channel(uint8_t channel);
    void set_radio_channels(uint8_t rx_channel, uint8_t tx_channel);

    void set_error(uint16_t flag);
    void clear_error(uint16_t flag);
    uint16_t get_error_flags() const;

   private:
    RobotState() = default;
    ~RobotState() = default;
    RobotState(const RobotState &) = delete;
    RobotState & operator=(const RobotState &) = delete;

    uint8_t m_robot_id = 0;
    uint8_t m_radio_channel = 76;
    uint8_t m_radio_tx_channel = 76;
    uint16_t m_error_flags = 0;

    static constexpr const char * NVS_NS = "robot_cfg";
    static constexpr const char * NVS_KEY = "robot_id";
    static constexpr const char * NVS_KEY_CHANNEL = "rf_ch";
    static constexpr const char * NVS_KEY_TX_CHANNEL = "rf_tx_ch";
};

#endif  // _ROBOT_STATE_H_

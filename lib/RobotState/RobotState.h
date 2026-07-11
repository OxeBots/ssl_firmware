#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

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

    void set_error(uint16_t flag);

    void clear_error(uint16_t flag);

    uint16_t get_error_flags() const;

   private:
    RobotState() = default;
    ~RobotState() = default;
    RobotState(const RobotState &) = delete;
    RobotState & operator=(const RobotState &) = delete;

    uint8_t m_robot_id = 0;
    uint16_t m_error_flags = 0;

    static constexpr const char * NVS_NS = "robot_cfg";
    static constexpr const char * NVS_KEY = "robot_id";
};

#endif  // ROBOT_STATE_H

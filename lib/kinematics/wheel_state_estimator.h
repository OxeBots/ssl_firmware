#ifndef _WHEEL_STATE_ESTIMATOR_H_
#define _WHEEL_STATE_ESTIMATOR_H_

#include <esp_adc/adc_continuous.h>
#include <esp_attr.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <array>

#include "AS5600.h"
#include "CD4052B.h"
#include "encoder_channel.h"

static constexpr adc_unit_t ADC_UNIT = ADC_UNIT_1;
static constexpr uint8_t NUM_ENC_CHANNELS = 4;
static constexpr int MIN_VALID_SWING_MV = 2000;

struct AS5600Settings
{
    AS5600::OutputStage output_stage =
      AS5600::OutputStage::ANALOG_REDUCED;  // 0.1 VCC to 0.9 VCC to be in the ESP ADC linear
                                            // response range
    AS5600::SlowFilter slow_filter =
      AS5600::SlowFilter::FILTER_2X;  // Fast response to slow changes, which is good for wheel
                                      // encoders
    AS5600::FastFilter fast_filter =
      AS5600::FastFilter::THRESH_6LSB;  // Fast filter to catch sudden spikes,
                                        // which can happen with noisy readings
    bool burn_settings = false;         // Dangerous! Keep false by default
};

class WheelStateEstimator
{
   public:
    static WheelStateEstimator & get_instance();
    WheelStateEstimator(const WheelStateEstimator &) = delete;
    WheelStateEstimator & operator=(const WheelStateEstimator &) = delete;

    // Lifecycle. Idempotent: returns ESP_OK when the estimator is already
    // running. Loads the persisted motor/encoder wiring map before creating
    // the encoder channels.
    esp_err_t init(const std::array<adc_channel_t, NUM_ENC_CHANNELS> & channels,
                   adc_atten_t attenuation = ADC_ATTEN_DB_12,
                   CD4052B * cd4052b = nullptr);

    void set_cd4052b(CD4052B * cd4052b);

    void suspend();
    void resume();

    // Analog range calibration loaded from NVS (non-blocking).
    esp_err_t load_calibration();

    // Encoder range calibration support: track the analog min/max voltage of
    // every encoder while the wheels are spun, then validate and persist it.
    void start_range_tracking();
    // Disable tracking without saving the tracked values (aborted run).
    void stop_range_tracking();
    // Disable tracking, validate every encoder swing and save the valid ranges
    // to NVS. Returns a bitmask of the analog input slots with a valid range.
    uint8_t finish_range_tracking();

    bool is_channel_calibrated(size_t wheel) const;

    // I2C configuration (advanced)
    esp_err_t configure_encoder(uint8_t channel_idx,
                                const AS5600Settings & settings = AS5600Settings());

    // -- Motor/encoder wiring map ---------------------------------------
    // The encoder harness is not keyed: the encoder of a motor can reach any
    // analog input and any CD4052B channel. analog_map[wheel] is the analog
    // input slot (index into the channel array given to init(), slot 0 =
    // ENC1) and mux_map[wheel] the CD4052B channel that reaches the same
    // encoder over I2C.
    std::array<uint8_t, NUM_ENC_CHANNELS> get_analog_mapping() const;
    // Reports whether the returned mux map was calibrated and stored in NVS.
    std::array<uint8_t, NUM_ENC_CHANNELS> get_mux_mapping(bool * calibrated = nullptr) const;
    // Apply a permutation pair, rebuild the encoder routing and persist both
    // maps in NVS. The caller must have verified both maps are permutations.
    esp_err_t apply_mapping(const std::array<uint8_t, NUM_ENC_CHANNELS> & analog_map,
                            const std::array<uint8_t, NUM_ENC_CHANNELS> & mux_map);

    // -- Wiring calibration support -------------------------------------
    // Latest raw ADC block mean of every analog input slot (init order).
    std::array<uint16_t, NUM_ENC_CHANNELS> get_raw_values() const;
    // Reads each mux input and optionally its magnetic field status.
    esp_err_t read_i2c_raw_angles(
      std::array<uint16_t, NUM_ENC_CHANNELS> * angles,
      std::array<EncoderChannel::MagnetField, NUM_ENC_CHANNELS> * magnet_fields = nullptr);
    // Raw I2C angle (0..4095) of one wheel's encoder: selects the wheel's mux
    // channel from the wiring map, reads its AS5600 and disables the mux again.
    esp_err_t read_wheel_i2c_raw_angle(uint8_t wheel, uint16_t * angle);
    // Anchors the applied raw ADC range of the encoder mapped to one wheel
    // (calibration ADC-I2C alignment). Fails when the range is implausible.
    esp_err_t set_wheel_raw_range(uint8_t wheel, uint16_t min_raw, uint16_t max_raw);

    // -- Encoder output stage -------------------------------------------
    // Applies the AS5600 analog output stage to every wheel: selects its
    // CD4052B channel, reads the configuration, and writes and verifies the
    // new stage when it differs. The slow/fast filters are left untouched and
    // no OTP is burned. Consumers that display the stage read it from the
    // AS5600 (read_output_stage) instead of caching it here.
    esp_err_t configure_encoder_output_stage(AS5600::OutputStage stage);
    // Stored raw ADC endpoints of the encoder mapped to every wheel (0 when the
    // stored range is absent or invalid).
    std::array<uint16_t, NUM_ENC_CHANNELS> get_range_raw_min() const;
    std::array<uint16_t, NUM_ENC_CHANNELS> get_range_raw_max() const;

    // Data accessors
    std::array<float, NUM_ENC_CHANNELS> get_filtered_angle_rad();
    std::array<float, NUM_ENC_CHANNELS> get_filtered_velocity_rad_s();
    // Smoothed velocity for the wheel-level control loops: the pulse-window
    // velocity is quantized in one-count steps (20 RPM of wheel speed at low
    // speed), which the EKF alone still shows as a sawtooth. Kinematics and
    // radio control keep using the unsmoothed estimate.
    std::array<float, NUM_ENC_CHANNELS> get_smoothed_velocity_rad_s();
    std::array<float, NUM_ENC_CHANNELS> get_measured_angle_rad();

    // Latest published values and the producer timestamp of each wheel.
    struct ControlSnapshot
    {
        std::array<float, NUM_ENC_CHANNELS> smoothed_velocity_rad_s = {};
        std::array<float, NUM_ENC_CHANNELS> measured_angle_rad = {};
        std::array<float, NUM_ENC_CHANNELS> fast_velocity_rad_s = {};
        uint8_t smoothed_valid_mask = 0;
        uint8_t fast_valid_mask = 0;
        uint8_t stuck_analog_mask = 0;
        uint8_t clamped_raw_mask = 0;
        std::array<int64_t, NUM_ENC_CHANNELS> sample_us = {};
    };
    bool get_control_snapshot(ControlSnapshot * snapshot) const;

   private:
    WheelStateEstimator();

    // Called from the acquisition task.
    void IRAM_ATTR on_sample(adc_channel_t channel, uint16_t raw);
    static void IRAM_ATTR s_on_sample(void * pvParameters);
    void IRAM_ATTR publish_control_sample(size_t wheel,
                                          const EncoderChannel & state,
                                          int64_t now_us);

    // Loads the persisted wiring map; identity when NVS has none.
    void load_mapping();
    // Creates the encoder of every wheel for the current wiring map.
    void rebuild_encoders();

    // Config helpers
    esp_err_t apply_i2c_settings(AS5600 * enc, const AS5600Settings & settings);
    esp_err_t verify_i2c_settings(AS5600 * enc, const AS5600Settings & settings);
    esp_err_t process_otp_burn(AS5600 * enc, const AS5600Settings & settings);

    // State variables
    bool m_initialized = false;
    bool m_calibrated = false;
    bool m_mapping_valid = false;

    // Analog input slots in init order: slot 0 = ENC1 ... slot 3 = ENC4.
    std::array<adc_channel_t, NUM_ENC_CHANNELS> m_channels = {};
    // One encoder channel per wheel, bound to the slot from analog_map.
    std::array<EncoderChannel, NUM_ENC_CHANNELS> m_enc_channels;
    std::array<EncoderChannel *, SOC_ADC_CHANNEL_NUM(ADC_UNIT)> m_channel_lookup;
    // Wiring map and its inverse (slot -> wheel).
    std::array<uint8_t, NUM_ENC_CHANNELS> m_analog_map = {0, 1, 2, 3};
    std::array<uint8_t, NUM_ENC_CHANNELS> m_mux_map = {0, 1, 2, 3};
    std::array<uint8_t, NUM_ENC_CHANNELS> m_slot_wheel = {0, 1, 2, 3};

    SemaphoreHandle_t m_data_mutex;
    mutable portMUX_TYPE m_control_mux = portMUX_INITIALIZER_UNLOCKED;
    ControlSnapshot m_published_control = {};
    CD4052B * m_cd4052b = nullptr;
};

#endif  // _WHEEL_STATE_ESTIMATOR_H_

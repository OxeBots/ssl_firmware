#include "wheel_state_estimator.h"

#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <sdkconfig.h>

#include "AdcStream.h"
#include "NVSManager.h"

static const char * TAG = "WheelStateEstimator";

// NVS namespace and keys of the persisted motor/encoder wiring map.
static constexpr const char * MAPPING_NVS_NS = "wheel_map";
static constexpr const char * MAPPING_ANALOG_KEY = "analog";
static constexpr const char * MAPPING_MUX_KEY = "mux";
static constexpr const char * MAPPING_VALID_KEY = "valid";

static const char * WHEEL_NAMES[NUM_ENC_CHANNELS] = {"M1", "M2", "M3", "M4"};

/**
 * @brief Checks that a wheel -> input map visits every input exactly once.
 */
static bool is_permutation(const std::array<uint8_t, NUM_ENC_CHANNELS> & map)
{
    uint8_t seen = 0;
    for (const uint8_t value : map)
    {
        if (value >= NUM_ENC_CHANNELS)
            return false;
        const uint8_t bit = static_cast<uint8_t>(1U << value);
        if ((seen & bit) != 0U)
            return false;
        seen |= bit;
    }
    return seen == 0x0FU;
}

WheelStateEstimator & WheelStateEstimator::get_instance()
{
    static WheelStateEstimator instance;
    return instance;
}

WheelStateEstimator::WheelStateEstimator() : m_data_mutex(xSemaphoreCreateMutex())
{
    m_channel_lookup.fill(nullptr);
}

/**
 * @brief Registers the encoder channels with the shared ADC stream.
 *
 * Loads the persisted motor/encoder wiring map first, then creates one AS5600
 * channel per wheel for the analog input that map assigns to it.
 *
 * @param channels ADC channels of the four encoder inputs, slot order ENC1..4.
 * @param attenuation The ADC attenuation setting.
 * @param cd4052b Analog mux used to reach the encoder I2C ports, may be null.
 * @return ESP_OK on success, or the ADC stream error.
 */
esp_err_t WheelStateEstimator::init(const std::array<adc_channel_t, NUM_ENC_CHANNELS> & channels,
                                    adc_atten_t attenuation,
                                    CD4052B * cd4052b)
{
    if (m_initialized)
        return ESP_OK;

    m_cd4052b = cd4052b;
    m_channels = channels;

    load_mapping();

    AdcStream & adc = AdcStream::get_instance();
    const esp_err_t adc_result = adc.init(channels, attenuation);
    if (adc_result != ESP_OK)
        return adc_result;

    for (const adc_channel_t channel : channels)
    {
        if (static_cast<int>(channel) >= SOC_ADC_CHANNEL_NUM(ADC_UNIT))
        {
            ESP_LOGW(TAG, "Skipping invalid ADC channel: %d", channel);
            continue;
        }
        adc.set_sample_handler(channel, &WheelStateEstimator::s_on_sample, this);
    }

    xSemaphoreTake(m_data_mutex, portMAX_DELAY);
    rebuild_encoders();
    xSemaphoreGive(m_data_mutex);
    m_initialized = true;

    ESP_LOGI(TAG,
             "Encoder routing: M1 analog=%u mux=%u | M2 analog=%u mux=%u | "
             "M3 analog=%u mux=%u | M4 analog=%u mux=%u",
             static_cast<unsigned>(m_analog_map[0] + 1),
             static_cast<unsigned>(m_mux_map[0] + 1),
             static_cast<unsigned>(m_analog_map[1] + 1),
             static_cast<unsigned>(m_mux_map[1] + 1),
             static_cast<unsigned>(m_analog_map[2] + 1),
             static_cast<unsigned>(m_mux_map[2] + 1),
             static_cast<unsigned>(m_analog_map[3] + 1),
             static_cast<unsigned>(m_mux_map[3] + 1));

    return ESP_OK;
}

void WheelStateEstimator::set_cd4052b(CD4052B * cd4052b)
{
    m_cd4052b = cd4052b;
}

/**
 * @brief Loads the motor/encoder wiring map from NVS.
 *
 * Falls back to the identity map (wheel i reads input i) when NVS holds no
 * map or a malformed one; the map only counts as calibrated when the valid
 * flag was stored by a calibration run.
 */
void WheelStateEstimator::load_mapping()
{
    m_analog_map = {0, 1, 2, 3};
    m_mux_map = {0, 1, 2, 3};
    m_slot_wheel = {0, 1, 2, 3};
    m_mapping_valid = false;

    std::array<uint8_t, NUM_ENC_CHANNELS> analog = {};
    std::array<uint8_t, NUM_ENC_CHANNELS> mux = {};
    size_t analog_len = sizeof(analog);
    size_t mux_len = sizeof(mux);
    const bool analog_ok =
      NVSManager::load_blob(MAPPING_NVS_NS, MAPPING_ANALOG_KEY, analog.data(), &analog_len) ==
        ESP_OK &&
      analog_len == sizeof(analog);
    const bool mux_ok =
      NVSManager::load_blob(MAPPING_NVS_NS, MAPPING_MUX_KEY, mux.data(), &mux_len) == ESP_OK &&
      mux_len == sizeof(mux);

    if (!analog_ok || !mux_ok)
    {
        ESP_LOGW(TAG, "No stored motor/encoder wiring map; using the identity map.");
        return;
    }
    if (!is_permutation(analog) || !is_permutation(mux))
    {
        ESP_LOGE(TAG, "Stored motor/encoder wiring map is invalid; using the identity map.");
        return;
    }

    m_analog_map = analog;
    m_mux_map = mux;
    for (size_t wheel = 0; wheel < NUM_ENC_CHANNELS; ++wheel)
        m_slot_wheel[m_analog_map[wheel]] = static_cast<uint8_t>(wheel);

    int32_t valid = 0;
    if (NVSManager::load_i32(MAPPING_NVS_NS, MAPPING_VALID_KEY, &valid) == ESP_OK && valid != 0)
        m_mapping_valid = true;

    ESP_LOGI(TAG,
             "Loaded motor/encoder wiring map from NVS (calibrated=%d): "
             "M1 ENC%u/mux%u | M2 ENC%u/mux%u | M3 ENC%u/mux%u | M4 ENC%u/mux%u",
             static_cast<int>(m_mapping_valid),
             static_cast<unsigned>(m_analog_map[0] + 1),
             static_cast<unsigned>(m_mux_map[0] + 1),
             static_cast<unsigned>(m_analog_map[1] + 1),
             static_cast<unsigned>(m_mux_map[1] + 1),
             static_cast<unsigned>(m_analog_map[2] + 1),
             static_cast<unsigned>(m_mux_map[2] + 1),
             static_cast<unsigned>(m_analog_map[3] + 1),
             static_cast<unsigned>(m_mux_map[3] + 1));
}

/**
 * @brief Creates one AS5600 channel per wheel for the current wiring map.
 *
 * Must run while the ADC dispatch task is suspended or stopped: it rewrites
 * the channel routing that the task dereferences on every sample.
 */
void WheelStateEstimator::rebuild_encoders()
{
    AdcStream & adc = AdcStream::get_instance();
    portENTER_CRITICAL(&m_control_mux);
    m_published_control = {};
    portEXIT_CRITICAL(&m_control_mux);

    for (size_t wheel = 0; wheel < NUM_ENC_CHANNELS; ++wheel)
    {
        const size_t slot = m_analog_map[wheel];
        const adc_channel_t channel = m_channels[slot];
        EncoderChannel & encoder_channel = m_enc_channels[wheel];

        encoder_channel.bind(channel, adc.get_cali_handle(channel), ADC_UNIT);

        m_channel_lookup[channel] = &encoder_channel;
        m_slot_wheel[slot] = static_cast<uint8_t>(wheel);
    }
}

/**
 * @brief Pauses the acquisition stream (flash/I2C critical sections).
 *
 * The pulse-feedback window is not flushed: after a resume the velocity row
 * is absent until the window refills, so the filters continue on the angle row
 * instead of fusing a stale zero.
 */
void WheelStateEstimator::suspend()
{
    AdcStream::get_instance().suspend();
    portENTER_CRITICAL(&m_control_mux);
    m_published_control = {};
    portEXIT_CRITICAL(&m_control_mux);
}

/**
 * @brief Resumes the acquisition stream.
 */
void WheelStateEstimator::resume()
{
    AdcStream::get_instance().resume();
}

/**
 * @brief Loads calibration from NVS for all encoder channels.
 * @return ESP_OK if all calibrations loaded, ESP_ERR_NOT_FOUND if any missing.
 */
esp_err_t WheelStateEstimator::load_calibration()
{
    // Suspend the acquisition task to prevent it from running flash-resident
    // code while NVS flash operations disable the cache.
    suspend();

    bool all_calibrated = true;

    for (size_t wheel = 0; wheel < NUM_ENC_CHANNELS; ++wheel)
    {
        AS5600 * encoder = m_enc_channels[wheel].get_encoder();
        if (encoder == nullptr || encoder->load_calibration_from_nvs() != ESP_OK)
        {
            ESP_LOGW(TAG, "Wheel %s encoder range calibration not found", WHEEL_NAMES[wheel]);
            all_calibrated = false;
        }
    }

    m_calibrated = all_calibrated;
    if (all_calibrated)
        ESP_LOGI(TAG, "All AS5600 calibrations loaded via NVSManager.");
    else
        ESP_LOGW(TAG, "Some encoder range calibrations are missing.");

    // Resume the acquisition task after NVS operations are complete.
    resume();
    return m_calibrated ? ESP_OK : ESP_ERR_NOT_FOUND;
}

/**
 * @brief Enables analog min/max tracking on every encoder.
 *
 * The wiring calibration spins every wheel afterwards, which sweeps each
 * encoder over its full analog output range.
 */
void WheelStateEstimator::start_range_tracking()
{
    xSemaphoreTake(m_data_mutex, portMAX_DELAY);
    for (auto & channel : m_enc_channels)
    {
        AS5600 * encoder = channel.get_encoder();
        if (encoder == nullptr)
            continue;
        encoder->reset_calibration_min_max();
        encoder->start_calibration_mode();
    }
    xSemaphoreGive(m_data_mutex);
    ESP_LOGI(TAG, "Encoder range tracking started.");
}

/**
 * @brief Disables analog min/max tracking without saving the tracked values.
 */
void WheelStateEstimator::stop_range_tracking()
{
    xSemaphoreTake(m_data_mutex, portMAX_DELAY);
    for (auto & channel : m_enc_channels)
    {
        if (AS5600 * encoder = channel.get_encoder())
            encoder->stop_calibration_mode();
    }
    xSemaphoreGive(m_data_mutex);
}

/**
 * @brief Validates and persists the analog range tracked for every encoder.
 *
 * @return Bitmask of the analog input slots with a valid stored range.
 */
uint8_t WheelStateEstimator::finish_range_tracking()
{
    uint8_t valid_slots = 0;
    bool all_calibrated = true;

    xSemaphoreTake(m_data_mutex, portMAX_DELAY);
    for (size_t wheel = 0; wheel < NUM_ENC_CHANNELS; ++wheel)
    {
        AS5600 * encoder = m_enc_channels[wheel].get_encoder();
        if (encoder == nullptr)
        {
            all_calibrated = false;
            continue;
        }

        encoder->stop_calibration_mode();
        // Validate the TRACKING range of this run; only save_calibration_to_nvs
        // (below) promotes it to the applied range, so an invalid run leaves the
        // stored calibration active.
        const int min_mv = encoder->get_track_min_mv();
        const int max_mv = encoder->get_track_max_mv();
        const int range_mv = max_mv - min_mv;
        const uint16_t min_raw = encoder->get_track_min_raw();
        const uint16_t max_raw = encoder->get_track_max_raw();
        const int range_raw = static_cast<int>(max_raw) - static_cast<int>(min_raw);
        ESP_LOGI(TAG,
                 "Wheel %s encoder range: raw [%u..%u] (span %d), [%d..%d] mV",
                 WHEEL_NAMES[wheel],
                 static_cast<unsigned>(min_raw),
                 static_cast<unsigned>(max_raw),
                 range_raw,
                 min_mv,
                 max_mv);
        // Both endpoints must be usable and plausible: the raw span drives the
        // mapping and the millivolt swing stays the diagnostic/compatibility
        // fallback.
        const bool raw_ok = min_raw >= AS5600::MIN_PLAUSIBLE_RAW_MIN &&
                            max_raw <= AS5600::MAX_PLAUSIBLE_RAW_MAX &&
                            range_raw >= static_cast<int>(AS5600::MIN_VALID_RAW_RANGE) &&
                            range_raw <= static_cast<int>(AS5600::MAX_VALID_RAW_RANGE);
        if (!raw_ok || range_mv < MIN_VALID_SWING_MV)
        {
            ESP_LOGE(TAG,
                     "Wheel %s encoder range implausible (raw [%u..%u] span %d, %d mV); keeping "
                     "the old calibration.",
                     WHEEL_NAMES[wheel],
                     static_cast<unsigned>(min_raw),
                     static_cast<unsigned>(max_raw),
                     range_raw,
                     range_mv);
            all_calibrated = false;
            continue;
        }

        if (encoder->save_calibration_to_nvs() == ESP_OK)
            valid_slots |= static_cast<uint8_t>(1U << m_analog_map[wheel]);
        else
            all_calibrated = false;
    }
    m_calibrated = all_calibrated;
    xSemaphoreGive(m_data_mutex);

    ESP_LOGI(TAG,
             "Encoder range tracking finished: valid slots=0x%02X calibrated=%d.",
             static_cast<unsigned>(valid_slots),
             static_cast<int>(m_calibrated));
    return valid_slots;
}

/**
 * @brief Checks if the encoder mapped to a wheel has a valid analog range.
 * @param wheel Wheel index (0-3).
 * @return true if the encoder range is valid.
 */
bool WheelStateEstimator::is_channel_calibrated(size_t wheel) const
{
    if (wheel >= NUM_ENC_CHANNELS)
        return false;

    const AS5600 * encoder = m_enc_channels[wheel].get_encoder();
    if (encoder == nullptr)
        return false;

    const int range = encoder->get_calib_max() - encoder->get_calib_min();
    const int range_raw = static_cast<int>(encoder->get_calib_max_raw()) -
                          static_cast<int>(encoder->get_calib_min_raw());
    return range >= MIN_VALID_SWING_MV ||
           range_raw >= static_cast<int>(AS5600::MIN_VALID_RAW_RANGE);
}

/**
 * @brief Retrieves the persisted motor/encoder wiring map (wheel -> slot).
 */
std::array<uint8_t, NUM_ENC_CHANNELS> WheelStateEstimator::get_analog_mapping() const
{
    std::array<uint8_t, NUM_ENC_CHANNELS> map = {0, 1, 2, 3};
    if (xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        map = m_analog_map;
        xSemaphoreGive(m_data_mutex);
    }
    return map;
}

/**
 * @brief Retrieves the persisted motor/encoder wiring map (wheel -> mux).
 */
std::array<uint8_t, NUM_ENC_CHANNELS> WheelStateEstimator::get_mux_mapping(bool * calibrated) const
{
    std::array<uint8_t, NUM_ENC_CHANNELS> map = {0, 1, 2, 3};
    bool valid = false;
    if (xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        map = m_mux_map;
        valid = m_mapping_valid;
        xSemaphoreGive(m_data_mutex);
    }
    if (calibrated != nullptr)
        *calibrated = valid;
    return map;
}

/**
 * @brief Applies a validated wiring map, rebuilds the routing and persists it.
 *
 * Replaces the channel routing while the analog range calibration of the
 * newly mapped encoders is read back from NVS.
 *
 * @param analog_map Wheel -> analog input slot.
 * @param mux_map Wheel -> CD4052B channel.
 * @return ESP_OK when the map was applied and queued for persistence.
 */
esp_err_t WheelStateEstimator::apply_mapping(
  const std::array<uint8_t, NUM_ENC_CHANNELS> & analog_map,
  const std::array<uint8_t, NUM_ENC_CHANNELS> & mux_map)
{
    if (!is_permutation(analog_map) || !is_permutation(mux_map))
    {
        ESP_LOGE(TAG, "Refusing an invalid motor/encoder wiring map.");
        return ESP_ERR_INVALID_ARG;
    }

    // The NVS reads below run through the single NVS writer task; flash
    // operations disable the caches and interrupts, so no other task can run
    // flash-resident code meanwhile. Do not stop the ADC stream here:
    // adc_continuous_stop() releases the ADC driver lock, which must be done
    // by the task that started the stream, not by a calibration task.
    xSemaphoreTake(m_data_mutex, portMAX_DELAY);

    m_analog_map = analog_map;
    m_mux_map = mux_map;
    for (size_t wheel = 0; wheel < NUM_ENC_CHANNELS; ++wheel)
        m_slot_wheel[m_analog_map[wheel]] = static_cast<uint8_t>(wheel);

    esp_err_t result = NVSManager::save_blob(
      MAPPING_NVS_NS, MAPPING_ANALOG_KEY, m_analog_map.data(), sizeof(m_analog_map));
    if (result == ESP_OK)
        result = NVSManager::save_blob(
          MAPPING_NVS_NS, MAPPING_MUX_KEY, m_mux_map.data(), sizeof(m_mux_map));
    const int32_t valid = 1;
    if (result == ESP_OK)
        result = NVSManager::save_i32(MAPPING_NVS_NS, MAPPING_VALID_KEY, valid);
    m_mapping_valid = result == ESP_OK;

    rebuild_encoders();

    bool all_calibrated = true;
    for (auto & channel : m_enc_channels)
    {
        AS5600 * encoder = channel.get_encoder();
        if (encoder == nullptr || encoder->load_calibration_from_nvs() != ESP_OK)
            all_calibrated = false;
    }
    m_calibrated = all_calibrated;

    xSemaphoreGive(m_data_mutex);

    ESP_LOGI(TAG,
             "Applied motor/encoder wiring map (saved=%d calibrated=%d): "
             "M1 ENC%u/mux%u | M2 ENC%u/mux%u | M3 ENC%u/mux%u | M4 ENC%u/mux%u",
             static_cast<int>(m_mapping_valid),
             static_cast<int>(m_calibrated),
             static_cast<unsigned>(m_analog_map[0] + 1),
             static_cast<unsigned>(m_mux_map[0] + 1),
             static_cast<unsigned>(m_analog_map[1] + 1),
             static_cast<unsigned>(m_mux_map[1] + 1),
             static_cast<unsigned>(m_analog_map[2] + 1),
             static_cast<unsigned>(m_mux_map[2] + 1),
             static_cast<unsigned>(m_analog_map[3] + 1),
             static_cast<unsigned>(m_mux_map[3] + 1));

    return result;
}

/**
 * @brief Returns the latest raw ADC block mean of every analog input slot.
 */
std::array<uint16_t, NUM_ENC_CHANNELS> WheelStateEstimator::get_raw_values() const
{
    std::array<uint16_t, NUM_ENC_CHANNELS> values = {};
    if (xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        for (size_t slot = 0; slot < NUM_ENC_CHANNELS; ++slot)
            values[slot] = m_enc_channels[m_slot_wheel[slot]].get_raw();
        xSemaphoreGive(m_data_mutex);
    }
    return values;
}

/**
 * @brief Reads the raw I2C angle of every CD4052B channel.
 *
 * Selects each encoder pair on the mux, reads its 12-bit raw angle, then
 * disables the mux again. A channel whose reading fails reports zero.
 *
 * @param angles Output for the raw angle of each mux channel (0..3).
 * @param magnet_fields Optional field status in the same mux order. A failed
 *        status read reports UNREADABLE separately from the angle result.
 * @return ESP_OK when all four channels were read, ESP_FAIL otherwise.
 */
esp_err_t WheelStateEstimator::read_i2c_raw_angles(
  std::array<uint16_t, NUM_ENC_CHANNELS> * angles,
  std::array<EncoderChannel::MagnetField, NUM_ENC_CHANNELS> * magnet_fields)
{
    if (angles == nullptr)
        return ESP_ERR_INVALID_ARG;
    if (!m_initialized || m_cd4052b == nullptr)
        return ESP_ERR_INVALID_STATE;

    angles->fill(0);
    if (magnet_fields != nullptr)
        magnet_fields->fill(EncoderChannel::MagnetField::UNREADABLE);
    esp_err_t result = ESP_OK;

    for (uint8_t mux = 0; mux < NUM_ENC_CHANNELS; ++mux)
    {
        if (m_cd4052b->select(mux) != ESP_OK)
        {
            result = ESP_FAIL;
            break;
        }
        // Analog switch settling before the I2C transfer.
        esp_rom_delay_us(50);
        uint16_t raw_angle = 0;
        if (AS5600::read_raw_angle(&raw_angle) != ESP_OK)
        {
            result = ESP_FAIL;
            continue;
        }
        (*angles)[mux] = raw_angle;
        if (magnet_fields != nullptr)
            (*magnet_fields)[mux] = EncoderChannel::read_magnet_field();
    }

    m_cd4052b->disable();
    return result;
}

/**
 * @brief Reads the raw I2C angle of one wheel's encoder through the CD4052B mux.
 *
 * The mux reaches one encoder pair at a time. The wheel's channel comes from
 * the persisted wiring map; the mux is disabled again after the transfer so the
 * rest of the firmware sees its resting state.
 *
 * @param wheel Wheel index (0..3).
 * @param angle Output for the 12-bit raw angle (0..4095).
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG/STATE on a bad argument or an
 *         uninitialized estimator, ESP_ERR_TIMEOUT when the map is locked,
 *         ESP_FAIL when the channel cannot be selected, otherwise the AS5600
 *         read result.
 */
esp_err_t WheelStateEstimator::read_wheel_i2c_raw_angle(uint8_t wheel, uint16_t * angle)
{
    if (angle == nullptr)
        return ESP_ERR_INVALID_ARG;
    if (!m_initialized || m_cd4052b == nullptr || wheel >= NUM_ENC_CHANNELS)
        return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(10)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    const uint8_t channel = m_mux_map[wheel];
    xSemaphoreGive(m_data_mutex);

    if (m_cd4052b->select(channel) != ESP_OK)
    {
        m_cd4052b->disable();
        return ESP_FAIL;
    }
    // Analog switch settling before the I2C transfer.
    esp_rom_delay_us(50);
    uint16_t raw_angle = 0;
    const esp_err_t result = AS5600::read_raw_angle(&raw_angle);
    m_cd4052b->disable();
    if (result != ESP_OK)
        return result;
    *angle = raw_angle;
    return ESP_OK;
}

/**
 * @brief Applies one AS5600 analog output stage to every wheel.
 *
 * The encoders share the I2C bus behind the CD4052B mux: each wheel's channel
 * is selected, its configuration register is read, and only when the stage
 * differs the new stage is written and read back for verification. The
 * slow/fast filters are left untouched and no OTP is burned. No ADC suspend is
 * needed: the mux/I2C path is safe while the acquisition stream runs.
 *
 * @param stage Requested analog output stage.
 * @return ESP_OK when every wheel was read and (when needed) verified in the
 *         requested stage, ESP_FAIL/ESP_ERR_* otherwise.
 */
esp_err_t WheelStateEstimator::configure_encoder_output_stage(AS5600::OutputStage stage)
{
    if (!m_initialized || m_cd4052b == nullptr)
        return ESP_ERR_INVALID_STATE;

    // Snapshot the wheel -> mux channel map and the per-wheel drivers; the I2C
    // transfers themselves run without the data mutex (the ADC dispatch task
    // keeps running and only uses the analog path).
    std::array<uint8_t, NUM_ENC_CHANNELS> mux_channels = {};
    std::array<AS5600 *, NUM_ENC_CHANNELS> encoders = {};
    if (xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(50)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    mux_channels = m_mux_map;
    for (size_t wheel = 0; wheel < NUM_ENC_CHANNELS; ++wheel)
        encoders[wheel] = m_enc_channels[wheel].get_encoder();
    xSemaphoreGive(m_data_mutex);

    bool all_ok = true;

    for (size_t wheel = 0; wheel < NUM_ENC_CHANNELS; ++wheel)
    {
        AS5600 * encoder = encoders[wheel];
        if (encoder == nullptr || m_cd4052b->select(mux_channels[wheel]) != ESP_OK)
        {
            ESP_LOGE(TAG, "%s encoder stage: mux select failed.", WHEEL_NAMES[wheel]);
            m_cd4052b->disable();
            all_ok = false;
            continue;
        }
        // Analog switch settling before the I2C transfer.
        esp_rom_delay_us(50);

        AS5600::OutputStage current = AS5600::OutputStage::ANALOG_FULL;
        bool verified = false;
        bool already = false;
        if (!encoder->is_i2c_initialized() && encoder->init_i2c() != ESP_OK)
        {
            ESP_LOGE(TAG, "%s encoder stage: I2C interface failed.", WHEEL_NAMES[wheel]);
        }
        else if (encoder->read_configuration(&current, nullptr, nullptr) != ESP_OK)
        {
            ESP_LOGE(TAG, "%s encoder stage: configuration read failed.", WHEEL_NAMES[wheel]);
        }
        else if (current == stage)
        {
            already = true;
            verified = true;
        }
        else
        {
            AS5600::OutputStage readback = current;
            const bool written = encoder->set_output_stage(stage) == ESP_OK;
            if (written)
                vTaskDelay(pdMS_TO_TICKS(10));
            verified = written &&
                       encoder->read_configuration(&readback, nullptr, nullptr) == ESP_OK &&
                       readback == stage;
            if (verified)
                ESP_LOGI(TAG,
                         "%s encoder stage: %s -> %s (verified)",
                         WHEEL_NAMES[wheel],
                         AS5600::stage_name(current),
                         AS5600::stage_name(stage));
            else
                ESP_LOGE(TAG,
                         "%s encoder stage: %s -> %s (verification failed)",
                         WHEEL_NAMES[wheel],
                         AS5600::stage_name(current),
                         AS5600::stage_name(stage));
        }
        m_cd4052b->disable();

        if (already)
            ESP_LOGI(
              TAG, "%s encoder stage: already %s", WHEEL_NAMES[wheel], AS5600::stage_name(stage));
        if (!verified)
            all_ok = false;
    }

    return all_ok ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Anchors the applied raw ADC range of the encoder mapped to one wheel.
 *
 * Called by the wiring calibration ADC-I2C alignment so the angle mapping uses
 * the standstill endpoints instead of the spin-distorted ones.
 *
 * @param wheel Wheel index (0..3).
 * @param min_raw Raw ADC count at the I2C angle 0.
 * @param max_raw Raw ADC count at the I2C angle 4095.
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE when the encoder is missing,
 *         otherwise the AS5600 validation/persistence result.
 */
esp_err_t WheelStateEstimator::set_wheel_raw_range(uint8_t wheel,
                                                   uint16_t min_raw,
                                                   uint16_t max_raw)
{
    if (wheel >= NUM_ENC_CHANNELS)
        return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(50)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    esp_err_t result = ESP_ERR_INVALID_STATE;
    AS5600 * encoder = m_enc_channels[wheel].get_encoder();
    if (encoder != nullptr)
        result = encoder->set_raw_calibration_range(min_raw, max_raw);

    xSemaphoreGive(m_data_mutex);
    return result;
}

/**
 * @brief Retrieves the stored raw ADC minimum of the encoder mapped to each
 *        wheel.
 *
 * @return Raw block-mean minimum per wheel; 0 when the stored range is absent
 *         or too small to be usable.
 */
std::array<uint16_t, NUM_ENC_CHANNELS> WheelStateEstimator::get_range_raw_min() const
{
    std::array<uint16_t, NUM_ENC_CHANNELS> mins = {};
    if (xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        for (size_t wheel = 0; wheel < NUM_ENC_CHANNELS; ++wheel)
        {
            const AS5600 * encoder = m_enc_channels[wheel].get_encoder();
            if (encoder == nullptr)
                continue;
            const int span = static_cast<int>(encoder->get_calib_max_raw()) -
                             static_cast<int>(encoder->get_calib_min_raw());
            if (span >= static_cast<int>(AS5600::MIN_VALID_RAW_RANGE))
                mins[wheel] = encoder->get_calib_min_raw();
        }
        xSemaphoreGive(m_data_mutex);
    }
    return mins;
}

/**
 * @brief Retrieves the stored raw ADC maximum of the encoder mapped to each
 *        wheel.
 *
 * @return Raw block-mean maximum per wheel; 0 when the stored range is absent
 *         or too small to be usable.
 */
std::array<uint16_t, NUM_ENC_CHANNELS> WheelStateEstimator::get_range_raw_max() const
{
    std::array<uint16_t, NUM_ENC_CHANNELS> maxs = {};
    if (xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        for (size_t wheel = 0; wheel < NUM_ENC_CHANNELS; ++wheel)
        {
            const AS5600 * encoder = m_enc_channels[wheel].get_encoder();
            if (encoder == nullptr)
                continue;
            const int span = static_cast<int>(encoder->get_calib_max_raw()) -
                             static_cast<int>(encoder->get_calib_min_raw());
            if (span >= static_cast<int>(AS5600::MIN_VALID_RAW_RANGE))
                maxs[wheel] = encoder->get_calib_max_raw();
        }
        xSemaphoreGive(m_data_mutex);
    }
    return maxs;
}

/**
 * @brief Retrieves the latest measured encoder-shaft angles for all wheels.
 *
 * The value is the unfiltered AS5600 reading, so it moves with the motor even
 * when the filter has not settled. One full encoder-shaft turn spans 2*pi.
 *
 * @return Array of angles in radians; zero when the mutex is unavailable.
 */
std::array<float, NUM_ENC_CHANNELS> WheelStateEstimator::get_measured_angle_rad()
{
    std::array<float, NUM_ENC_CHANNELS> angles = {};
    if (xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        for (size_t wheel = 0; wheel < NUM_ENC_CHANNELS; ++wheel)
            angles[wheel] = m_enc_channels[wheel].get_measured_angle_rad();
        xSemaphoreGive(m_data_mutex);
    }
    return angles;
}

/**
 * @brief Retrieves the latest filtered angles (in radians) for all wheels.
 * @return Array of angles in radians; zero when the mutex is unavailable.
 */
std::array<float, NUM_ENC_CHANNELS> WheelStateEstimator::get_filtered_angle_rad()
{
    std::array<float, NUM_ENC_CHANNELS> angles = {};

    if (xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        for (size_t wheel = 0; wheel < NUM_ENC_CHANNELS; ++wheel)
            angles[wheel] = m_enc_channels[wheel].get_filtered_angle_rad();

        xSemaphoreGive(m_data_mutex);
    }
    return angles;
}

/**
 * @brief Retrieves the latest filtered angular velocities (in rad/s) for all wheels.
 * @return Array of velocities in rad/s; zero when the mutex is unavailable.
 */
std::array<float, NUM_ENC_CHANNELS> WheelStateEstimator::get_filtered_velocity_rad_s()
{
    std::array<float, NUM_ENC_CHANNELS> velocities = {};
    if (xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        for (size_t wheel = 0; wheel < NUM_ENC_CHANNELS; ++wheel)
            velocities[wheel] = m_enc_channels[wheel].get_filtered_velocity_rad_s();

        xSemaphoreGive(m_data_mutex);
    }
    return velocities;
}

/**
 * @brief Copies the current publications without waiting for filter or bus work.
 */
bool WheelStateEstimator::get_control_snapshot(ControlSnapshot * snapshot) const
{
    if (snapshot == nullptr)
        return false;
    portENTER_CRITICAL(&m_control_mux);
    *snapshot = m_published_control;
    portEXIT_CRITICAL(&m_control_mux);
    return true;
}

/**
 * @brief Retrieves the smoothed wheel-shaft velocities used by the wheel loops.
 *
 * A light low-pass on the filter velocity removes the one-count sawtooth of
 * the pulse-window velocity (20 RPM of wheel speed at low speeds). The
 * kinematics and radio paths keep the unsmoothed estimate.
 */
std::array<float, NUM_ENC_CHANNELS> WheelStateEstimator::get_smoothed_velocity_rad_s()
{
    std::array<float, NUM_ENC_CHANNELS> velocities = {};
    if (xSemaphoreTake(m_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        for (size_t wheel = 0; wheel < NUM_ENC_CHANNELS; ++wheel)
            velocities[wheel] = m_enc_channels[wheel].get_smoothed_velocity_rad_s();
        xSemaphoreGive(m_data_mutex);
    }
    return velocities;
}

/**
 * @brief Routes a sample to the mapped wheel and publishes its state.
 */
void WheelStateEstimator::on_sample(adc_channel_t channel, uint16_t raw)
{
    const int64_t now_us = esp_timer_get_time();
    // Skip this sample if a remap is holding the mutex to replace encoders.
    if (xSemaphoreTake(m_data_mutex, 0) != pdTRUE)
        return;
    EncoderChannel * state = m_channel_lookup[channel];
    if (state == nullptr)
    {
        xSemaphoreGive(m_data_mutex);
        return;
    }

    const size_t wheel = static_cast<size_t>(state - m_enc_channels.data());
    const bool processed = state->process_sample(wheel, raw, now_us);
    if (processed)
        publish_control_sample(wheel, *state, now_us);
    xSemaphoreGive(m_data_mutex);

#if CONFIG_ENCODER_DIAG
    if (processed)
        state->record_sample_timing(wheel);
#endif
}

void WheelStateEstimator::publish_control_sample(size_t wheel,
                                                 const EncoderChannel & state,
                                                 int64_t now_us)
{
    const uint8_t bit = static_cast<uint8_t>(1U << wheel);
    portENTER_CRITICAL(&m_control_mux);
    m_published_control.sample_us[wheel] = now_us;
    m_published_control.measured_angle_rad[wheel] = state.get_measured_angle_rad();
    m_published_control.smoothed_velocity_rad_s[wheel] = state.get_smoothed_velocity_rad_s();
    m_published_control.fast_velocity_rad_s[wheel] = state.get_fast_velocity_rad_s();
    m_published_control.smoothed_valid_mask =
      (m_published_control.smoothed_valid_mask & ~bit) | (state.has_smoothed_velocity() ? bit : 0);
    m_published_control.fast_valid_mask =
      (m_published_control.fast_valid_mask & ~bit) | (state.has_fast_velocity() ? bit : 0);
    m_published_control.stuck_analog_mask =
      (m_published_control.stuck_analog_mask & ~bit) | (state.is_analog_stuck() ? bit : 0);
    m_published_control.clamped_raw_mask =
      (m_published_control.clamped_raw_mask & ~bit) | (state.is_raw_clamped() ? bit : 0);
    portEXIT_CRITICAL(&m_control_mux);
}

void WheelStateEstimator::s_on_sample(void * pvParameters)
{
    auto * event = static_cast<AdcStream::SampleEvent *>(pvParameters);
    static_cast<WheelStateEstimator *>(event->user_data)->on_sample(event->channel, event->raw);
}

/**
 * @brief Configures the I2C registers for a SPECIFIC sensor channel.
 * @warning Stops the ADC task during execution.
 * @warning YOU MUST ENSURE ONLY ONE SENSOR IS CONNECTED TO I2C BUS (Addr 0x36).
 * @param channel_idx The wheel index (0-3) whose encoder is configured.
 * @param settings The configuration settings to apply.
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if sensor not found.
 */
esp_err_t WheelStateEstimator::configure_encoder(uint8_t channel_idx,
                                                 const AS5600Settings & settings)
{
    if (channel_idx >= NUM_ENC_CHANNELS)
        return ESP_ERR_INVALID_ARG;

    ESP_LOGW(TAG,
             "Configuring wheel %s encoder. Ensure ONLY this sensor is connected to I2C!",
             WHEEL_NAMES[channel_idx]);

    // Suspend ADC to prevent I2C/Flash conflicts
    suspend();

    esp_err_t ret = ESP_OK;
    AS5600 * enc = m_enc_channels[channel_idx].get_encoder();

    if (m_cd4052b != nullptr)
        ret = m_cd4052b->select(m_mux_map[channel_idx]);

    if (ret == ESP_OK && enc != nullptr && (enc->is_i2c_initialized() || enc->init_i2c() == ESP_OK))
    {
        ret = apply_i2c_settings(enc, settings);

        if (ret == ESP_OK)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            ret = verify_i2c_settings(enc, settings);
        }
        else
            ESP_LOGE(
              TAG, "I2C Config failed to write settings on wheel %s", WHEEL_NAMES[channel_idx]);

        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "I2C Config & Verification Success.");
            ret = process_otp_burn(enc, settings);
        }
        else
            ESP_LOGE(TAG,
                     "I2C Config Failed during settings verification on wheel %s",
                     WHEEL_NAMES[channel_idx]);
    }
    else
    {
        if (m_cd4052b == nullptr)
            ESP_LOGE(TAG, "CD4052B is not configured.");
        else
            ESP_LOGE(TAG, "Sensor not found on I2C bus (Address 0x%02X).", AS5600::AS5600_ADDR);
    }

    if (m_cd4052b != nullptr)
    {
        esp_err_t mux_err = m_cd4052b->disable();
        if (ret == ESP_OK && mux_err != ESP_OK)
            ret = mux_err;
    }

    // Resume ADC Task
    resume();
    return ret;
}

/**
 * @brief Writes configuration settings to the AS5600 via I2C.
 * @param enc Pointer to the AS5600 encoder instance.
 * @param settings The configuration settings to apply.
 * @return ESP_OK on success.
 */
esp_err_t WheelStateEstimator::apply_i2c_settings(AS5600 * enc, const AS5600Settings & settings)
{
    if (enc->set_output_stage(settings.output_stage) != ESP_OK)
        return ESP_FAIL;
    if (enc->set_slow_filter(settings.slow_filter) != ESP_OK)
        return ESP_FAIL;
    if (enc->set_fast_filter(settings.fast_filter) != ESP_OK)
        return ESP_FAIL;
    return ESP_OK;
}

/**
 * @brief Reads back AS5600 registers to verify they match the desired settings.
 * @param enc Pointer to the AS5600 encoder instance.
 * @param settings The configuration parameters expected.
 * @return ESP_OK if the readback values match.
 */
esp_err_t WheelStateEstimator::verify_i2c_settings(AS5600 * enc, const AS5600Settings & settings)
{
    AS5600::OutputStage read_stage;
    AS5600::SlowFilter read_slow;
    AS5600::FastFilter read_fast;

    if (enc->read_configuration(&read_stage, &read_slow, &read_fast) != ESP_OK)
    {
        ESP_LOGE(TAG, "Verify Failed: Could not read back configuration register.");
        return ESP_FAIL;
    }

    bool match = true;

    if (read_stage != settings.output_stage)
    {
        ESP_LOGE(TAG,
                 "Verify Failed: Output Stage mismatch (Exp: 0x%02X, Got: 0x%02X)",
                 (uint8_t)settings.output_stage,
                 (uint8_t)read_stage);
        match = false;
    }

    if (read_slow != settings.slow_filter)
    {
        ESP_LOGE(TAG,
                 "Verify Failed: Slow Filter mismatch (Exp: 0x%02X, Got: 0x%02X)",
                 (uint8_t)settings.slow_filter,
                 (uint8_t)read_slow);
        match = false;
    }

    if (read_fast != settings.fast_filter)
    {
        ESP_LOGE(TAG,
                 "Verify Failed: Fast Filter mismatch (Exp: 0x%02X, Got: 0x%02X)",
                 (uint8_t)settings.fast_filter,
                 (uint8_t)read_fast);
        match = false;
    }

    return match ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

/**
 * @brief Permanently burns configuration settings into the AS5600 OTP memory.
 * @warning This operation is irreversible.
 * @param enc Pointer to the AS5600 encoder instance.
 * @param settings Configuration object containing the `burn_settings` flag.
 * @return ESP_OK on success.
 */
esp_err_t WheelStateEstimator::process_otp_burn(AS5600 * enc, const AS5600Settings & settings)
{
    if (!settings.burn_settings)
        return ESP_OK;

    return enc->burn_settings();
}

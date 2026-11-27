# SSL Robot Firmware

Firmware for small-size league (SSL) robots based on the ESP32. It controls a 4-wheel omni-directional robot with closed-loop motor control, IMU-based orientation sensing, and ESP-NOW communication.

## Setup

This project uses **git submodules** for some components. When cloning, you **must** include the `--recurse-submodules` flag so all dependencies are fetched.

### Cloning

```bash
git clone --recurse-submodules https://github.com/OxeBots/ssl_firmware.git
```

If you already cloned without submodules, fetch them manually:

```bash
git submodule update --init --recursive
```

### Start the development environment

The project is fully configured for Docker-based development. You only need VS Code with the Dev Containers extension installed.

1. Open VS Code
2. Open this folder
3. When prompted, click **"Reopen in Container"** (or run `Dev Containers: Rebuild Container` from the command palette)

The container includes PlatformIO, the ESP-IDF toolchain, and all build dependencies. No manual setup is required.

### Using AI agents (optional)

If you want to use AI coding assistants, edit `.devcontainer/devcontainer.json` **before** reopening the container. Set the `INSTALL_` variables for the provider you want:

```json
"args": {
  "INSTALL_CLAUDE": "0",
  "INSTALL_GEMINI": "0",
  "INSTALL_QWEN": "0",
  "INSTALL_CODEX": "0",
  "INSTALL_OPENCODE": "0",
  "INSTALL_PI": "0"
}
```

Set to `"1"` to install, `"0"` to skip. You can enable multiple agents at once.


## Hardware used

- **MCU**: ESP32 (esp32dev)
- **Radio**: ESP-NOW on the ESP32's built-in 2.4 GHz radio
- **IMU**: GY-85 board (ADXL345 accelerometer, ITG3200 gyroscope, QMC5883L magnetometer)
- **Motor**: BL48250
- **Wheel encoders**: AS5600 magnetic encoders (in analog mode)

## Building

Open the VS Code terminal (inside the dev container) and run:

```bash
# Build
pio run

# Upload robot firmware to /dev/ttyUSB1
pio run -e esp32dev -t upload

# Build and upload the USB gateway to /dev/ttyUSB0
pio run -e esp_link -t upload

# Open serial monitor (115200 baud)
pio device monitor

# Check code quality (clang-tidy + cppcheck)
pio check

# Generate compile_commands.json for IDE linting/intellisense
pio run -t compiledb
```

### ESP-NOW gateway

`esp32dev` builds the robot firmware (`src/robot`) and `esp_link` builds the USB
gateway firmware (`src/esp_link`), both using
`components/embedded-drivers/ESP_NOW_Dev`, and both default to channel 6, which
you must change in both targets through `CONFIG_ESP_NOW_CHANNEL` in `menuconfig`
(one gateway per channel, and the radio link is not encrypted). The gateway is
just a bridge: the computer plugs into it over USB and never needs a radio of
its own, sending frames at 921600 baud, 8N1, each one framed by zero bytes
around a COBS-encoded block holding the marker byte `A5`, the payload and one
CRC-8 byte (polynomial `0x07`, initial value `0x00`, no reflection or final XOR,
computed over `A5` plus the payload before COBS), with the payload defined in
[`proto/ssl_robot_protocol.bitproto`](proto/ssl_robot_protocol.bitproto). The
gateway learns each robot's address from its telemetry and unicasts while the
route is less than one second old, broadcasting unknown routes, team velocity
messages and broadcast robot IDs, and an addressed robot answers straight back
to the gateway that asked. Frame type `0xA7` forces a broadcast downlink and
`0xA6` asks for a USB-only echo that never reaches the air. Once ready it prints
`ESP_LINK ready` behind a zero byte, and while idle it keeps the radio asleep
with a 20 ms listening window every 100 ms, waking it for 150 ms per real radio
request. Robot Bench opens `/dev/ttyUSB0` unless `ESP_LINK_PORT` is set, and
`ESP_LINK_BAUD` must match `monitor_speed` in `[env:esp_link]`.

### Tests

```bash
# Logic tests (no hardware required)
pio test -e test_logic

# Hardware tests (sensors and motors must be connected)
pio test -e test_hardware

# Calibration tests (interactive)
pio test -e test_calibration

# Run a specific test (with -f matching the path under test/)
pio test -vv -f kinematics/test_omnidirectional_robot

# Other examples
pio test -vv -e test_hardware -f ADXL345/test_ADXL345
pio test -vv -e test_hardware -f control/test_WheelController
```

## Configuration

GPIO pins are set in `sdkconfig.defaults`. The key groups are:

| Function                   | Pins                                                       |
| -------------------------- | ---------------------------------------------------------- |
| I2C (IMU)                  | SDA=21, SCL=22                                             |
| SPI (74HC595)              | MISO=19, MOSI=23, SCLK=18, latch=17                        |
| Motor PWM                  | FL=32, BL=26, BR=14, FR=12                                 |
| Motor direction and mux    | 74HC595 outputs: DIR1-4 = bits 0-3, MUX A/B/INH = bits 4-6 |
| Motor pulse feedback       | FL=25, BL=27, BR=13, FR=2                                  |
| Motor encoder ADC channels | FL=0, BL=7, BR=6, FR=3                                     |

### SAFE_STOP at flash time

Run `pio run -e esp32dev -t menuconfig` in the dev container. Under **Oxebots Robot Configuration**, select **Disable SAFE_STOP and report health faults as warnings** (`CONFIG_DISABLE_SAFE_STOP`), save, and build or upload. The default build retains `SAFE_STOP`.

With the option selected, health faults appear as warnings without blocking valid radio or USB commands. The firmware still applies command timeouts, control ownership, and actuator prerequisites such as calibrated encoders for position moves.

## License

See `LICENSE`.

# SSL Robot Firmware

Firmware for small-size league (SSL) robots based on the ESP32. It controls a 4-wheel omni-directional robot with closed-loop motor control, IMU-based orientation sensing, and radio communication via nRF24L01.

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

### Start the devlopment environment

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
  "INSTALL_OPENCODE": "0"
}
```

Set to `"1"` to install, `"0"` to skip. You can enable multiple agents at once.


## Hardware used

- **MCU**: ESP32 (esp32dev)
- **Radio**: nRF24L01 (2.4 GHz)
- **IMU**: GY-85 board (ADXL345 accelerometer, ITG3200 gyroscope, QMC5883L magnetometer)
- **Motor**: BL48250
- **Wheel encoders**: AS5600 magnetic encoders (in analog mode)

## Building

Open the VS Code terminal (inside the dev container) and run:

```bash
# Build
pio run

# Upload
pio run -t upload

# Open serial monitor (115200 baud)
pio device monitor

# Check code quality (clang-tidy + cppcheck)
pio check

# Generate compile_commands.json for IDE linting/intellisense
pio run -t compiledb
```

### Tests

```bash
# Logic tests (no hardware required)
pio test -e test_logic

# Hardware tests (sensors and motors must be connected)
pio test -e test_hardware

# Calibration tests (interactive)
pio test -e test_calibration
```

## Configuration

GPIO pins are set in `sdkconfig.defaults`. The key groups are:

| Function                 | Pins                                          |
| ------------------------ | --------------------------------------------- |
| I2C (IMU)                | SDA=21, SCL=22                                |
| SPI (nRF24L01)           | MISO=19, MOSI=23, SCLK=18, CE=2, CSN=5, IRQ=4 |
| Motor PWM                | FL=32, BL=26, BR=14, FR=16                    |
| Motor DIR                | FL=33, BL=25, BR=27, FR=17                    |
| Motor ENC (ADC channels) | FL=0, BL=7, BR=6, FR=3                        |

Change radio channel and other settings in `sdkconfig.defaults` under the `# Radio` section.

## License

See `LICENSE`.

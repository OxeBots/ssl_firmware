# bench_dashboard

Live GUI dashboard for the SSL robot serial debug stream. Pairs with the
main firmware built with `CONFIG_SERIAL_DEBUG=y` (menuconfig or
`sdkconfig.defaults`), which streams binary debug frames over the serial
console as long as a dashboard client keeps the connection alive.

## Run

From the repository root (host or devcontainer):

```bash
python3 -m bench_dashboard                        # port picker, then dashboard
python3 -m bench_dashboard --port /dev/ttyUSB0    # skip the picker
python3 -m bench_dashboard --demo                 # synthetic data, no hardware
python3 -m bench_dashboard --tab 2                # open on the Sensing tab
python3 -m bench_dashboard --screenshot-dir tmp/gui   # save PNG frames
```

Requires `pygame` and `pyserial` (installed by the devcontainer image).

## GUI

Resizable window (minimum 1024x640, maximizable to full screen) with three
tabs:

| Tab              | Content                                                          |
| ---------------- | ---------------------------------------------------------------- |
| Motors & Control | 4 wheel cards (PWM, direction, target/speed), speed chart, keys  |
| Health & State   | System (uptime, heap), safety lifecycle, error flags, power, IMU |
| Sensing          | VL53L5CX 8x8 heatmap, AHRS cube, raw accel/gyro/mag              |

Serial console and status bar stay visible on every tab. Keys:
`[1-4]` select wheel, `[UP/DOWN]` target +/-100 RPM, `[LEFT/RIGHT]` +/-500,
`[SPACE]` stop all motors, `[W]` dashboard-driven +/-1000 RPM square sweep,
`[F1-F3]` switch tabs, `[Q]` quit.

## Modules

| Module         | Responsibility                                                        |
| -------------- | --------------------------------------------------------------------- |
| `config.py`    | Window sizing, timing windows, fonts, color palette                   |
| `state.py`     | `BenchState`: thread-shared telemetry snapshot + command routing      |
| `protocol.py`  | COBS + CRC8 framing, bitproto binary message parsers, control encoder |
| `serial_io.py` | `BenchReader`: connect/reconnect thread, stream subscription, TX      |
| `discovery.py` | USB port listing + `PortScanner` firmware probe (picker data source)  |
| `demo.py`      | `DemoSource`: synthetic data generator for GUI work without hardware  |
| `rendering.py` | Drawing primitives, colormap, 3D cube math                            |
| `panels.py`    | Tab bar and panels for the three tabs, console, status                |
| `picker.py`    | First-screen port picker rendering + click resolution                 |
| `app.py`       | Resizable window, main loop, phases (picker/dashboard), input         |
| `cli.py`       | Argument parsing and wiring                                           |

## Serial protocol

Defined in `proto/serial_debug_protocol.bitproto`. Every frame is
`COBS(msg_type | bitproto message | crc8)` followed by a `0x00` delimiter:

| Direction     | Message        | Rate     | Content                                     |
| ------------- | -------------- | -------- | ------------------------------------------- |
| robot -> host | `DebugState`   | 25 Hz    | 4 wheels PWM/speed/target, attitude         |
| robot -> host | `DebugImu`     | 10 Hz    | calibrated accel / gyro / magnetometer      |
| robot -> host | `DebugTof`     | ~15 Hz   | 8x8 range frame, 0 = no valid target        |
| robot -> host | `DebugHealth`  | 2 Hz     | lifecycle, faults, supply voltage, heap     |
| robot -> host | `DebugText`    | per line | captured ESP_LOG console lines              |
| host -> robot | `DebugControl` | on event | start/stop stream, keepalive, wheel targets |

The firmware starts streaming only after a `START_STREAM` control frame and
stops when keepalives (sent at ~1 Hz by the dashboard) stop arriving, so
closing the port always ends the debug dump. Both the picker probe and the
dashboard re-send `START_STREAM` until the first frame arrives, so a board
that reboots as the port opens is still detected. The picker enables
**Connect** only on ports where the probe actually flagged the firmware.

## Debugging

- `--demo` runs the full GUI without hardware (synthetic IMU/ToF/motor data).
- `--screenshot-dir DIR` saves a PNG every `--screenshot-period` seconds;
  combined with `SDL_VIDEODRIVER=dummy` it renders headless, e.g.:

  ```bash
  SDL_VIDEODRIVER=dummy python3 -m bench_dashboard --demo \
      --screenshot-dir tmp/gui --screenshot-period 2
  ```

- `protocol.py` and `state.py` are pygame-free; import them directly to test
  parsing of captured serial logs.

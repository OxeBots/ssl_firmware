# bench_dashboard

Live GUI dashboard for the protoboard test bench. Pairs with the
`BenchDashboard` firmware test (`pio test -e test_bench`), which streams
sensor telemetry over the serial console.

## Run

From the repository root (host or devcontainer):

```bash
python3 -m bench_dashboard                        # port picker, then dashboard
python3 -m bench_dashboard --port /dev/ttyUSB0    # skip the picker
python3 -m bench_dashboard --demo                 # synthetic data, no hardware
python3 -m bench_dashboard --screenshot-dir tmp/gui   # save PNG frames
```

Requires `pygame` and `pyserial` (installed by the devcontainer image).

## Modules

| Module         | Responsibility                                                        |
| -------------- | --------------------------------------------------------------------- |
| `config.py`    | Protocol framing sizes, timing windows, fonts, color palette           |
| `state.py`     | `BenchState`: thread-shared telemetry snapshot + command routing       |
| `protocol.py`  | `parse_line`: firmware `S` (state) and `T` (ToF frame) line parsing    |
| `serial_io.py` | `BenchReader`: connect/reconnect thread, TX of commands                |
| `discovery.py` | USB port listing + `PortScanner` firmware probe (picker data source)   |
| `demo.py`      | `DemoSource`: synthetic data generator for GUI work without hardware   |
| `rendering.py` | Drawing primitives, colormap, 3D cube math                             |
| `panels.py`    | Dashboard panels: ToF heatmap, orientation cube, motor gauges, console |
| `picker.py`    | First-screen port picker rendering + click resolution                  |
| `app.py`       | Window, main loop, phases (picker/dashboard), key and mouse input      |
| `cli.py`       | Argument parsing and wiring                                            |

## Serial protocol

The firmware test emits two line types and accepts short commands:

```text
S,<ms>,<yaw>,<pitch>,<roll>,<pwm>,<vel_rpm>,<tgt_rpm>,<enc_raw>,<enc_ok>,<tof_seq>
T,<seq>,<d0>,...,<d63>            # 8x8 distances in mm, 0 = no valid target
```

Commands accepted from the dashboard: `T<rpm>` (motor 1 target), `0` (stop),
`W` (+/-1000 RPM square sweep), `Q` (firmware exits the test). The dashboard
sends `0` before quitting and `Q` on `X` (exit firmware test).

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
- Serial console (bottom-left panel) mirrors every non-telemetry line the
  firmware prints, including `INIT WARN` lines about missing sensors.

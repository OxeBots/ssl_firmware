# Robot Bench

Live diagnostics and motor controls for the SSL robot serial debug stream. Pairs with the
main firmware built with `CONFIG_SERIAL_DEBUG=y` (menuconfig or
`sdkconfig.defaults`), which streams binary debug frames over the serial
console as long as a dashboard client keeps the connection alive.

## Run

From the repository root (host or devcontainer):

```bash
python3 -m robot_bench                        # port picker, then dashboard
python3 -m robot_bench --port /dev/ttyUSB0    # skip the picker
python3 -m robot_bench --demo                 # synthetic data, no hardware
python3 -m robot_bench --tab 1                # open on the Calibração tab
python3 -m robot_bench --tab 3                # open on the ToF tab
python3 -m robot_bench --tab 4                # open on the IMU tab
python3 -m robot_bench --tab 5                # open on the Console tab
python3 -m robot_bench --tab 6                # open on the Controle tab
python3 -m robot_bench --screenshot-dir tmp/gui   # save PNG frames
```

Requires `dearpygui` and `pyserial`. In the devcontainer, use `python -m robot_bench`;
these dependencies are installed in the PlatformIO Python environment, selected by `python`.
The dashboard renders through OpenGL; the devcontainer image installs the Mesa software
renderer and forwards the host X11 socket, so it works without a GPU.

## GUI

Resizable window (minimum 1024x640, maximizable to full screen) with seven
tabs:

| Tab         | Content                                                                 |
| ----------- | ----------------------------------------------------------------------- |
| Motors      | Four selectable wheel cards, speed/target plots, angle readout, motor and sweep buttons |
| Calibração  | Motor/encoder wiring calibration controls, live detection bars and the stored map |
| Health      | System, safety lifecycle, error flags, power and sensor availability     |
| ToF         | Full-size 8x8 distance heatmap with millimetre readings and colour scale |
| IMU         | Orientation box and twelve orientation / sensor plots                   |
| Console     | Full-size serial log with history scrolling and text selection          |
| Controle    | Wheel control calibration (PID + feedforward auto-tuning) and wheel position control |

Connection status stays at the top and stream details at the bottom. The
serial console has a dedicated tab; messages continue to arrive while other
tabs are open. Each wheel card shows the encoder angle next to the RPM
reading; the angle is a live number only and is not plotted. Click a wheel to
select it, then use the speed adjustment, sweep or **Stop all motors** buttons.
Stopping cancels the sweep so it cannot restart the motors. All keyboard
shortcuts remain available:
`[1-4]` select wheel, `[UP/DOWN]` target +/-100 RPM, `[LEFT/RIGHT]` +/-500,
`[SPACE]` stop all motors, `[W]` dashboard-driven +/-1000 RPM square sweep,
`[F1-F7]` switch tabs, `[Q/ESC]` quit.

**Bench motion override** (Motors -> Control) lets the dashboard wheel targets
through while the health monitor inhibits motion, for bring-up on a bare bench
without the power-board supply sense or without encoder calibration. It only
lifts the gate on the dashboard target path: the reported health state is
unchanged, radio commands keep their safety gating, and the firmware clears the
override and zeroes the wheels when the streaming session ends.

On the Console tab, scroll up to browse history; scroll down to return to live
messages. Drag across console text to copy the selection to the clipboard.
Missing or stale health data stays unknown. Motor, attitude and range readings
expire after 1 second; health expires after 1.5 seconds. Disconnects and reboots
clear freshness immediately. Zero orientation angles are valid.

The ToF tab receives all 64 raw distances in millimetres with their sensor status,
before the firmware's control validity filter. Status 5 is valid; statuses 6, 9 and
10 with a positive range have amber borders and a `~` prefix. Other failed readings
have red borders and a `?` suffix. Status 0 means no update and 255 means no target;
these cells show missing measurements explicitly. Every cell shows its status code.
Hover over a cell for its raw distance and status description, including raw values
that do not represent a detected target. Counts distinguish valid, uncertain,
rejected and missing zones. A stale stream is labelled separately from a new frame
with missing targets. There is no 1 m distance cutoff.

On the IMU tab, a left column shows the orientation box with its X, Y and Z
reference axes.
Four aligned rows show yaw / pitch / roll, acceleration (g), rotation (°/s), and
magnetic field (mG). Each measurement axis has a scrolling 12-second history plot,
its current reading, and an automatic vertical scale. All twelve plots fit on the
IMU tab together. Traces break at gaps in telemetry; orientation traces also
break at angle wraparound. Stale readings show a dash while recent history remains visible.

## Calibração dos motores

The Calibração tab drives the firmware's motor/encoder wiring calibration. The
robot spins one motor at a time in open loop and watches every encoder input:
the analog output of each connector (ENCM1..ENCM4) and every channel of the
CD4052B I2C mux. The detected map is `wheel -> analog input` (the path used for
closed-loop control) plus `wheel -> I2C mux channel` (the encoder pair behind
the mux).

The safety warning on the tab is mandatory: check **Robô levantado · rodas
livres** and keep the robot lifted or the wheels free before pressing
**Calibrar motores**. The wheels spin during the routine and must not be
touched; **Parar calibração** aborts the run. Wheel targets are ignored while
the routine runs, so the Motors tab cannot fight the calibration.

While the routine runs the tab shows its progress, which wheel is spinning, the
movement measured on each analog input and mux channel (one bar each, normalised
to the column peak), and the per-wheel detection result (both encoder paths,
analog only, failed or pending). The right panel shows the stored map and whether
it came from a calibration run or is still the default identity map. The
firmware saves the detected map in NVS and applies it immediately, so it
survives reboots. In `--demo` mode the tab simulates the same 7.2 s routine so
the layout can be exercised without hardware.

## Calibração do controle

The Controle tab drives the firmware's per-wheel control calibration. For each wheel the
firmware measures the static friction duty and the plant gain with open-loop duty steps,
synthesizes a PID plus friction and velocity feedforward from that model, validates the
closed-loop speed step, refines the gains when the step does not meet the criteria, and stores
the winning parameters in NVS. The same safety confirmation as the wiring calibration applies
(**Robô levantado · rodas livres**): the wheels spin one at a time and **Parar calibração**
aborts the run. Wheel targets are ignored while the routine runs.

Progress, the current wheel and its phase (Atrito, Identificação, Validação, Refino, Gravação)
are shown live, together with the speed step response of the wheel under test and the per-wheel
result: validated gains (Kp, Ki, Kd), friction feedforward duty, velocity feedforward, rise
time, overshoot and steady-state error. The right panel shows the parameters currently in
effect, which at idle are the ones stored in NVS. A wheel that fails keeps its previous
parameters; the run only fails when no wheel produces a valid step. Run the wiring
calibration (F2) first: control calibration needs a valid motor/encoder map.

### Posição das rodas

The lower panel of the Controle tab commands wheel position moves. Enter an angle in degrees,
choose **Ângulo absoluto** or leave it relative to the current angle, set the speed cap and
press **Girar** for one wheel. The firmware runs a cascade position loop (PD outer loop driving
the tuned velocity loop), holds the target when it is reached and stops after a timeout.
**Parar posição** cancels every move. Absolute angles count from boot; relative moves start
from the measured angle. Measured angle, error and state of each wheel are shown live. Position
moves use the same motion gate (and bench override) as wheel targets and are refused while a
calibration runs.

In `--demo` mode both features simulate the firmware behavior: a 10.4 s control calibration
per run and position moves with a proportional approach.

## Modules

| Module         | Responsibility                                                        |
| -------------- | --------------------------------------------------------------------- |
| `config.py`    | Window sizing, timing windows, fonts, color palette                   |
| `state.py`     | `BenchState`: thread-shared telemetry snapshot + command routing      |
| `protocol.py`  | COBS + CRC8 framing, bitproto binary message parsers, control encoder |
| `serial_io.py` | `BenchReader`: connect/reconnect thread, stream subscription, TX      |
| `discovery.py` | USB port listing + `PortScanner` firmware probe (picker data source)  |
| `demo.py`      | `DemoSource`: synthetic data generator for GUI work without hardware  |
| `rendering.py` | Dear PyGui theme, fonts, panel/plot helpers, colormap and cube math |
| `motors.py`    | Wheel cards, speed plots, angle readout and clickable motor controls |
| `calibration.py` | Motor/encoder calibration controls, detection bars and stored map  |
| `tuning.py`    | Control calibration (PID auto-tuning) and wheel position controls      |
| `health.py`    | System, safety, power and sensor status                              |
| `sensing.py`   | Distance heatmap, orientation and sensor measurements                 |
| `console.py`   | Console drawlist, history scrolling and text selection                |
| `picker.py`    | First-screen port picker window and scan status                       |
| `app.py`       | Dear PyGui viewport, main loop, phases (picker/dashboard), input      |
| `cli.py`       | Argument parsing and wiring                                           |

The view modules build Dear PyGui widgets once and refresh their values from
`BenchState` every frame. `app.py` routes mouse and keyboard input through
`perform_action`; commands stay in `BenchState`. The demo source produces
telemetry and uses the same sweep timing as a real session.

## GUI checks

Run inside the devcontainer, from `/workspaces/ssl_firmware`:

```bash
ruff check robot_bench/
python -m unittest discover -s robot_bench/tests -v
```

The unit tests cover state handling, commands, formatting and protocol
parsing; they do not open a window, so they also run without a display.

The checks cover stop/sweep behavior, target changes, motion override arming and
command routing; motor calibration freshness, button rules, level normalisation,
result/error wording, start/stop routing and the simulated routine frames; layout
space reservation and connection/status wording; picker ordering; console history
clamping and anchoring, hit testing, selection text, clipboard copy and scrolling;
wheel card status and hit testing; health freshness, lifecycle, flags, supply,
IMU calibration and connection rows; history sample filtering, gap and wraparound
breaks, automatic scaling, distance colours and rotation math; raw range transport
(including fragmented frames, quality labels, hover text, stale/waiting modes);
the IMU freshness and layout helpers; control-calibration freshness, gating, phase/result
wording, metric formatting, demo frames and command routing; and wheel position parsing,
frame encoding, move/stop gating, row formatting and tab/F7 binding. Run a firmware build
first to generate the bindings.

## Serial protocol

Defined in `proto/serial_debug_protocol.bitproto`. Every frame is
`COBS(msg_type | bitproto message | crc8)` followed by a `0x00` delimiter:

| Direction     | Message        | Rate     | Content                                     |
| ------------- | -------------- | -------- | ------------------------------------------- |
| robot -> host | `DebugState`   | 25 Hz    | 4 wheels PWM/speed/target/angle, attitude   |
| robot -> host | `DebugImu`     | 10 Hz    | calibrated accel / gyro / magnetometer      |
| robot -> host | `DebugTof`     | ~15 Hz   | 64 raw distances + 64 sensor status codes  |
| robot -> host | `DebugHealth`  | 2 Hz     | lifecycle, faults, supply voltage, heap     |
| robot -> host | `DebugText`    | per line | captured ESP_LOG console lines              |
| robot -> host | `DebugCalibration` | ~5 Hz | routine state/progress, mapping and live levels |
| robot -> host | `DebugTuning` | ~5 Hz | control-calibration state/phase, per-wheel gains and step metrics |
| robot -> host | `DebugPosition` | 10 Hz | position-control masks, continuous wheel angles and targets |
| host -> robot | `DebugControl` | on event | start/stop stream, keepalive, wheel targets, motion override, wiring calibration, control calibration |
| host -> robot | `DebugPositionControl` | on event | relative/absolute wheel position moves and stop |

`DebugTof` is 196 bytes: a 4-byte sequence, 64 signed 16-bit distances, and
64 unsigned 8-bit statuses. `DebugCalibration` is 36 bytes: routine state and
current wheel, progress, result masks, stored-map flag, four analog and four mux
mappings, eight signed 16-bit movement levels, range mask, error code and a
sequence counter. `DebugTuning` is 66 bytes: state, wheel, phase, progress,
result and error masks, four gain/feedforward arrays (Kp x100, Ki x100,
Kd x10000, friction duty, velocity feedforward x100), four rise times, overshoot
and steady-state error percentages and a sequence counter. `DebugPosition` is
39 bytes: active/done masks, error code, four measured and four target continuous
wheel angles in 0.001 degree units and a sequence counter. `DebugPositionControl`
is 25 bytes: operation, mode, wheel mask, four requested angles (0.001 degree),
speed cap and a command counter. Rebuild the firmware and dashboard together when
changing this layout; generated C and Python definitions share the same source.

The firmware starts streaming only after a `START_STREAM` control frame.
Telemetry and captured application logs stop on `STOP_STREAM` or 1.5 seconds
after the last keepalive (sent at ~1 Hz by the dashboard). Keepalives alone cannot
start a session. While disconnected, no diagnostic frames or application log text
are sent; normal boot output can precede service initialization. Sensor acquisition
and robot control continue independently of the bench connection. Both the picker
probe and the dashboard re-send `START_STREAM` until the first frame arrives, so a board
that reboots as the port opens is still detected. The picker enables
**Connect** only on ports where the probe actually flagged the firmware.

## Debugging

- `--demo` runs the full GUI without hardware (synthetic IMU/ToF/motor data, a
  simulated motor calibration run started from the Calibração tab, a simulated
  control calibration and wheel position moves started from the Controle tab).
- `--screenshot-dir DIR` saves a PNG of the rendered frame every
  `--screenshot-period` seconds, e.g.:

  ```bash
  python -m robot_bench --demo --screenshot-dir tmp/gui --screenshot-period 2
  ```

  Screenshots are read from the Dear PyGui frame buffer, so a display (or
  Xvfb) is required even when the window is not watched.

- `protocol.py` and `state.py` contain no drawing logic; use them to test
  parsing of captured serial logs.

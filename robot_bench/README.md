# Robot Bench

Live diagnostics and motor controls for the SSL robot serial debug stream. Pairs with the
main firmware built with `CONFIG_SERIAL_DEBUG=y` (menuconfig or
`sdkconfig.defaults`), which streams binary debug frames over the serial
console as long as a dashboard client keeps the connection alive. The dashboard
also serves a local JSON HTTP API, so scripts and LLM agents can read the same
telemetry and send the same commands while the window stays open (see
**Local API** below). The connection picker can also link the dashboard to the
robot over the nRF24L01 radio bridge, simultaneously with the USB debug link
(see **Conexão por rádio** below).

## Run

From the repository root (host or devcontainer):

```bash
python3 -m robot_bench                        # connection picker (USB or radio), then dashboard
python3 -m robot_bench --port /dev/ttyUSB0    # skip the picker
python3 -m robot_bench --demo                 # synthetic data, no hardware
python3 -m robot_bench --tab 1                # open on the Calibração tab
python3 -m robot_bench --tab 3                # open on the ToF tab
python3 -m robot_bench --tab 4                # open on the IMU tab
python3 -m robot_bench --tab 5                # open on the Console tab
python3 -m robot_bench --screenshot-dir tmp/gui   # save PNG frames
python3 -m robot_bench --api-port 9000            # local API on another port
python3 -m robot_bench --api-port 0 --api-socket ""   # local API disabled
```

Requires `dearpygui` and `pyserial`. In the devcontainer, use `python -m robot_bench`;
these dependencies are installed in the PlatformIO Python environment, selected by `python`.
The dashboard renders through OpenGL; the devcontainer image installs the Mesa software
renderer and forwards the host X11 socket, so it works without a GPU.

## GUI

Resizable window (minimum 1024x640, maximizable to full screen) with six
tabs:

| Tab         | Content                                                                 |
| ----------- | ----------------------------------------------------------------------- |
| Motors      | Sub-tabs **Visão geral** (four selectable wheel cards with analog/FG feedback, speed/target/pulse plots, motor controls, sweep, position panel) and **Dados brutos** (raw encoder/motor values and plots) |
| Calibração  | Sub-tabs: encoder wiring alignment (Encoders) and control calibration (Controlador) |
| Health      | System, lifecycle, warnings, power and sensor availability             |
| ToF         | Full-size 8x8 distance heatmap with millimetre readings and colour scale |
| IMU         | Orientation box and twelve orientation / sensor plots                   |
| Console     | Full-size serial log with history scrolling and text selection          |

Connection status stays at the top and stream details at the bottom. The
serial console has a dedicated tab; messages continue to arrive while other
tabs are open. Each wheel card shows the wheel name and the RPM reading, with
the live values below them (angle, target, PWM, direction, pulse speed and
count); the angle is a live number only and is not plotted. The speed plot fills
whatever the rows leave, so it stays the largest element of the card. The card
also shows the digital FG pulse feedback of the BL48250 motors: the pulse-window
speed next to the analog speed and the cumulative FG edge count, both shown as
"—" until the firmware reports a pulse baseline for that wheel. The speed plot
has a third magenta pulse trace with a small colour legend in the caption. Click a wheel to
select it, then use the speed adjustment, sweep or **Stop all motors** buttons.
Stopping cancels the sweep so it cannot restart the motors. The adjustment buttons
and the square sweep are fractions of the calibrated wheel-speed limit, which the
sidebar shows as "Máximo calibrado: ±138 RPM"; before a control calibration the
fallback limit is 4000/3 RPM and the line says it is not calibrated. The sidebar
switches between **Controle** (wheel target and sweep), **Posição** (wheel
position moves), and **Ajuste** (live velocity-controller settings). Keyboard shortcuts remain
available: `[1-4]` select wheel, `[UP/DOWN]` adjust by the RPM step shown in Controle,
`[LEFT/RIGHT]` +/-100 %, `[SPACE]` stop all motors, `[W]` square sweep between
+/- the limit, `[F1-F7]` switch tabs (`F7` is Rádio while connected), `[Q/ESC]` quit.

In **Ajuste**, select a wheel, enter a signed speed and press **Definir RPM**.
The plots keep showing measured speed and target while you edit. The **Ativo**
column shows the values read from the controller. Hover over a parameter name
for its effect and units. The PID gains are shared between directions and apply
in continuous mode. **Frente (+)** and **Ré (−)** select the feedforward,
starting duty and low-speed pulse settings for each direction. **Aplicar ao vivo**
sends the edited fields together at the current target and reports the values
accepted by the firmware. **Recarregar** discards pending edits. Manual changes
are held in RAM and the stored profile loads again at boot.

The **Dados brutos** sub-tab of Motors shows the unfiltered sensor path one wheel at a
time, so the plots get most of the tab height. A selector row has one button per motor
(M1..M4): the selected wheel's button is wide with its full name and the accent colour,
while the other three stay compact, so the selected motor clearly owns most of the row. The
selection follows the global wheel choice (button click, 1-4 keys or an overview card) and
focuses the shared CD4052B mux on that wheel's encoder; leaving the raw view releases the
focus so the other encoders are no longer read. The wide header lists the selected motor's raw analog ADC
mean, raw AS5600 I2C angle (counts and degrees; "—" when the focused mux read failed),
analog encoder angle, cumulative FG count, FG delta of the last 100 ms, EKF speed (less
filtered than the control value), filter pulse speed, smoothed control RPM and the applied
signed PWM with its direction, plus the raw/I2C/stale status. Three large 12-second plots
fill the rest: the encoder angle (cyan, "ADC (ângulo)") and the raw I2C angle (amber,
"I2C (ângulo)") on one 12-bit count scale, FG delta (violet) and the
EKF/pulse/control speed traces. Every history plot labels its vertical scale on the left
and fits it to the visible trace. The ADC trace converts the raw ADC mean to equivalent 12-bit
encoder-angle counts with the wheel's stored adc_min_raw/adc_max_raw endpoints (clamped to
0..4095), so the two encoder readings are comparable; the raw ADC counts stay visible in the
header. Without a valid stored range the trace falls back to the raw ADC counts and is labeled
"ADC bruto (sem faixa)".

The Health tab reports health faults as warnings while wheel commands remain available.
The firmware stops wheel targets set by the dashboard when its streaming session ends.

On the Console tab, scroll up to browse history; scroll down to return to live
messages. Drag across console text to copy the selection to the clipboard. The
**Limpar** button in the panel header drops the history and returns the view to
the live messages. The console also shows the plain text the board prints
directly on the port, so a reset is readable from `ets Jun  8 2016` and the
ESP-IDF startup log onwards, before the firmware starts sending its own framed
messages.
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

## Conexão por rádio (nRF24L01)

The connection picker offers two links: a robot on a USB debug port
(**Conectar USB**) or the robot over the nRF24L01 radio bridge (**Conectar
rádio**). Both can be connected **at the same time**: the radio link appears as
a **Rádio** tab in the dashboard, so the USB tabs keep driving the robot while
the radio tab follows its communication. The **Conexões** button in the
dashboard header reopens the picker to add, change or disconnect a link; the
picker's **Voltar** button returns to the dashboard. The bridge is the CH340 USB
adapter (`1a86:7523`) that `scripts/nrf24_usb.py` uses, and the picker detects
it automatically. Only one process can hold the adapter, so close other radio
tools first. The adapter is never probed as a robot port: every byte written to
it is transmitted over the air.

While the picker is open the dashboard auto-discovers the active robots: it
sends a telemetry request to every robot id 0..10 over the radio and lists every
robot that answers, with round-trip time, battery and error flags (**Buscar
robôs** repeats the search). **Conectar rádio** on a result row already uses the
discovered id, so the manual **Robô alvo** field is only needed when discovery
is not usable. If no robot answers, the picker retries while it remains open.

A fresh radio motion command (pose or velocity fields) takes control of the wheels.
An all-zero radio command (**Parar**) or an expired command zeroes the radio targets
and releases control. With both links connected, the USB dashboard can then send
new targets until the next radio motion command.

The radio link uses `proto/ssl_robot_protocol.bitproto`, like `scripts/nrf24_usb.py`.
The session alternates commands and unicast `RobotTelemetryRequest` packets
with at least 50 ms between adapter writes, for about 10 Hz each when replies
arrive promptly. The firmware replies with `RobotTelemetry` and stops the
wheels if commands expire after 250 ms.

The **Rádio** tab shows up in the dashboard while the link is up. Open
**Teclado** and select **Ativar teclado**. Hold **W/A/S/D** or the arrow keys
to translate in the robot frame, and **Q/E** to turn. Two perpendicular keys
move diagonally. The keyboard speed is in wheel RPM and starts at the
50 RPM minimum. Release the keys or press **Space** to stop. The wheels also
stop while the dashboard window is unfocused, and a key held through a focus
change only drives again after it is released. Leaving the keyboard view
disarms it. Motion also stops if fresh radio replies are lost.

Other radio controls:

- **Telemetria**: link state, target robot, round-trip time, TX/RX counters,
  battery percentage, kicker voltage, yaw, pose and active warnings.
- **Manual**: x, y, angle, vx, vy and w fields (mm, 0.01 degree, mm/s,
  0.01 rad/s) with **Enviar movimento contínuo** and **Parar**, plus a one-shot kick
  (1..255) with **Chutar**.
- **Configuração do robô**: control calibration, magnetometer calibration, set
  robot id and factory reset. Calibration and the id change need the **Robô
  levantado · rodas livres** confirmation; the factory reset has its own
  confirmation (it erases NVS and reboots the robot).
- **Mensagens**: the link and command log.

**Desconectar** stops the command stream; when only the radio is connected it
returns to the connection picker, otherwise the dashboard stays open with the
USB link. The robot stops on its own once the commands stop.

## Local API (automation and LLM agents)

The dashboard owns the serial port, so it is the only process that talks to the
robot. It also serves a small JSON HTTP API over that live state: reads return
the latest telemetry frames, writes call exactly the commands the GUI controls
call. A human keeps using the window while an agent reads frames, moves the
wheels, runs calibrations and follows the console in parallel. Telemetry comes
from the streams requested by the visible tab and sub-tab. Other frame
endpoints return stale data or an empty result until their screen is opened.
Attitude is returned by `/api/imu`.

Two local-only listeners serve the same API:

| Listener     | Address                    | Reaches                                                |
| ------------ | -------------------------- | ------------------------------------------------------ |
| TCP loopback | `http://127.0.0.1:8765`    | processes in the same machine or container             |
| TCP (opt-in) | `--api-host 0.0.0.0`       | local-network machines; public/web peers are refused   |
| Unix socket  | `tmp/robot_bench_api.sock` | host and container agents (bind-mounted repo), mode 0600 |

The console prints both addresses at startup. The defaults need no flags: an
agent on the host reaches a dashboard running inside the devcontainer through
the same socket path in the bind-mounted repository, and processes in the
container can use loopback TCP.

Access is limited to this machine and the local network; the web cannot use the
API. The TCP listener binds loopback by default, and `--api-host 0.0.0.0` opens
it to the local network. Every request is checked before it reaches any command:
the client must be loopback, a private (RFC1918) or link-local address, and the
request must not carry browser or proxy headers (`Origin`, `Sec-Fetch-Site`,
`X-Forwarded-For`, `Forwarded`); anything else gets HTTP 403. The Unix socket is
created with mode 0600, so only the dashboard user can connect. There is still no
authentication, so do not create port forwards, tunnels or reverse proxies to
this port: they run inside the machine and are outside the API's control.

```bash
python3 -m robot_bench                                  # 127.0.0.1:8765 + socket
python3 -m robot_bench --api-port 9000                  # another TCP port
python3 -m robot_bench --api-host 0.0.0.0               # add the local network (web stays blocked)
python3 -m robot_bench --api-port 0 --api-socket ""     # disable the API
```

```bash
curl -s http://127.0.0.1:8765/api/state | jq .data.wheels
curl -s --unix-socket tmp/robot_bench_api.sock http://localhost/api/snapshot
curl -s -X POST http://127.0.0.1:8765/api/targets -d '{"wheel": 0, "rpm": 500}'
curl -s -X POST http://127.0.0.1:8765/api/calibration -d '{"action": "start", "confirm": true}'
curl -s 'http://127.0.0.1:8765/api/console?since=0&limit=200'
```

`GET /` lists every endpoint in JSON; HTTP clients get the same catalogue
without reading this file. Wheel indices are 0..3 (M1..M4). Frame reads answer
`{"ok": true, "live": ..., "age_s": ..., "data": ...}` (`/api/status` and
`/api/console` answer their own fields): `live` follows the same freshness
windows as the GUI (1 s for state, IMU, ToF and position; 0.5 s for raw; 1.5 s for
health, calibration and tuning) and `data` is `null` before the first frame arrives.
Errors answer `{"ok": false, "error": "..."}` with 400 (malformed body), 403
(client outside the local network, or a browser/proxy request), 404 (unknown
endpoint), 405 (unsupported method), 409 (command refused by the dashboard), 413 (body too
large) or 500.

| Method | Path                | Body                                                       | Content |
| ------ | ------------------- | ---------------------------------------------------------- | ------- |
| GET | `/api/status` | – | link state, frame counters, dashboard options, the calibrated `max_rpm` and `raw_encoder_wheel` |
| GET | `/api/state` | – | four wheels (PWM, RPM, target, angle, FG pulse speed and count), encoder and pulse masks |
| GET | `/api/health` | – | safety lifecycle, faults, supply voltage and heap |
| GET | `/api/tof` | – | 8x8 distance grid in mm with the sensor status codes |
| GET | `/api/imu` | – | acceleration (g), rotation (°/s) and magnetic field (mG) |
| GET | `/api/calibration` | – | wiring calibration frame with maps, per-wheel encoder stages and `adc_min_raw`/`adc_max_raw` endpoints, plus `state_name`, `error_name`, `wheel_error_names` and `stage_names` |
| GET | `/api/tuning` | – | control calibration frame plus `state_name`, `phase_name`, `error_name`, forward/reverse parameter/metric arrays, `max_rpm` and `wheel_max_rpm` |
| GET | `/api/position` | – | position frame plus `error_name` |
| GET | `/api/raw` | – | raw ADC/I2C encoder values, FG pulse deltas, EKF RPM and applied PWM |
| POST | `/api/raw_encoder` | `{"wheel": 0..3}` or `{"wheel": null}` | focus the raw I2C read on one wheel's encoder (null releases) |
| GET | `/api/console` | `?limit=100&since=N` | console lines; `since` returns only newer lines, `next_since` is the next cursor |
| GET | `/api/snapshot` | `?console=100` | status and every frame in one call |
| POST | `/api/targets` | `{"targets_rpm": [r1, r2, r3, r4]}` or `{"wheel": 0, "rpm": 500}` | set wheel targets, clamped to the calibrated limit |
| POST | `/api/stop` | – | zero every wheel target and cancel the sweep |
| POST | `/api/sweep` | `{"enabled": true}` | dashboard square sweep on the selected wheel |
| POST | `/api/calibration` | `{"action": "start", "confirm": true}` or `{"action": "stop"}` | wiring calibration |
| POST | `/api/tuning` | `{"action": "start", "confirm": true}` or `{"action": "stop"}` | control calibration |
| POST | `/api/position` | `{"action": "move", "moves": [{"wheel": 0, "angle_deg": 45}], "absolute": false, "max_rpm": 300}` or `{"action": "stop"}` | wheel position moves |

The safety rules are the same for both paths. A calibration start needs
`"confirm": true` (the dashboard arms its own confirmation checkbox, so the
human sees it), wheel targets are refused while a calibration runs or without a
live link, and position moves are refused without calibrated encoders. Every API
command is written to the Console tab, so the human sees what the agent did.
Commands from the API and the window share one `BenchState`; the last writer wins
and `/api/console?since=N` is the reliable way to follow a running routine.

## Posição das rodas

The **Posição** sub-tab of the Motors sidebar commands wheel position moves. Enter an angle in
degrees, choose **Ângulo absoluto** or leave it relative to the current angle, set the speed cap
and press **Girar** for one wheel. The firmware runs a cascade position loop (PD outer loop
driving the tuned velocity loop), holds the target when it is reached and stops after a timeout.
**Parar posição** cancels every move. Absolute angles count from boot; relative moves start from
the measured angle. Measured angle, error and state of each wheel are shown live. Position moves
require calibrated encoders and are refused while a radio motion command or a calibration
controls the wheels. In `--demo` mode the panel simulates the proportional approach.

## Calibração dos motores (Encoders)

The Calibração tab has two internal sub-tabs. **Encoders** drives the firmware's
motor/encoder wiring calibration; **Controlador** (next section) drives the per-wheel control
calibration. Run Encoders first: the control calibration needs a valid motor/encoder map.

The Encoders sub-tab makes the robot spin one motor at a time in open loop and watches every
encoder input:
the analog output of each connector (ENCM1..ENCM4) and every channel of the
CD4052B I2C mux. The detected map is `wheel -> analog input` (the path used for
closed-loop control) plus `wheel -> I2C mux channel` (the encoder pair behind
the mux).

The safety warning on the sub-tab is mandatory: check **Robô levantado · rodas
livres** and keep the robot lifted or the wheels free before pressing
**Alinhar encoders**. The wheels spin during the routine and must not be
touched; **Parar calibração** aborts the run. Wheel targets are ignored while
the routine runs, so the Motors tab cannot fight the calibration.

While the routine runs the tab shows its progress, which wheel is spinning, the
movement measured on each analog input and mux channel (one bar each, normalised
to the column peak), and the per-wheel detection result (both encoder paths,
analog only, failed or pending). A wheel that did not match shows **Falhou** and
the status line names each failed wheel with the path that failed (analógico or
I2C) and its reason; the routine still visits all four wheels, and when a wheel
fails the map stored in NVS is left untouched.
The right panel shows the stored map and whether it came from a calibration run
or is still the default identity map. The firmware saves the detected map in NVS
and applies it immediately, so it survives reboots.

The firmware also enforces the encoder operation mode at boot and when a calibration starts:
the AS5600 analog output must be in REDUCED mode (0.1..0.9 VCC) so its voltage stays inside
the ESP32 ADC linear range (the factory FULL mode spans 0..VCC and can clip). The right panel
adds a **Modo** column per wheel (REDUCED green, FULL amber, PWM red, "—" unknown/unverified)
and a badge: green "Encoders em modo REDUCED" when all four are verified REDUCED, otherwise a
warning naming the wheels that are FULL/PWM/unknown; a non-REDUCED encoder is not usable for
the analog control path. Opening **Encoders** and finishing alignment each request one readback
while the wheels have zero targets. Motor telemetry uses the last readback. The same table
has an **ADC mín–máx** column with the stored raw ADC
endpoints of each wheel's encoder (the 0 and 360 degree values, e.g. "410–3680"), or "—" when
no range is stored (range_mask bit clear) or when the stored pair is unusable (max <= min, for
example a wheel whose raw range was rejected while the mV fallback kept the mask bit, like an
intermittent analog harness). Those raw endpoints are what the firmware prefers for the angle
mapping, and they make the 0/360 degree wrap verifiable. In `--demo` mode the sub-tab
simulates the same 7.2 s routine with all four encoders REDUCED and plausible ADC ranges so the
layout, the good badge and the endpoint column can be exercised without hardware.

## Calibração do controle (Controlador)

Start in **Calibração → Controlador** with a valid encoder map and
**Robô levantado · rodas livres** checked. Keep the robot lifted throughout
the run. **Parar calibração** aborts it.

The firmware measures breakaway duty, the motor response time, and the lowest
stable continuous speed for each wheel in both directions. Below that speed,
the controller adjusts pulsed drive from encoder travel and coasts early when
the wheel is predicted to reach the target. Calibration compares pulse
amplitude, drive between pulses, pulse spacing, and coast timing. It chooses the
candidate with the smallest measured peak error, using RMS ripple to break ties.
If the first settings give a surge on one side and underspeed on the other,
calibration tests coast timing between those measured outcomes.

The common maximum is the smallest of the four measured wheel maxima. Each
wheel's maximum is the lower of its full-duty forward and reverse speeds.
Calibration checks both sides of each wheel's transition, then runs all four
wheels together at 50 RPM intervals and at the exact common maximum. The
shared checks use 12-second captures for pulsed operation and cross-check the
encoder travel with motor feedback counts. The console reports mean speed,
minimum and maximum speeds, and RMS ripple from sliding 100 ms travel windows.

At every shared checkpoint, each mean must be within 5% of target and the
spread across wheel and direction means must also be within 5%. Each window
must stay within 35% of target under pulsed drive and within 18% under
continuous drive. Calibration tries alternate qualified pulse
settings on wheels that miss a checkpoint and saves the profile only after
the shared range passes. A failed or interrupted run keeps the previous settings. Wheel target
commands are ignored during the run, which can take up to 20 minutes.

Progress, the current wheel and its phase (Atrito, Identificação, Velocidade máxima,
Validação, Refino, Gravação) are shown live, together with the speed step response of the
wheel under test and the per-wheel result. The results table has two rows per wheel, **M1 +**
(forward, positive target) and **M1 −** (reverse): validated HIGH-speed rise time, overshoot
and steady-state error of each direction; the low-speed validation results are logged to the
Console. The parameters table also shows two rows per wheel: Kp/Ki/Kd are common (Ki is the
calibrated integral gain, duty per rad/s per second), while the friction intercept (Avanço)
and the velocity feedforward (v_ff) are per direction. At idle the values shown are the ones
stored in NVS. A line under
the tables shows each wheel's measured maximum and the common limit ("Limite comum:
138.0 RPM"), with "—" for wheels not measured in the session, and the finished status names the
stored limit. Run the wiring calibration first (Encoders sub-tab): control calibration needs a
valid motor/encoder map. In `--demo` mode the sub-tab simulates a 20 s two-direction control
calibration with asymmetric forward/reverse values and wheel maxima 175/170/165/138 RPM that
produce a 138 RPM limit.

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
| `motors.py`    | Wheel cards with FG pulse feedback, plots, motor controls and the position panel |
| `calibration.py` | Calibração tab: Encoders/Controlador sub-tabs, detection bars and stored map |
| `tuning.py`    | Control calibration content (PID auto-tuning) embedded in the Controlador sub-tab |
| `raw.py`       | Motors "Dados brutos" sub-tab: wide selected-motor selector, mux focus, raw values and three large plots |
| `health.py`    | System, safety, power and sensor status                              |
| `sensing.py`   | Distance heatmap, orientation and sensor measurements                 |
| `console.py`   | Console drawlist, history scrolling, text selection and clear button  |
| `radio.py`     | nRF24L01 USB adapter transport, robot radio codec, command session    |
| `radio_panel.py` | Radio link screen: telemetry, movement commands and robot configuration |
| `picker.py`    | First-screen connection picker: USB debug ports and radio adapters    |
| `app.py`       | Dear PyGui viewport, main loop, phases (picker/dashboard/radio), input |
| `cli.py`       | Argument parsing and wiring                                           |
| `api.py`       | Local JSON HTTP API over the shared dashboard state                   |

The view modules build Dear PyGui widgets once and refresh their values from
`BenchState` every frame. `app.py` routes mouse and keyboard input through
`perform_action`; commands stay in `BenchState`. The demo source produces
telemetry and uses the same sweep timing as a real session.
The API module runs its listeners in their own threads and reads/writes the same
`BenchState`; it never opens the serial port itself.

## GUI checks

Run inside the devcontainer, from `/workspaces/ssl_firmware`:

```bash
ruff check scripts/ robot_bench/
python -m unittest discover -s robot_bench/tests -v
```

The unit tests cover state handling, commands, formatting and protocol
parsing; they do not open a window, so they also run without a display.

The checks cover stop/sweep behavior, target changes and
command routing; motor calibration freshness, button rules, level normalisation,
result/error wording, per-wheel encoder-mode text and the REDUCED/non-REDUCED badge,
the ADC min–max endpoint column (stored range vs range_mask), start/stop routing and the
simulated routine frames (all REDUCED with plausible ADC ranges in the demo); layout
space reservation and connection/status wording; picker ordering; console history
clamping and anchoring, hit testing, selection text, clipboard copy, clearing and scrolling;
wheel card status, digital pulse text and hit testing; the seven-tab layout with the
radio tab; the simultaneous USB + radio mode (tab visibility per link, combined header
badge, radio panel placeholder, keep-port scanner exclusion, discovery rows); health
freshness, lifecycle, flags, supply, IMU
calibration and connection rows; history sample filtering, gap and wraparound
breaks, automatic scaling, distance colours and rotation math; raw range transport
(including fragmented frames, quality labels, hover text, stale/waiting modes);
the IMU freshness and layout helpers; control-calibration freshness, gating, phase/result
wording, per-direction row/metric formatting, max-speed phase/limit texts, demo frames and
command routing; wheel position parsing, frame encoding, move/stop gating and row formatting
(Motors sidebar); the Calibração sub-tab height and selection helpers; calibrated max-RPM
tracking, target clamping and sweep amplitude, Motors fraction buttons/labels and the
configurable arrow-key RPM step (Controle panel, default 10 RPM); the raw view helpers, per-wheel raw history, I2C sentinel handling, selector
labels/widths and sync, distinct ADC/I2C plot colours, plot height, mux focus command/clearing,
status and stale/disconnect clearing; and the local API (index, freshness payloads with the FG pulse
fields and the raw frame, calibrated limit in the status, target clamping, console cursor,
command validation, safety gating and both listeners); and the radio link
(bitproto command, config and telemetry-request encoders cross-checked against generated
bindings, telemetry decoding, adapter detection, discovery, command/request alternation,
write spacing, session snapshots, panel formatting, and picker radio rows). Run a firmware
build first to generate the bindings.

## Serial protocol

Defined in `proto/serial_debug_protocol.bitproto`. Every frame is
`COBS(msg_type | bitproto message | crc8)` followed by a `0x00` delimiter:

| Direction     | Message        | Rate     | Content                                     |
| ------------- | -------------- | -------- | ------------------------------------------- |
| robot -> host | `DebugLink`    | 4 Hz     | uptime and link heartbeat                   |
| robot -> host | `DebugState`   | ~83 Hz   | 4 wheels PWM/speed/target/angle + FG pulse speed/count and masks |
| robot -> host | `DebugImu`     | 10 Hz    | attitude and calibrated accel / gyro / magnetometer |
| robot -> host | `DebugTof`     | ~15 Hz   | 64 raw distances + 64 sensor status codes  |
| robot -> host | `DebugHealth`  | 2 Hz     | lifecycle, faults, supply voltage, heap     |
| robot -> host | `DebugText`    | per line | captured ESP_LOG console lines              |
| robot -> host | `DebugCalibration` | ~5 Hz | routine state/progress, mapping and live levels |
| robot -> host | `DebugTuning` | ~5 Hz | control-calibration state/phase, per-wheel gains and forward/reverse step metrics, calibrated speed limit and per-wheel maxima |
| robot -> host | `DebugRaw` | ~10 Hz | raw ADC/I2C encoder values, FG pulse deltas, EKF speed and applied PWM |
| robot -> host | `DebugPosition` | 10 Hz | position-control masks, continuous wheel angles and targets |
| host -> robot | `DebugSubscribe` | on view change and every 0.9 s | requested stream mask and keepalive |
| host -> robot | `DebugControl` | on event | stop stream, wheel targets, wiring/control calibration, raw encoder focus |
| host -> robot | `DebugPositionControl` | on event | relative/absolute wheel position moves and stop |

`DebugState` uses 61 payload bytes, including four 13-byte wheel records.
Encoder angles use 0.1° units and FG pulse speeds use 1 RPM units. Measured
speed and target retain 0.1 RPM units. Attitude travels in `DebugImu`.
`DebugTof` is 196 bytes: a 4-byte
sequence, 64 signed 16-bit distances, and 64 unsigned 8-bit statuses. `DebugCalibration` is 70 bytes: routine state and
current wheel, progress, per-path masks (analog, I2C and both-paths result),
stored-map flag, four analog and four mux mappings, eight signed 16-bit movement
levels, range mask, abort-level error code, four per-wheel failure reasons, a sequence
counter, four per-wheel AS5600 output stages (0 FULL, 1 REDUCED, 2 PWM, 3 unknown), a
validity mask for those stages, and four per-wheel raw ADC minima plus four maxima (the stored
0/360 degree endpoints, 0 when that wheel has no stored range), followed by four magnet warnings in
mux input order and the current run's wheel-to-mux assignments, valid where the matching I2C bit is set. `DebugTuning` is
108 bytes: state, wheel, phase, progress, result and error masks, four gain/feedforward arrays
(Kp x100, Ki x100, Kd x10000, forward friction intercept, forward velocity feedforward x100),
four forward rise times, forward overshoot and steady-state error percentages, a sequence
counter, then the reverse direction: four reverse friction intercepts, four reverse velocity
feedforwards (x100), four reverse rise times, reverse overshoot and steady-state error
percentages, and finally the common calibrated speed limit plus the four measured wheel
maxima (both in 0.1 RPM units, 0 when not measured or stored). `DebugRaw` is 44 bytes:
a sequence, four unsigned 16-bit analog encoder ADC means, four unsigned 16-bit AS5600 I2C
raw angles (0xFFFF when unreadable), four unsigned 16-bit FG pulse deltas, four signed 16-bit
EKF speeds (0.1 RPM) and four signed 16-bit applied PWM duties. `DebugPosition` is
39 bytes: active/done masks, error code, four measured and four target continuous
wheel angles in 0.001 degree units and a sequence counter. `DebugPositionControl`
is 25 bytes: operation, mode, wheel mask, four requested angles (0.001 degree),
speed cap and a command counter. Rebuild the firmware and dashboard together when
changing this layout; generated C and Python definitions share the same source.

The picker requests the link heartbeat to detect the firmware. The dashboard
renews its two-byte subscription at 115200 baud and changes the mask as the
visible screen changes. Telemetry stops on `STOP_STREAM` or 1.5 seconds after
the last subscription. Sensor acquisition and robot control continue
independently of the bench connection.

## Debugging

- `--demo` runs the full GUI without hardware (synthetic IMU/ToF/motor data, a
  simulated wiring calibration started from the Calibração tab's Encoders sub-tab, a
  simulated control calibration from the Controlador sub-tab and wheel position moves
  from the Motors sidebar's Posição sub-tab).
- `--screenshot-dir DIR` saves a PNG of the rendered frame every
  `--screenshot-period` seconds, e.g.:

  ```bash
  python -m robot_bench --demo --screenshot-dir tmp/gui --screenshot-period 2
  ```

  Screenshots are read from the Dear PyGui frame buffer, so a display (or
  Xvfb) is required even when the window is not watched.

- `protocol.py` and `state.py` contain no drawing logic; use them to test
  parsing of captured serial logs.

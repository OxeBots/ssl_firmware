# AGENTS.md - SSL Robot Firmware

## Build / Lint / Test

```bash
pio run                                    # Build
pio run -t upload                          # Upload
pio device monitor                         # Serial monitor

# compile_commands.json for clangd/clang-tidy intellisense. A bare
# `pio run -t compiledb` is also sufficient.
./scripts/regenerate_compiledb.sh

pio check                                  # Lint C/C++ (clang-tidy + cppcheck)
ruff check scripts/                        # Lint Python (rules: ruff.toml)

pio test -e test_logic                     # Logic tests (no hardware)
pio test -e test_hardware                  # Hardware tests (sensors required)
pio test -e test_calibration               # Calibration (interactive)
pio test -vv -f kinematics/test_omnidirectional_robot   # Single test file

python scripts/compile_bitprotos.py        # Compile bitproto definitions
```

Test environments (`platformio.ini`): `test_logic` (kinematics, IMU orientation), `test_hardware` (ADXL345, ITG3200, QMC5883L, BL48250, WheelController), `test_calibration` (interactive sensor calibration).

## Conversational Style

- Keep answers short and concise.
- No emojis in commits, issues, PR comments, or code.
- No fluff or cheerful filler text (e.g., "Thanks @user" not "Thanks so much @user!").
- Technical prose only, be direct.
- When the user asks a question, answer it first before making edits or running implementation commands.
- When responding to user feedback or an analysis, explicitly say whether you agree or disagree before saying what you changed.
- Do only what the user explicitly asked - nothing more, nothing less.
- At the end of each plan, list unresolved questions (if any), ultra-concise.

## Code Quality

- Read files in full before wide-ranging changes, before editing files you have not fully inspected, and when asked to investigate or audit. Do not rely on search snippets for broad changes.
- Implement only what the code needs now - no speculative abstractions, enums, constants, or methods. If a method/enum/parameter has zero callers, delete it. Favor flat code (a single-file driver beats an unasked-for class hierarchy); avoid inheritance, factories, and interfaces unless they serve a concrete, present need. When in doubt, write the simplest thing that works.
- Inline single-line helpers that have only one call site.
- Check modules for external API types; don't guess.
- No inline imports (dynamic type imports). Top-level includes/imports only.
- Never remove or downgrade code to fix type errors from outdated deps; upgrade the dep instead.
- Always ask before removing functionality or code that appears intentional.
- Do not preserve backward compatibility unless the user asks for it.
- Avoid acronyms in your comments.

## Commands

- For ad-hoc scripts, `write` them to a temp file (e.g. `/tmp`), run, edit if needed, remove when done. Don't embed multi-line scripts in `bash` commands.

## Git

- Only use git commands if the user requested.
- Never commit unless the user asks.

If the user's instructions conflict with any rule in this document, ask for explicit confirmation before overriding. Only then execute their instructions.

## Code Style

**Formatting**: enforced by `.clang-format` - run `clang-format -i <file>` before committing. (4-space indent, 100 cols, Allman braces, middle pointer align `int * ptr`, EOF newline.)

**Includes**: system headers `<...>`, project headers `"..."`. Include guards: `#ifndef _FILENAME_H_` / `#define _FILENAME_H_` / `#endif`. Sorting/regrouping handled by clang-format.

**Naming**: classes `PascalCase`; functions/variables/members `snake_case`; constants/macros `UPPER_SNAKE_CASE`; tags `static const char * TAG = "MODULE_NAME"`.

**Types**: fixed-width `<stdint.h>` (`int16_t`, `uint8_t`, ...); ESP-IDF `esp_err_t`, `TickType_t`; linear algebra via `vt::numeric_vector<N>` / `vt::generic_vector<T, N>`.

**Error handling**: `ESP_ERROR_CHECK()` for mandatory HW init; return `esp_err_t` from init and propagate; `ESP_LOGE/W/I(TAG, ...)`; never crash silently - log and recover.

**Concurrency**: FreeRTOS primitives (`SemaphoreHandle_t`, `QueueHandle_t`, mutexes); protect shared state with `data_mutex`; NVS writes from ISR/high-priority tasks go through the `NvsWriteRequest` queue (never direct); singletons via `get_instance()` with static local.

**Architecture**: `src/main.cpp` is a pure init sequencer - no globals, no free-standing runtime functions; all runtime logic in `lib/`; init in dependency order.

**Serialization (bitproto)**: `memset` structs to zero before `Decode*` (bitproto uses bitwise OR); definitions in `proto/ssl_robot_protocol.bitproto`.

**Documentation (Doxygen)**: Doxygen blocks (`/** ... */`) live ONLY in `*.cpp` (file-level and per-function); headers use only `//` comments - guards, includes, declarations, minimal comments. Plain language, no jargon or buzzwords.

## Testing (Unity)

- Path: `test/<COMPONENT>/<test_name>/<test_name>.cpp`. Include the implementation `.cpp` to unit-test internals; include headers as `"omni_robot.h"` (resolves relative to `lib/` subdirs, not `"kinematics/omni_robot.h"`).
- `TEST_ASSERT_*` macros; helper functions for vector comparison with epsilon tolerance.
- I2C config in tests: `.flags = {.enable_internal_pullup = true, .allow_pd = false}` (ESP-IDF v5.5+).
- When changing tested code (API, params, renamed functions, config structs), update the affected `test/` files in the same change. Don't leave tests broken.

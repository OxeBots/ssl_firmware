#!/usr/bin/env bash
# Run the hardware test suite with gcov instrumentation and produce an HTML
# coverage report (ESP-IDF gcov over JTAG).
#
# Requirements:
#   - ESP32 with a JTAG probe wired up (TDI/TDO/TCK/TMS/GND)
#   - Test sensors connected (same setup as `pio test -e test_hardware`)
#   - socat installed on the host
#
# Usage:
#   scripts/run_coverage.sh -f interface/ftdi/esp32_devkitj_v1.cfg -f target/esp32.cfg
#
# While the device idles after the Unity summary, OpenOCD preempts it through
# the gcov debug stubs (`esp gcov dump`) and writes the .gcda counters next to
# the .gcno files in the build directory. gcovr then generates the report in
# .pio/coverage/.

set -euo pipefail

PROJECT_DIR=$(cd "$(dirname "$0")/.." && pwd)
ENV_NAME=test_coverage
UPLOAD_PORT=${UPLOAD_PORT:-/dev/ttyUSB0}
SERIAL_TIMEOUT=${SERIAL_TIMEOUT:-120}

PIO_HOME=${PIO_HOME:-$HOME/.platformio}
OPENOCD=${PIO_HOME}/packages/tool-openocd-esp32/bin/openocd
OPENOCD_SCRIPTS=${PIO_HOME}/packages/tool-openocd-esp32/share/openocd/scripts
GCOV=${PIO_HOME}/packages/toolchain-xtensa-esp-elf/bin/xtensa-esp-elf-gcov
BUILD_DIR=${PROJECT_DIR}/.pio/build/${ENV_NAME}
REPORT_DIR=${PROJECT_DIR}/.pio/coverage
SDKCONFIG=${PROJECT_DIR}/sdkconfig.${ENV_NAME}

usage() {
    echo "usage: $0 <openocd args>   e.g. -f interface/ftdi/esp32_devkitj_v1.cfg -f target/esp32.cfg" >&2
    exit 1
}

enable_sdkconfig_gcov() {
    # The first build generates sdkconfig.test_coverage from sdkconfig.defaults.
    if [ ! -f "${SDKCONFIG}" ]; then
        echo "Generating ${SDKCONFIG} (initial build)..." >&2
        pio run -d "${PROJECT_DIR}" -e "${ENV_NAME}"
    fi
    # Select JTAG as trace destination and switch on gcov transfer. The choice
    # default (DEST_NONE) must be commented out or kconfig keeps both selected.
    # GCOV_ENABLE has no line in a freshly generated sdkconfig because its
    # dependency (APPTRACE_ENABLE) is not selected yet, so append it instead
    # of substituting.
    sed -i \
        -e 's/^CONFIG_APPTRACE_DEST_NONE=y/# CONFIG_APPTRACE_DEST_NONE is not set/' \
        -e 's|^# CONFIG_APPTRACE_DEST_JTAG is not set|CONFIG_APPTRACE_DEST_JTAG=y|' \
        "${SDKCONFIG}"
    grep -q '^CONFIG_APPTRACE_DEST_JTAG=y' "${SDKCONFIG}" || {
        echo "error: could not set CONFIG_APPTRACE_DEST_JTAG in ${SDKCONFIG}" >&2
        exit 1
    }
    grep -q '^CONFIG_APPTRACE_GCOV_ENABLE=y' "${SDKCONFIG}" || \
        echo 'CONFIG_APPTRACE_GCOV_ENABLE=y' >> "${SDKCONFIG}"
}

wait_for_sentinel() {
    # Block until the merged binary announces that every suite has run (or
    # timeout). The final Unity summary lines stream by before the sentinel.
    "${PIO_HOME}/penv/bin/python" - "${UPLOAD_PORT}" "${SERIAL_TIMEOUT}" <<'PYEOF'
import sys
import time

import serial

port, timeout = sys.argv[1], int(sys.argv[2])
ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = port, 115200, 1
ser.dtr, ser.rts = False, False  # Avoid holding the auto-reset circuit
ser.open()
ser.dtr, ser.rts = False, False
deadline = time.time() + timeout
while time.time() < deadline:
    line = ser.readline()
    if b"ALL SUITES DONE" in line:
        sys.stdout.write(line.decode(errors="replace"))
        sys.exit(0)
sys.exit("error: no ALL SUITES DONE sentinel on serial within timeout")
PYEOF
}

dump_coverage() {
    # OpenOCD preempts the idle device through the debug stubs and writes the
    # .gcda files; retry until fresh counters appear.
    local marker="${BUILD_DIR}/.dump_marker"
    touch "${marker}"
    "${OPENOCD}" -s "${OPENOCD_SCRIPTS}" "$@" > "${BUILD_DIR}/openocd.log" 2>&1 &
    local oocd_pid=$!
    local dumped=0
    for _ in $(seq 1 10); do
        sleep 1
        printf 'esp gcov dump\nexit\n' | socat -T5 - TCP:127.0.0.1:4444 \
            >> "${BUILD_DIR}/openocd.log" 2>&1 || true
        if find "${BUILD_DIR}" -name '*.gcda' -newer "${marker}" -print -quit | grep -q .; then
            dumped=1
            break
        fi
    done
    kill "${oocd_pid}" 2>/dev/null || true
    wait "${oocd_pid}" 2>/dev/null || true
    [ "${dumped}" -eq 1 ] || {
        echo "error: gcov dump produced no .gcda files (see ${BUILD_DIR}/openocd.log)" >&2
        exit 1
    }
}

watch_serial() {
    # Background serial watcher: exits 0 once the sentinel line arrives.
    "${PIO_HOME}/penv/bin/python" - "${UPLOAD_PORT}" "${SERIAL_TIMEOUT}" \
        > "${BUILD_DIR}/serial_capture.log" 2>&1 <<'PYEOF' &
import sys
import time

import serial

port, timeout = sys.argv[1], int(sys.argv[2])
ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = port, 115200, 1
ser.dtr, ser.rts = False, False  # Avoid holding the auto-reset circuit
ser.open()
ser.dtr, ser.rts = False, False
deadline = time.time() + timeout
while time.time() < deadline:
    line = ser.readline()
    sys.stdout.write(line.decode(errors="replace"))
    sys.stdout.flush()
    if b"ALL SUITES DONE" in line:
        sys.exit(0)
sys.exit("error: no ALL SUITES DONE sentinel on serial within timeout")
PYEOF
}

main() {
    [ $# -ge 1 ] || usage
    command -v socat > /dev/null || { echo "error: socat is required" >&2; exit 1; }

    enable_sdkconfig_gcov
    find "${BUILD_DIR}" -name '*.gcda' -delete 2>/dev/null || true

    # Pass 1: compile the merged suite. On the first run after source changes
    # the link fails on cross-suite symbol duplicates - that is expected, the
    # objects are what this pass is for.
    echo "=== merged binary: compile (link errors are expected here) ===" >&2
    pio test -d "${PROJECT_DIR}" -e "${ENV_NAME}" -f test_all \
        --without-uploading --without-testing || true

    # Pass 2: objcopy each wrapper object (rename entries/hooks, localize
    # everything else) so the merged link cannot collide.
    echo "=== symbol fixup ===" >&2
    "${PIO_HOME}/penv/bin/python" "${PROJECT_DIR}/scripts/fixup_merged_objects.py" \
        "${PROJECT_DIR}"

    # Pass 3: relink with the fixed objects.
    echo "=== relink ===" >&2
    pio test -d "${PROJECT_DIR}" -e "${ENV_NAME}" -f test_all \
        --without-uploading --without-testing

    # Flash and let the binary run all suites while the serial watcher is
    # already listening (the sentinel cannot be missed).
    echo "=== flash and run ===" >&2
    watch_serial
    local watcher=$!
    pio test -d "${PROJECT_DIR}" -e "${ENV_NAME}" -f test_all \
        --upload-port "${UPLOAD_PORT}" --without-testing
    if ! wait "${watcher}"; then
        echo "error: serial watcher failed (see ${BUILD_DIR}/serial_capture.log)" >&2
        exit 1
    fi
    dump_coverage "$@"

    echo "=== generating report ===" >&2
    mkdir -p "${REPORT_DIR}"
    (cd "${BUILD_DIR}" && gcovr -r "${PROJECT_DIR}" --gcov-executable "${GCOV}" -s \
        --html-details "${REPORT_DIR}/index.html")
    echo "Report: ${REPORT_DIR}/index.html" >&2
}

main "$@"

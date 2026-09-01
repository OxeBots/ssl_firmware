"""Parser for the serial telemetry lines produced by the BenchDashboard test."""

from . import config


def parse_line(state, line):
    """Parse one serial line, updating the shared state."""
    if line.startswith("S,"):
        fields = line.split(",")
        if len(fields) == config.STATE_FIELD_COUNT:
            try:
                state.update_state(
                    int(fields[1]),
                    float(fields[2]),
                    float(fields[3]),
                    float(fields[4]),
                    float(fields[5]),
                    float(fields[6]),
                    float(fields[7]),
                    int(fields[8]),
                    int(fields[9]),
                    int(fields[10]),
                )
            except ValueError:
                pass
    elif line.startswith("T,"):
        fields = line.split(",")
        if len(fields) == config.TOF_FIELD_COUNT:
            try:
                state.update_tof(int(fields[1]), [int(v) for v in fields[2:]])
            except ValueError:
                pass
    elif line:
        # Boot-time line noise can carry non-printable bytes; drop them.
        state.console.append("".join(c for c in line if c.isprintable()))

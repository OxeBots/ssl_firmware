"""Distance heatmap, robot attitude and sensor measurements."""

import dearpygui.dearpygui as dpg

from . import config, rendering

# Vertical space between stacked widgets inside a tab (matches the theme item spacing).
STACK_GAP = 8

# ---------------------------------------------------------------------------
# ToF zone model
# ---------------------------------------------------------------------------

TOF_VALID_STATUS = 5
TOF_UNCERTAIN_STATUSES = (6, 9, 10)
TOF_MISSING_STATUSES = (0, 255)
TOF_NO_RANGE_LABEL = "Sem distância"
TOF_UNKNOWN_LABEL = "Desconhecido"
TOF_STATUS_LABELS = {
    0: "Sem leitura",
    1: "Sinal fraco",
    2: "Erro de fase",
    3: "Ruído alto",
    4: "Inconsistente",
    5: "Válido",
    6: "Incerto",
    7: "Inconsistente",
    8: "Sinal fraco",
    9: "Incerto",
    10: "Novo alvo",
    11: "Inconsistente",
    12: "Desfocado",
    13: "Inconsistente",
    255: "Sem alvo",
}

TOF_MISSING_TEXT = "—"
TOF_STALE_LABEL = "Obsoleto"
TOF_WAITING_LABEL = "Sem dados"
TOF_HINT = "Milímetros · ~ âmbar: incerto · ? vermelho: rejeitado · —: ausente · Passe o mouse para dados brutos"
TOF_TITLE_TEXT = "Distância · 8 × 8"
TOF_STALE_OVERLAY = "Dados de distância desatualizados"
TOF_WAITING_OVERLAY = "Nenhum quadro de distância recebido"
TOF_DARK_TEXT = (20, 22, 26)
TOF_DARK_TEXT_MM = 1400
TOF_LABEL_CELL = 80
# Rendered advance of the monospace font per pixel of drawlist font size.
TOF_MONO_ADVANCE = 0.55
TOF_MIN_GRID = 200
TOF_LEGEND_W = 180
TOF_LEGEND_MAX_H = 420
TOF_LEGEND_OVERHEAD = 40
TOF_HEADER_EXTRA = 20
TOF_GRID_GAP = 24
TOF_BAR_X = 6.0
TOF_BAR_W = 28.0
TOF_BAR_MARGIN = 8.0
TOF_TICK_LENGTH = 8.0
TOF_LEGEND_TICKS = ("0", "0.5", "1.2", "2.5", "4")
TOF_LEGEND_SEGMENTS = 4
TOF_LEGEND_SLICES = 32

TOF_ROOT = "tof_root"
TOF_TITLE = "tof_title"
TOF_SUMMARY = "tof_summary"
TOF_DETAIL = "tof_detail"
TOF_ROW = "tof_row"
TOF_GRID_PAD = "tof_grid_pad"
TOF_GRID = "tof_grid"
TOF_GRID_BG = "tof_grid_bg"
TOF_CELL_BOX = "tof_cell_box"
TOF_CELL_VALUE = "tof_cell_value"
TOF_CELL_STATUS = "tof_cell_status"
TOF_CELL_LABEL = "tof_cell_label"
TOF_HOVER = "tof_hover"
TOF_OVERLAY_BG = "tof_overlay_bg"
TOF_OVERLAY = "tof_overlay"
TOF_LEGEND_COL = "tof_legend_col"
TOF_LEGEND_PAD = "tof_legend_pad"
TOF_LEGEND_TITLE = "tof_legend_title"
TOF_LEGEND = "tof_legend"
TOF_LEGEND_BAR = "tof_legend_bar"
TOF_LEGEND_SLICE = "tof_legend_slice"
TOF_LEGEND_TICK = "tof_legend_tick"
TOF_LEGEND_TICK_LABEL = "tof_legend_tick_label"

# ---------------------------------------------------------------------------
# IMU model
# ---------------------------------------------------------------------------

IMU_ROOT = "imu_root"
IMU_BODY = "imu_body"
IMU_ROWS = "imu_rows"
IMU_ATTITUDE_PANEL = "imu_attitude_panel"
IMU_ATTITUDE_PAD = "imu_attitude_pad"
IMU_CUBE_SPACER = "imu_cube_spacer"
IMU_CUBE = "imu_cube"
IMU_CUBE_PREFIX = "imu_cube_item"
IMU_READOUTS = "imu_readouts"
IMU_STATUS = "imu_attitude_status"
IMU_HINT = "imu_axes_hint"
IMU_PANEL_MARGIN = 34
IMU_PANEL_SAFETY = 26
IMU_ATTITUDE_LABELS = ("Guinada", "Arfagem", "Rolagem")
IMU_READOUT_COLORS = (config.VELOCITY_COLOR, config.OK_COLOR, config.PWM_COLOR)
IMU_ORIENTATION_CHANNELS = (
    ("Guinada", config.VELOCITY_COLOR),
    ("Arfagem", config.OK_COLOR),
    ("Rolagem", config.PWM_COLOR),
)
IMU_SENSOR_CHANNELS = (
    (rendering.AXES[0][1], rendering.AXES[0][2]),
    (rendering.AXES[1][1], rendering.AXES[1][2]),
    (rendering.AXES[2][1], rendering.AXES[2][2]),
)
IMU_ROWS_MODEL = (
    ("Orientação", "°", 30.0, "orientation"),
    ("Aceleração", "g", 0.1, "accel"),
    ("Rotação", "°/s", 1.0, "gyro"),
    ("Campo magnético", "mG", 10.0, "mag"),
)
IMU_SENSOR_INDEX = {"accel": 1, "gyro": 2, "mag": 3}
IMU_WAITING_TEXT = "Aguardando dados"

_TEXT_SIZES = {}
_TOF_STATE = {}
_IMU_STATE = {}


# ---------------------------------------------------------------------------
# Pure helpers: ToF classification and text
# ---------------------------------------------------------------------------


def tof_zone_quality(distance, status):
    """Classify one raw zone reading as valid, uncertain, invalid or missing."""
    if status in TOF_MISSING_STATUSES:
        return "missing"
    if distance > 0 and status == TOF_VALID_STATUS:
        return "valid"
    if distance > 0 and status in TOF_UNCERTAIN_STATUSES:
        return "uncertain"
    return "invalid"


def tof_status_label(distance, status):
    """Label a zone reading, including valid status with no measured range."""
    if distance <= 0 and status == TOF_VALID_STATUS:
        return TOF_NO_RANGE_LABEL
    return TOF_STATUS_LABELS.get(status, TOF_UNKNOWN_LABEL)


def tof_cell_texts(distance, status, mode, show_label):
    """Value and status lines for one heatmap cell."""
    if mode != "live":
        return TOF_MISSING_TEXT, (TOF_STALE_LABEL if mode == "stale" else TOF_WAITING_LABEL)
    quality = tof_zone_quality(distance, status)
    if distance <= 0 or quality == "missing":
        value = TOF_MISSING_TEXT
    elif quality == "uncertain":
        value = f"~{distance}"
    elif quality == "invalid":
        value = f"{distance}?"
    else:
        value = str(distance)
    status_text = f"S{status}"
    if show_label:
        status_text += " " + tof_status_label(distance, status)
    return value, status_text


def tof_cell_style(distance, status, mode):
    """Fill, border, border width and text color for one heatmap cell."""
    if mode != "live":
        return config.INSET, config.PANEL_BORDER, 1.0, config.TEXT_DIM
    quality = tof_zone_quality(distance, status)
    if quality == "missing":
        return config.INSET, config.PANEL_BORDER, 1.0, config.TEXT_DIM
    fill = rendering.distance_color(distance)
    dark = 0 < distance < TOF_DARK_TEXT_MM
    text_color = TOF_DARK_TEXT if dark else config.TEXT
    if quality == "valid":
        return fill, config.BACKGROUND, 1.0, text_color
    if quality == "uncertain":
        return fill, config.DEMO_COLOR, 2.0, text_color
    return fill, config.BAD_COLOR, 2.0, text_color


def tof_zone_counts(distances, statuses):
    """Number of zones in each quality class."""
    counts = {"valid": 0, "uncertain": 0, "invalid": 0, "missing": 0}
    for distance, status in zip(distances, statuses):
        counts[tof_zone_quality(distance, status)] += 1
    return counts


def tof_stream_mode(state, now):
    """Freshness of the range stream: live, stale, or waiting for the first frame."""
    if not state.last_tof_t:
        return "waiting"
    if now - state.last_tof_t > config.SENSOR_STALE_S:
        return "stale"
    return "live"


def tof_summary_text(state, now):
    """One line of frame, quality counts or the stale wording."""
    mode = tof_stream_mode(state, now)
    if mode == "waiting":
        return "Aguardando dados de distância"
    if mode == "stale":
        return f"Dados de distância desatualizados · último quadro {state.tof_seq_seen}"
    counts = tof_zone_counts(state.tof, state.tof_status)
    return (
        f"Quadro {state.tof_seq_seen} · Válidos {counts['valid']} · Incertos {counts['uncertain']}"
        f" · Rejeitados {counts['invalid']} · Ausentes {counts['missing']}"
    )


def tof_hover_text(row, column, distance, status):
    """Detail line for the zone under the mouse."""
    return (
        f"Zona {row + 1},{column + 1} · Distância bruta {distance} mm"
        f" · {tof_status_label(distance, status)} (status {status})"
    )


def tof_layout(body_w, body_h, header_h):
    """Grid and legend sizes for the ToF tab."""
    min_grid = rendering.px(TOF_MIN_GRID)
    legend_w = rendering.px(TOF_LEGEND_W)
    grid_gap = rendering.px(TOF_GRID_GAP)
    stack_gap = rendering.px(STACK_GAP)
    grid_side = int(max(min_grid, min(body_h - header_h, body_w - legend_w - grid_gap)))
    legend_h = int(max(rendering.px(120), min(rendering.px(TOF_LEGEND_MAX_H), grid_side - rendering.px(52))))
    return {
        "grid_side": grid_side,
        "cell_side": grid_side / float(config.TOF_SIDE),
        "legend_h": legend_h,
        "legend_pad": max(0, (grid_side - rendering.px(TOF_LEGEND_OVERHEAD) - legend_h) // 2),
        "left_pad": max(
            0, (body_w - (grid_side + grid_gap + 2 * stack_gap + legend_w)) // 2 - stack_gap
        ),
    }


def tof_legend_colors():
    """Heat colours sampled over the four legend segments."""
    colors = []
    stops = rendering.TOF_COLOR_STOPS
    for (_start_value, start), (_end_value, end) in zip(stops, stops[1:]):
        for slot in range(TOF_LEGEND_SLICES):
            fraction = (slot + 0.5) / TOF_LEGEND_SLICES
            colors.append(tuple(int(a + (b - a) * fraction) for a, b in zip(start, end)))
    return colors


def tof_legend_geometry(legend_h):
    """Bar rectangle and tick positions inside the vertical legend drawlist."""
    top = rendering.pxf(TOF_BAR_MARGIN)
    bar_x = rendering.pxf(TOF_BAR_X)
    bar_w = rendering.pxf(TOF_BAR_W)
    bar_height = max(rendering.pxf(20.0), legend_h - 2.0 * top)
    step = bar_height / (len(TOF_LEGEND_TICKS) - 1)
    ticks = tuple(top + step * index for index in range(len(TOF_LEGEND_TICKS)))
    return {
        "bar": (bar_x, top, bar_x + bar_w, top + bar_height),
        "ticks": ticks,
    }


# ---------------------------------------------------------------------------
# Pure helpers: IMU freshness and formatting
# ---------------------------------------------------------------------------


def imu_attitude_fresh(state, now):
    """True while the latest IMU frame with attitude data is recent."""
    return imu_sensor_fresh(state, now)


def imu_sensor_fresh(state, now):
    """True while the latest raw IMU frame is recent."""
    return bool(state.last_imu_t) and now - state.last_imu_t <= config.SENSOR_STALE_S


def imu_attitude_status(state, now):
    """Status label for the attitude panel and its cube."""
    if not state.last_imu_t:
        return "Aguardando atitude"
    if now - state.last_imu_t > config.SENSOR_STALE_S:
        return "Atitude desatualizada"
    return "Atitude atual"


def format_angle(value):
    """Current orientation reading with one decimal and the degree sign."""
    return f"{value:+.1f}°"


def format_reading(value):
    """Current raw sensor reading with three decimals."""
    return f"{value:+.3f}"


def format_scale(peak):
    """Automatic vertical scale label for one history plot."""
    return f"±{peak:.0f}" if peak >= 10 else f"±{peak:.2g}"


def history_span_label(unit):
    """Row subtitle describing the scrolling history and its unit."""
    return f"Histórico de {config.CHART_WINDOW_S:g} s · {unit}"


def recent_samples(samples, now, window_s=config.CHART_WINDOW_S):
    """Drop samples older than the chart window; the newest sample stays last."""
    cutoff = now - window_s
    start = 0
    for index in range(len(samples) - 1, -1, -1):
        if samples[index][0] < cutoff:
            start = index + 1
            break
    return samples[start:]


def orientation_samples(samples, axis):
    """(timestamp, angle) pairs for one yaw/pitch/roll axis."""
    return [(sample[0], sample[axis + 1]) for sample in samples]


def sensor_samples(samples, sensor_index, axis):
    """(timestamp, reading) pairs for one axis of one raw IMU sensor."""
    return [(sample[0], sample[sensor_index][axis]) for sample in samples]


def imu_layout(body_w, body_h, header_h, label_w):
    """Panel, row and plot sizes for the IMU tab."""
    stack_gap = rendering.px(STACK_GAP)
    attitude_w = int(max(rendering.px(260), min(rendering.px(360), body_w * 0.27)))
    right_w = body_w - attitude_w - stack_gap
    row_h = (body_h - 3 * stack_gap) // len(IMU_ROWS_MODEL)
    return {
        "attitude_w": attitude_w,
        "body_h": body_h,
        "label_w": label_w,
        "row_h": row_h,
        "plot_w": max(rendering.px(80), (right_w - label_w - 3 * stack_gap) // 3),
        "plot_h": max(rendering.px(40), row_h - header_h - stack_gap),
    }


# ---------------------------------------------------------------------------
# Shared drawlist measurements
# ---------------------------------------------------------------------------


def _text_size(text, font_name):
    """Measured (width, height) of a fixed text in drawlist coordinates."""
    key = (text, font_name, rendering.ui_scale())
    cached = _TEXT_SIZES.get(key)
    if cached is None:
        font = rendering.font(font_name)
        measured = dpg.get_text_size(text, font=font) if font is not None else dpg.get_text_size(text)
        cached = (float(measured[0]), float(measured[1]))
        _TEXT_SIZES[key] = cached
    return cached


def _mono_width(text, size=None):
    """Width of a text rendered with the drawlist monospace font at one size."""
    px_size = rendering.font_size("mono") if size is None else size
    return len(text) * TOF_MONO_ADVANCE * px_size


def _cell_text_size(text, cell_side):
    """Largest monospace size that keeps one cell line inside the cell width."""
    limit = cell_side - rendering.px(8)
    min_size = max(8, rendering.px(10))
    for size in range(rendering.font_size("mono"), min_size - 1, -1):
        if _mono_width(text, size) <= limit:
            return size
    return min_size


def _item_height(tag, fallback):
    """Height of an already laid out item, or a fallback before the first layout."""
    return int(dpg.get_item_rect_size(tag)[1]) or fallback


# ---------------------------------------------------------------------------
# ToF tab
# ---------------------------------------------------------------------------


def build_tof(state):
    """Create the ToF tab contents once."""
    min_grid = rendering.px(TOF_MIN_GRID)
    mono_sz = rendering.font_size("mono")
    with dpg.group(tag=TOF_ROOT):
        with dpg.group(horizontal=True):
            rendering.add_text(TOF_TITLE_TEXT, "title", config.TEXT, tag=TOF_TITLE)
            dpg.add_spacer(width=rendering.px(16))
            rendering.add_text("", "small", config.TEXT_DIM, tag=TOF_SUMMARY)
        rendering.add_text(TOF_HINT, "small", config.TEXT_DIM, tag=TOF_DETAIL)
        with dpg.group(horizontal=True, tag=TOF_ROW):
            dpg.add_spacer(width=0, tag=TOF_GRID_PAD)
            with dpg.drawlist(width=min_grid, height=min_grid, tag=TOF_GRID):
                dpg.draw_rectangle(
                    (0.0, 0.0),
                    (0.0, 0.0),
                    fill=rendering.rgba(config.INSET),
                    color=rendering.rgba(config.PANEL_BORDER),
                    tag=TOF_GRID_BG,
                )
                for index in range(config.TOF_SIDE * config.TOF_SIDE):
                    dpg.draw_rectangle(
                        (0.0, 0.0),
                        (0.0, 0.0),
                        fill=rendering.rgba(config.INSET),
                        color=rendering.rgba(config.PANEL_BORDER),
                        tag=f"{TOF_CELL_BOX}_{index}",
                    )
                    value = dpg.draw_text(
                        (0.0, 0.0),
                        "",
                        size=mono_sz,
                        color=rendering.rgba(config.TEXT),
                        tag=f"{TOF_CELL_VALUE}_{index}",
                    )
                    rendering.bind_font(value, "mono")
                    status = dpg.draw_text(
                        (0.0, 0.0),
                        "",
                        size=mono_sz,
                        color=rendering.rgba(config.TEXT),
                        tag=f"{TOF_CELL_STATUS}_{index}",
                    )
                    rendering.bind_font(status, "mono")
                    label_item = dpg.draw_text(
                        (0.0, 0.0),
                        "",
                        size=mono_sz,
                        color=rendering.rgba(config.TEXT),
                        tag=f"{TOF_CELL_LABEL}_{index}",
                        show=False,
                    )
                    rendering.bind_font(label_item, "mono")
                dpg.draw_rectangle(
                    (0.0, 0.0),
                    (0.0, 0.0),
                    fill=(0, 0, 0, 0),
                    color=rendering.rgba(config.ACCENT),
                    thickness=rendering.pxf(2.0),
                    tag=TOF_HOVER,
                    show=False,
                )
                dpg.draw_rectangle(
                    (0.0, 0.0),
                    (0.0, 0.0),
                    fill=rendering.rgba(config.INSET, 235),
                    color=rendering.rgba(config.BAD_COLOR),
                    tag=TOF_OVERLAY_BG,
                    show=False,
                )
                overlay = dpg.draw_text(
                    (0.0, 0.0),
                    TOF_STALE_OVERLAY,
                    size=rendering.font_size("title"),
                    color=rendering.rgba(config.BAD_COLOR),
                    tag=TOF_OVERLAY,
                    show=False,
                )
                rendering.bind_font(overlay, "title")
            dpg.add_spacer(width=rendering.px(TOF_GRID_GAP))
            with dpg.group(tag=TOF_LEGEND_COL):
                dpg.add_spacer(height=0, tag=TOF_LEGEND_PAD)
                rendering.add_text("Metros", "small", config.TEXT_DIM, tag=TOF_LEGEND_TITLE)
                with dpg.drawlist(width=rendering.px(TOF_LEGEND_W), height=rendering.px(200), tag=TOF_LEGEND):
                    for slot, color in enumerate(tof_legend_colors()):
                        dpg.draw_rectangle(
                            (0.0, 0.0),
                            (0.0, 0.0),
                            fill=rendering.rgba(color),
                            tag=f"{TOF_LEGEND_SLICE}_{slot}",
                        )
                    for index in range(len(TOF_LEGEND_TICKS)):
                        dpg.draw_line(
                            (0.0, 0.0),
                            (0.0, 0.0),
                            color=rendering.rgba(config.TEXT_DIM),
                            tag=f"{TOF_LEGEND_TICK}_{index}",
                        )
                        label = dpg.draw_text(
                            (0.0, 0.0),
                            TOF_LEGEND_TICKS[index],
                            size=rendering.font_size("small"),
                            color=rendering.rgba(config.TEXT_DIM),
                            tag=f"{TOF_LEGEND_TICK_LABEL}_{index}",
                        )
                        rendering.bind_font(label, "small")
                    dpg.draw_rectangle(
                        (0.0, 0.0),
                        (0.0, 0.0),
                        color=rendering.rgba(config.PANEL_BORDER),
                        tag=TOF_LEGEND_BAR,
                    )


def update_tof(state, layout, now):
    """Refresh the ToF tab from the latest range frame."""
    body_w = int(layout["body_w"])
    body_h = int(layout["body_h"])
    stack_gap = rendering.px(STACK_GAP)
    title_h = _item_height(TOF_TITLE, rendering.px(24))
    summary_h = _item_height(TOF_SUMMARY, rendering.px(20))
    detail_h = _item_height(TOF_DETAIL, rendering.px(19))
    header_h = max(title_h, summary_h) + stack_gap + detail_h + stack_gap + rendering.px(TOF_HEADER_EXTRA)
    geometry = tof_layout(body_w, body_h, header_h)
    if geometry != _TOF_STATE.get("geometry"):
        _apply_tof_geometry(geometry)
        _TOF_STATE["geometry"] = geometry

    with state.history_lock:
        distances = list(state.tof)
        statuses = list(state.tof_status)
        sequence = state.tof_seq_seen
    mode = tof_stream_mode(state, now)
    rendering.set_text(
        TOF_SUMMARY, tof_summary_text(state, now), config.BAD_COLOR if mode != "live" else config.TEXT
    )
    cells_key = (sequence, mode, geometry["grid_side"])
    if cells_key != _TOF_STATE.get("cells_key"):
        _update_tof_cells(distances, statuses, mode, geometry)
        _TOF_STATE["cells_key"] = cells_key
    _update_tof_hover(distances, statuses, mode, geometry)
    _update_tof_overlay(mode, geometry)


def _tof_cell_lines(distance, status, mode, cell, labelled):
    """Value, status and optional full status label lines for one cell."""
    if mode != "live":
        value, status_text = tof_cell_texts(distance, status, mode, False)
        return value, status_text, ""
    label = tof_status_label(distance, status)
    combined = f"S{status} {label}"
    if labelled and _mono_width(combined) <= cell - rendering.px(8):
        value, status_text = tof_cell_texts(distance, status, mode, True)
        return value, status_text, ""
    value, status_text = tof_cell_texts(distance, status, mode, False)
    if labelled and label:
        return value, status_text, label
    return value, status_text, ""


def _apply_tof_geometry(geometry):
    """Resize the grid and legend, then place the legend bar and its labels."""
    grid_side = geometry["grid_side"]
    tick_len = rendering.pxf(TOF_TICK_LENGTH)
    dpg.configure_item(TOF_GRID, width=grid_side, height=grid_side)
    dpg.configure_item(TOF_GRID_BG, pmin=(0.0, 0.0), pmax=(float(grid_side), float(grid_side)))
    dpg.configure_item(TOF_GRID_PAD, width=geometry["left_pad"])
    dpg.configure_item(TOF_LEGEND, width=rendering.px(TOF_LEGEND_W), height=geometry["legend_h"])
    dpg.configure_item(TOF_LEGEND_PAD, height=geometry["legend_pad"])
    legend = tof_legend_geometry(geometry["legend_h"])
    left, top, right, bottom = legend["bar"]
    segment_h = (bottom - top) / TOF_LEGEND_SEGMENTS
    slice_h = segment_h / TOF_LEGEND_SLICES
    for slot in range(TOF_LEGEND_SEGMENTS * TOF_LEGEND_SLICES):
        slice_top = top + slot * slice_h
        dpg.configure_item(
            f"{TOF_LEGEND_SLICE}_{slot}",
            pmin=(left, slice_top),
            pmax=(right, slice_top + slice_h + 0.5),
        )
    for index, tick_y in enumerate(legend["ticks"]):
        dpg.configure_item(
            f"{TOF_LEGEND_TICK}_{index}",
            p1=(right, tick_y),
            p2=(right + tick_len, tick_y),
        )
        _label_width, label_h = _text_size(TOF_LEGEND_TICKS[index], "small")
        dpg.configure_item(
            f"{TOF_LEGEND_TICK_LABEL}_{index}",
            pos=(right + tick_len + rendering.pxf(5.0), tick_y - label_h / 2.0),
        )
    dpg.configure_item(TOF_LEGEND_BAR, pmin=(left, top), pmax=(right, bottom))


def _update_tof_cells(distances, statuses, mode, geometry):
    """Recolour and relabel every heatmap cell."""
    cell = geometry["cell_side"]
    labelled = cell >= rendering.px(TOF_LABEL_CELL)
    mono_sz = rendering.font_size("mono")
    line_gap = rendering.px(2)
    for index, (distance, status) in enumerate(zip(distances, statuses)):
        row, column = divmod(index, config.TOF_SIDE)
        value, status_text, label_text = _tof_cell_lines(distance, status, mode, cell, labelled)
        fill, border, thickness, text_color = tof_cell_style(distance, status, mode)
        left = column * cell
        top = row * cell
        dpg.configure_item(
            f"{TOF_CELL_BOX}_{index}",
            pmin=(left + 1.0, top + 1.0),
            pmax=(left + cell - 1.0, top + cell - 1.0),
            fill=rendering.rgba(fill),
            color=rendering.rgba(border),
            thickness=thickness * rendering.ui_scale(),
        )
        lines = [(f"{TOF_CELL_VALUE}_{index}", value, mono_sz)]
        status_size = _cell_text_size(status_text, cell)
        lines.append((f"{TOF_CELL_STATUS}_{index}", status_text, status_size))
        if label_text:
            lines.append((f"{TOF_CELL_LABEL}_{index}", label_text, _cell_text_size(label_text, cell)))
        stack = sum(size for _tag, _text, size in lines) + line_gap * (len(lines) - 1)
        y = top + (cell - stack) / 2.0
        for tag, text, size in lines:
            dpg.configure_item(
                tag,
                pos=(left + (cell - _mono_width(text, size)) / 2.0, y),
                text=text,
                size=float(size),
                color=rendering.rgba(text_color),
                show=True,
            )
            y += size + line_gap
        if not label_text:
            dpg.configure_item(f"{TOF_CELL_LABEL}_{index}", show=False)


def _update_tof_hover(distances, statuses, mode, geometry):
    """Detail line and outline for the zone under the mouse."""
    hovered = _tof_hovered_zone(geometry) if mode == "live" else None
    if hovered is None:
        rendering.set_text(TOF_DETAIL, TOF_HINT, config.TEXT_DIM)
        dpg.configure_item(TOF_HOVER, show=False)
        return
    row, column = hovered
    index = row * config.TOF_SIDE + column
    rendering.set_text(
        TOF_DETAIL, tof_hover_text(row, column, distances[index], statuses[index]), config.TEXT
    )
    cell = geometry["cell_side"]
    left = column * cell
    top = row * cell
    dpg.configure_item(
        TOF_HOVER,
        pmin=(left, top),
        pmax=(left + cell, top + cell),
        show=True,
    )


def _tof_hovered_zone(geometry):
    """Zone under the mouse, or None when the mouse is outside the grid."""
    mouse = dpg.get_mouse_pos(local=False)
    origin = dpg.get_item_rect_min(TOF_GRID)
    x = mouse[0] - origin[0]
    y = mouse[1] - origin[1]
    side = geometry["grid_side"]
    if 0 <= x < side and 0 <= y < side:
        cell = geometry["cell_side"]
        return int(y // cell), int(x // cell)
    return None


def _update_tof_overlay(mode, geometry):
    """Centred stale message over the grid."""
    if mode == "live":
        dpg.configure_item(TOF_OVERLAY, show=False)
        dpg.configure_item(TOF_OVERLAY_BG, show=False)
        return
    text = TOF_STALE_OVERLAY if mode == "stale" else TOF_WAITING_OVERLAY
    width, height = _text_size(text, "title")
    side = geometry["grid_side"]
    pos = ((side - width) / 2.0, (side - height) / 2.0)
    dpg.configure_item(
        TOF_OVERLAY,
        text=text,
        pos=pos,
        color=rendering.rgba(config.BAD_COLOR),
        show=True,
    )
    dpg.configure_item(
        TOF_OVERLAY_BG,
        pmin=(pos[0] - rendering.pxf(18.0), pos[1] - rendering.pxf(10.0)),
        pmax=(pos[0] + width + rendering.pxf(18.0), pos[1] + height + rendering.pxf(10.0)),
        show=True,
    )


# ---------------------------------------------------------------------------
# IMU tab
# ---------------------------------------------------------------------------


def _imu_tag(kind, row, axis=None):
    """Item tag for one IMU widget."""
    if axis is None:
        return f"imu_{kind}_{row}"
    return f"imu_{kind}_{row}_{axis}"


def build_imu(state):
    """Create the IMU tab contents once."""
    with dpg.group(tag=IMU_ROOT):
        with dpg.group(horizontal=True, tag=IMU_BODY):
            with rendering.panel("Atitude", IMU_ATTITUDE_PANEL, width=280, height=400):
                dpg.add_spacer(height=0, tag=IMU_ATTITUDE_PAD)
                dpg.add_spacer(height=0, tag=IMU_CUBE_SPACER)
                with dpg.drawlist(width=240, height=240, tag=IMU_CUBE, show=False):
                    rendering.add_cube(IMU_CUBE, IMU_CUBE_PREFIX)
                with dpg.group(tag=IMU_READOUTS):
                    for axis, label in enumerate(IMU_ATTITUDE_LABELS):
                        with dpg.group(horizontal=True):
                            rendering.add_text(f"{label:<5}", "mono", config.TEXT_DIM)
                            rendering.add_text(
                                TOF_MISSING_TEXT,
                                "title",
                                IMU_READOUT_COLORS[axis],
                                tag=_imu_tag("readout", axis),
                            )
                rendering.add_text("Aguardando atitude", "small", config.TEXT_DIM, tag=IMU_STATUS)
                rendering.add_text("Eixos de referência X / Y / Z", "micro", config.TEXT_DIM, tag=IMU_HINT)
            with dpg.group(tag=IMU_ROWS):
                for row, (title, unit, _minimum, source) in enumerate(IMU_ROWS_MODEL):
                    channels = IMU_ORIENTATION_CHANNELS if source == "orientation" else IMU_SENSOR_CHANNELS
                    with dpg.group(horizontal=True, tag=_imu_tag("row", row)):
                        with dpg.group(tag=_imu_tag("label", row)):
                            rendering.add_text(title, "mono", config.TEXT_DIM)
                            rendering.add_text(history_span_label(unit), "micro", config.TEXT_DIM)
                            dpg.add_spacer(width=0, height=0, tag=_imu_tag("label_pad", row))
                        for axis, (channel, color) in enumerate(channels):
                            with dpg.group(tag=_imu_tag("cell", row, axis)):
                                with dpg.group(horizontal=True, tag=_imu_tag("header", row, axis)):
                                    rendering.add_text(
                                        channel, "tiny", color, tag=_imu_tag("channel", row, axis)
                                    )
                                    rendering.add_text(
                                        TOF_MISSING_TEXT,
                                        "tiny",
                                        config.TEXT_DIM,
                                        tag=_imu_tag("value", row, axis),
                                    )
                                    rendering.add_text(
                                        "",
                                        "micro",
                                        config.TEXT_DIM,
                                        tag=_imu_tag("scale", row, axis),
                                    )
                                rendering.add_plot(_imu_tag("plot", row, axis), height=64, width=120)
                                rendering.add_series(_imu_tag("plot", row, axis), 0, color)


def update_imu(state, layout, now):
    """Refresh the IMU tab from the latest attitude and raw sensor frames."""
    body_w = int(layout["body_w"])
    body_h = int(layout["body_h"])
    header_h = _item_height(_imu_tag("header", 0, 0), 16)
    label_w = max(
        max(_text_size(title, "mono")[0] for title, _unit, _minimum, _source in IMU_ROWS_MODEL),
        max(_text_size(history_span_label(unit), "micro")[0] for _title, unit, _minimum, _source in IMU_ROWS_MODEL),
    )
    geometry = imu_layout(body_w, body_h, header_h, label_w)
    if geometry != _IMU_STATE.get("geometry"):
        _apply_imu_geometry(geometry)
        _IMU_STATE["geometry"] = geometry
    _apply_imu_cube_geometry(geometry)

    attitude = imu_attitude_fresh(state, now)
    sensors = imu_sensor_fresh(state, now)
    with state.history_lock:
        orientation = recent_samples(list(state.orientation_history), now)
        sensor_rows = recent_samples(list(state.imu_history), now)
    _update_imu_attitude(state, attitude, now)
    _update_imu_rows(state, attitude, sensors, orientation, sensor_rows, now)


def _apply_imu_geometry(geometry):
    """Resize the attitude panel and all twelve history plots."""
    rendering.resize_item(IMU_ATTITUDE_PANEL, width=geometry["attitude_w"], height=geometry["body_h"])
    for row in range(len(IMU_ROWS_MODEL)):
        dpg.configure_item(_imu_tag("label_pad", row), width=geometry["label_w"])
        for axis in range(3):
            rendering.resize_item(
                _imu_tag("plot", row, axis), width=geometry["plot_w"], height=geometry["plot_h"]
            )


def _apply_imu_cube_geometry(geometry):
    """Size the orientation cube inside the attitude panel."""
    margin = rendering.px(IMU_PANEL_MARGIN)
    stack_gap = rendering.px(STACK_GAP)
    inner_w = geometry["attitude_w"] - margin
    inner_h = geometry["body_h"] - margin
    title_h = _item_height(f"{IMU_ATTITUDE_PANEL}_title", rendering.px(26))
    readouts_h = _item_height(IMU_READOUTS, rendering.px(90))
    status_h = _item_height(IMU_STATUS, rendering.px(19))
    hint_h = _item_height(IMU_HINT, rendering.px(14))
    fixed = title_h + 5 * stack_gap + readouts_h + status_h + hint_h + rendering.px(IMU_PANEL_SAFETY)
    max_cube = inner_h - fixed
    cube_h = max(rendering.px(140), min(int(inner_w * 1.5), max_cube))
    pad = max(0, (max_cube - cube_h) // 2)
    cube = {"width": inner_w, "height": cube_h, "pad": pad}
    if cube == _IMU_STATE.get("cube"):
        return
    _IMU_STATE["cube"] = cube
    rendering.resize_item(IMU_CUBE, width=inner_w, height=cube_h)
    dpg.configure_item(IMU_ATTITUDE_PAD, height=pad)


def _update_imu_attitude(state, fresh, now):
    """Drive the cube, the big readouts and the attitude status label."""
    dpg.configure_item(IMU_CUBE, show=fresh)
    dpg.configure_item(IMU_CUBE_SPACER, height=0 if fresh else _IMU_STATE["cube"]["height"])
    if fresh:
        rendering.update_cube(IMU_CUBE, IMU_CUBE_PREFIX, state.yaw, state.pitch, state.roll)
    values = (state.yaw, state.pitch, state.roll)
    for axis, value in enumerate(values):
        rendering.set_text(
            _imu_tag("readout", axis),
            format_angle(value) if fresh else TOF_MISSING_TEXT,
            IMU_READOUT_COLORS[axis] if fresh else config.TEXT_DIM,
        )
    status = imu_attitude_status(state, now)
    status_color = config.BAD_COLOR
    if status == "Atitude atual":
        status_color = config.OK_COLOR
    elif status == "Aguardando atitude":
        status_color = config.TEXT_DIM
    rendering.set_text(IMU_STATUS, status, status_color)


def _update_imu_rows(state, attitude, sensors, orientation, sensor_rows, now):
    """Update the twelve history plots with traces, current values and scales."""
    for row, (_title, _unit, minimum, source) in enumerate(IMU_ROWS_MODEL):
        orientation_row = source == "orientation"
        samples = orientation if orientation_row else sensor_rows
        fresh = attitude if orientation_row else sensors
        if orientation_row:
            values = (state.yaw, state.pitch, state.roll)
        else:
            values = getattr(state, source)
        sensor_index = IMU_SENSOR_INDEX.get(source, 0)
        for axis in range(3):
            if orientation_row:
                series = orientation_samples(samples, axis)
            else:
                series = sensor_samples(samples, sensor_index, axis)
            xs, ys = rendering.history_series(series, 1, now, wrap=orientation_row)
            rendering.set_series(_imu_tag("plot", row, axis), 0, xs, ys)
            peak = rendering.history_peak(series, 1, now, config.CHART_WINDOW_S, minimum)
            rendering.set_y_limits(_imu_tag("plot", row, axis), -peak, peak)
            value_text = (format_angle if orientation_row else format_reading)(values[axis])
            channel_color = (IMU_ORIENTATION_CHANNELS if orientation_row else IMU_SENSOR_CHANNELS)[axis][1]
            rendering.set_text(
                _imu_tag("value", row, axis),
                value_text if fresh else TOF_MISSING_TEXT,
                channel_color if fresh else config.TEXT_DIM,
            )
            rendering.set_text(
                _imu_tag("scale", row, axis),
                format_scale(peak) if samples else IMU_WAITING_TEXT,
            )

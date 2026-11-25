"""Dear PyGui theme, fonts, panel/plot helpers and orientation geometry."""

import math
import os
from contextlib import contextmanager

import dearpygui.dearpygui as dpg

from . import config

FONT_SIZES = {
    "big": 26,
    "title": 18,
    "label": 16,
    "small": 14,
    "tiny": 12,
    "micro": 10,
}
MONO_FONT_SIZE = 14

# Distance heatmap colormap stops (mm -> RGB), near = hot, far = cold
TOF_COLOR_STOPS = [
    (0.0, (255, 64, 48)),
    (500.0, (255, 190, 40)),
    (1200.0, (96, 220, 96)),
    (2500.0, (64, 150, 255)),
    (4000.0, (150, 100, 240)),
]

CUBE_VERTS = [
    (-1, -1, -1),
    (1, -1, -1),
    (1, 1, -1),
    (-1, 1, -1),
    (-1, -1, 1),
    (1, -1, 1),
    (1, 1, 1),
    (-1, 1, 1),
]
CUBE_FACES = [(0, 1, 2, 3), (5, 4, 7, 6), (4, 0, 3, 7), (1, 5, 6, 2), (4, 5, 1, 0), (3, 2, 6, 7)]
CUBE_EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
AXES = [((1, 0, 0), "X", (235, 90, 80)), ((0, 1, 0), "Y", (110, 220, 130)), ((0, 0, 1), "Z", (90, 160, 255))]

_FONTS = {}
_PLOT_THEME = None


def rgba(color, alpha=255):
    """Convert an RGB palette entry to the RGBA tuple Dear PyGui expects."""
    return (color[0], color[1], color[2], alpha)


def _font_file(candidates):
    for path in candidates:
        if os.path.exists(path):
            return path
    return None


def load_fonts():
    """Create the interface fonts; call once after dpg.create_context()."""
    text_file = _font_file(config.FONT_FILES)
    if text_file is None:
        return
    mono_file = _font_file(config.MONO_FONT_FILES) or text_file
    with dpg.font_registry():
        for name, size in FONT_SIZES.items():
            _FONTS[name] = dpg.add_font(text_file, size)
        _FONTS["mono"] = dpg.add_font(mono_file, MONO_FONT_SIZE)
    dpg.bind_font(_FONTS["label"])


def font(name):
    """Font item for one FONT_SIZES entry, or None when no font file was found."""
    return _FONTS.get(name)


def bind_font(item, name):
    item_font = _FONTS.get(name)
    if item_font is not None:
        dpg.bind_item_font(item, item_font)


def add_text(text="", font_name="label", color=config.TEXT, tag=None, **kwargs):
    """Add a text item with one of the named fonts and an RGB palette color."""
    if tag is not None:
        kwargs["tag"] = tag
    item = dpg.add_text(text, color=rgba(color), **kwargs)
    bind_font(item, font_name)
    return item


def set_text(tag, text, color=None):
    """Update a text item, optionally recoloring it."""
    settings = {"default_value": text}
    if color is not None:
        settings["color"] = rgba(color)
    dpg.configure_item(tag, **settings)


def create_theme(colors=None, styles=None, component=None):
    """Create a theme from {color target: RGB(A)} and {style target: values}."""
    with dpg.theme() as theme:
        with dpg.theme_component(component if component is not None else dpg.mvAll):
            for target, color in (colors or {}).items():
                dpg.add_theme_color(target, rgba(color) if len(color) == 3 else color)
            for target, values in (styles or {}).items():
                dpg.add_theme_style(target, *values)
    return theme


def apply_theme():
    """Install the dark dashboard theme and the shared history plot theme."""
    global _PLOT_THEME
    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvAll):
            for target, color in (
                (dpg.mvThemeCol_WindowBg, config.BACKGROUND),
                (dpg.mvThemeCol_ChildBg, config.PANEL),
                (dpg.mvThemeCol_PopupBg, config.PANEL),
                (dpg.mvThemeCol_Border, config.PANEL_BORDER),
                (dpg.mvThemeCol_Text, config.TEXT),
                (dpg.mvThemeCol_TextDisabled, config.TEXT_DIM),
                (dpg.mvThemeCol_FrameBg, config.INSET),
                (dpg.mvThemeCol_FrameBgHovered, config.TAB_ACTIVE),
                (dpg.mvThemeCol_FrameBgActive, config.TAB_ACTIVE),
                (dpg.mvThemeCol_Button, config.INSET),
                (dpg.mvThemeCol_ButtonHovered, config.TAB_ACTIVE),
                (dpg.mvThemeCol_ButtonActive, config.ACCENT),
                (dpg.mvThemeCol_Tab, config.INSET),
                (dpg.mvThemeCol_TabHovered, config.TAB_ACTIVE),
                (dpg.mvThemeCol_TabSelected, config.TAB_ACTIVE),
                (dpg.mvThemeCol_TabSelectedOverline, config.ACCENT),
                (dpg.mvThemeCol_TabDimmed, config.INSET),
                (dpg.mvThemeCol_TabDimmedSelected, config.TAB_ACTIVE),
                (dpg.mvThemeCol_TabDimmedSelectedOverline, config.ACCENT),
                (dpg.mvThemeCol_Header, config.TAB_ACTIVE),
                (dpg.mvThemeCol_HeaderHovered, config.TAB_ACTIVE),
                (dpg.mvThemeCol_HeaderActive, config.ACCENT),
                (dpg.mvThemeCol_Separator, config.PANEL_BORDER),
                (dpg.mvThemeCol_ScrollbarBg, config.BACKGROUND),
                (dpg.mvThemeCol_ScrollbarGrab, config.PANEL_BORDER),
                (dpg.mvThemeCol_TableHeaderBg, config.PANEL),
                (dpg.mvThemeCol_TableBorderStrong, config.PANEL_BORDER),
                (dpg.mvThemeCol_TableBorderLight, config.PANEL),
            ):
                dpg.add_theme_color(target, rgba(color))
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 16, 16)
            dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 10, 5)
            dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 8, 8)
            dpg.add_theme_style(dpg.mvStyleVar_WindowRounding, 0)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 8)
            dpg.add_theme_style(dpg.mvStyleVar_ChildBorderSize, 1)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 4)
            dpg.add_theme_style(dpg.mvStyleVar_TabRounding, 5)
            dpg.add_theme_style(dpg.mvStyleVar_ScrollbarRounding, 4)
            dpg.add_theme_style(dpg.mvStyleVar_GrabRounding, 4)
    dpg.bind_theme(theme)

    with dpg.theme() as plot_theme:
        with dpg.theme_component(dpg.mvPlot):
            for target, color in (
                (dpg.mvPlotCol_PlotBg, config.INSET),
                (dpg.mvPlotCol_FrameBg, config.INSET),
                (dpg.mvPlotCol_PlotBorder, config.PANEL_BORDER),
                (dpg.mvPlotCol_AxisGrid, config.PANEL_BORDER),
                (dpg.mvPlotCol_AxisText, config.TEXT_DIM),
                (dpg.mvPlotCol_AxisTick, config.PANEL_BORDER),
            ):
                dpg.add_theme_color(target, rgba(color), category=dpg.mvThemeCat_Plots)
            dpg.add_theme_style(dpg.mvPlotStyleVar_LineWeight, 1.5, category=dpg.mvThemeCat_Plots)
            # The charts carry no titles or legends; a small padding leaves the
            # pixels to the traces instead of to the empty border.
            dpg.add_theme_style(dpg.mvPlotStyleVar_PlotPadding, 2.0, 2.0, category=dpg.mvThemeCat_Plots)
    _PLOT_THEME = plot_theme


@contextmanager
def panel(title, tag, width=0, height=0):
    """Child window styled as a dashboard panel with an optional title."""
    with dpg.child_window(tag=tag, width=width, height=height, no_scrollbar=True):
        if title:
            add_text(title, "title", config.TEXT, tag=tag + "_title")
        yield


def resize_item(tag, width=None, height=None):
    """Resize a plot, panel or drawlist while keeping the other dimension."""
    settings = {}
    if width is not None:
        settings["width"] = int(width)
    if height is not None:
        settings["height"] = int(height)
    dpg.configure_item(tag, **settings)


def item_height(tag):
    """Rendered height of an item, or 0 while Dear PyGui has not laid it out yet."""
    if not dpg.does_item_exist(tag):
        return 0
    size = dpg.get_item_state(tag).get("rect_size")
    return size[1] if size else 0


def add_plot(tag, height, width=0):
    """Create an empty history plot with a hidden time axis and a fixed window."""
    with dpg.plot(
        tag=tag,
        height=height,
        width=width,
        no_title=True,
        no_menus=True,
        no_box_select=True,
        no_mouse_pos=True,
        no_inputs=True,
        no_frame=True,
    ):
        dpg.add_plot_axis(
            dpg.mvXAxis, no_tick_labels=True, no_tick_marks=True, no_gridlines=True, tag=tag + "_x"
        )
        # The vertical axis is the scale the traces are read against, so it keeps
        # its tick labels; the time axis stays implicit.
        with dpg.plot_axis(dpg.mvYAxis, tag=tag + "_y"):
            pass
    if _PLOT_THEME is not None:
        dpg.bind_item_theme(tag, _PLOT_THEME)
    dpg.set_axis_limits(tag + "_x", -config.CHART_WINDOW_S, 0.0)
    return tag


def add_series(plot_tag, index, color, thickness=2.0):
    """Add one scrolling line series to a plot; gaps in the data stay gaps."""
    tag = f"{plot_tag}_s{index}"
    dpg.add_line_series([], [], parent=plot_tag + "_y", tag=tag, skip_nan=True)
    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvLineSeries):
            dpg.add_theme_color(dpg.mvPlotCol_Line, rgba(color), category=dpg.mvThemeCat_Plots)
            dpg.add_theme_style(dpg.mvPlotStyleVar_LineWeight, thickness, category=dpg.mvThemeCat_Plots)
    dpg.bind_item_theme(tag, theme)
    return tag


def set_series(plot_tag, index, xs, ys):
    """Replace the samples of one series; NaN entries break the trace."""
    dpg.set_value(f"{plot_tag}_s{index}", (list(xs), list(ys)))


def set_y_limits(plot_tag, low, high):
    dpg.set_axis_limits(plot_tag + "_y", low, high)


def history_series(samples, value_index, now, window_s=config.CHART_WINDOW_S, wrap=False):
    """Plot coordinates for the samples inside the chart window.

    A NaN pair separates samples that are too far apart in time, and, for
    angle series, samples across a 180 degree wrap-around.
    """
    xs, ys = [], []
    previous = None
    for sample in samples:
        timestamp = sample[0]
        if timestamp < now - window_s or timestamp > now:
            continue
        if previous is not None and (
            timestamp - previous[0] > 1.0
            or (wrap and abs(sample[value_index] - previous[value_index]) > 180.0)
        ):
            xs.append(float("nan"))
            ys.append(float("nan"))
        xs.append(timestamp - now)
        ys.append(sample[value_index])
        previous = sample
    return xs, ys


def history_peak(samples, value_index, now, window_s, minimum):
    """Automatic vertical scale with a headroom margin and a lower bound."""
    peak = minimum
    for sample in samples:
        value = sample[value_index]
        if now - window_s <= sample[0] <= now and math.isfinite(value):
            peak = max(peak, abs(value) * 1.15)
    return peak


def series_bounds(series, minimum_span):
    """Vertical limits fitted to the plotted values with a margin.

    Missing values are ignored. A flat or empty trace keeps a range of at
    least minimum_span, so the axis never collapses.
    """
    low = high = None
    for ys in series:
        for value in ys:
            if not math.isfinite(value):
                continue
            low = value if low is None else min(low, value)
            high = value if high is None else max(high, value)
    if low is None:
        return 0.0, minimum_span
    if high - low < minimum_span:
        middle = (low + high) / 2.0
        low, high = middle - minimum_span / 2.0, middle + minimum_span / 2.0
    margin = (high - low) * 0.08
    return low - margin, high + margin


def distance_color(distance_mm):
    """Map a distance in mm to a heatmap color; 0 or negative means invalid."""
    if distance_mm <= 0:
        return (44, 48, 58)
    value = min(float(distance_mm), config.TOF_MAX_MM)
    for (v0, c0), (v1, c1) in zip(TOF_COLOR_STOPS, TOF_COLOR_STOPS[1:]):
        if value <= v1:
            fraction = (value - v0) / (v1 - v0)
            return tuple(int(a + (b - a) * fraction) for a, b in zip(c0, c1))
    return TOF_COLOR_STOPS[-1][1]


def euler_to_matrix(yaw_deg, pitch_deg, roll_deg):
    """Convert yaw, pitch and roll in degrees to a rotation matrix."""
    cy, sy = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    cp, sp = math.cos(math.radians(pitch_deg)), math.sin(math.radians(pitch_deg))
    cr, sr = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def rotate(matrix, vector):
    x, y, z = vector
    return (
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z,
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z,
        matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z,
    )


def project(vertex, matrix, scale, view_dist, cx, cy):
    """Perspective projection of one cube vertex into drawlist coordinates."""
    x, y, z = rotate(matrix, vertex)
    factor = scale * view_dist / (z + view_dist)
    return (cx + x * factor, cy - y * factor, z)


def add_cube(drawlist_tag, prefix):
    """Create the orientation box items once, then drive them with update_cube."""
    for index in range(len(CUBE_FACES)):
        dpg.draw_polygon(
            [(0, 0)] * 4,
            fill=rgba((48, 62, 78)),
            color=rgba(config.BACKGROUND, 0),
            parent=drawlist_tag,
            tag=f"{prefix}_face_{index}",
            show=False,
        )
    for index in range(len(CUBE_EDGES)):
        dpg.draw_line(
            (0, 0),
            (0, 0),
            color=rgba((12, 16, 22)),
            parent=drawlist_tag,
            tag=f"{prefix}_edge_{index}",
        )
    for index, (_vector, label, color) in enumerate(AXES):
        dpg.draw_line(
            (0, 0), (0, 0), color=rgba(color), thickness=2.0,
            parent=drawlist_tag, tag=f"{prefix}_axis_{index}",
        )
        dpg.draw_text(
            (0, 0), label, color=rgba(color), size=14.0,
            parent=drawlist_tag, tag=f"{prefix}_axis_label_{index}",
        )


def update_cube(drawlist_tag, prefix, yaw_deg, pitch_deg, roll_deg):
    """Project and shade the orientation box into its drawlist."""
    width, height = dpg.get_item_rect_size(drawlist_tag)
    if width < 40 or height < 40:
        return
    cx, cy = width / 2.0, height / 2.0
    scale = min(width, height) * 0.27
    view_dist = 4.2
    matrix = euler_to_matrix(yaw_deg, pitch_deg, roll_deg)
    points = [project(vertex, matrix, scale, view_dist, cx, cy) for vertex in CUBE_VERTS]
    light = (-0.45, -0.6, -0.66)
    for face_index, face in enumerate(CUBE_FACES):
        corners = [(points[vertex][0], points[vertex][1]) for vertex in face]
        centroid = rotate(matrix, tuple(sum(CUBE_VERTS[v][k] for v in face) / 4.0 for k in range(3)))
        edge1 = [points[face[1]][k] - points[face[0]][k] for k in range(3)]
        edge2 = [points[face[2]][k] - points[face[0]][k] for k in range(3)]
        normal = [
            edge1[1] * edge2[2] - edge1[2] * edge2[1],
            edge1[2] * edge2[0] - edge1[0] * edge2[2],
            edge1[0] * edge2[1] - edge1[1] * edge2[0],
        ]
        length = math.sqrt(sum(component * component for component in normal)) or 1.0
        normal = [component / length for component in normal]
        if sum(n * c for n, c in zip(normal, centroid)) < 0:
            normal = [-component for component in normal]
        visible = normal[2] < 0.0
        lambert = abs(sum(n * lum for n, lum in zip(normal, light)))
        shade = int(60 + 140 * lambert)
        dpg.configure_item(
            f"{prefix}_face_{face_index}",
            points=corners,
            fill=rgba((int(shade * 0.55), int(shade * 0.75), shade)),
            show=visible,
        )
    for index, (a, b) in enumerate(CUBE_EDGES):
        dpg.configure_item(f"{prefix}_edge_{index}", p1=points[a][:2], p2=points[b][:2])
    origin = project((0, 0, 0), matrix, scale, view_dist, cx, cy)
    for index, (axis_vector, _label, _color) in enumerate(AXES):
        tip = project(axis_vector, matrix, scale, view_dist, cx, cy)
        dpg.configure_item(f"{prefix}_axis_{index}", p1=origin[:2], p2=tip[:2])
        label_pos = (origin[0] + (tip[0] - origin[0]) * 1.12, origin[1] + (tip[1] - origin[1]) * 1.12)
        dpg.configure_item(f"{prefix}_axis_label_{index}", pos=label_pos)

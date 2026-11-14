"""Scrollable serial console with history scrolling and drag text selection."""

import math

import dearpygui.dearpygui as dpg

from . import config, rendering

CONSOLE_LINE_H = 21
CONSOLE_PAD_X = 12
# Height of the panel header inside the drawlist, and the empty space below the last line.
CONSOLE_HEADER_H = 36
CONSOLE_BOTTOM_PAD = 28
CONSOLE_SELECT_COLOR = (56, 90, 148)
CONSOLE_EMPTY_TEXT = "As mensagens do console aparecem aqui"
CONSOLE_LIVE_HINT = "arraste para copiar | role para ver o histórico"
CONSOLE_HISTORY_HINT = "role para baixo para voltar ao vivo"
# Reusable draw items; the tallest supported window needs about 100 rows.
CONSOLE_POOL = 128
# Glyph advance of the monospace font, measured once the viewport exists.
CONSOLE_CHAR_SAMPLE = 64
CONSOLE_CHAR_W = 7.25
CONSOLE_SCROLL_LINES = 3

DRAWLIST = "console_drawlist"

_panel_item = None
_status_item = None
_hint_item = None
_empty_item = None
_highlights = []
_lines = []
_char_w = None
_shown = 0
_clipboard_reported = False
_last_copied_text = None


def console_visible_lines(state, width, height):
    """Console lines drawn in the view and the first line's history index."""
    capacity = _visible_capacity(height)
    lines = list(state.console)
    if not lines:
        return [], 0
    # The history offset never scrolls past the oldest full window.
    offset = min(state.console_offset, max(0, len(lines) - capacity))
    end = len(lines) - offset
    start = max(0, end - capacity)
    return lines[start:end], start


def console_hit_test(state, width, height, pos):
    """(history_line_index, char_index) under a drawlist-local position, or None."""
    lines, start = console_visible_lines(state, width, height)
    if not lines:
        return None
    # The last visible line sits above the bottom padding, older ones above it.
    row = len(lines) - 1 - int(round((height - CONSOLE_BOTTOM_PAD - pos[1]) / CONSOLE_LINE_H))
    row = max(0, min(len(lines) - 1, row))
    text = lines[row]
    x = pos[0] - CONSOLE_PAD_X
    if x <= 0:
        return start + row, 0
    char = min(len(text), int(math.ceil(x / _char_width())))
    return start + row, char


def console_selection_text(state):
    """Clipboard text for the active selection ('' when empty)."""
    if not state.console_sel:
        return ""
    a_line, a_ch = state.console_sel["anchor"]
    b_line, b_ch = state.console_sel["focus"]
    if (a_line, a_ch) > (b_line, b_ch):
        (a_line, a_ch), (b_line, b_ch) = (b_line, b_ch), (a_line, a_ch)
    lines = list(state.console)
    a_line = max(0, a_line)
    b_line = min(len(lines) - 1, b_line)
    if not lines or a_line > b_line:
        return ""
    if a_line == b_line:
        return lines[a_line][a_ch:b_ch]
    parts = [lines[a_line][a_ch:]]
    parts.extend(lines[i] for i in range(a_line + 1, b_line))
    parts.append(lines[b_line][:b_ch])
    return "\n".join(parts)


def build(state):
    """Create the console drawlist, panel header and reusable line items."""
    global _panel_item, _status_item, _hint_item, _empty_item
    with dpg.drawlist(tag=DRAWLIST, width=400, height=300):
        _panel_item = dpg.draw_rectangle(
            (0, 0),
            (400, 300),
            fill=rendering.rgba(config.PANEL),
            color=rendering.rgba(config.PANEL_BORDER),
            rounding=8.0,
        )
        _add_text((CONSOLE_PAD_X, 6), "Console serial", "title", config.TEXT)
        _status_item = _add_text((0, 9), "", "small", config.OK_COLOR)
        _hint_item = _add_text((0, 9), "", "small", config.TEXT_DIM)
        _empty_item = _add_text((0, 0), CONSOLE_EMPTY_TEXT, "small", config.TEXT_DIM, show=False)
        for _ in range(CONSOLE_POOL):
            _highlights.append(
                dpg.draw_rectangle(
                    (0, 0),
                    (0, 0),
                    fill=rendering.rgba(CONSOLE_SELECT_COLOR, 160),
                    color=rendering.rgba(config.PANEL, 0),
                    show=False,
                )
            )
        for _ in range(CONSOLE_POOL):
            _lines.append(_add_text((0, 0), "", "mono", config.TEXT_DIM, show=False))


def update(state, layout):
    """Refresh the console text, selection highlight and header hint."""
    global _char_w, _shown
    width = int(layout["body_w"])
    height = int(layout["body_h"])
    if width <= 0 or height <= 0:
        return
    dpg.configure_item(DRAWLIST, width=width, height=height)
    dpg.configure_item(_panel_item, pmin=(0, 0), pmax=(float(width), float(height)))
    if _char_w is None:
        _char_w = _measure_char_width()
    _update_header(state, width)
    lines, start = console_visible_lines(state, width, height)
    if not lines:
        _hide_lines(0)
        _shown = 0
        message_width = _text_width(CONSOLE_EMPTY_TEXT, "small")
        dpg.configure_item(
            _empty_item,
            pos=((width - message_width) / 2.0, (CONSOLE_HEADER_H + height - CONSOLE_BOTTOM_PAD) / 2.0 - 7.0),
            show=True,
        )
        return
    dpg.configure_item(_empty_item, show=False)
    a_line = b_line = a_ch = b_ch = 0
    if state.console_sel:
        a_line, a_ch = state.console_sel["anchor"]
        b_line, b_ch = state.console_sel["focus"]
        if (a_line, a_ch) > (b_line, b_ch):
            (a_line, a_ch), (b_line, b_ch) = (b_line, b_ch), (a_line, a_ch)
    char_w = _char_width()
    max_text_w = width - 2 * CONSOLE_PAD_X
    for row, raw in enumerate(lines):
        line_top = height - CONSOLE_BOTTOM_PAD - (len(lines) - 1 - row) * CONSOLE_LINE_H
        text = _fit_line(raw, max_text_w)
        color = config.BAD_COLOR if raw.startswith("ERR") else (config.TEXT if raw.startswith(">") else config.TEXT_DIM)
        dpg.configure_item(
            _lines[row],
            text=text,
            pos=(float(CONSOLE_PAD_X), float(line_top + 2)),
            color=rendering.rgba(color),
            show=True,
        )
        absolute = start + row
        highlight = _highlights[row]
        if state.console_sel and a_line <= absolute <= b_line:
            left_ch = 0 if absolute > a_line else min(a_ch, len(text))
            right_ch = len(text) if absolute < b_line else min(b_ch, len(text))
            if right_ch > left_ch:
                dpg.configure_item(
                    highlight,
                    pmin=(CONSOLE_PAD_X + left_ch * char_w, line_top),
                    pmax=(CONSOLE_PAD_X + right_ch * char_w, line_top + CONSOLE_LINE_H - 2),
                    show=True,
                )
                continue
        dpg.configure_item(highlight, show=False)
    _hide_lines(len(lines))
    _shown = len(lines)


def press(state, pos):
    """Start a text selection at a screen position."""
    width, height = dpg.get_item_rect_size(DRAWLIST)
    x, y = _local_position(pos)
    if not (0 <= x < width and 0 <= y < height):
        state.console_sel = None
        return
    hit = console_hit_test(state, width, height, (x, y))
    state.console_sel = {"anchor": hit, "focus": hit} if hit else None


def drag(state, pos):
    """Extend the active text selection to a screen position."""
    if not state.console_sel:
        return
    width, height = dpg.get_item_rect_size(DRAWLIST)
    hit = console_hit_test(state, width, height, _local_position(pos))
    if hit:
        state.console_sel["focus"] = hit


def release(state, pos):
    """Copy the selected console text to the clipboard."""
    global _clipboard_reported, _last_copied_text
    text = console_selection_text(state)
    if not text:
        return
    try:
        dpg.set_clipboard_text(text)
    except Exception as error:
        if not _clipboard_reported:
            _clipboard_reported = True
            reason = str(error) or type(error).__name__
            state.add_console(f"> área de transferência indisponível: {reason}")
        return
    _clipboard_reported = False
    if text != _last_copied_text:
        _last_copied_text = text
        state.add_console(f"> {len(text)} caracteres copiados para a área de transferência")


def wheel(state, delta, pos):
    """Scroll the console history while the pointer is over the console."""
    width, height = dpg.get_item_rect_size(DRAWLIST)
    x, y = _local_position(pos)
    if not (0 <= x < width and 0 <= y < height):
        return
    # Positive deltas scroll up, back into the history.
    offset = state.console_offset + int(delta) * CONSOLE_SCROLL_LINES
    state.console_offset = max(0, min(offset, max(0, len(state.console) - _visible_capacity(height))))


def _add_text(pos, text, font_name, color, show=True):
    """Draw text with an interface font at its configured pixel size."""
    size = rendering.MONO_FONT_SIZE if font_name == "mono" else rendering.FONT_SIZES[font_name]
    item = dpg.draw_text(pos, text, color=rendering.rgba(color), size=float(size), show=show)
    rendering.bind_font(item, font_name)
    return item


def _visible_capacity(height):
    """Console rows that fit between the panel header and its bottom padding."""
    return max(1, (height - CONSOLE_HEADER_H) // CONSOLE_LINE_H)


def _local_position(pos):
    """Drawlist-local position of a screen position."""
    min_x, min_y = dpg.get_item_rect_min(DRAWLIST)
    return pos[0] - min_x, pos[1] - min_y


def _measure_char_width():
    """Advance width of the monospace font, or None before the first frame."""
    sample = "M" * CONSOLE_CHAR_SAMPLE
    font = rendering.font("mono")
    size = dpg.get_text_size(sample, font=font) if font is not None else dpg.get_text_size(sample)
    if not size or size[0] <= 0:
        return None
    return size[0] / CONSOLE_CHAR_SAMPLE


def _char_width():
    """Glyph advance used for hit testing and selection geometry."""
    return _char_w if _char_w else CONSOLE_CHAR_W


def _text_width(text, font_name):
    """Rendered width of a UI string, with a per-character estimate as fallback."""
    font = rendering.font(font_name)
    size = dpg.get_text_size(text, font=font) if font is not None else dpg.get_text_size(text)
    if not size:
        return len(text) * CONSOLE_CHAR_W
    return size[0]


def _fit_line(text, max_w):
    """Truncate one log line with an ellipsis so it fits the console width."""
    char_w = _char_width()
    if len(text) * char_w <= max_w:
        return text
    max_chars = max(0, int(max_w / char_w))
    text = text[:max_chars]
    while text and (len(text) + 3) * char_w > max_w:
        text = text[:-1]
    return text + "..." if text else ""


def _update_header(state, width):
    """Right-align the live or history hint in the panel header."""
    if state.console_offset > 0:
        status, status_color, hint = "Histórico", config.DEMO_COLOR, CONSOLE_HISTORY_HINT
    else:
        status, status_color, hint = "Ao vivo", config.OK_COLOR, CONSOLE_LIVE_HINT
    hint = " | " + hint
    status_width = _text_width(status, "small")
    hint_width = _text_width(hint, "small")
    x = width - CONSOLE_PAD_X - status_width - hint_width
    dpg.configure_item(_status_item, text=status, pos=(float(x), 9.0), color=rendering.rgba(status_color))
    dpg.configure_item(_hint_item, text=hint, pos=(float(x + status_width), 9.0))


def _hide_lines(first):
    """Hide pool items from first upwards that the previous frame displayed."""
    for index in range(first, _shown):
        dpg.configure_item(_lines[index], show=False)
        dpg.configure_item(_highlights[index], show=False)

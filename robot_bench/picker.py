"""First-screen port picker window."""

import dearpygui.dearpygui as dpg

from . import config, rendering

WINDOW = "picker_window"
WIDTH = 760
ROW_COUNT = 5

STATUS_ORDER = {"firmware": 0, "probing": 1, "empty": 2, "busy": 3}
STATUS_TEXT = {
    "probing": ("Verificando firmware...", config.TEXT_DIM),
    "empty": ("Sem resposta", config.TEXT_DIM),
    "busy": ("Porta em uso", config.BAD_COLOR),
}

_rows = [None] * ROW_COUNT
_rows_built = 0
_on_connect = None


def sorted_entries(scan_results):
    """Ports ordered by probe readiness, then by device name."""
    return sorted(scan_results.items(), key=lambda item: (STATUS_ORDER.get(item[1]["status"], 9), item[0]))


def device_for_row(index):
    """Port device last shown on a picker row, or None while unusable."""
    if index is None or not 0 <= index < ROW_COUNT:
        return None
    return _rows[index]


def _add_row(index):
    """Create one port row; the caller rebuilds rows when the port count changes."""
    with dpg.group(tag=f"picker_row_{index}", parent="picker_rows"):
        with dpg.group(horizontal=True):
            rendering.add_text("", "label", tag=f"picker_device_{index}", wrap=230)
            dpg.add_spacer(width=10)
            rendering.add_text("", "tiny", config.TEXT_DIM, tag=f"picker_desc_{index}", wrap=320)
            dpg.add_spacer(width=10)
            dpg.add_button(label="Conectar", tag=f"picker_connect_{index}", callback=_on_connect, user_data=index)
            rendering.add_text("", "small", config.TEXT_DIM, tag=f"picker_status_{index}", wrap=140)
        dpg.add_separator()


def build(on_connect, on_rescan, on_quit):
    """Create the picker window; the caller controls its visibility."""
    global _on_connect
    _on_connect = on_connect
    with dpg.window(
        tag=WINDOW,
        no_title_bar=True,
        no_resize=True,
        no_move=True,
        no_collapse=True,
        no_scrollbar=True,
        width=WIDTH,
        show=False,
    ):
        rendering.add_text(config.APP_NAME, "big", tag="picker_title")
        rendering.add_text("Conecte um robô para ver a telemetria e testar os motores.", "small", config.TEXT_DIM)
        dpg.add_group(tag="picker_rows")
        rendering.add_text("", "title", tag="picker_empty")
        rendering.add_text("", "small", config.TEXT_DIM, tag="picker_hint")
        rendering.add_text("", "tiny", config.TEXT_DIM, tag="picker_more")
        with dpg.group(horizontal=True):
            dpg.add_button(label="Buscar portas", callback=on_rescan, width=130)
            dpg.add_button(label="Sair", callback=on_quit, width=130)
        rendering.add_text("Sem resposta? Feche outros monitores seriais.", "tiny", config.TEXT_DIM)


def update(scan_results, lock):
    """Refresh the scanned port rows."""
    global _rows_built
    with lock:
        entries = sorted_entries(scan_results)[:ROW_COUNT]
    if len(entries) != _rows_built:
        for index in range(_rows_built):
            dpg.delete_item(f"picker_row_{index}")
        for index in range(len(entries)):
            _add_row(index)
        _rows_built = len(entries)
    for index, (device, info) in enumerate(entries):
        found = info["status"] == "firmware"
        _rows[index] = device if found else None
        rendering.set_text(f"picker_device_{index}", device, config.TEXT)
        rendering.set_text(f"picker_desc_{index}", info["desc"], config.TEXT_DIM)
        dpg.configure_item(f"picker_connect_{index}", show=found)
        dpg.configure_item(f"picker_status_{index}", show=not found)
        if not found:
            text, color = STATUS_TEXT.get(info["status"], ("Desconhecido", config.TEXT_DIM))
            rendering.set_text(f"picker_status_{index}", text, color)
    for index in range(len(entries), ROW_COUNT):
        _rows[index] = None
    for tag in ("picker_empty", "picker_hint"):
        dpg.configure_item(tag, show=not entries)
    if not entries:
        rendering.set_text("picker_empty", "Nenhum robô encontrado", config.TEXT)
        rendering.set_text(
            "picker_hint", "Conecte o cabo USB. As portas são verificadas automaticamente.", config.TEXT_DIM
        )
    extra = max(0, len(scan_results) - ROW_COUNT)
    dpg.configure_item("picker_more", show=bool(extra))
    if extra:
        rendering.set_text("picker_more", f"+{extra} dispositivos", config.TEXT_DIM)

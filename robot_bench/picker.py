"""First-screen connection picker window: USB debug port or nRF24L01 radio adapter."""

import dearpygui.dearpygui as dpg

from . import config, radio, rendering

WINDOW = "picker_window"
WIDTH = 760
HEIGHT = 560
ROW_COUNT = 5
RADIO_ROBOT_MAX = radio.ROBOT_ID_MAX

STATUS_ORDER = {"firmware": 0, "probing": 1, "empty": 2, "busy": 3, "unavailable": 4}
STATUS_TEXT = {
    "probing": ("Verificando firmware...", config.TEXT_DIM),
    "empty": ("Sem resposta", config.TEXT_DIM),
    "busy": ("Porta em uso", config.BAD_COLOR),
    "unavailable": ("Porta indisponível", config.BAD_COLOR),
}

# Auto-discovery of active robots: the results are one row per robot, inside
# a scrollable child window so up to ROBOT_ID_MAX + 1 robots fit the picker.
DISCOVERY_HINT = (
    f"A busca consulta a telemetria dos robôs com ID 0..{RADIO_ROBOT_MAX}."
)
DISCOVERY_EMPTY = "Nenhum robô ativo no rádio."
DISCOVERY_RUNNING = "Buscando robôs ativos no rádio..."
DISCOVERY_CHILD_HEIGHT = 110

_rows = [None] * ROW_COUNT
_rows_built = 0
_radio_devices = []
_radio_discovered = []
_on_connect = None
_on_connect_radio = None
_on_discover_radio = None
_on_back = None


def sorted_entries(scan_results):
    """Ports ordered by probe readiness, then by device name."""
    return sorted(scan_results.items(), key=lambda item: (STATUS_ORDER.get(item[1]["status"], 9), item[0]))


def device_for_row(index):
    """Port device last shown on a picker row, or None while unusable."""
    if index is None or not 0 <= index < ROW_COUNT:
        return None
    return _rows[index]


def radio_robot_id():
    """Robot id selected for radio links, clamped to the protocol range."""
    value = int(dpg.get_value("picker_radio_robot"))
    return max(0, min(RADIO_ROBOT_MAX, value))


def set_radio_status(text, color=None):
    """Show one status line under the radio section; an empty text hides it."""
    dpg.configure_item("picker_radio_status", show=bool(text))
    if text:
        rendering.set_text("picker_radio_status", text, color)


def discovered_label(telemetry):
    """Row label of one robot found by the radio discovery."""
    return f"Robô {telemetry['robot_id']}"


def discovered_detail(telemetry):
    """Telemetry summary shown next to one discovered robot."""
    errors = radio.error_flag_names(telemetry["error_flags"])
    errors_text = "Erros: " + ", ".join(errors) if errors else "sem erros"
    return f"RTT {telemetry['rtt_ms']:.0f} ms · Bateria {telemetry['battery_pct']} % · {errors_text}"


def _connect_radio(_sender, _app_data, user_data):
    if _on_connect_radio is not None:
        _on_connect_radio(user_data)


def _discover_radio(_sender=None, _app_data=None, _user_data=None):
    if _on_discover_radio is not None:
        _on_discover_radio()


def _connect_discovered(_sender, _app_data, user_data):
    if not _radio_devices or _on_connect_radio is None:
        return
    dpg.set_value("picker_radio_robot", int(user_data["robot_id"]))
    _on_connect_radio(_radio_devices[0][0])


def _add_row(index):
    """Create one port row; the caller rebuilds rows when the port count changes."""
    with dpg.group(tag=f"picker_row_{index}", parent="picker_rows"):
        with dpg.group(horizontal=True):
            rendering.add_text("", "label", tag=f"picker_device_{index}", wrap=230)
            dpg.add_spacer(width=10)
            rendering.add_text("", "tiny", config.TEXT_DIM, tag=f"picker_desc_{index}", wrap=320)
            dpg.add_spacer(width=10)
            dpg.add_button(
                label="Conectar USB", tag=f"picker_connect_{index}", callback=_on_connect, user_data=index
            )
            rendering.add_text("", "small", config.TEXT_DIM, tag=f"picker_status_{index}", wrap=140)
        dpg.add_separator()


def _add_radio_row(index, device, description):
    """Create one adapter row; the caller rebuilds rows when the list changes."""
    with dpg.group(tag=f"picker_radio_row_{index}", parent="picker_radio_rows"):
        with dpg.group(horizontal=True):
            rendering.add_text(device, "label", config.TEXT, tag=f"picker_radio_device_{index}", wrap=230)
            dpg.add_spacer(width=10)
            rendering.add_text(description, "tiny", config.TEXT_DIM, tag=f"picker_radio_desc_{index}", wrap=320)
            dpg.add_spacer(width=10)
            dpg.add_button(
                label="Conectar rádio",
                tag=f"picker_radio_connect_{index}",
                width=130,
                callback=_connect_radio,
                user_data=device,
            )
        dpg.add_separator()


def _add_discovered_row(index, telemetry):
    """Create one discovery result row; its button connects to that robot."""
    with dpg.group(tag=f"picker_radio_found_row_{index}", parent="picker_radio_found"):
        with dpg.group(horizontal=True):
            rendering.add_text(discovered_label(telemetry), "label", config.TEXT, wrap=90)
            dpg.add_spacer(width=10)
            rendering.add_text(discovered_detail(telemetry), "tiny", config.TEXT_DIM, wrap=380)
            dpg.add_spacer(width=10)
            dpg.add_button(
                label="Conectar rádio",
                tag=f"picker_radio_found_connect_{index}",
                width=130,
                callback=_connect_discovered,
                user_data=telemetry,
            )
        dpg.add_separator()


def build(on_connect, on_rescan, on_quit, on_connect_radio, on_discover_radio, on_back):
    """Create the picker window; the caller controls its visibility."""
    global _on_connect, _on_connect_radio, _on_discover_radio, _on_back
    _on_connect = on_connect
    _on_connect_radio = on_connect_radio
    _on_discover_radio = on_discover_radio
    _on_back = on_back
    with dpg.window(
        tag=WINDOW,
        no_title_bar=True,
        no_resize=True,
        no_move=True,
        no_collapse=True,
        width=WIDTH,
        height=HEIGHT,
        show=False,
    ):
        rendering.add_text(config.APP_NAME, "big", tag="picker_title")
        rendering.add_text("Conecte um robô para ver a telemetria e testar os motores.", "small", config.TEXT_DIM)
        rendering.add_text(
            "USB e rádio podem ficar conectados ao mesmo tempo.",
            "tiny",
            config.TEXT_DIM,
        )
        rendering.add_text("Rádio nRF24L01", "title", tag="picker_radio_title")
        with dpg.group(horizontal=True):
            rendering.add_text("Robô alvo", "label")
            dpg.add_spacer(width=10)
            dpg.add_input_int(
                tag="picker_radio_robot",
                default_value=0,
                width=90,
                min_value=0,
                max_value=RADIO_ROBOT_MAX,
                min_clamped=True,
                max_clamped=True,
                step=1,
            )
        dpg.add_group(tag="picker_radio_rows")
        with dpg.group(horizontal=True):
            dpg.add_button(
                label="Buscar robôs",
                tag="picker_radio_discover",
                width=130,
                callback=_discover_radio,
            )
            rendering.add_text(
                "", "tiny", config.TEXT_DIM, tag="picker_radio_scan", wrap=540, show=False
            )
        with dpg.child_window(
            tag="picker_radio_found", width=-1, height=DISCOVERY_CHILD_HEIGHT, show=False
        ):
            pass
        rendering.add_text(DISCOVERY_EMPTY, "small", config.TEXT_DIM, tag="picker_radio_none", show=False)
        rendering.add_text(
            DISCOVERY_HINT,
            "tiny",
            config.TEXT_DIM,
            wrap=720,
            tag="picker_radio_discovery_hint",
            show=False,
        )
        rendering.add_text(
            "Adaptador nRF24L01 não encontrado (USB).",
            "title",
            config.TEXT,
            tag="picker_radio_empty",
            show=False,
        )
        rendering.add_text(
            "Conecte o adaptador USB do rádio (CH340).",
            "small",
            config.TEXT_DIM,
            tag="picker_radio_hint",
            show=False,
        )
        rendering.add_text("", "small", config.BAD_COLOR, tag="picker_radio_status", show=False)
        dpg.add_separator()
        dpg.add_group(tag="picker_rows")
        rendering.add_text("", "title", tag="picker_empty")
        rendering.add_text("", "small", config.TEXT_DIM, tag="picker_hint")
        rendering.add_text("", "tiny", config.TEXT_DIM, tag="picker_more")
        with dpg.group(horizontal=True):
            dpg.add_button(
                tag="picker_back",
                label="Voltar",
                width=130,
                callback=on_back,
                show=False,
            )
            dpg.add_button(label="Buscar portas", callback=on_rescan, width=130)
            dpg.add_button(label="Sair", callback=on_quit, width=130)
        rendering.add_text("Sem resposta? Feche outros monitores seriais.", "tiny", config.TEXT_DIM)


def update(
    scan_results,
    lock,
    radio_devices,
    discovered=None,
    discovering=False,
    discovery_error="",
    can_back=False,
    usb_connected=False,
    radio_connected=False,
):
    """Refresh the scanned ports, the radio adapters and the discovered robots."""
    global _rows_built, _radio_devices, _radio_discovered
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
        dpg.configure_item(f"picker_connect_{index}", show=found, enabled=not usb_connected)
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

    devices = list(radio_devices)
    if devices != _radio_devices:
        for index in range(len(_radio_devices)):
            dpg.delete_item(f"picker_radio_row_{index}")
        for index, (device, description) in enumerate(devices):
            _add_radio_row(index, device, description)
        _radio_devices = devices
    for tag in ("picker_radio_empty", "picker_radio_hint"):
        dpg.configure_item(tag, show=not devices)

    radio_ready = bool(devices) and not radio_connected
    for index in range(len(devices)):
        dpg.configure_item(
            f"picker_radio_connect_{index}", enabled=not discovering and not radio_connected
        )
    dpg.configure_item(
        "picker_radio_discover",
        show=bool(devices),
        enabled=radio_ready and not discovering,
    )
    dpg.configure_item("picker_radio_discovery_hint", show=bool(devices))
    found_rows = list(discovered or []) if devices else []
    if found_rows != _radio_discovered:
        for index in range(len(_radio_discovered)):
            dpg.delete_item(f"picker_radio_found_row_{index}")
        for index, telemetry in enumerate(found_rows):
            _add_discovered_row(index, telemetry)
        _radio_discovered = found_rows
    dpg.configure_item("picker_radio_found", show=bool(found_rows))
    for index in range(len(found_rows)):
        dpg.configure_item(
            f"picker_radio_found_connect_{index}", enabled=not radio_connected
        )
    dpg.configure_item("picker_back", show=can_back)
    if radio_connected:
        rendering.set_text(
            "picker_radio_scan", "Rádio conectado; desconecte na aba Rádio para trocar.", config.OK_COLOR
        )
        dpg.configure_item("picker_radio_scan", show=True)
    elif discovering:
        rendering.set_text("picker_radio_scan", DISCOVERY_RUNNING, config.TEXT_DIM)
        dpg.configure_item("picker_radio_scan", show=True)
    elif discovery_error:
        rendering.set_text("picker_radio_scan", f"Falha na busca: {discovery_error}", config.BAD_COLOR)
        dpg.configure_item("picker_radio_scan", show=True)
    else:
        dpg.configure_item("picker_radio_scan", show=False)
    dpg.configure_item(
        "picker_radio_none",
        show=bool(devices) and discovered is not None and not found_rows and not discovering,
    )

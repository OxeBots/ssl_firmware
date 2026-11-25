"""System, safety, power and sensor status panels."""

import time

import dearpygui.dearpygui as dpg

from . import config, rendering

GAP = 8

ERROR_FLAG_NAMES = (
    "Comunicação perdida",
    "Falha da IMU",
    "Falha do sensor de distância",
    "Subtensão da bateria",
    "Sobretensão da bateria",
    "Kicker sem carga",
    "Encoder inválido",
    "Aviso de temperatura",
    "Falha do motor 0",
    "Falha do motor 1",
    "Falha do motor 2",
    "Falha do motor 3",
)
LIFECYCLE_NAMES = ("Inicialização", "Autoteste", "Desarmado", "Ativo", "Degradado", "Parada segura", "Calibração")
LIFECYCLE_COLORS = {
    3: config.OK_COLOR,
    4: config.DEMO_COLOR,
    5: config.BAD_COLOR,
    6: config.ACCENT,
}

SYSTEM_PANEL = "health_system"
SAFETY_PANEL = "health_safety"
POWER_PANEL = "health_power"
SUPPLY_VALUE = "health_power_supply_value"
SUPPLY_STATUS = "health_power_supply_status"
ERROR_LIST = "health_safety_errors"

SYSTEM_ROWS = (
    ("ID do robô", "robot_id"),
    ("Tempo ativo", "uptime"),
    ("Memória livre", "free_memory"),
    ("Memória mínima", "minimum_memory"),
    ("Quadros de estado", "state_frames"),
    ("Quadros de distância", "range_frames"),
)
SAFETY_ROWS = (
    ("Ciclo de vida", "lifecycle"),
    ("Kicker", "kicker"),
)
POWER_ROWS = (
    ("Calibração da IMU", "imu_cal"),
    ("Sensor de distância", "range_sensor"),
    ("Rádio", "radio"),
    ("Estação base", "base_station"),
)


def live_health(state):
    """Health snapshot while it is fresh, otherwise an empty mapping."""
    if not state.last_health_t:
        return {}
    if time.monotonic() - state.last_health_t > config.HEALTH_STALE_S:
        return {}
    return state.health


def missing_health_text(state):
    return "Dados de saúde desatualizados" if state.last_health_t else "Aguardando dados de saúde"


def format_uptime(milliseconds):
    seconds = int(milliseconds // 1000)
    return f"{seconds // 3600:02d}:{seconds // 60 % 60:02d}:{seconds % 60:02d}"


def lifecycle_text(lifecycle):
    if lifecycle is None:
        return "—"
    if 0 <= lifecycle < len(LIFECYCLE_NAMES):
        return LIFECYCLE_NAMES[lifecycle]
    return f"Desconhecido ({lifecycle})"


def lifecycle_color(lifecycle):
    if lifecycle is None:
        return config.TEXT_DIM
    return LIFECYCLE_COLORS.get(lifecycle, config.TEXT)


def inhibit_text(inhibited):
    if inhibited is None:
        return "—"
    return "Inibido" if inhibited else "Permitido"


def inhibit_color(inhibited):
    if inhibited is None:
        return config.TEXT_DIM
    return config.BAD_COLOR if inhibited else config.OK_COLOR


def error_flag_names(flags):
    """Names of the set error flag bits, lowest bit first."""
    return [f"{bit}: {name}" for bit, name in enumerate(ERROR_FLAG_NAMES) if flags & (1 << bit)]


def supply_text(voltage):
    return "—" if voltage is None else f"{voltage:.2f} V"


def supply_color(flags):
    if flags is None:
        return config.TEXT_DIM
    if flags & 6:
        return config.DEMO_COLOR
    if flags & 1:
        return config.OK_COLOR
    return config.TEXT_DIM


def supply_status(state, flags):
    if flags is None:
        return missing_health_text(state), config.TEXT_DIM
    if flags & 2:
        return "Subtensão", config.DEMO_COLOR
    if flags & 4:
        return "Sobretensão", config.DEMO_COLOR
    if flags & 1:
        return "Alimentação normal", config.OK_COLOR
    return "Monitor de tensão indisponível", config.TEXT_DIM


def imu_cal_text(flags):
    if flags is None:
        return "—"
    return " ".join(letter if flags & (1 << bit) else "—" for bit, letter in enumerate("AGM"))


def imu_cal_color(flags):
    return config.OK_COLOR if flags == 7 else config.TEXT_DIM


def connection_text(value):
    if value is None:
        return "—"
    return "Conectado" if value else "Desconectado"


def connection_color(value):
    if value is None:
        return config.TEXT_DIM
    return config.OK_COLOR if value else config.DEMO_COLOR


def _add_rows(panel, rows):
    with dpg.table(tag=panel + "_table", header_row=False, policy=dpg.mvTable_SizingStretchProp):
        dpg.add_table_column(width_fixed=True, init_width_or_weight=150)
        dpg.add_table_column(width_stretch=True, init_width_or_weight=1.0)
        for label, part in rows:
            with dpg.table_row():
                rendering.add_text(label, "small", config.TEXT_DIM)
                rendering.add_text("—", "label", config.TEXT, tag=f"{panel}_{part}")


def _set(panel, part, text, color):
    rendering.set_text(f"{panel}_{part}", text, color)


def build(state):
    """Create the Health tab contents once."""
    with dpg.group(horizontal=True):
        with rendering.panel("Sistema", SYSTEM_PANEL, width=1, height=1):
            _add_rows(SYSTEM_PANEL, SYSTEM_ROWS)
        with rendering.panel("Segurança", SAFETY_PANEL, width=1, height=1):
            _add_rows(SAFETY_PANEL, SAFETY_ROWS)
            dpg.add_spacer(height=6)
            dpg.add_separator()
            rendering.add_text("Avisos ativos", "title", config.TEXT)
            rendering.add_text("Aguardando dados de saúde", "small", config.TEXT_DIM, tag=ERROR_LIST)
        with rendering.panel("Alimentação e sensores", POWER_PANEL, width=1, height=1):
            rendering.add_text("Alimentação", "small", config.TEXT_DIM)
            rendering.add_text("—", "big", config.TEXT_DIM, tag=SUPPLY_VALUE)
            rendering.add_text("Aguardando dados de saúde", "small", config.TEXT_DIM, tag=SUPPLY_STATUS)
            dpg.add_spacer(height=4)
            dpg.add_separator()
            _add_rows(POWER_PANEL, POWER_ROWS)
            dpg.add_spacer(height=4)
            rendering.add_text("A: acelerômetro · G: giroscópio · M: magnetômetro", "micro", config.TEXT_DIM)


def _update_system(state, health, now):
    robot_id = health.get("robot_id")
    robot_text = "—" if robot_id is None else str(robot_id)
    _set(SYSTEM_PANEL, "robot_id", robot_text, config.TEXT if robot_id is not None else config.TEXT_DIM)
    uptime_live = state.link_fresh(now)
    _set(
        SYSTEM_PANEL,
        "uptime",
        format_uptime(state.uptime_ms) if uptime_live else "—",
        config.TEXT if uptime_live else config.TEXT_DIM,
    )
    for part, key in (("free_memory", "free_heap"), ("minimum_memory", "min_free_heap")):
        value = health.get(key)
        text = "—" if value is None else f"{value / 1024:.0f} KB"
        _set(SYSTEM_PANEL, part, text, config.TEXT_DIM if value is None else config.TEXT)
    _set(SYSTEM_PANEL, "state_frames", f"{state.state_count:,}", config.TEXT)
    _set(SYSTEM_PANEL, "range_frames", f"{state.tof_count:,}", config.TEXT)


def _update_safety(state, health):
    lifecycle = health.get("lifecycle")
    _set(SAFETY_PANEL, "lifecycle", lifecycle_text(lifecycle), lifecycle_color(lifecycle))
    kicker = health.get("kick_inhibited")
    _set(SAFETY_PANEL, "kicker", inhibit_text(kicker), inhibit_color(kicker))
    flags = health.get("error_flags")
    if flags is not None:
        flags |= health.get("active_faults", 0)
    if flags is None:
        text, color = missing_health_text(state), config.TEXT_DIM
    elif flags == 0:
        text, color = "Nenhum aviso ativo", config.OK_COLOR
    else:
        text, color = "\n".join(error_flag_names(flags)), config.DEMO_COLOR
    rendering.set_text(ERROR_LIST, text, color)


def _update_power(state, health):
    flags = health.get("supply_flags")
    rendering.set_text(SUPPLY_VALUE, supply_text(health.get("supply_voltage")), supply_color(flags))
    status, status_color = supply_status(state, flags)
    rendering.set_text(SUPPLY_STATUS, status, status_color)
    calibration = health.get("imu_cal_flags")
    _set(POWER_PANEL, "imu_cal", imu_cal_text(calibration), imu_cal_color(calibration))
    for part, key in (
        ("range_sensor", "tof_available"),
        ("radio", "radio_ready"),
        ("base_station", "base_station_connected"),
    ):
        value = health.get(key)
        _set(POWER_PANEL, part, connection_text(value), connection_color(value))


def update(state, layout, now):
    """Refresh the Health tab contents."""
    panel_w = max(240, (layout["body_w"] - 2 * GAP) // 3)
    panel_h = layout["body_h"]
    inner_w = panel_w - 34
    health = live_health(state)
    for panel in (SYSTEM_PANEL, SAFETY_PANEL, POWER_PANEL):
        dpg.configure_item(panel, width=panel_w, height=panel_h)
        dpg.configure_item(panel + "_table", width=inner_w)
    _update_system(state, health, now)
    _update_safety(state, health)
    _update_power(state, health)

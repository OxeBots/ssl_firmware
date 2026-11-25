"""Live velocity-controller editor for the selected wheel."""

import math
import time

import dearpygui.dearpygui as dpg

from . import config, rendering
from . import protocol as proto

PANEL = "motors_wheel_tuning"
TARGET = PANEL + "_target"
SEND_TARGET = PANEL + "_send_target"
APPLY = PANEL + "_apply"
RELOAD = PANEL + "_reload"
STATUS = PANEL + "_status"
MODE = PANEL + "_mode"
DIRECTION = PANEL + "_direction"
SECTIONS = PANEL + "_sections"
PULSES = PANEL + "_pulses"
BRANCHES = ("Frente (+)", "Ré (−)")
MODES = ("Parado", "Contínuo · PID", "Pulsado", "Pulsado bloqueado")
POLL_S = 0.75
REPLY_TIMEOUT_S = 1.5

# Display multiplier converts rad/s to RPM and seconds to milliseconds.
GAINS = (
    ("Kp", 1, "Corrige o erro instantâneo de velocidade (PWM por rad/s)."),
    (
        "Ki",
        1,
        "Acumula o erro ao longo do tempo (PWM por rad/s por segundo). "
        "Abaixo de 1, desativa a integral e usa o PWM de partida como piso contínuo.",
    ),
    ("Kd", 1, "Amortece mudanças rápidas da velocidade medida (PWM por rad/s²). Valores altos amplificam o ruído."),
)
DIRECTIONAL = (
    (
        "Intercepto PWM",
        1,
        "Parcela fixa e com sinal do avanço. Soma-se ao ganho de avanço "
        "vezes o alvo em rad/s. Em ré, use o sinal do PWM de ré.",
    ),
    ("Ganho de avanço", 1, "PWM antecipado por rad/s de alvo. Multiplica o alvo com sinal."),
    (
        "PWM de partida",
        1,
        "Duty necessário para sair do repouso. Define o impulso de partida "
        "e ajuda a definir a amplitude inicial no modo pulsado.",
    ),
    (
        "Transição (RPM)",
        30 / math.pi,
        "Abaixo desta velocidade em módulo, o controle usa pulsos. "
        "Zero usa o limiar automático. A transição tem histerese.",
    ),
    ("PWM do pulso", 1, "Amplitude mínima do pulso após a partida. Zero usa o modelo de partida."),
    ("PWM entre pulsos", 1, "Duty mantido nos intervalos entre pulsos, limitado ao PWM do pulso."),
    (
        "PWM médio mín.",
        1,
        "Piso para a fração de tempo em que o pulso fica ligado. "
        "O piso é calculado a partir do PWM do pulso e do intervalo.",
    ),
    (
        "Resposta (ms)",
        1000,
        "Tempo de resposta usado na integral do modo pulsado. Maior tempo reduz essa correção. Há um mínimo interno.",
    ),
    (
        "Antecipação (ms)",
        1000,
        "Projeta a velocidade a partir da aceleração e corta o pulso "
        "antes de ultrapassar o alvo. Zero desativa a antecipação.",
    ),
)
INTERLEAVE = (
    "Intercalar pulsos",
    1,
    "Distribui os instantes ligados ao longo do ciclo de 5 ms. Desmarcado, agrupa-os no início.",
)
PARAMETERS = GAINS + DIRECTIONAL * 2 + (INTERLEAVE,) * 2
INPUTS = tuple(f"{PANEL}_input_{index}" for index in range(len(PARAMETERS)))
ACTIVE = tuple(f"{PANEL}_active_{index}" for index in range(len(PARAMETERS)))
LAST_NUMERIC = len(GAINS) + 2 * len(DIRECTIONAL)


def _display(index, value):
    if index >= LAST_NUMERIC:
        return bool(value)
    return f"{value * PARAMETERS[index][1]:.7g}"


def _changes(values, draft, baseline):
    """Convert only edited fields back to controller units."""
    proposed = list(values)
    fields = 0
    for index, (text, original) in enumerate(zip(draft, baseline)):
        if text == original:
            continue
        try:
            value = float(text.replace(",", ".")) if isinstance(text, str) else float(text)
            if not math.isfinite(value):
                raise ValueError
        except ValueError:
            raise ValueError(PARAMETERS[index][0]) from None
        value /= PARAMETERS[index][1]
        if _display(index, value) != _display(index, values[index]):
            fields |= 1 << index
            proposed[index] = value
    return fields, proposed


class Editor:
    def __init__(self, state, width, button_height, table_theme, stop_theme):
        self.state = state
        self.table_theme = table_theme
        self.wheel = None
        self.sequence = 0
        self.pending = None
        self.last_request = 0.0
        self.last_reply = 0.0
        self.values = None
        self.baseline = [None] * len(PARAMETERS)
        self.feedback = None
        self.enabled = None

        with rendering.panel("Ajuste · M1", PANEL, width=width, height=1):
            with dpg.group(horizontal=True):
                dpg.add_input_float(
                    tag=TARGET,
                    width=75,
                    step=0.0,
                    format="%.1f",
                    on_enter=True,
                    callback=self.set_target,
                )
                dpg.add_button(tag=SEND_TARGET, label="Definir RPM", callback=self.set_target)
                stop = dpg.add_button(label="Parar", callback=lambda: state.stop_motors())
                dpg.bind_item_theme(stop, stop_theme)
            rendering.add_text("Modo —", "tiny", config.TEXT_DIM, tag=MODE)
            with dpg.group(horizontal=True):
                dpg.add_button(
                    tag=APPLY,
                    label="Aplicar ao vivo",
                    height=button_height,
                    callback=self.apply,
                )
                dpg.add_button(
                    tag=RELOAD,
                    label="Recarregar",
                    height=button_height,
                    callback=self.reload,
                )
            rendering.add_text("Lendo controlador…", "micro", config.TEXT_DIM, tag=STATUS, wrap=222)
            rendering.add_text("Ajustes em RAM até reiniciar.", "micro", config.TEXT_DIM)
            with dpg.child_window(height=-1, border=False):
                rendering.add_text("Passe sobre o nome para ver o efeito.", "micro", config.TEXT_DIM)
                dpg.add_combo(
                    BRANCHES,
                    tag=DIRECTION,
                    default_value=BRANCHES[0],
                    width=126,
                    callback=self.select_direction,
                )
                with dpg.tab_bar(tag=SECTIONS):
                    with dpg.tab(label="PID e avanço"):
                        self.build_table(range(len(GAINS)))
                        self.build_directions(pulses=False)
                    with dpg.tab(label="Pulsos", tag=PULSES):
                        self.build_directions(pulses=True)

    def build_directions(self, *, pulses):
        for branch in range(len(BRANCHES)):
            offset = len(GAINS) + branch * len(DIRECTIONAL)
            with dpg.group(tag=f"{PANEL}_{pulses}_{branch}", show=branch == 0):
                if pulses:
                    self.build_table((*range(offset + 3, offset + len(DIRECTIONAL)), LAST_NUMERIC + branch))
                else:
                    self.build_table(range(offset, offset + 3), header=False)

    def build_table(self, indices, *, header=True):
        with dpg.table(
            policy=dpg.mvTable_SizingStretchProp,
            borders_innerH=True,
            header_row=header,
        ) as table:
            rendering.bind_font(table, "tiny")
            dpg.bind_item_theme(table, self.table_theme)
            dpg.add_table_column(label="Parâmetro", init_width_or_weight=0.45)
            dpg.add_table_column(label="Ativo", init_width_or_weight=0.23)
            dpg.add_table_column(label="Editar", init_width_or_weight=0.32)
            for index in indices:
                label, _, explanation = PARAMETERS[index]
                with dpg.table_row():
                    tag = f"{PANEL}_label_{index}"
                    rendering.add_text(label, "tiny", config.TEXT, tag=tag)
                    with dpg.tooltip(tag):
                        rendering.add_text(explanation, "small", config.TEXT, wrap=305)
                    rendering.add_text("—", "tiny", config.TEXT_DIM, tag=ACTIVE[index])
                    if index >= LAST_NUMERIC:
                        dpg.add_checkbox(tag=INPUTS[index])
                    else:
                        dpg.add_input_text(tag=INPUTS[index], width=-1)
                        rendering.bind_font(INPUTS[index], "tiny")

    def select_direction(self, _sender, direction, _user_data=None):
        for branch, name in enumerate(BRANCHES):
            for pulses in (False, True):
                dpg.configure_item(f"{PANEL}_{pulses}_{branch}", show=direction == name)

    def editing(self):
        return dpg.is_item_active(TARGET) or any(dpg.is_item_active(tag) for tag in INPUTS)

    def set_target(self, *_args):
        if self.wheel != self.state.selected_wheel or not self.state._live_link():
            return
        rpm = dpg.get_value(TARGET)
        if math.isfinite(rpm):
            self.state.sweep = False
            self.state.set_selected_target(rpm)

    def send(self, now, fields=0, values=None):
        self.sequence += 1
        self.last_request = now
        if self.state.request_wheel_tuning(self.wheel, self.sequence, fields, values):
            self.pending = (self.sequence, fields)
            return True
        return False

    def apply(self, *_args):
        if not self.enabled or self.wheel != self.state.selected_wheel:
            return
        try:
            fields, proposed = _changes(
                self.values,
                [dpg.get_value(tag) for tag in INPUTS],
                self.baseline,
            )
        except ValueError as error:
            self.feedback = (f"Valor inválido em {error}.", config.BAD_COLOR)
            return
        if fields and self.send(time.monotonic(), fields, proposed):
            self.feedback = ("Aguardando confirmação…", config.TEXT_DIM)
            self.set_enabled(False)
        elif fields:
            self.feedback = ("Envio indisponível. Confira o link e os valores.", config.BAD_COLOR)
        else:
            self.feedback = ("Sem alterações.", config.TEXT_DIM)

    def reload(self, *_args):
        if self.values is not None:
            self.show_values(self.values, replace=(1 << len(PARAMETERS)) - 1)
            self.feedback = None

    def show_values(self, values, replace=0):
        for index, value in enumerate(values):
            displayed = _display(index, value)
            text = ("Sim" if displayed else "Não") if isinstance(displayed, bool) else displayed
            rendering.set_text(ACTIVE[index], text, config.TEXT)
            if (
                replace & (1 << index)
                or self.baseline[index] is None
                or (dpg.get_value(INPUTS[index]) == self.baseline[index] and not dpg.is_item_active(INPUTS[index]))
            ):
                dpg.set_value(INPUTS[index], displayed)
                self.baseline[index] = displayed
        self.values = values

    def set_enabled(self, enabled):
        if self.enabled != enabled:
            for tag in (*INPUTS, APPLY, RELOAD):
                dpg.configure_item(tag, enabled=enabled)
            self.enabled = enabled

    def update(self, now):
        live = self.state._live_link(now)
        wheel = self.state.selected_wheel
        if wheel != self.wheel or (not live and self.values is not None):
            self.wheel = wheel
            self.values = self.pending = self.feedback = None
            self.last_request = self.last_reply = 0.0
            self.baseline = [None] * len(PARAMETERS)
            rendering.set_text(PANEL + "_title", f"Ajuste · M{wheel + 1}")
            for index, tag in enumerate(INPUTS):
                dpg.set_value(tag, False if index >= LAST_NUMERIC else "")
                rendering.set_text(ACTIVE[index], "—", config.TEXT_DIM)
            dpg.set_value(TARGET, 0.0)

        busy = self.state.calibration_running() or self.state.tuning_running()
        reply = self.state.wheel_tuning_reply
        mode = "Modo —"
        if live and self.pending is not None:
            sequence, fields = self.pending
            if reply and (reply["wheel"], reply["sequence"], reply["fields"]) == (
                wheel,
                sequence,
                fields,
            ):
                self.pending = None
                self.last_reply = now
                accepted = reply["status"] == proto.WHEEL_TUNING_OK
                self.show_values(reply["values"], replace=fields if accepted else 0)
                if fields:
                    self.feedback = {
                        proto.WHEEL_TUNING_OK: ("Aplicado · confira os valores ativos.", config.OK_COLOR),
                        proto.WHEEL_TUNING_BUSY: (
                            "Recusado · calibração, rádio ou bloqueio de movimento.",
                            config.BAD_COLOR,
                        ),
                        proto.WHEEL_TUNING_INVALID: ("Recusado · parâmetro inválido.", config.BAD_COLOR),
                    }[reply["status"]]
            elif now - self.last_request >= REPLY_TIMEOUT_S:
                self.pending = None
                if fields:
                    self.feedback = ("Sem confirmação. Confira os valores ativos.", config.BAD_COLOR)

        if live and self.pending is None and now - self.last_request >= POLL_S:
            self.send(now)
        fresh = self.last_reply > 0.0 and now - self.last_reply <= REPLY_TIMEOUT_S
        if live and fresh and reply and reply["wheel"] == wheel and reply["mode"] < len(MODES):
            mode = f"Modo · {MODES[reply['mode']]}"
        applying = self.pending is not None and bool(self.pending[1])
        self.set_enabled(live and fresh and not busy and not applying)
        dpg.configure_item(SEND_TARGET, enabled=live and not busy)
        dpg.configure_item(TARGET, enabled=live and not busy)
        rendering.set_text(MODE, mode, config.TEXT if fresh else config.TEXT_DIM)
        if not live:
            status = ("Sem link serial.", config.TEXT_DIM)
        elif busy:
            status = ("Calibração em andamento.", config.DEMO_COLOR)
        elif not fresh:
            status = self.feedback or ("Aguardando valores do firmware…", config.TEXT_DIM)
        else:
            status = self.feedback or (
                ("Demonstração · valores simulados.", config.DEMO_COLOR)
                if self.state.demo
                else ("Valores ativos sincronizados.", config.TEXT_DIM)
            )
        rendering.set_text(STATUS, *status)

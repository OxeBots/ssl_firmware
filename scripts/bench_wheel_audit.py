#!/usr/bin/env python3
"""Auditoria detalhada de bancada das 4 rodas via API local do Robot Bench.

Coleta dados suficientes para validar e ajustar:
  1. Driver BL48250 (curva real PWM -> RPM nos 2 sentidos, saturacao, deadzone,
     contagem exata de pulsos FG por volta da roda, crosstalk em repouso/giro,
     comportamento em reversao brusca).
  2. Estimador AS5600 + EKF + FG (ruido em repouso, atraso de degrau entre
     deslocamento real do AS5600 vs EKF vs FG, coerencia de sinal e escala).
  3. WheelController e calibracao (erro real do feedforward atual ff + vff*w,
     ripple na faixa pulsada e continua, tempo de subida, overshoot, coast-down,
     sincronia das 4 rodas simultaneas).

Uso:
    python3 scripts/bench_wheel_audit.py
    python3 scripts/bench_wheel_audit.py --wheels 0,1,2,3 --socket tmp/robot_bench_api.sock
    python3 scripts/bench_wheel_audit.py --quick
"""

import argparse
import http.client
import json
import math
import os
import signal
import socket
import statistics
import sys
import time
from datetime import datetime
from typing import Any

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SOCKET = os.path.join(REPO_ROOT, "tmp", "robot_bench_api.sock")
DEFAULT_OUT_DIR = os.path.join(REPO_ROOT, "tmp")

RPM_TO_RAD_S = math.pi / 30.0
RAD_S_TO_RPM = 30.0 / math.pi
WHEEL_NAMES = ("M1", "M2", "M3", "M4")
WHEEL_REDUCTION = 3.0  # reducao eixo do motor -> eixo da roda (BL48250.h)
EXPECTED_FG_COUNTS_PER_MOTOR_REV = 10.0  # CONFIG_MOTOR_FB_COUNTS_PER_REV
EXPECTED_FG_COUNTS_PER_WHEEL_REV = EXPECTED_FG_COUNTS_PER_MOTOR_REV * WHEEL_REDUCTION


class UnixHTTPConnection(http.client.HTTPConnection):
    """HTTPConnection sobre Unix domain socket."""

    def __init__(self, socket_path: str, timeout: float = 5.0):
        super().__init__("localhost", timeout=timeout)
        self._socket_path = socket_path

    def connect(self) -> None:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(self.timeout)
        sock.connect(self._socket_path)
        self.sock = sock


class BenchClient:
    """Cliente leve para a API JSON do Robot Bench (Unix socket ou HTTP loopback)."""

    def __init__(self, socket_path: str | None = None, host: str = "127.0.0.1", port: int = 8765):
        self.socket_path = socket_path
        self.host = host
        self.port = port

    def _conn(self) -> http.client.HTTPConnection:
        if self.socket_path and os.path.exists(self.socket_path):
            return UnixHTTPConnection(self.socket_path, timeout=5.0)
        return http.client.HTTPConnection(self.host, self.port, timeout=5.0)

    def request(self, method: str, path: str, body: dict[str, Any] | None = None) -> dict[str, Any]:
        conn = self._conn()
        try:
            headers = {"Accept": "application/json"}
            payload = None
            if body is not None:
                payload = json.dumps(body).encode("utf-8")
                headers["Content-Type"] = "application/json"
            conn.request(method, path, body=payload, headers=headers)
            resp = conn.getresponse()
            raw = resp.read().decode("utf-8")
            data = json.loads(raw)
            if resp.status >= 400 or not data.get("ok", False):
                raise RuntimeError(f"HTTP {resp.status} em {method} {path}: {data}")
            return data
        finally:
            conn.close()

    def get(self, path: str) -> dict[str, Any]:
        return self.request("GET", path)

    def post(self, path: str, body: dict[str, Any] | None = None) -> dict[str, Any]:
        return self.request("POST", path, body if body is not None else {})

    def stop(self) -> None:
        try:
            self.post("/api/stop", {})
        except Exception:
            pass

    def set_wheel(self, wheel: int, rpm: float) -> dict[str, Any]:
        return self.post("/api/targets", {"wheel": wheel, "rpm": float(rpm)})

    def set_targets(self, targets_rpm: list[float]) -> dict[str, Any]:
        return self.post("/api/targets", {"targets_rpm": [float(v) for v in targets_rpm]})


def wrap_deg_delta(delta_deg: float) -> float:
    """Envolve diferenca angular em [-180, 180)."""
    return (delta_deg + 180.0) % 360.0 - 180.0


def unwrap_angles_deg(angles_deg: list[float]) -> list[float]:
    """Desenrola serie de angulos [0, 360) em graus continuos."""
    if not angles_deg:
        return []
    out = [angles_deg[0]]
    for ang in angles_deg[1:]:
        step = wrap_deg_delta(ang - angles_deg[len(out) - 1])
        out.append(out[-1] + step)
    return out


def sample_state_stream(
    client: BenchClient, duration_s: float, poll_s: float = 0.005
) -> list[dict[str, Any]]:
    """Coleta quadros unicos de /api/state (por uptime_ms, ~83 Hz) durante duration_s."""
    samples: list[dict[str, Any]] = []
    last_uptime = None
    t0 = time.monotonic()
    while True:
        now = time.monotonic()
        if now - t0 >= duration_s:
            break
        resp = client.get("/api/state")
        data = resp.get("data")
        if data is not None:
            up = data.get("uptime_ms")
            if up != last_uptime:
                last_uptime = up
                samples.append({"t_host": now - t0, "uptime_ms": up, "wheels": data["wheels"]})
        time.sleep(poll_s)
    return samples


def wait_until_stopped(
    client: BenchClient, wheels: list[int], timeout_s: float = 2.5, rpm_tol: float = 4.0
) -> float:
    """Aguarda as rodas pararem e retorna o tempo de coast-down ate |rpm| <= rpm_tol."""
    t0 = time.monotonic()
    stopped_since = None
    coast_s = timeout_s
    while time.monotonic() - t0 < timeout_s:
        data = client.get("/api/state").get("data")
        if data is not None:
            ws = data["wheels"]
            still = all(
                abs(ws[w]["rpm"]) <= rpm_tol and abs(ws[w]["pulse_rpm"]) <= rpm_tol for w in wheels
            )
            if still:
                if stopped_since is None:
                    stopped_since = time.monotonic()
                    coast_s = stopped_since - t0
                elif time.monotonic() - stopped_since >= 0.18:
                    return coast_s
            else:
                stopped_since = None
        time.sleep(0.02)
    return coast_s


def compute_angle_velocity_series(
    times_s: list[float], unwrapped_motor_deg: list[float], window_s: float = 0.06
) -> list[float]:
    """Velocidade instantanea da RODA em RPM a partir do angulo bruto do eixo do motor (AS5600)."""
    n = len(times_s)
    vels = [0.0] * n
    if n < 2:
        return vels
    left = 0
    for right in range(1, n):
        while left + 1 < right and (times_s[right] - times_s[left + 1]) >= window_s:
            left += 1
        dt = times_s[right] - times_s[left]
        if dt > 1e-3:
            # 1 volta do motor = 360 deg; 1 volta da roda = 360 * WHEEL_REDUCTION deg
            # 6 * WHEEL_REDUCTION deg_motor/s = 1 RPM na roda
            vels[right] = (unwrapped_motor_deg[right] - unwrapped_motor_deg[left]) / (
                6.0 * WHEEL_REDUCTION * dt
            )
    return vels


def first_crossing_ms(
    times_s: list[float], values: list[float], threshold: float, use_abs: bool = True
) -> float | None:
    """Retorna o primeiro instante (ms) em que |values[i]| >= threshold."""
    for t, v in zip(times_s, values):
        val = abs(v) if use_abs else v
        if val >= threshold:
            return round(t * 1000.0, 1)
    return None


def analyze_step_samples(
    wheel: int,
    target_rpm: float,
    samples: list[dict[str, Any]],
    tuning_data: dict[str, Any] | None,
    steady_window_s: float = 0.85,
) -> dict[str, Any]:
    """Extrai todas as metricas dinamicas, estacionarias, FG vs AS5600 e crosstalk."""
    if len(samples) < 5:
        return {"wheel": wheel, "target_rpm": target_rpm, "valid": False, "n_samples": len(samples)}

    t0_up = samples[0]["uptime_ms"]
    times_s = [(s["uptime_ms"] - t0_up) / 1000.0 for s in samples]
    pwms = [float(s["wheels"][wheel]["pwm"]) for s in samples]
    rpms_ekf = [float(s["wheels"][wheel]["rpm"]) for s in samples]
    rpms_fg = [float(s["wheels"][wheel]["pulse_rpm"]) for s in samples]
    tgts = [float(s["wheels"][wheel]["target_rpm"]) for s in samples]
    angles_raw = [float(s["wheels"][wheel]["angle_deg"]) for s in samples]
    counts = [int(s["wheels"][wheel]["pulse_count"]) for s in samples]

    unwrapped = unwrap_angles_deg(angles_raw)
    rpms_angle_inst = compute_angle_velocity_series(times_s, unwrapped, window_s=0.06)

    # Indice onde o alvo efetivamente apareceu no firmware
    cmd_idx = 0
    for idx, tgt in enumerate(tgts):
        if abs(tgt) > 1.0:
            cmd_idx = idx
            break
    t_cmd = times_s[cmd_idx]
    rel_t = [t - t_cmd for t in times_s[cmd_idx:]]
    ekf_post = rpms_ekf[cmd_idx:]
    fg_post = rpms_fg[cmd_idx:]
    ang_vel_post = rpms_angle_inst[cmd_idx:]
    unw_post = unwrapped[cmd_idx:]
    ang_disp_post = [abs(a - unw_post[0]) for a in unw_post]

    # Atraso de resposta: angulo bruto (>=3 deg) vs EKF (>=10 RPM) vs FG (>=10 RPM)
    move_thr = min(10.0, 0.2 * abs(target_rpm))
    t_angle_move_ms = first_crossing_ms(rel_t, ang_disp_post, 3.0)
    t_ekf_move_ms = first_crossing_ms(rel_t, ekf_post, move_thr)
    t_fg_move_ms = first_crossing_ms(rel_t, fg_post, move_thr)
    t_angvel_move_ms = first_crossing_ms(rel_t, ang_vel_post, move_thr)

    # Tempo de subida 10-90% e overshoot (no sentido do alvo)
    sign = 1.0 if target_rpm >= 0 else -1.0
    mag = abs(target_rpm)
    signed_ekf = [sign * v for v in ekf_post]
    t10_ekf = first_crossing_ms(rel_t, signed_ekf, 0.10 * mag, use_abs=False)
    t90_ekf = first_crossing_ms(rel_t, signed_ekf, 0.90 * mag, use_abs=False)
    rise_ekf_ms = (
        round(t90_ekf - t10_ekf, 1)
        if (t10_ekf is not None and t90_ekf is not None and t90_ekf >= t10_ekf)
        else None
    )

    # Regiao estacionaria: ultimos steady_window_s segundos
    t_end = times_s[-1]
    ss_idx = next((i for i, t in enumerate(times_s) if (t_end - t) <= steady_window_s), len(times_s) // 2)
    ss_dt = times_s[-1] - times_s[ss_idx]

    ss_ekf = rpms_ekf[ss_idx:]
    ss_fg = rpms_fg[ss_idx:]
    ss_pwm = pwms[ss_idx:]
    ss_ang_vel = rpms_angle_inst[ss_idx:]

    # Velocidade real absoluta da roda pelo deslocamento total do AS5600 (eixo do motor / 3)
    delta_motor_deg = unwrapped[-1] - unwrapped[ss_idx]
    rpm_as5600_true = (delta_motor_deg / (6.0 * WHEEL_REDUCTION * ss_dt)) if ss_dt > 0.05 else 0.0
    as5600_motor_revs = abs(delta_motor_deg) / 360.0
    as5600_wheel_revs = as5600_motor_revs / WHEEL_REDUCTION

    # Verifica maior salto de angulo por quadro na janela estacionaria (alerta de aliasing se >150 deg)
    max_frame_step_deg = max(
        (abs(unwrapped[k] - unwrapped[k - 1]) for k in range(max(1, ss_idx), len(unwrapped))),
        default=0.0,
    )

    # Velocidade real absoluta pelos pulsos FG acumulados na janela estacionaria
    delta_fg_counts = counts[-1] - counts[ss_idx]
    rpm_fg_true_mag = (
        (delta_fg_counts / EXPECTED_FG_COUNTS_PER_WHEEL_REV) * (60.0 / ss_dt)
        if ss_dt > 0.05
        else 0.0
    )
    fg_counts_per_motor_rev = (
        (delta_fg_counts / as5600_motor_revs) if as5600_motor_revs > 0.5 else None
    )
    fg_counts_per_wheel_rev = (
        (delta_fg_counts / as5600_wheel_revs) if as5600_wheel_revs > 0.15 else None
    )
    fg_vs_as5600_ratio = (
        (rpm_fg_true_mag / abs(rpm_as5600_true)) if abs(rpm_as5600_true) > 5.0 else None
    )

    ekf_mean = statistics.fmean(ss_ekf)
    ekf_std = statistics.pstdev(ss_ekf) if len(ss_ekf) > 1 else 0.0
    fg_mean = statistics.fmean(ss_fg)
    ang_inst_std = statistics.pstdev(ss_ang_vel) if len(ss_ang_vel) > 1 else 0.0
    pwm_mean = statistics.fmean(ss_pwm)
    pwm_std = statistics.pstdev(ss_pwm) if len(ss_pwm) > 1 else 0.0

    # Overshoot (sobre o EKF e sobre a velocidade rapida do AS5600)
    peak_ekf = max(signed_ekf) if signed_ekf else 0.0
    overshoot_ekf_pct = max(0.0, (peak_ekf - mag) / mag * 100.0) if mag > 0 else 0.0

    # Sinal do angulo bruto relativo a rotacao comandada (+1 se delta_ang tem mesmo sinal do alvo)
    raw_angle_dir = None
    if abs(delta_motor_deg) > 15.0:
        raw_angle_dir = 1 if (delta_motor_deg * sign > 0) else -1

    # Comparacao com o modelo de feedforward atualmente salvo na NVS
    ff_model_pwm = None
    ff_error_pwm = None
    if tuning_data is not None:
        w_rad_s = target_rpm * RPM_TO_RAD_S
        if target_rpm >= 0:
            ff_i = float(tuning_data["feedforward"][wheel])
            vff = float(tuning_data["velocity_ff"][wheel])
        else:
            ff_i = float(tuning_data["feedforward_rev"][wheel])
            vff = float(tuning_data["velocity_ff_rev"][wheel])
        ff_model_pwm = ff_i + vff * w_rad_s
        ff_error_pwm = ff_model_pwm - pwm_mean

    # Crosstalk nas outras 3 rodas durante o teste
    crosstalk = {}
    for other in range(4):
        if other == wheel:
            continue
        dc = int(samples[-1]["wheels"][other]["pulse_count"]) - int(samples[0]["wheels"][other]["pulse_count"])
        max_other_rpm = max(abs(float(s["wheels"][other]["rpm"])) for s in samples)
        if dc != 0 or max_other_rpm > 1.0:
            crosstalk[WHEEL_NAMES[other]] = {"delta_fg": dc, "max_rpm": round(max_other_rpm, 2)}

    # Diferenca EKF vs velocidade verdadeira do AS5600 (detecta erro de sinal ou escala)
    as5600_signed_in_motor_frame = abs(rpm_as5600_true) * (
        1.0 if (raw_angle_dir == -1 and target_rpm >= 0) or (raw_angle_dir == 1 and target_rpm < 0) else -1.0
    )
    ekf_vs_as5600_mag_pct = (
        ((abs(ekf_mean) - abs(rpm_as5600_true)) / abs(rpm_as5600_true) * 100.0)
        if abs(rpm_as5600_true) > 5.0
        else None
    )

    return {
        "wheel": wheel,
        "wheel_name": WHEEL_NAMES[wheel],
        "target_rpm": target_rpm,
        "applied_target_rpm": round(tgts[-1], 1),
        "valid": True,
        "n_samples": len(samples),
        "duration_s": round(times_s[-1], 3),
        "steady_dt_s": round(ss_dt, 3),
        "t_angle_move_ms": t_angle_move_ms,
        "t_angvel_move_ms": t_angvel_move_ms,
        "t_ekf_move_ms": t_ekf_move_ms,
        "t_fg_move_ms": t_fg_move_ms,
        "ekf_lag_vs_angle_ms": (
            round(t_ekf_move_ms - t_angle_move_ms, 1)
            if (t_ekf_move_ms is not None and t_angle_move_ms is not None)
            else None
        ),
        "fg_lag_vs_angle_ms": (
            round(t_fg_move_ms - t_angle_move_ms, 1)
            if (t_fg_move_ms is not None and t_angle_move_ms is not None)
            else None
        ),
        "rise_ekf_ms": rise_ekf_ms,
        "overshoot_ekf_pct": round(overshoot_ekf_pct, 1),
        "ekf_rpm_mean": round(ekf_mean, 2),
        "ekf_rpm_std": round(ekf_std, 2),
        "ekf_rpm_min": round(min(ss_ekf), 1),
        "ekf_rpm_max": round(max(ss_ekf), 1),
        "fg_rpm_window_mean": round(fg_mean, 2),
        "as5600_true_rpm_mag": round(abs(rpm_as5600_true), 2),
        "as5600_true_rpm_signed": round(rpm_as5600_true, 2),
        "as5600_inst_std_rpm": round(ang_inst_std, 2),
        "as5600_dir_vs_cmd": raw_angle_dir,
        "fg_true_rpm_mag": round(rpm_fg_true_mag, 2),
        "delta_fg_counts": delta_fg_counts,
        "as5600_motor_revs": round(as5600_motor_revs, 3),
        "as5600_wheel_revs": round(as5600_wheel_revs, 3),
        "max_frame_step_deg": round(max_frame_step_deg, 1),
        "fg_counts_per_motor_rev": (
            round(fg_counts_per_motor_rev, 3) if fg_counts_per_motor_rev is not None else None
        ),
        "fg_counts_per_wheel_rev": (
            round(fg_counts_per_wheel_rev, 3) if fg_counts_per_wheel_rev is not None else None
        ),
        "fg_vs_as5600_ratio": round(fg_vs_as5600_ratio, 4) if fg_vs_as5600_ratio is not None else None,
        "ekf_vs_as5600_mag_pct": (
            round(ekf_vs_as5600_mag_pct, 2) if ekf_vs_as5600_mag_pct is not None else None
        ),
        "ss_error_vs_as5600_pct": (
            round((abs(rpm_as5600_true) - mag) / mag * 100.0, 2) if mag > 0 else 0.0
        ),
        "pwm_mean": round(pwm_mean, 1),
        "pwm_std": round(pwm_std, 1),
        "pwm_min": round(min(ss_pwm), 0),
        "pwm_max": round(max(ss_pwm), 0),
        "ff_model_pwm": round(ff_model_pwm, 1) if ff_model_pwm is not None else None,
        "ff_error_pwm": round(ff_error_pwm, 1) if ff_error_pwm is not None else None,
        "crosstalk": crosstalk,
    }


def run_rest_noise_check(client: BenchClient, duration_s: float = 1.5) -> dict[str, Any]:
    """Mede ruido de angulo/velocidade e pulsos FG fantasma com todos os motores em 0."""
    client.stop()
    wait_until_stopped(client, [0, 1, 2, 3], timeout_s=2.0)
    samples = sample_state_stream(client, duration_s=duration_s, poll_s=0.015)
    per_wheel = {}
    for w in range(4):
        angs = unwrap_angles_deg([float(s["wheels"][w]["angle_deg"]) for s in samples])
        rpms = [float(s["wheels"][w]["rpm"]) for s in samples]
        fg_rpms = [float(s["wheels"][w]["pulse_rpm"]) for s in samples]
        dc = int(samples[-1]["wheels"][w]["pulse_count"]) - int(samples[0]["wheels"][w]["pulse_count"])
        per_wheel[WHEEL_NAMES[w]] = {
            "angle_std_deg": round(statistics.pstdev(angs), 4) if len(angs) > 1 else 0.0,
            "angle_span_deg": round(max(angs) - min(angs), 4) if angs else 0.0,
            "max_abs_ekf_rpm": round(max(abs(v) for v in rpms), 3) if rpms else 0.0,
            "max_abs_fg_rpm": round(max(abs(v) for v in fg_rpms), 3) if fg_rpms else 0.0,
            "phantom_fg_counts": dc,
        }
    return {"duration_s": duration_s, "n_samples": len(samples), "wheels": per_wheel}


def run_reversal_check(client: BenchClient, wheel: int, rpm: float = 240.0) -> dict[str, Any]:
    """Testa reversao brusca +rpm -> -rpm -> 0 e detecta erro de sinal do FG durante inversao."""
    client.stop()
    wait_until_stopped(client, [wheel], timeout_s=2.0)

    client.set_wheel(wheel, rpm)
    time.sleep(0.95)
    pre_samples = sample_state_stream(client, duration_s=0.18, poll_s=0.005)

    # Captura a transicao +rpm -> -rpm em alta taxa
    client.set_wheel(wheel, -rpm)
    samples = sample_state_stream(client, duration_s=1.5, poll_s=0.005)
    client.stop()
    wait_until_stopped(client, [wheel], timeout_s=2.0)

    if len(samples) < 6:
        return {"wheel": wheel, "valid": False}

    t0 = samples[0]["uptime_ms"]
    times_s = [(s["uptime_ms"] - t0) / 1000.0 for s in samples]
    angs = unwrap_angles_deg([float(s["wheels"][wheel]["angle_deg"]) for s in samples])
    ekfs = [float(s["wheels"][wheel]["rpm"]) for s in samples]
    fgs = [float(s["wheels"][wheel]["pulse_rpm"]) for s in samples]
    pwms = [float(s["wheels"][wheel]["pwm"]) for s in samples]
    ang_vels = compute_angle_velocity_series(times_s, angs, window_s=0.04)

    # Infere o sinal do AS5600 durante o patamar estavel +rpm antes da inversao
    init_sign = -1.0
    if len(pre_samples) >= 3:
        pre_angs = unwrap_angles_deg([float(s["wheels"][wheel]["angle_deg"]) for s in pre_samples])
        if abs(pre_angs[-1] - pre_angs[0]) > 5.0:
            init_sign = 1.0 if (pre_angs[-1] - pre_angs[0]) > 0 else -1.0
    motor_ang_vels = [init_sign * v for v in ang_vels]

    # Conta amostras onde o FG ja inverteu sinal (negativo) enquanto o AS5600 ainda gira positivo
    fg_wrong_sign_samples = 0
    max_fg_wrong_rpm = 0.0
    for v_true, v_fg in zip(motor_ang_vels, fgs):
        if v_true > 15.0 and v_fg < -15.0:
            fg_wrong_sign_samples += 1
            max_fg_wrong_rpm = max(max_fg_wrong_rpm, abs(v_fg))

    # Tempo ate cruzar 0 e ate atingir -90% do alvo
    t_zero_true_ms = first_crossing_ms(times_s, [-v for v in motor_ang_vels], 0.0, use_abs=False)
    t_zero_ekf_ms = first_crossing_ms(times_s, [-v for v in ekfs], 0.0, use_abs=False)
    t_90_ekf_ms = first_crossing_ms(times_s, [-v for v in ekfs], 0.90 * rpm, use_abs=False)
    final_ekf_rpm = statistics.fmean(ekfs[-max(3, len(ekfs) // 5) :])
    reversed_ok = final_ekf_rpm < -0.5 * rpm
    min_pwm = min(pwms)
    max_pwm = max(pwms)

    return {
        "wheel": wheel,
        "wheel_name": WHEEL_NAMES[wheel],
        "rpm": rpm,
        "valid": True,
        "reversed_ok": reversed_ok,
        "final_ekf_rpm": round(final_ekf_rpm, 1),
        "t_zero_as5600_ms": t_zero_true_ms,
        "t_zero_ekf_ms": t_zero_ekf_ms,
        "t_90_neg_ekf_ms": t_90_ekf_ms,
        "fg_wrong_sign_samples": fg_wrong_sign_samples,
        "max_fg_wrong_sign_rpm": round(max_fg_wrong_rpm, 1),
        "pwm_range": [round(min_pwm, 0), round(max_pwm, 0)],
    }


def run_sync_check(client: BenchClient, profiles: list[tuple[str, list[float]]]) -> list[dict[str, Any]]:
    """Roda as 4 rodas juntas e mede sincronia (spread max-min) e deslocamento de PWM."""
    out = []
    for label, targets in profiles:
        client.stop()
        wait_until_stopped(client, [0, 1, 2, 3], timeout_s=2.0)
        client.set_targets(targets)
        samples = sample_state_stream(client, duration_s=2.0, poll_s=0.005)
        client.stop()
        wait_until_stopped(client, [0, 1, 2, 3], timeout_s=2.0)

        t0 = samples[0]["uptime_ms"]
        times_s = [(s["uptime_ms"] - t0) / 1000.0 for s in samples]
        t_end = times_s[-1]
        ss_idx = next((i for i, t in enumerate(times_s) if (t_end - t) <= 0.8), len(samples) // 2)
        ss_dt = times_s[-1] - times_s[ss_idx]

        wheel_stats = []
        abs_speeds = []
        abs_ekfs = []
        for w in range(4):
            unw = unwrap_angles_deg([float(s["wheels"][w]["angle_deg"]) for s in samples])
            rpm_true = (
                abs(unw[-1] - unw[ss_idx]) / (6.0 * WHEEL_REDUCTION * ss_dt)
                if ss_dt > 0.05
                else 0.0
            )
            ekf_m = statistics.fmean(float(s["wheels"][w]["rpm"]) for s in samples[ss_idx:])
            pwm_m = statistics.fmean(float(s["wheels"][w]["pwm"]) for s in samples[ss_idx:])
            abs_speeds.append(rpm_true)
            abs_ekfs.append(abs(ekf_m))
            wheel_stats.append(
                {
                    "wheel": WHEEL_NAMES[w],
                    "target_rpm": targets[w],
                    "as5600_true_rpm_mag": round(rpm_true, 2),
                    "ekf_rpm_mean": round(ekf_m, 2),
                    "pwm_mean": round(pwm_m, 1),
                }
            )
        spread = round(max(abs_speeds) - min(abs_speeds), 2)
        mean_spd = statistics.fmean(abs_speeds)
        spread_pct = round(spread / mean_spd * 100.0, 2) if mean_spd > 1.0 else 0.0
        spread_ekf = round(max(abs_ekfs) - min(abs_ekfs), 2)
        mean_ekf = statistics.fmean(abs_ekfs)
        spread_ekf_pct = round(spread_ekf / mean_ekf * 100.0, 2) if mean_ekf > 1.0 else 0.0
        out.append(
            {
                "label": label,
                "targets_rpm": targets,
                "spread_rpm": spread,
                "spread_pct": spread_pct,
                "spread_ekf_rpm": spread_ekf,
                "spread_ekf_pct": spread_ekf_pct,
                "wheels": wheel_stats,
            }
        )
    return out


def fetch_new_console(client: BenchClient, cursor: int) -> tuple[list[str], int]:
    resp = client.get(f"/api/console?since={cursor}&limit=500")
    return resp.get("lines", []), int(resp.get("next_since", cursor))


def main() -> int:
    parser = argparse.ArgumentParser(description="Auditoria completa das rodas via Robot Bench API.")
    parser.add_argument("--socket", default=DEFAULT_SOCKET, help="Caminho do Unix socket da API.")
    parser.add_argument("--host", default="127.0.0.1", help="Host TCP fallback.")
    parser.add_argument("--port", type=int, default=8765, help="Porta TCP fallback.")
    parser.add_argument("--wheels", default="0,1,2,3", help="Indices das rodas (ex.: 0,1,2,3).")
    parser.add_argument("--quick", action="store_true", help="Menos degraus de velocidade (~45 s).")
    parser.add_argument("--hold-s", type=float, default=1.8, help="Duracao de cada degrau (s).")
    parser.add_argument("--out", default="", help="Arquivo JSON de saida (padrao em tmp/).")
    args = parser.parse_args()

    wheels = [int(x.strip()) for x in args.wheels.split(",") if x.strip() != ""]
    client = BenchClient(socket_path=args.socket, host=args.host, port=args.port)

    def _on_sigint(_sig: int, _frame: Any) -> None:
        print("\n[ABORT] Interrompido — parando motores...", file=sys.stderr)
        client.stop()
        sys.exit(130)

    signal.signal(signal.SIGINT, _on_sigint)
    signal.signal(signal.SIGTERM, _on_sigint)

    try:
        snap = client.get("/api/snapshot?console=10")
        status = snap["status"]
        if not status.get("live"):
            print("ERRO: Robot Bench nao esta com link serial ativo.", file=sys.stderr)
            return 1
        if status["calibration"]["running"] or status["tuning"]["running"]:
            print("ERRO: ha uma calibracao em andamento no Robot Bench.", file=sys.stderr)
            return 1

        max_rpm = float(status.get("max_rpm") or 450.0)
        calib_data = snap["calibration"].get("data")
        tuning_data = snap["tuning"].get("data")
        health_data = snap["health"].get("data")
        console_cursor = int(snap["console"].get("next_since", 0))

        print("=== AUDITORIA DE BANCADA DAS RODAS ===")
        print(
            f"Conexao: {status['connected']} | uptime={status['uptime_ms']/1000:.1f}s | "
            f"max_rpm={max_rpm:.1f} (calibrado={status['max_rpm_calibrated']})"
        )
        if health_data:
            print(f"Health: {health_data}")
        else:
            print("Health: indisponivel (aba Saude nao aberta no GUI; opcional)")

        if calib_data:
            print(
                f"Wiring: analog_map={calib_data.get('analog_map')} | "
                f"mux_map={calib_data.get('mux_map')} | "
                f"range_mask=0x{calib_data.get('range_mask', 0):x} | "
                f"adc_min={calib_data.get('adc_min_raw')} | adc_max={calib_data.get('adc_max_raw')}"
            )
        if tuning_data:
            print(f"Tuning Kp={tuning_data['kp']} Ki={tuning_data['ki']} Kd={tuning_data['kd']}")
            print(
                f"  FF+={tuning_data['feedforward']}  VFF+={tuning_data['velocity_ff']}\n"
                f"  FF-={tuning_data['feedforward_rev']}  VFF-={tuning_data['velocity_ff_rev']}"
            )

        # 1. Ruido em repouso
        print("\n[1/4] Medindo ruido em repouso e pulsos FG fantasma (1.5 s)...")
        rest = run_rest_noise_check(client, duration_s=1.5)
        for name, st in rest["wheels"].items():
            print(
                f"  {name}: ang_std={st['angle_std_deg']:.4f} deg (span={st['angle_span_deg']:.3f}) | "
                f"max|ekf|={st['max_abs_ekf_rpm']:.2f} RPM | phantom_FG={st['phantom_fg_counts']}"
            )

        # Preflight: confirma que a alimentacao dos motores esta ligada antes da varredura
        probe_w = wheels[0]
        probe_rpm = min(200.0, max_rpm * 0.5)
        client.set_wheel(probe_w, probe_rpm)
        probe_samples = sample_state_stream(client, duration_s=0.7, poll_s=0.01)
        client.stop()
        wait_until_stopped(client, [probe_w], timeout_s=1.5)
        probe_ang = unwrap_angles_deg([float(s["wheels"][probe_w]["angle_deg"]) for s in probe_samples])
        probe_move_deg = abs(probe_ang[-1] - probe_ang[0]) if len(probe_ang) >= 2 else 0.0
        probe_fg = (
            int(probe_samples[-1]["wheels"][probe_w]["pulse_count"])
            - int(probe_samples[0]["wheels"][probe_w]["pulse_count"])
            if len(probe_samples) >= 2
            else 0
        )
        if probe_move_deg < 5.0 and probe_fg == 0:
            print(
                f"\nABORTADO: {WHEEL_NAMES[probe_w]} nao se moveu com alvo +{probe_rpm:.0f} RPM "
                f"(delta_ang={probe_move_deg:.1f} deg, delta_FG={probe_fg}). "
                "Verifique se a bateria/fonte de potencia dos motores esta ligada."
            )
            return 2

        # 2. Varredura individual nos 2 sentidos
        if args.quick:
            base_rpms = [60.0, 140.0, 260.0, min(420.0, round(max_rpm * 0.92, 0))]
        else:
            top_rpm = min(480.0, math.floor(max_rpm * 0.96))
            base_rpms = [50.0, 80.0, 120.0, 180.0, 250.0, 340.0, top_rpm]
        # Remove duplicatas mantendo ordem
        seen: set[float] = set()
        magnitudes = []
        for r in base_rpms:
            r = min(r, max_rpm)
            if r not in seen and r >= 50.0:
                seen.add(r)
                magnitudes.append(r)

        print(f"\n[2/4] Varredura individual por roda ({len(magnitudes)} velocidades x 2 sentidos)...")
        step_results: list[dict[str, Any]] = []
        console_events: list[dict[str, Any]] = []

        for w in wheels:
            print(f"\n--- {WHEEL_NAMES[w]} ---")
            print(
                f"{'Alvo':>7} | {'AS5600':>7} | {'EKF':>7} | {'FG':>7} | {'FG/rev':>6} | "
                f"{'PWM':>6} | {'FFerr':>6} | {'LagE':>5} | {'LagF':>5} | {'Rise':>5} | {'Ovr%':>5} | {'Coast':>5}"
            )
            for sign in (1.0, -1.0):
                for mag in magnitudes:
                    target = sign * mag
                    client.stop()
                    wait_until_stopped(client, [w], timeout_s=2.0)
                    _, console_cursor = fetch_new_console(client, console_cursor)

                    client.set_wheel(w, target)
                    samples = sample_state_stream(client, duration_s=args.hold_s, poll_s=0.005)
                    client.stop()
                    coast_s = wait_until_stopped(client, [w], timeout_s=2.2)

                    new_lines, console_cursor = fetch_new_console(client, console_cursor)
                    diag_lines = [
                        ln for ln in new_lines
                        if any(tok in ln for tok in ("low-speed", "continuous", "LS ", "RIP ", "guard", "fault", "blocked"))
                    ]
                    if diag_lines:
                        console_events.append({"wheel": WHEEL_NAMES[w], "target_rpm": target, "lines": diag_lines})

                    m = analyze_step_samples(w, target, samples, tuning_data)
                    m["coast_s"] = round(coast_s, 3)
                    m["mode_logs"] = diag_lines
                    step_results.append(m)

                    def _fmt(val: Any, fmt: str = "{:.1f}") -> str:
                        return "—" if val is None else fmt.format(val)

                    print(
                        f"{target:+7.0f} | {m['as5600_true_rpm_mag']:7.1f} | {m['ekf_rpm_mean']:+7.1f} | "
                        f"{m['fg_true_rpm_mag']:7.1f} | {_fmt(m['fg_counts_per_wheel_rev'], '{:.2f}'):>6} | "
                        f"{m['pwm_mean']:+6.0f} | {_fmt(m['ff_error_pwm'], '{:+.0f}'):>6} | "
                        f"{_fmt(m['ekf_lag_vs_angle_ms'], '{:.0f}'):>5} | {_fmt(m['fg_lag_vs_angle_ms'], '{:.0f}'):>5} | "
                        f"{_fmt(m['rise_ekf_ms'], '{:.0f}'):>5} | {m['overshoot_ekf_pct']:5.1f} | {coast_s*1000:5.0f}"
                    )

        # 3. Teste de reversao brusca
        rev_rpm = min(240.0, round(max_rpm * 0.55, 0))
        print(f"\n[3/4] Teste de reversao brusca (+{rev_rpm:.0f} -> -{rev_rpm:.0f} RPM)...")
        reversals = []
        for w in wheels:
            rev = run_reversal_check(client, w, rpm=rev_rpm)
            reversals.append(rev)
            status_tag = "OK" if rev.get("reversed_ok") else "FALHOU"
            print(
                f"  {WHEEL_NAMES[w]} [{status_tag}, final={rev.get('final_ekf_rpm')} RPM]: "
                f"zero_AS5600={rev.get('t_zero_as5600_ms')}ms | "
                f"zero_EKF={rev.get('t_zero_ekf_ms')}ms | -90%_EKF={rev.get('t_90_neg_ekf_ms')}ms | "
                f"FG_sinal_invertido={rev.get('fg_wrong_sign_samples')} amostras "
                f"(max={rev.get('max_fg_wrong_sign_rpm')} RPM)"
            )

        # 4. Teste das 4 rodas simultaneas
        print("\n[4/4] Teste de sincronia das 4 rodas juntas...")
        hi_sync = min(380.0, math.floor(max_rpm * 0.80))
        sync_profiles = [
            ("low_all_fwd_100", [100.0, 100.0, 100.0, 100.0]),
            ("mid_omni_240", [240.0, -240.0, 240.0, -240.0]),
            (f"high_all_fwd_{int(hi_sync)}", [hi_sync, hi_sync, hi_sync, hi_sync]),
        ]
        sync_results = run_sync_check(client, sync_profiles)
        for sr in sync_results:
            summary = " | ".join(
                f"{w['wheel']}={abs(w['ekf_rpm_mean']):.1f}RPM(pwm={w['pwm_mean']:+.0f})"
                for w in sr["wheels"]
            )
            print(
                f"  {sr['label']}: spread_EKF={sr['spread_ekf_rpm']:.1f} RPM ({sr['spread_ekf_pct']:.1f}%) | "
                f"{summary}"
            )

        # Resumo analitico
        fg_per_rev_all = [
            r["fg_counts_per_wheel_rev"]
            for r in step_results
            if r.get("fg_counts_per_wheel_rev") is not None and abs(r["target_rpm"]) >= 180.0
        ]
        ekf_lags = [
            r["ekf_lag_vs_angle_ms"]
            for r in step_results
            if r.get("ekf_lag_vs_angle_ms") is not None and abs(r["target_rpm"]) >= 180.0
        ]
        fg_lags = [
            r["fg_lag_vs_angle_ms"]
            for r in step_results
            if r.get("fg_lag_vs_angle_ms") is not None and abs(r["target_rpm"]) >= 180.0
        ]
        print("\n=== RESUMO DIAGNOSTICO ===")
        if fg_per_rev_all:
            print(
                f"  FG counts/volta_roda (esperado {EXPECTED_FG_COUNTS_PER_WHEEL_REV:.1f}): "
                f"mediana={statistics.median(fg_per_rev_all):.3f} "
                f"[min={min(fg_per_rev_all):.3f}, max={max(fg_per_rev_all):.3f}]"
            )
        if ekf_lags:
            print(
                f"  Atraso EKF vs AS5600 bruto (>=180 RPM): "
                f"mediana={statistics.median(ekf_lags):.1f} ms [max={max(ekf_lags):.1f} ms]"
            )
        if fg_lags:
            print(
                f"  Atraso FG vs AS5600 bruto  (>=180 RPM): "
                f"mediana={statistics.median(fg_lags):.1f} ms [max={max(fg_lags):.1f} ms]"
            )

        report = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "status": status,
            "health": health_data,
            "calibration": calib_data,
            "tuning": tuning_data,
            "rest_noise": rest,
            "steps": step_results,
            "reversals": reversals,
            "sync": sync_results,
            "console_events": console_events,
        }
        out_path = args.out or os.path.join(
            DEFAULT_OUT_DIR, f"wheel_audit_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        )
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(out_path, "w", encoding="utf-8") as fh:
            json.dump(report, fh, indent=2, ensure_ascii=False)
        print(f"\nRelatorio completo salvo em: {out_path}")
        return 0
    finally:
        client.stop()


if __name__ == "__main__":
    raise SystemExit(main())

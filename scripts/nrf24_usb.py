"""Interactive & CLI tool for controlling SSL robots via the ESP32 Base Station.

Speaks COBS+CRC8 Bitproto (proto/base_station_protocol.bitproto) over UART0 @ 921600 baud.
"""

import argparse
import os
import sys
import time

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_SCRIPT_DIR)
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from robot_bench import base_station, radio  # noqa: E402


class Colors:
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"
    BOLD = "\033[1m"
    RESET = "\033[0m"


def connect_base(port=None, baud=base_station.DEFAULT_BAUD):
    if port:
        link = base_station.BaseStationLink(port, baud=baud)
        info = link.open()
        return link, info
    adapters = radio.find_adapters(baud=baud)
    if not adapters:
        details = []
        for p in sorted(radio.serial.tools.list_ports.comports(), key=lambda x: x.device):
            if not any(tok in p.device for tok in ("ttyUSB", "ttyACM", "usbserial", "usbmodem", "COM")):
                continue
            try:
                base_station.probe_base_station(p.device, baud=baud, timeout_s=0.25)
                details.append(f"{p.device}: sem resposta ao PING")
            except OSError as exc:
                details.append(f"{p.device}: {exc} (feche outro processo usando a porta, ex: nrf24_bridge_node)")
        suffix = f" Portas verificadas: {'; '.join(details)}" if details else " Nenhuma porta /dev/ttyUSB* ou /dev/ttyACM* visível no container."
        raise base_station.BaseStationError(
            f"Nenhuma ESP32 Base Station encontrada nas portas USB.{suffix}"
        )
    link = base_station.BaseStationLink(adapters[0][0], baud=baud)
    info = link.open()
    return link, info


def print_info(info, port, baud):
    rate = base_station.RATE_NAMES.get(info["data_rate"], "?")
    mask = info.get("radio_ready_mask", 1 if info["radio_ready"] else 0)
    tx_ok = "PRONTO" if (mask & 0x01) else "ERRO"
    rx_ok = "PRONTO (Full-Duplex)" if (mask & 0x02) else "AUSENTE (Fallback 1-Rádio)"
    rx_ch = info.get("rx_channel", info["channel"])
    sep = abs(rx_ch - info["channel"])
    print(f"\n{Colors.BOLD}=== ESP32 BASE STATION ({port} @ {baud} baud) ==={Colors.RESET}")
    print(f"  Protocolo : v{info['protocol_version']} | TX: {tx_ok} | RX: {rx_ok}")
    print(
        f"  Canais RF : TX={info['channel']} ({2400 + info['channel']} MHz)"
        f" | RX={rx_ch} ({2400 + rx_ch} MHz) | sep={sep} MHz | Taxa: {rate} | PA: {info['pa_level']}"
    )
    print(
        f"  Retries   : ARD={info['retransmit_delay']} ({(info['retransmit_delay'] + 1) * 250} us)"
        f" | ARC={info['retransmit_count']}"
    )
    print(
        f"  Contadores: TX OK={info['tx_ok_count']} | TX Fail={info['tx_fail_count']}"
        f" | RX={info['rx_count']} | Reinits={info['radio_reinit_count']}\n"
    )


def print_scan(scan):
    best_tx = scan["best_channel"]
    best_rx = scan.get("best_rx_channel", best_tx)
    sep = abs(best_rx - best_tx)
    print(
        f"\n{Colors.BOLD}=== SCAN RPD (melhor par: TX={best_tx} / {2400 + best_tx} MHz, "
        f"RX={best_rx} / {2400 + best_rx} MHz, sep={sep} MHz) ==={Colors.RESET}"
    )
    for i, pct in enumerate(scan["busy_pct"]):
        ch = scan["start_channel"] + i
        if pct == 0 and ch not in (best_tx, best_rx) and ch % 10 != 0:
            continue
        color = Colors.RED if pct >= 50 else (Colors.YELLOW if pct >= 15 else Colors.GREEN)
        tags = []
        if ch == best_tx:
            tags.append("RECOMENDADO TX")
        if ch == best_rx:
            tags.append("RECOMENDADO RX")
        tag = f" {Colors.CYAN}<-- {' / '.join(tags)}{Colors.RESET}" if tags else ""
        print(f"  ch {ch:3d} ({2400 + ch} MHz) [{pct:3d}%] {color}{'#' * (pct // 5)}{Colors.RESET}{tag}")
    print()


def send_telemetry_request(link, robot_id, timeout_s=0.05):
    ts = int(time.time() * 1000) & 0xFFFFFFFF
    req = radio.encode_telemetry_request(robot_id, ts)
    link.clear_rx()
    t0 = time.monotonic()
    tx_st = link.send_rf(robot_id, req, tx_flags=base_station.BS_TX_FLAG_EXPECT_REPLY, timeout_s=0.03)
    if tx_st is None or tx_st["status"] != base_station.BS_TX_STATUS_OK:
        return None, tx_st, 0.0
    deadline = t0 + timeout_s
    while time.monotonic() < deadline:
        rx = link.wait_rf_rx(timeout_s=max(0.002, deadline - time.monotonic()))
        if rx is None:
            break
        tel = radio.decode_telemetry(rx["payload"])
        if tel is not None and tel["robot_id"] == robot_id:
            return tel, tx_st, (time.monotonic() - t0) * 1000.0
    return None, tx_st, 0.0


def switch_channel(link, new_channel, rx_channel=None, notify_robots=True, max_attempts=15):
    info = link.get_info()
    if info is None:
        print(f"{Colors.RED}Falha ao ler info da Base Station.{Colors.RESET}")
        return False
    old_tx = info["channel"]
    old_rx = info.get("rx_channel", old_tx)
    target_rx = new_channel if not info.get("dual_radio") else (
        rx_channel if rx_channel is not None else new_channel
    )
    param = base_station.encode_channel_pair_param(new_channel, target_rx)

    active_ids = []
    if notify_robots:
        for rid in range(radio.ROBOT_ID_MAX + 1):
            req = radio.encode_telemetry_request(rid, int(time.time() * 1000) & 0xFFFFFFFF)
            st = link.send_rf(rid, req, timeout_s=0.025)
            if st and st["status"] == base_station.BS_TX_STATUS_OK:
                active_ids.append(rid)

    updated = link.set_config(
        channel=new_channel,
        rx_channel=target_rx,
        data_rate=info["data_rate"],
        pa_level=info["pa_level"],
        retransmit_delay=info["retransmit_delay"],
        retransmit_count=info["retransmit_count"],
        persist_nvs=not notify_robots or not active_ids,
    )
    if not updated:
        return False

    if notify_robots and active_ids:
        pending = set(active_ids)
        confirmed = set()
        for attempt in range(1, max_attempts + 1):
            if not pending:
                break
            for rid in list(pending):
                cfg = radio.encode_config(rid, radio.CONFIG_FLAG_SET_CHANNEL, param)
                link.send_rf_on_channel(rid, old_tx, cfg, timeout_s=0.025)
                if new_channel != old_tx:
                    probe = radio.encode_telemetry_request(rid, int(time.time() * 1000) & 0xFFFFFFFF)
                    st = link.send_rf(rid, probe, timeout_s=0.025)
                    ok = st is not None and st["status"] == base_station.BS_TX_STATUS_OK
                else:
                    tel, _, _ = send_telemetry_request(link, rid, timeout_s=0.04)
                    ok = tel is not None
                if ok:
                    pending.remove(rid)
                    confirmed.add(rid)
                    print(
                        f"{Colors.GREEN}  ✓ Robô {rid} confirmado em TX={new_channel} RX={target_rx} "
                        f"(tentativa {attempt}/{max_attempts}){Colors.RESET}"
                    )
            if pending and attempt % 3 == 0:
                bcast = radio.encode_config(radio.ROBOT_ID_BROADCAST, radio.CONFIG_FLAG_SET_CHANNEL, param)
                link.send_rf_on_channel(
                    radio.ROBOT_ID_BROADCAST,
                    old_tx,
                    bcast,
                    tx_flags=base_station.BS_TX_FLAG_NO_ACK,
                    timeout_s=0.025,
                )
            if pending:
                time.sleep(0.03)

        if not confirmed:
            print(f"{Colors.RED}✗ Nenhum robô confirmou — rollback para TX={old_tx} RX={old_rx}.{Colors.RESET}")
            link.set_config(
                channel=old_tx,
                rx_channel=old_rx,
                data_rate=info["data_rate"],
                pa_level=info["pa_level"],
                retransmit_delay=info["retransmit_delay"],
                retransmit_count=info["retransmit_count"],
                persist_nvs=False,
            )
            return False

        updated = link.set_config(
            channel=new_channel,
            rx_channel=target_rx,
            data_rate=info["data_rate"],
            pa_level=info["pa_level"],
            retransmit_delay=info["retransmit_delay"],
            retransmit_count=info["retransmit_count"],
            persist_nvs=True,
        )
    elif notify_robots:
        bcast = radio.encode_config(radio.ROBOT_ID_BROADCAST, radio.CONFIG_FLAG_SET_CHANNEL, param)
        for _ in range(3):
            link.send_rf_on_channel(
                radio.ROBOT_ID_BROADCAST,
                old_tx,
                bcast,
                tx_flags=base_station.BS_TX_FLAG_NO_ACK,
                timeout_s=0.025,
            )

    if updated:
        print(
            f"{Colors.GREEN}✓ Base Station agora em TX={updated['channel']} ({2400 + updated['channel']} MHz) "
            f"| RX={updated['rx_channel']} ({2400 + updated['rx_channel']} MHz){Colors.RESET}"
        )
        return True
    return False


def main():
    parser = argparse.ArgumentParser(description="CLI da ESP32 Base Station nRF24L01+")
    parser.add_argument("-p", "--port", default=None, help="Porta serial (padrão: auto-detecção)")
    parser.add_argument("-b", "--baud", type=int, default=base_station.DEFAULT_BAUD)
    parser.add_argument("--info", action="store_true", help="Exibe informações da Base Station")
    parser.add_argument("--scan", action="store_true", help="Escaneia espectro RPD (0..125)")
    parser.add_argument("--set-channel", type=int, default=None, help="Migra robôs + Base para canal TX 0..125")
    parser.add_argument("--rx-channel", type=int, default=None, help="Canal RX Full-Duplex 0..125 (usado com --set-channel)")
    parser.add_argument("--auto-channel", action="store_true", help="Escaneia e migra para o melhor par de canais")
    parser.add_argument("--tel", type=int, default=None, help="Solicita telemetria de um robot_id")
    parser.add_argument("--id", type=int, default=0, help="Robot ID alvo para comandos")
    parser.add_argument("--vx", type=int, default=None, help="Envia comando de velocidade vx (mm/s)")
    parser.add_argument("--vy", type=int, default=0)
    parser.add_argument("--w", type=int, default=0)
    parser.add_argument("--kick", type=int, default=0)
    parser.add_argument("--set-id", type=int, default=None, help="Altera ID do robô --id para novo ID")
    parser.add_argument("--calibrate", action="store_true")
    parser.add_argument("--factory-reset", action="store_true")
    args = parser.parse_args()

    try:
        link, info = connect_base(args.port, args.baud)
    except Exception as exc:
        print(f"{Colors.RED}Erro: {exc}{Colors.RESET}")
        sys.exit(1)

    try:
        if args.info or not any(
            x is not None and x is not False
            for x in (
                args.scan,
                args.set_channel,
                args.auto_channel,
                args.tel,
                args.vx,
                args.set_id,
                args.calibrate,
                args.factory_reset,
            )
        ):
            print_info(info, link.port, link.baud)
        if args.scan:
            scan = link.scan_channels(0, 125, 32)
            if scan:
                print_scan(scan)
        if args.auto_channel:
            scan = link.scan_channels(0, 125, 32)
            if scan:
                print_scan(scan)
                switch_channel(
                    link,
                    scan["best_channel"],
                    rx_channel=scan.get("best_rx_channel"),
                    notify_robots=True,
                )
        if args.set_channel is not None:
            switch_channel(link, args.set_channel, rx_channel=args.rx_channel, notify_robots=True)
        if args.tel is not None:
            tel, tx_st, rtt = send_telemetry_request(link, args.tel)
            if tel:
                print(
                    f"{Colors.GREEN}✓ Robô {tel['robot_id']} | RTT={rtt:.2f}ms | "
                    f"Batt={tel['battery_pct']}% | Kicker={tel['kicker_voltage_v']:.2f}V | "
                    f"Err=0x{tel['error_flags']:04X}{Colors.RESET}"
                )
            else:
                print(f"{Colors.RED}✗ Sem resposta do robô {args.tel} (tx={tx_st}){Colors.RESET}")
        if args.vx is not None:
            cmd = radio.encode_command(args.id, vx=args.vx, vy=args.vy, w=args.w, kick=args.kick)
            st = link.send_rf(args.id, cmd)
            print(f"CMD -> robô {args.id}: {st}")
        if args.set_id is not None:
            cfg = radio.encode_config(args.id, radio.CONFIG_FLAG_SET_ID, args.set_id)
            print(f"SET_ID {args.id}->{args.set_id}: {link.send_rf(args.id, cfg)}")
        if args.calibrate:
            cfg = radio.encode_config(args.id, radio.CONFIG_FLAG_RUN_CALIBRATION, 0)
            print(f"CALIBRATE robô {args.id}: {link.send_rf(args.id, cfg)}")
        if args.factory_reset:
            cfg = radio.encode_config(args.id, radio.CONFIG_FLAG_FACTORY_RESET, 0)
            print(f"FACTORY_RESET robô {args.id}: {link.send_rf(args.id, cfg)}")
    finally:
        link.close()


if __name__ == "__main__":
    main()

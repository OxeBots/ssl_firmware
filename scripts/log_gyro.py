#!/usr/bin/env python3
import argparse
import csv
from datetime import datetime
import serial

GYRO_LSB_PER_DPS = 14.375


def temp_raw_to_c(raw: int) -> float:
    return 35.0 + (raw + 13200) / 280.0


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Coleta dados do ITG-3200 via serial e salva em CSV."
    )
    parser.add_argument(
        "-p", "--port", default="/dev/ttyUSB0", help="Porta serial (padrão: /dev/ttyUSB0)"
    )
    parser.add_argument(
        "-b", "--baud", type=int, default=115200, help="Baud rate (padrão: 115200)"
    )
    parser.add_argument("-o", "--out", default=None, help="Arquivo CSV de saída")
    parser.add_argument(
        "--print-every",
        type=int,
        default=10,
        help="Atualiza tela a cada N amostras (padrão: 10 = 10 Hz)",
    )
    args = parser.parse_args()

    out_path = args.out or f"gyro_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    count = 0
    t0_us = None

    print(f"Conectando em {args.port} @ {args.baud} baud...")
    print(f"Salvando em: {out_path} (Ctrl+C para encerrar)\n")

    with serial.Serial(args.port, args.baud, timeout=1.0) as ser, open(
        out_path, "w", newline=""
    ) as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp_us",
            "temp_raw", "temp_c",
            "gx_raw", "gy_raw", "gz_raw",
            "gx_dps", "gy_dps", "gz_dps",
        ])

        try:
            while True:
                raw_line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not raw_line.startswith("GYRO,"):
                    continue

                parts = raw_line.split(",")
                if len(parts) != 6:
                    continue

                try:
                    ts_us = int(parts[1])
                    temp_raw = int(parts[2])
                    gx_raw = int(parts[3])
                    gy_raw = int(parts[4])
                    gz_raw = int(parts[5])
                except ValueError:
                    continue

                if t0_us is None:
                    t0_us = ts_us

                temp_c = temp_raw_to_c(temp_raw)
                gx_dps = gx_raw / GYRO_LSB_PER_DPS
                gy_dps = gy_raw / GYRO_LSB_PER_DPS
                gz_dps = gz_raw / GYRO_LSB_PER_DPS

                writer.writerow([
                    ts_us,
                    temp_raw, f"{temp_c:.4f}",
                    gx_raw, gy_raw, gz_raw,
                    f"{gx_dps:.5f}", f"{gy_dps:.5f}", f"{gz_dps:.5f}",
                ])
                count += 1

                if count % 100 == 0:
                    f.flush()

                if count % args.print_every == 0:
                    elapsed_s = (ts_us - t0_us) / 1e6
                    print(
                        f"[{elapsed_s:7.2f}s | {count:6d} pts] "
                        f"T={temp_c:6.2f}°C ({temp_raw:6d}) | "
                        f"X={gx_dps:+7.3f}  Y={gy_dps:+7.3f}  Z={gz_dps:+7.3f} °/s  "
                        f"(raw: {gx_raw:+5d}, {gy_raw:+5d}, {gz_raw:+5d})"
                    )
        except KeyboardInterrupt:
            print(f"\nFinalizado! {count} amostras salvas em {out_path}")


if __name__ == "__main__":
    main()

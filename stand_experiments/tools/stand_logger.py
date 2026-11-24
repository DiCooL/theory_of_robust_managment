import argparse
import csv
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

FRAME_LEN = 66
HEADER = b"\x42\x40"
FIELDS = [
    "t_us",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "gyro_x_dps",
    "gyro_y_dps",
    "gyro_z_dps",
    "acc_x",
    "acc_y",
    "acc_z",
    "esc1",
    "esc2",
    "esc3",
    "esc4",
]


def checksum_ok(frame):
    expected = (0xFFFF - sum(frame[:64])) & 0xFFFF
    received = frame[64] | (frame[65] << 8)
    return expected == received


def decode(frame):
    t_hi, t_lo = struct.unpack_from("<HH", frame, 2)
    angles = struct.unpack_from("<3h", frame, 6)
    gyro = struct.unpack_from("<3h", frame, 12)
    acc = struct.unpack_from("<3h", frame, 18)
    esc = struct.unpack_from("<4H", frame, 24)
    return [
        (t_hi << 16) | t_lo,
        angles[0] / 100.0,
        angles[1] / 100.0,
        angles[2] / 100.0,
        gyro[0] / 100.0,
        gyro[1] / 100.0,
        gyro[2] / 100.0,
        acc[0] / 10000.0,
        acc[1] / 10000.0,
        acc[2] / 10000.0,
        esc[0],
        esc[1],
        esc[2],
        esc[3],
    ]


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()
        self.good = 0
        self.bad = 0

    def feed(self, data):
        self.buffer.extend(data)
        rows = []
        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                del self.buffer[: max(0, len(self.buffer) - 1)]
                break
            if start > 0:
                del self.buffer[:start]
            if len(self.buffer) < FRAME_LEN:
                break
            frame = bytes(self.buffer[:FRAME_LEN])
            if checksum_ok(frame):
                rows.append(decode(frame))
                self.good += 1
                del self.buffer[:FRAME_LEN]
            else:
                self.bad += 1
                del self.buffer[:1]
        return rows


def list_ports():
    from serial.tools import list_ports as lp

    ports = list(lp.comports())
    if not ports:
        print("COM-порты не найдены")
    for p in ports:
        print(f"{p.device}: {p.description}")


def run(port, baud, out_path, duration, quiet, every):
    import serial

    parser = FrameParser()
    started = time.time()
    last_print = started
    prev_t = None
    elapsed_us = 0
    gaps = 0
    max_gap_us = 0
    last_row = None
    active = False
    roll_min = None
    roll_max = None
    esc_max = 0
    with serial.Serial(port, baud, timeout=0.2) as ser, open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["pc_time_s"] + FIELDS)
        ser.reset_input_buffer()
        print(f"Порт {port}, {baud} бод. Запись в {out_path}. Остановка: Ctrl+C")
        try:
            while duration is None or time.time() - started < duration:
                data = ser.read(4096)
                now = time.time()
                for row in parser.feed(data):
                    writer.writerow([f"{now - started:.4f}"] + row)
                    if prev_t is not None:
                        dt = (row[0] - prev_t) & 0xFFFFFFFF
                        elapsed_us += dt
                        max_gap_us = max(max_gap_us, dt)
                        if dt > 5000:
                            gaps += 1
                    prev_t = row[0]
                    last_row = row
                    roll_min = row[1] if roll_min is None else min(roll_min, row[1])
                    roll_max = row[1] if roll_max is None else max(roll_max, row[1])
                    esc_max = max(esc_max, max(row[10:14]))
                    is_active = max(row[10:14]) > 1010
                    if is_active != active:
                        active = is_active
                        state = "ВКЛЮЧИЛИСЬ" if active else "выключились"
                        print(
                            f">>> {now - started:6.2f} с: моторы {state} | "
                            f"крен {row[1]:7.2f}° | ESC {row[10]} {row[11]} {row[12]} {row[13]}"
                        )
                if not quiet and now - last_print >= every:
                    last_print = now
                    if last_row is None:
                        print(f"кадров нет, плохих {parser.bad}, байт в буфере {len(parser.buffer)}")
                    else:
                        rate = parser.good / (elapsed_us / 1e6) if elapsed_us > 0 else 0.0
                        print(
                            f"{now - started:6.1f} с | кадров {parser.good}, плохих {parser.bad}, ~{rate:.0f} Гц | "
                            f"крен {last_row[1]:7.2f}°  гиро X {last_row[4]:8.2f}°/с | "
                            f"ESC {last_row[10]} {last_row[11]} {last_row[12]} {last_row[13]}"
                        )
        except KeyboardInterrupt:
            pass
    print("-" * 60)
    print(f"Готово: кадров {parser.good}, плохих {parser.bad}, файл {out_path}")
    if parser.good:
        print(f"Длительность по часам платы: {elapsed_us / 1e6:.1f} с")
        print(f"Пропусков больше 5 мс: {gaps}, самый длинный {max_gap_us / 1000:.1f} мс")
        print(f"Крен: от {roll_min:.2f}° до {roll_max:.2f}°")
        print(f"Максимальный ШИМ мотора: {esc_max}" + ("  (моторы не включались)" if esc_max <= 1010 else ""))
        print("Проверка знака реакции: py check_response.py")


def main():
    ap = argparse.ArgumentParser(description="Запись телеметрии стенда (UART2 прошивки vimana) в CSV")
    ap.add_argument("--port", help="например COM5")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--out", help="имя CSV-файла; по умолчанию logs\\stand_ДАТА_ВРЕМЯ.csv")
    ap.add_argument("--duration", type=float, help="длительность записи, с; по умолчанию до Ctrl+C")
    ap.add_argument("--list", action="store_true", help="показать COM-порты и выйти")
    ap.add_argument("--every", type=float, default=1.0, help="как часто печатать строку состояния, с")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    if args.list:
        list_ports()
        return 0
    if not args.port:
        ap.error("нужен --port (список портов: --list)")
    if args.out:
        out_path = Path(args.out)
    else:
        out_path = Path(__file__).resolve().parent / "logs" / f"stand_{datetime.now():%Y%m%d_%H%M%S}.csv"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    run(args.port, args.baud, out_path, args.duration, args.quiet, args.every)
    return 0


if __name__ == "__main__":
    sys.exit(main())

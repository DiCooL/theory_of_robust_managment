import csv
import math
import sys
from pathlib import Path

IDLE = 1010


def load(path):
    rows = []
    with open(path, newline="") as f:
        for d in csv.DictReader(f):
            rows.append({k: float(v) for k, v in d.items()})
    t = []
    acc = 0
    prev = None
    for r in rows:
        v = int(r["t_us"])
        if prev is not None:
            acc += (v - prev) & 0xFFFFFFFF
        prev = v
        t.append(acc / 1e6)
    return rows, t


def corr(x, y):
    n = min(len(x), len(y))
    if n < 10:
        return 0.0
    x = x[:n]
    y = y[:n]
    mx = sum(x) / n
    my = sum(y) / n
    sx = math.sqrt(sum((a - mx) ** 2 for a in x))
    sy = math.sqrt(sum((b - my) ** 2 for b in y))
    if sx < 1e-9 or sy < 1e-9:
        return 0.0
    return sum((a - mx) * (b - my) for a, b in zip(x, y)) / (sx * sy)


def segments(rows, t):
    act = [max(r["esc1"], r["esc2"], r["esc3"], r["esc4"]) > IDLE for r in rows]
    segs = []
    start = None
    last = None
    for i, a in enumerate(act):
        if a:
            if start is None:
                start = i
            elif t[i] - t[last] > 0.3:
                segs.append((start, last))
                start = i
            last = i
    if start is not None:
        segs.append((start, last))
    return [s for s in segs if t[s[1]] - t[s[0]] > 0.5]


def analyse(rows, t, a, b, props):
    seg = rows[a : b + 1]
    ts = t[a : b + 1]
    dur = ts[-1] - ts[0]
    print(f"\nМоторы работали с {ts[0]:.2f} с по {ts[-1]:.2f} с ({dur:.1f} с)")
    for k in ("esc1", "esc2", "esc3", "esc4"):
        v = [r[k] for r in seg]
        print(f"  {k}: от {min(v):.0f} до {max(v):.0f}, среднее {sum(v) / len(v):.0f}")
    roll = [r["roll_deg"] for r in seg]
    diff = [(r["esc1"] + r["esc2"]) - (r["esc3"] + r["esc4"]) for r in seg]
    print(f"  крен за это время: от {min(roll):.2f}° до {max(roll):.2f}°")
    c = corr(roll, diff)
    print(f"\nПроверка А (логика прошивки): корреляция крена и разницы (ESC1+ESC2)-(ESC3+ESC4) = {c:+.2f}")
    if c > 0.5:
        print("  Крен положительный, сильнее работает пара 1-2. Прошивка делает то, что в ней написано.")
    elif c < -0.5:
        print("  Наоборот: при положительном крене сильнее пара 3-4. Это не совпадает с кодом, нужно разбираться.")
    else:
        print("  Связи не видно: крен был слишком мал, или значения упёрлись в ограничения 1110/1490.")
    if not props:
        print("\nПроверка Б (реальная реакция тяги) пропущена: запуск без ключа --props, то есть без винтов или с наклоном рукой.")
        return
    step = max(1, len(seg) // max(1, int(dur * 80)))
    d = diff[::step]
    g = [r["gyro_x_dps"] for r in seg][::step]
    fs = len(d) / dur if dur > 0 else 0
    best = (0.0, 0.0)
    for lag_ms in range(0, 601, 20):
        k = int(lag_ms / 1000 * fs)
        if len(d) - k < 20:
            break
        cc = corr(d[: len(d) - k], g[k:])
        if abs(cc) > abs(best[0]):
            best = (cc, lag_ms / 1000)
    print(f"\nПроверка Б (реальная реакция): сильнейшая связь разницы ESC со скоростью крена {best[0]:+.2f} при задержке {best[1]:.2f} с")
    if abs(best[0]) < 0.3:
        print("  Связь слабая. Без винтов или при ручном наклоне эта проверка ничего не показывает.")
    elif best[0] < 0:
        print("  Когда сильнее пара 1-2, крен уменьшается: обратная связь отрицательная, рама возвращается к нулю.")
    else:
        print("  Когда сильнее пара 1-2, крен растёт: обратная связь положительная. Так нельзя, нужно менять знак.")


def main():
    args = [x for x in sys.argv[1:] if not x.startswith("--")]
    props = "--props" in sys.argv
    if args:
        path = Path(args[0])
    else:
        logs = sorted((Path(__file__).resolve().parent / "logs").glob("stand_*.csv"))
        if not logs:
            print("Нет файлов в logs")
            return 1
        path = logs[-1]
    rows, t = load(path)
    print(f"Файл {path.name}: {len(rows)} кадров, {t[-1]:.1f} с")
    segs = segments(rows, t)
    if not segs:
        mx = max(max(r["esc1"], r["esc2"], r["esc3"], r["esc4"]) for r in rows)
        print(f"Моторы не включались: максимальный ШИМ {mx:.0f}, нужно больше {IDLE}.")
        print("Проверь тумблер канала 5 и что после включения платы прошло не меньше 20 с.")
        return 0
    for a, b in segs:
        analyse(rows, t, a, b, props)
    return 0


if __name__ == "__main__":
    sys.exit(main())

import re
import sys
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial

# ──── 설정 ────
COM_PORT = "COM10"       # 맞는 포트로 변경
BAUD = 115200
SENSOR_NUM = 4

# 센서 방향 벡터 (반시계 방향, S0 = 12시 = (0,1))
angle_inc = 2 * np.pi / SENSOR_NUM
sensor_dirs = np.array([
    [-np.sin(i * angle_inc), np.cos(i * angle_inc)]  # CCW from +Y
    for i in range(SENSOR_NUM)
])

# UART 출력이 SUM= 뒤에서 줄바꿈될 수 있으므로 F=[ ] 만 매칭
line_pattern = re.compile(
    r"RAW=\[(?P<raw>[^\]]+)\]\s*"
    r"F=\[(?P<f>[^\]]+)\]\s*"
    r"V=\[(?P<v>[^\]]+)\]\s*"
    r"SUM=\s*(?P<sum>[-+]?\d*\.?\d+)\s*"
    r"(?:ANG=\s*(?P<ang>[-+]?\d*\.?\d+))?\s*"
    r"(?:x=\s*(?P<x>[-+]?\d*\.?\d+))?\s*"
    r"(?:y=\s*(?P<y>[-+]?\d*\.?\d+))?"
)

def parse_vector(text):
    return [float(x) for x in text.split()]

def compute_fire_vector(f_vals):
    f = np.array(f_vals, dtype=float)
    norm = np.sqrt(np.sum(f ** 2))
    if norm < 1e-6:
        return 0.0, 0.0, 0.0
    f_n = f / norm
    vx = -np.sum(sensor_dirs[:, 0] * f_n) / SENSOR_NUM
    vy = -np.sum(sensor_dirs[:, 1] * f_n) / SENSOR_NUM
    return vx, vy, np.mean(f)

# ──── 시리얼 열기 ────
try:
    ser = serial.Serial(COM_PORT, BAUD, timeout=0.05)
except serial.SerialException as e:
    print(f"Cannot open {COM_PORT}: {e}")
    sys.exit(1)

print(f"Opened {COM_PORT} @ {BAUD} baud. Waiting for data...")

# ──── 최신 파싱 결과 저장 ────
latest = {"f": np.zeros(SENSOR_NUM), "vx": 0.0, "vy": 0.0, "ang": 0.0, "x": 0.0, "y": 0.0, "sum": 0.0}
_serial_buf = ""  # 줄바꿈으로 잘린 데이터 버퍼

def read_serial():
    """시리얼 버퍼에서 가장 최신 유효 라인을 파싱 (줄바꿈 잘림 대응)."""
    global _serial_buf
    last_match = None

    n_waiting = ser.in_waiting
    if n_waiting == 0:
        return                       # 데이터 없으면 바로 리턴 (블로킹 방지)
    raw_bytes = ser.read(n_waiting)
    if not raw_bytes:
        return
    text = raw_bytes.decode("utf-8", errors="ignore")
    _serial_buf += text

    # RAW= 로 시작하는 구간별로 쪼개기
    parts = re.split(r"(?=RAW=)", _serial_buf)
    # 마지막 조각은 아직 완성 안 됐을 수 있으므로 버퍼에 남김
    _serial_buf = parts[-1]
    for part in parts[:-1]:
        joined = part.replace("\r", "").replace("\n", " ").strip()
        m = line_pattern.search(joined)
        if m:
            last_match = m

    if last_match:
        f_vals = parse_vector(last_match.group("f"))
        vx, vy, _ = compute_fire_vector(f_vals)
        latest["f"] = np.array(f_vals)
        latest["vx"] = vx
        latest["vy"] = vy
        latest["ang"] = float(last_match.group("ang") or 0.0)
        latest["x"] = float(last_match.group("x") or 0.0)
        latest["y"] = float(last_match.group("y") or 0.0)
        latest["sum"] = float(last_match.group("sum") or 0.0)
        print(f"\rF={f_vals} vx={vx:.4f} vy={vy:.4f} ang={latest['ang']:.2f}° x={latest['x']:.4f} y={latest['y']:.4f} sum={latest['sum']:.2f}", end="", flush=True)

# ──── 그래프 세팅 ────
fig, (ax_arrow, ax_bar) = plt.subplots(
    1, 2, figsize=(10, 5), gridspec_kw={"width_ratios": [2, 1]}
)
fig.canvas.manager.set_window_title("Fire Direction — Live")
colors_bar = ["tab:blue", "tab:orange", "tab:green", "tab:red"]

def update(frame_idx):
    read_serial()

    ax_arrow.cla()
    ax_bar.cla()

    vx, vy = latest["vx"], latest["vy"]
    mag = np.sqrt(vx**2 + vy**2)

    # 왼쪽: 화살표
    ax_arrow.set_xlim(-1.5, 1.5)
    ax_arrow.set_ylim(-1.5, 1.5)
    ax_arrow.set_aspect("equal")
    ax_arrow.set_title("Fire Direction (Live)", fontsize=13)
    ax_arrow.grid(True, linestyle="--", alpha=0.25)
    ax_arrow.axhline(0, color="gray", lw=0.5)
    ax_arrow.axvline(0, color="gray", lw=0.5)

    # 크기 기준 동심원 (0.02 ~ 0.10)
    for ref_mag in [0.02, 0.04, 0.06, 0.08, 0.10]:
        r = ref_mag / 0.1 * 1.2       # 같은 스케일
        circle = plt.Circle((0, 0), r, fill=False, color="lightgray",
                             linestyle="--", lw=0.7)
        ax_arrow.add_patch(circle)
        ax_arrow.text(r + 0.03, 0, f"{ref_mag:.2f}",
                      fontsize=7, color="gray", va="center")

    for i in range(SENSOR_NUM):
        sx, sy = sensor_dirs[i]
        ax_arrow.plot(sx, sy, "o", color="steelblue", markersize=8)
        ax_arrow.text(sx * 1.18, sy * 1.18, f"S{i}",
                      ha="center", va="center", fontsize=9, color="steelblue")

    if mag > 1e-6:
        ux, uy = vx / mag, vy / mag
        arrow_len = mag / 0.1 * 1.2    # mag 0~0.1 → 화살표 0~1.2
        ax_arrow.annotate(
            "", xy=(ux * arrow_len, uy * arrow_len), xytext=(0, 0),
            arrowprops=dict(arrowstyle="->", color="red", lw=3),
        )
        ax_arrow.text(0, -1.35, f"mag={mag:.4f} | ANG={latest['ang']:.2f}° | x={latest['x']:.4f} | y={latest['y']:.4f} | SUM={latest['sum']:.2f}", ha="center", fontsize=9, color="red")
    else:
        ax_arrow.plot(0, 0, "x", color="red", markersize=12, mew=2)

    # 오른쪽: F 바 차트
    ax_bar.barh(
        [f"F[{i}]" for i in range(SENSOR_NUM)],
        latest["f"],
        color=[colors_bar[i % len(colors_bar)] for i in range(SENSOR_NUM)],
    )
    ax_bar.set_xlim(0, max(latest["f"].max() * 1.2, 1))
    ax_bar.set_title("Sensor F values", fontsize=12)
    ax_bar.grid(True, axis="x", linestyle="--", alpha=0.3)

ani = animation.FuncAnimation(fig, update, interval=50, cache_frame_data=False)
plt.tight_layout()
plt.show()

ser.close()

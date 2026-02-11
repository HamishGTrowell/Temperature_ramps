# pip install pywinauto
from pywinauto import Desktop, keyboard
import time, re, statistics
from collections import deque
from dataclasses import dataclass

# =============================== CONFIG ===============================
TITLE_RE_UVVIS   = r"^Kinetics\s+.*-\s*Online$"
TITLE_RE_TEMPAPP = r"^T-App\s+-\s+Temperature Control Application\s+on\s+COM4$"
NUM_PATTERN = re.compile(r"^-?\d+\.\d{1,4}$")

# Approximate coordinates of the command Edit box (from Inspect.exe)
TEMPAPP_EDIT_REF_X = 1312
TEMPAPP_EDIT_REF_Y = 800

START_TEMP = 45.0
MAX_TEMP   = 100.0
MAX_RAMP   = 10.0
FAST_RAMP  = 2.0
COOL_RAMP  = -1.0

CHECK_PERIOD_S = 30           # control tick (s)
SLOPE_WIN_SEC  = 30           # absorbance slope window (s)
EXPECTED_MAX_ABS = 1.0
SWITCH_TIME_HRS = 5.0         # defines reference slope for 5 h → A=1
S_REF          = EXPECTED_MAX_ABS / (SWITCH_TIME_HRS*60.0)  # AU/min
Kp             = 100.0       # °C/min per (AU/min) slope error
S_FLOOR        = 1e-5

STABILITY_WINDOW_MIN   = 10
STABILITY_MAX_RANGE_AU = 0.05
STABILITY_MAX_SLOPE    = 0.005
PLATEAU_TICKS          = 3
# ======================================================================


@dataclass
class ControlUpdate:
    time_s: float
    state: str
    absorbance: float
    slope_au_per_min: float
    temp_target_c: float
    ramp_rate_c_per_min: float
    notes: str = ""


# ========================== UV-Vis READOUT =============================
def get_uvvis_window():
    win = Desktop(backend="uia").window(title_re=TITLE_RE_UVVIS, control_type="Window")
    win.wait("exists ready", timeout=20)
    return win

def get_absorbance_element(win):
    candidates = []
    for el in win.descendants():
        s = (el.window_text() or "").strip()
        if NUM_PATTERN.match(s):
            r = el.rectangle()
            candidates.append(((r.top, r.left), el))
    if not candidates:
        raise RuntimeError("No numeric UI element found in UV-Vis window.")
    candidates.sort(key=lambda x: (x[0][0], x[0][1]))
    return candidates[0][1]

def read_absorbance(el):
    s = (el.window_text() or "").strip()
    if NUM_PATTERN.match(s):
        return float(s)
    return None
# =======================================================================


# ======================== TEMPERATURE APP ==============================
def _pick_tempapp_edit(win):
    edits = win.descendants(control_type="Edit")
    if not edits:
        raise RuntimeError("No Edit controls found in Temperature App window.")
    target_x, target_y = TEMPAPP_EDIT_REF_X, TEMPAPP_EDIT_REF_Y
    best = min(edits, key=lambda e: abs(e.rectangle().left - target_x) + abs(e.rectangle().top - target_y))
    return best

def send_temp_command(command: str):
    try:
        win = Desktop(backend="uia").window(title_re=TITLE_RE_TEMPAPP)
        win.wait("exists ready", timeout=10)
        edit = _pick_tempapp_edit(win)
        edit.set_focus()
        edit.type_keys("^a{BACKSPACE}", with_spaces=True, set_foreground=True)
        edit.type_keys(command, with_spaces=True, pause=0.02, set_foreground=True)
        keyboard.send_keys("{ENTER}")
        print(f"[T-App] Sent: {command}")
    except Exception as e:
        print(f"[T-App] ERROR sending command: {e}")
# =======================================================================


# ======================== DATA UTILITIES ===============================
def median_last_seconds(history: deque, seconds: int):
    cutoff = time.time() - seconds
    vals = [a for (t,a) in history if t >= cutoff]
    return statistics.median(vals) if vals else None

def slope_over_window(history: deque, window_sec: int):
    cutoff = time.time() - window_sec
    pts = [(t,a) for (t,a) in history if t >= cutoff]
    if len(pts) < 6:
        return None
    t0 = pts[0][0]
    xs = [(t - t0)/60.0 for (t,_) in pts]
    ys = [a for (_,a) in pts]
    n = len(xs)
    mx = sum(xs)/n; my = sum(ys)/n
    num = sum((x-mx)*(y-my) for x,y in zip(xs,ys))
    den = sum((x-mx)**2 for x in xs)
    return (num/den) if den != 0 else None

def range_over_window(history: deque, window_min: float):
    cutoff = time.time() - window_min*60
    vals = [a for (t,a) in history if t >= cutoff]
    return (max(vals) - min(vals)) if vals else None
# =======================================================================


# =========================== MAIN CONTROL ==============================
def control_loop():
    uvwin = get_uvvis_window()
    absel = get_absorbance_element(uvwin)
    print("UV-Vis readout connected.")

    # States
    STATE_LINEAR    = "FAST_LINEAR_RAMP"
    STATE_NONLINEAR = "PROPORTIONAL_CONTROL"
    STATE_HOLD      = "HOLD"
    STATE_COOL      = "COOL"
    STATE_DONE      = "DONE"

    state = STATE_LINEAR
    history = deque(maxlen=100000)
    temp = START_TEMP
    ramp = FAST_RAMP
    plateau_counter = 0

    print("Controller started (proportional-only). Updates every 30 s.")

    while True:
        # ---- collect absorbance data for one period ----
        t0 = time.time()
        while time.time() - t0 < CHECK_PERIOD_S:
            val = read_absorbance(absel)
            if val is not None:
                history.append((time.time(), val))
            time.sleep(0.1)

        A_med = median_last_seconds(history, 30)
        s_meas = slope_over_window(history, SLOPE_WIN_SEC) or 0.0
        notes = ""

        # ================== STATE LOGIC ==================
        if state == STATE_LINEAR:
            ramp = FAST_RAMP
            temp += ramp * (CHECK_PERIOD_S / 60.0)
            if s_meas >= S_REF:
                state = STATE_NONLINEAR
                ramp = 0
                temp -= 2* ramp * (CHECK_PERIOD_S / 60.0) #! should change to read probe temp then set holder temp accordingly
                notes = "→ proportional control"

        elif state == STATE_NONLINEAR:
            # --- PID-based control for smoother ramp adjustment ---
            # Initialize persistent variables once
            if "integral" not in locals():
                integral = 0.0
                prev_error = 0.0

            e = S_REF - s_meas
            dt_min = CHECK_PERIOD_S / 60.0

            # PID gains (tune as needed)
            Kp_pid = 80.0      # proportional gain
            Ki_pid = 10.0      # integral gain
            Kd_pid = 5.0       # derivative gain

            # Update integral and derivative
            integral += e * dt_min
            derivative = (e - prev_error) / dt_min

            # Anti-windup: clamp integral term
            integral = max(min(integral, 0.01), -0.01)

            # Compute new ramp
            ramp = Kp_pid * e + Ki_pid * integral + Kd_pid * derivative
            ramp = max(-10.0, min(MAX_RAMP, ramp))

            # Update temperature target
            temp += ramp * dt_min
            temp = max(START_TEMP, min(MAX_TEMP, temp))

            # Save previous error for next cycle
            prev_error = e

            # Plateau check (unchanged)
            if (temp >= MAX_TEMP and ramp > 0) or (ramp >= MAX_RAMP and e > 0):
                plateau_counter += 1
            else:
                plateau_counter = 0
            if plateau_counter >= PLATEAU_TICKS:
                state = STATE_HOLD
                ramp = 0.0
                temp = MAX_TEMP
                notes = "Reached limit → HOLD"

        elif state == STATE_HOLD:
            ramp = 0.0
            temp = MAX_TEMP
            rng = range_over_window(history, STABILITY_WINDOW_MIN)
            s10 = slope_over_window(history, int(STABILITY_WINDOW_MIN*60)) or 0.0
            if rng is not None and rng < STABILITY_MAX_RANGE_AU and abs(s10) < STABILITY_MAX_SLOPE:
                state = STATE_COOL
                ramp = COOL_RAMP
                temp = START_TEMP
                notes = "Stable → COOL"

        elif state == STATE_COOL:
            ramp = COOL_RAMP
            temp = START_TEMP
            if temp <= START_TEMP:
                temp = START_TEMP
                ramp = 0.0
                state = STATE_DONE
                notes = "Reached 20 °C → done."

        # ---- send to controller ----
        send_temp_command(f"[F1 RR S {ramp:.2f}]")
        send_temp_command(f"[F1 TT S {temp:.2f}]")

        # ---- logging ----
        print(f"[{time.strftime('%H:%M:%S')}] {state:18s} "
              f"T={temp:6.2f} °C  ramp={ramp:6.2f} °C/min  "
              f"A={A_med if A_med else 0:.4f}  dA/dt={s_meas:.5f}  {notes}")

        if state == STATE_DONE:
            break
# =======================================================================


if __name__ == "__main__":
    control_loop()

#!/usr/bin/env python3
"""xArm besturen met het toetsenbord (ingedrukt houden = blijven bewegen).

Vereist:  pip install pynput

Starten:  python3 xarm_toetsenbord.py
Voor echt precies ingedrukt houden (loslaten, Shift, Ctrl, meerdere toetsen tegelijk):
draai het op de computer zelf (niet in Docker) met  pip install pynput

Toetsen (houd ingedrukt):
    pijl omhoog / omlaag    : +X / -X  (vooruit / achteruit)
    pijl links / rechts     : +Y / -Y
    Shift  (of Page Up)     : Z omhoog
    Ctrl   (of Page Down)   : Z omlaag
    +  /  -                 : grijper open / dicht  (numpad +/- aanbevolen)

Losse toetsen:
    s : snelheid wisselen            g : grijpersnelheid wisselen
    o / c : grijper helemaal open / dicht
    h : naar vak 1 op verplaatsingshoogte
    k : huidige X/Y = midden van vak 1 (herkalibreren)
    p : positie tonen
    spatiebalk : alles direct stoppen (terminalmodus)
    q of Esc : stoppen   (Ctrl+C in de terminal werkt ook)

Coördinatenstelsel "dammen": nulpunt = midden van vak 1 (via world offset).

TERMINALMODUS (automatisch in Docker/SSH, zonder pynput): een terminal kan niet zien dat
je een toets loslaat, en ook Shift/Ctrl niet lezen. Daarom: één druk op een pijl,
Page Up/Down (of r / f) of + / - laat de arm/grijper doorlopen tot je op de SPATIEBALK
drukt. Z omhoog = Page Up of r, Z omlaag = Page Down of f.

LET OP (pynput-modus): de toetsen worden globaal gelezen, dus ook als het terminalvenster niet
actief is. Sluit het programma af voordat je in een ander venster gaat typen.
Als de arm gaat bewegen: loslaten stopt hem meteen; de noodstop blijft binnen bereik.
"""

import os
import re
import select
import sys
import threading
import time

from xarm.wrapper import XArmAPI

try:
    from pynput import keyboard
    HAVE_PYNPUT = True
except Exception:          # niet geïnstalleerd, of geen beeldscherm (bijv. in Docker)
    keyboard = None
    HAVE_PYNPUT = False

# pynput heeft een echt toetsenbord/beeldscherm nodig. In Docker/SSH zonder scherm
# valt het programma terug op lezen via de terminal.
USE_PYNPUT = HAVE_PYNPUT and (os.name == "nt" or sys.platform == "darwin"
                              or bool(os.environ.get("DISPLAY")))

# ================= CONFIGURATIE =================
ROBOT_IP = "192.168.1.237"

# Het "dammen"-stelsel (nulpunt = midden van vak 1). Zet op None als dit stelsel
# al actief is op de robot (bijvoorbeeld via xArm Studio).
# Gaat de arm niet naar het juiste punt? Probeer [210, 196, -65, 0, 0, 0].
WORLD_OFFSET = [-210, -196, 65, 0, 0, 0]

# Midden van vak 1 in het dammen-stelsel (mm). Herkalibreren kan met 'k'.
VAK1_X = 0.0
VAK1_Y = 0.0

SQUARE_SIZE_MM = 45.0
HALF = SQUARE_SIZE_MM / 2.0
BOARD_STEP_X = 9 * SQUARE_SIZE_MM       # vak 1 -> vak 50 in X (405 mm)
BOARD_STEP_Y = 9 * SQUARE_SIZE_MM       # vak 1 -> vak 50 in Y (405 mm)

# Hoogtegrenzen (mm) in het dammen-stelsel: pas aan op jouw opstelling!
# Z_MIN is de laagste hoogte waarop de grijper nog veilig boven het bord blijft.
Z_MIN = 0.0
Z_MAX = 250.0
Z_START = 150.0

# Roll/pitch/yaw worden niet met de toetsen aangestuurd.
# Standaard: roll 180, pitch 0, yaw 0 (gereedschap loodrecht naar beneden).
ORIENTATION = [180.0, 0.0, 0.0]
USE_CURRENT_ORIENTATION = False

# Richting van de toetsen. Staat een as de verkeerde kant op: zet op -1.
# Pijl omhoog = +X, pijl omlaag = -X, pijl links = +Y, pijl rechts = -Y.
DIR_X = 1
DIR_Y = 1

# Snelheden (mm/s), te wisselen met 's'
SPEEDS_MM_S = [10.0, 30.0, 60.0, 100.0]
SPEED_IDX = 1

# Z beweegt langzamer dan X/Y: Z-snelheid = gekozen snelheid x deze factor.
# Kleiner = fijner (0.25 -> bij 30 mm/s beweegt Z met 7,5 mm/s).
Z_SPEED_FACTOR = 0.25

# Grijper (xArm Gripper: 0 = dicht, 850 = open), snelheid in eenheden per seconde
GRIPPER_OPEN = 850
GRIPPER_CLOSED = 0
GRIPPER_SPEED = 3000
# Gewone grijpersnelheid tijdens bedienen (eenheden/s), te wisselen met 'g'.
# De grijper loopt soepel door naar open/dicht zolang je de toets vasthoudt en stopt bij loslaten.
GRIPPER_RATES = [10, 30, 100, 300]
GRIPPER_RATE_IDX = 1

ACCEL_MM_S2 = 150.0         # zacht optrekken en afremmen (mm/s^2); Z gebruikt dit x Z_SPEED_FACTOR
LOOP_DT = 0.05              # 20 Hz regelcyclus
VC_DURATION = 0.25          # veiligheid: snelheid valt na 0,25 s vanzelf op 0 als het programma stokt
BRAKE_GAIN = 5.0            # afremmen bij de bordrand (1/s): remt vanaf ~ snelheid/5 mm van de rand
# ================================================

LIM = {}


def update_limits() -> None:
    LIM["x_min"] = VAK1_X - HALF
    LIM["x_max"] = VAK1_X + BOARD_STEP_X + HALF
    LIM["y_max"] = VAK1_Y + HALF                  # vak 1 zit aan de +Y kant
    LIM["y_min"] = VAK1_Y - BOARD_STEP_Y - HALF
    LIM["z_min"] = Z_MIN
    LIM["z_max"] = Z_MAX


update_limits()


# ---------- toetsen ----------
pressed: set[str] = set()                # pynput-modus: nu ingedrukt
active_until: dict[str, float] = {}      # terminalmodus: toets geldt als ingedrukt tot dit tijdstip
events: list[str] = []
last_single: dict[str, float] = {}
lock = threading.Lock()

HOLD_KEYS = {"up", "down", "left", "right", "pgup", "pgdn", "+", "-"}
SINGLE_KEYS = {"q", "s", "g", "o", "c", "h", "k", "p"}


def push_event(n: str) -> None:
    """Losse toets (lock moet vastgehouden zijn). Herhaling binnen 0,3 s negeren."""
    now = time.time()
    if now - last_single.get(n, 0.0) > 0.3:
        events.append(n)
    last_single[n] = now


if HAVE_PYNPUT:
    SPECIAL = {
        keyboard.Key.up: "up", keyboard.Key.down: "down",
        keyboard.Key.left: "left", keyboard.Key.right: "right",
        keyboard.Key.page_up: "pgup", keyboard.Key.page_down: "pgdn",
        keyboard.Key.shift: "shift", keyboard.Key.shift_l: "shift", keyboard.Key.shift_r: "shift",
        keyboard.Key.ctrl: "ctrl", keyboard.Key.ctrl_l: "ctrl", keyboard.Key.ctrl_r: "ctrl",
        keyboard.Key.esc: "q",
    }
    NUMPAD_PLUS = {107, 0xFFAB, 65451}
    NUMPAD_MINUS = {109, 0xFFAD, 65453}

    def name_of(key) -> str:
        if key in SPECIAL:
            return SPECIAL[key]
        vk = getattr(key, "vk", None)
        if vk in NUMPAD_PLUS:
            return "+"
        if vk in NUMPAD_MINUS:
            return "-"
        ch = getattr(key, "char", None)
        if not ch:
            return ""
        ch = ch.lower()
        if ch in ("+", "="):
            return "+"
        if ch in ("-", "_"):
            return "-"
        return ch

    def on_press(key) -> None:
        n = name_of(key)
        if not n:
            return
        with lock:
            if n in pressed:
                return                  # auto-repeat negeren
            pressed.add(n)
            if n in SINGLE_KEYS:
                push_event(n)

    def on_release(key) -> None:
        n = name_of(key)
        with lock:
            pressed.discard(n)


# Terminalmodus: een terminal geeft GEEN "toets losgelaten" door. Daarom:
#  TERMINAL_LATCH = True : één druk op een pijl/Page-toets/+/- laat de arm (of grijper) soepel
#                          doorlopen tot je op de SPATIEBALK drukt. Geen stappen.
#                          (de tegenovergestelde toets keert om, spatiebalk stopt alles)
#  TERMINAL_LATCH = False: toetsherhaling gebruiken als "ingedrukt houden" (kan stappen geven)
TERMINAL_LATCH = False
OPPOSITE = {"up": "down", "down": "up", "left": "right", "right": "left",
            "pgup": "pgdn", "pgdn": "pgup", "+": "-", "-": "+"}

# Alleen voor TERMINAL_LATCH = False: toetsherhaling gebruiken als "ingedrukt houden"
# De herhaalvertraging en herhaalsnelheid van jouw toetsenbord worden automatisch geleerd
# tijdens het eerste gebruik; de beginwaarden hieronder gelden tot dan.
FIRST_TIMEOUT = [0.65]   # eerste aanslag: zo lang actief (moet langer zijn dan de herhaalvertraging)
REPEAT_TIMEOUT = [0.15]  # bij herhaling: zo lang actief na de laatste herhaling
_first_t: dict[str, float] = {}
_last_t: dict[str, float] = {}
_rep_n: dict[str, int] = {}
ESC_KEYS = {"[A": "up", "[B": "down", "[C": "right", "[D": "left",
            "OA": "up", "OB": "down", "OC": "right", "OD": "left",
            "[5~": "pgup", "[6~": "pgdn"}


def terminal_reader(stop_evt: threading.Event) -> None:
    fd = sys.stdin.fileno()
    while not stop_evt.is_set():
        if not select.select([fd], [], [], 0.05)[0]:
            continue
        data = os.read(fd, 32).decode(errors="ignore")
        # Een lezing kan meerdere toetsen bevatten (toetsherhaling): splitsen
        i = 0
        names = []
        while i < len(data):
            if data[i] == "\x1b":
                j = i + 1
                while j < len(data) and data[j] != "\x1b":
                    j += 1
                seq = data[i + 1:j]
                names.append("q" if seq == "" else ESC_KEYS.get(seq, ""))
                i = j
            else:
                ch = data[i].lower()
                i += 1
                if ch == " ":
                    with lock:
                        active_until.clear()          # spatiebalk = alles direct stoppen
                    continue
                if ch in ("+", "="):
                    ch = "+"
                elif ch in ("-", "_"):
                    ch = "-"
                elif ch == "r":
                    ch = "pgup"
                elif ch == "f":
                    ch = "pgdn"
                names.append(ch)
        now = time.time()
        with lock:
            for n in names:
                if n in HOLD_KEYS and TERMINAL_LATCH:
                    active_until[n] = float("inf")           # blijft actief tot spatiebalk
                    active_until.pop(OPPOSITE[n], None)
                elif n in HOLD_KEYS:
                    first = active_until.get(n, 0.0) < now
                    if first:
                        _first_t[n], _rep_n[n] = now, 0
                    else:
                        if _rep_n.get(n) == 0:                      # eerste herhaling = herhaalvertraging
                            delay = now - _first_t[n]
                            if 0.1 < delay < 1.0:
                                FIRST_TIMEOUT[0] = delay + 0.08
                        elif _rep_n.get(n) == 1:                    # tweede herhaling = herhaalinterval
                            gap = now - _last_t[n]
                            if 0.005 < gap < 0.2:
                                REPEAT_TIMEOUT[0] = min(0.3, max(0.08, gap * 3))
                        _rep_n[n] = _rep_n.get(n, 0) + 1
                    _last_t[n] = now
                    active_until[n] = now + (FIRST_TIMEOUT[0] if first else REPEAT_TIMEOUT[0])
                elif n in SINGLE_KEYS:
                    push_event(n)


# ---- Terminals met het "kitty keyboard protocol": echte indrukken EN loslaten ----
# Ondersteund door o.a. kitty, WezTerm, foot, Ghostty, Alacritty (>= 0.13), iTerm2 (>= 3.5).
# Het script vraagt dit bij het starten aan de terminal; werkt het, dan is het precies zoals
# met pynput (ingedrukt = bewegen, loslaten = stop, Shift/Ctrl, meerdere toetsen tegelijk).
KITTY = [False]
KITTY_RE = re.compile(r"\x1b\[([0-9:;]*)([A-Za-z~])")
KITTY_LETTERS = {"q", "s", "g", "o", "c", "h", "k", "p"}


def detect_kitty(fd: int) -> bool:
    sys.stdout.write("\x1b[?u\x1b[c")
    sys.stdout.flush()
    resp = ""
    end = time.time() + 0.5
    while time.time() < end:
        if select.select([fd], [], [], 0.05)[0]:
            resp += os.read(fd, 128).decode(errors="ignore")
            if re.search(r"\x1b\[\?[\d;]*c", resp):
                break
    return re.search(r"\x1b\[\?\d+u", resp) is not None


def kitty_key_name(cp: int, final: str) -> str:
    if final == "A":
        return "up"
    if final == "B":
        return "down"
    if final == "C":
        return "right"
    if final == "D":
        return "left"
    if final == "~":
        return {5: "pgup", 6: "pgdn"}.get(cp, "")
    if final != "u":
        return ""
    if cp in (57441, 57447):
        return "shift"
    if cp in (57442, 57448):
        return "ctrl"
    if cp in (57413, 43, 61):
        return "+"
    if cp in (57414, 45, 95):
        return "-"
    if cp == 27:
        return "q"
    if cp == 32:
        return "space"
    if cp == 114:
        return "pgup"
    if cp == 102:
        return "pgdn"
    ch = chr(cp).lower() if 0 < cp < 0x110000 else ""
    return ch if ch in KITTY_LETTERS else ""


def kitty_reader(stop_evt: threading.Event) -> None:
    fd = sys.stdin.fileno()
    buf = ""
    while not stop_evt.is_set():
        if not select.select([fd], [], [], 0.05)[0]:
            continue
        buf += os.read(fd, 256).decode(errors="ignore")
        while True:
            m = KITTY_RE.search(buf)
            if not m:
                if len(buf) > 64:
                    buf = ""
                break
            buf = buf[m.end():]
            params, final = m.group(1), m.group(2)
            fields = params.split(";")
            try:
                cp = int(fields[0].split(":")[0]) if fields[0] else 1
                mods_field = fields[1] if len(fields) > 1 else "1"
                parts = mods_field.split(":")
                mods = int(parts[0]) if parts[0] else 1
                etype = int(parts[1]) if len(parts) > 1 and parts[1] else 1
            except ValueError:
                continue
            name = kitty_key_name(cp, final)
            if not name:
                continue
            with lock:
                if etype == 3:                          # losgelaten
                    pressed.discard(name)
                    continue
                if name == "space":                     # spatiebalk = alles direct stoppen
                    pressed.clear()
                    continue
                if cp == 99 and (mods - 1) & 4:          # Ctrl+C
                    events.append("q")
                    continue
                if name in HOLD_KEYS or name in ("shift", "ctrl"):
                    pressed.add(name)
                elif name in SINGLE_KEYS and etype == 1:
                    push_event(name)


def calibrate_keyboard(fd: int) -> None:
    """Meet de herhaalvertraging en -snelheid van het toetsenbord (alleen terminalmodus)."""
    import statistics
    import termios
    print("\nKALIBRATIE: houd nu ongeveer 2 seconden de pijl OMHOOG ingedrukt en laat los.")
    print("(Spatiebalk of Enter = overslaan)")
    termios.tcflush(fd, termios.TCIFLUSH)
    stamps = []
    end_wait = time.time() + 20
    while time.time() < end_wait and not stamps:
        if select.select([fd], [], [], 0.1)[0]:
            data = os.read(fd, 256)
            if data in (b" ", b"\r", b"\n"):
                print("Overgeslagen.")
                return
            stamps.append(time.time())
    # blijven lezen tot het toetsenbord ~0,9 s stil is
    last = time.time()
    while time.time() - last < 0.9:
        if select.select([fd], [], [], 0.02)[0]:
            os.read(fd, 256)
            now = time.time()
            stamps.append(now)
            last = now
    termios.tcflush(fd, termios.TCIFLUSH)
    if len(stamps) < 4:
        print("Te weinig toetsherhaling gemeten (houd de toets langer vast). Standaardwaarden gebruikt.")
        return
    delay = stamps[1] - stamps[0]
    gaps = [b - a for a, b in zip(stamps[2:], stamps[3:])]
    interval = statistics.median(gaps) if gaps else 0.03
    if not (0.15 < delay < 1.5):
        print(f"Onverwachte herhaalvertraging ({delay:.2f} s), standaardwaarden gebruikt.")
        return
    FIRST_TIMEOUT[0] = delay + 0.1
    REPEAT_TIMEOUT[0] = min(0.3, max(0.08, interval * 3 + 0.03))
    print(f"Gemeten: herhaalvertraging {delay:.2f} s, herhaalinterval {interval * 1000:.0f} ms "
          f"-> eerste aanslag {FIRST_TIMEOUT[0]:.2f} s, daarna stop {REPEAT_TIMEOUT[0] * 1000:.0f} ms na loslaten.")


def current_held() -> set:
    now = time.time()
    with lock:
        if USE_PYNPUT or KITTY[0]:
            return set(pressed)
        return {k for k, t in active_until.items() if t > now}


# ---------- robot ----------
def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def limited_velocity(v: float, pos: float, lo: float, hi: float) -> float:
    """Beperk snelheid v zodat de arm de grenzen niet overschrijdt (met afremmen)."""
    if v > 0:
        return min(v, max(0.0, hi - pos) * BRAKE_GAIN)
    if v < 0:
        return -min(-v, max(0.0, pos - lo) * BRAKE_GAIN)
    return 0.0


def stop_motion(arm) -> None:
    try:
        arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0], is_radian=False, duration=0)
    except Exception:
        pass


def enter_mode(arm, mode: int) -> None:
    arm.set_mode(mode)
    arm.set_state(0)
    time.sleep(0.2)


def recover(arm) -> None:
    arm.clean_warn()
    arm.clean_error()
    arm.motion_enable(enable=True)
    enter_mode(arm, 0)


def move_to(arm, x: float, y: float, z: float) -> bool:
    """Positiemodus-beweging (wacht tot klaar), met vaste oriëntatie."""
    if arm.mode != 0 or arm.error_code != 0 or arm.state in (4, 5):
        recover(arm)
    code = arm.set_position(x=x, y=y, z=z,
                            roll=ORIENTATION[0], pitch=ORIENTATION[1], yaw=ORIENTATION[2],
                            speed=60.0, mvacc=300.0, wait=True)
    if code != 0 or arm.error_code != 0:
        print(f"Beweging mislukt (code {code}, fout {arm.error_code}, "
              f"waarschuwing {arm.warn_code}).")
        recover(arm)
        return False
    return True


def angle_diff(a: float, b: float) -> float:
    return (a - b + 180.0) % 360.0 - 180.0


def main() -> None:
    global VAK1_X, VAK1_Y
    speed_idx = SPEED_IDX
    rate_idx = GRIPPER_RATE_IDX

    print(f"Verbinden met xArm op {ROBOT_IP}...")
    arm = XArmAPI(ROBOT_IP)
    time.sleep(1.0)
    if not arm.connected:
        print("Geen verbinding. Controleer kabel en IP.")
        sys.exit(1)

    arm.clean_warn()
    arm.clean_error()
    arm.motion_enable(enable=True)
    if WORLD_OFFSET is not None:
        arm.set_world_offset(WORLD_OFFSET)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(1.0)

    arm.set_gripper_enable(True)
    arm.set_gripper_mode(0)
    arm.set_gripper_speed(GRIPPER_SPEED)
    time.sleep(0.5)

    code, gpos = arm.get_gripper_position()
    grip = float(gpos) if code == 0 and gpos is not None else float(GRIPPER_OPEN)
    gdir = 0                       # -1 dicht, 0 stil, +1 open
    last_grip_read = 0.0

    code, pos = arm.get_position()
    if code != 0 or pos is None:
        print("Kan de huidige positie niet lezen.")
        arm.disconnect()
        sys.exit(1)
    x, y, z = pos[0], pos[1], pos[2]
    if USE_CURRENT_ORIENTATION:
        ORIENTATION[:] = pos[3:6]

    print(f"Huidige positie: X={x:.1f} Y={y:.1f} Z={z:.1f}")
    print(f"Oriëntatie: roll={pos[3]:.1f} pitch={pos[4]:.1f} yaw={pos[5]:.1f} "
          f"(doel: {ORIENTATION[0]:.1f}/{ORIENTATION[1]:.1f}/{ORIENTATION[2]:.1f})")
    print(f"Grenzen: X {LIM['x_min']:.1f}..{LIM['x_max']:.1f}, "
          f"Y {LIM['y_min']:.1f}..{LIM['y_max']:.1f}, Z {LIM['z_min']:.1f}..{LIM['z_max']:.1f}")

    binnen = (LIM["x_min"] <= x <= LIM["x_max"] and LIM["y_min"] <= y <= LIM["y_max"]
              and LIM["z_min"] <= z <= LIM["z_max"])
    if not binnen:
        antw = input("De arm staat buiten de bordlimieten. Naar vak 1 op verplaatsingshoogte? (j/n): ")
        if antw.strip().lower() != "j":
            arm.disconnect()
            sys.exit(0)
        x, y, z = VAK1_X, VAK1_Y, Z_START
        if not move_to(arm, x, y, z):
            arm.disconnect()
            sys.exit(1)
    elif (any(abs(angle_diff(pos[3 + i], ORIENTATION[i])) > 2.0 for i in range(3))):
        antw = input("De oriëntatie wijkt af van de standaard. Arm hier rechtzetten? (j/n): ")
        if antw.strip().lower() == "j":
            if not move_to(arm, x, y, z):
                arm.disconnect()
                sys.exit(1)

    print(__doc__)
    if USE_PYNPUT:
        print("Toetsmodus: pynput (Shift = omhoog, Ctrl = omlaag).")
    else:
        if not sys.stdin.isatty():
            print("Geen interactieve terminal. Start met: docker exec -it <container> python3 ...")
            arm.disconnect()
            sys.exit(1)
        print("Toetsmodus: terminal (er wordt gecontroleerd of je terminal echt loslaten kan melden).")

    # Terminal-echo uit (Linux/macOS), of cbreak-modus in terminalmodus
    old_term = None
    stop_reader = threading.Event()
    if os.name != "nt":
        import termios
        import tty
        try:
            fd = sys.stdin.fileno()
            old_term = termios.tcgetattr(fd)
            if USE_PYNPUT:
                new = termios.tcgetattr(fd)
                new[3] &= ~termios.ECHO
                termios.tcsetattr(fd, termios.TCSADRAIN, new)
            else:
                tty.setcbreak(fd)
        except Exception:
            old_term = None

    if not USE_PYNPUT and os.name != "nt" and old_term is not None:
        if detect_kitty(sys.stdin.fileno()):
            sys.stdout.write("\x1b[>11u")             # alle toetsen met indrukken/herhalen/loslaten melden
            sys.stdout.flush()
            KITTY[0] = True
            print("Terminal meldt loslaten: ingedrukt houden = bewegen, loslaten = stop. "
                  "Shift = omhoog, Ctrl = omlaag, spatiebalk = alles stoppen.")
        elif TERMINAL_LATCH:
            print("Deze terminal kan loslaten NIET melden. Latch-modus: één druk laat de arm "
                  "doorlopen, SPATIEBALK stopt. Z: Page Up/r omhoog, Page Down/f omlaag.")
        else:
            print("Deze terminal kan loslaten niet rechtstreeks melden: de toetsherhaling van je "
                  "toetsenbord wordt gebruikt. Z: Page Up/r omhoog, Page Down/f omlaag. "
                  "Spatiebalk stopt alles.")
            calibrate_keyboard(sys.stdin.fileno())

    enter_mode(arm, 5)                       # cartesische snelheidsmodus
    listener = None
    if USE_PYNPUT:
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
    else:
        reader = kitty_reader if KITTY[0] else terminal_reader
        threading.Thread(target=reader, args=(stop_reader,), daemon=True).start()

    running = True
    last_print = 0.0
    vprev = [0.0, 0.0, 0.0]

    try:
        while running:
            t0 = time.time()
            held = current_held()
            with lock:
                todo = events[:]
                events.clear()

            # Losse toetsen
            for ev in todo:
                if ev == "q":
                    running = False
                elif ev == "s":
                    speed_idx = (speed_idx + 1) % len(SPEEDS_MM_S)
                    print(f"\nSnelheid: {SPEEDS_MM_S[speed_idx]:.0f} mm/s")
                elif ev == "g":
                    rate_idx = (rate_idx + 1) % len(GRIPPER_RATES)
                    print(f"\nGrijpersnelheid: {GRIPPER_RATES[rate_idx]} eenheden/s")
                elif ev in ("o", "c"):
                    arm.set_gripper_speed(GRIPPER_SPEED)
                    grip = float(GRIPPER_OPEN if ev == "o" else GRIPPER_CLOSED)
                    arm.set_gripper_position(int(grip), wait=False)
                    gdir = 0
                elif ev == "p":
                    print(f"\nX={x:.1f} Y={y:.1f} Z={z:.1f}")
                elif ev == "k":
                    VAK1_X, VAK1_Y = x, y
                    update_limits()
                    print(f"\nVak 1 = X {x:.1f}, Y {y:.1f}. Grenzen: "
                          f"X {LIM['x_min']:.1f}..{LIM['x_max']:.1f}, "
                          f"Y {LIM['y_min']:.1f}..{LIM['y_max']:.1f}")
                elif ev == "h":
                    stop_motion(arm)
                    enter_mode(arm, 0)
                    move_to(arm, VAK1_X, VAK1_Y, Z_START)
                    enter_mode(arm, 5)
                    vprev = [0.0, 0.0, 0.0]
            if not running:
                break

            # Foutafhandeling
            if arm.error_code != 0 or arm.state in (4, 5):
                print(f"\nArm in foutstatus (fout={arm.error_code}), herstellen...")
                stop_motion(arm)
                recover(arm)
                enter_mode(arm, 5)
                vprev = [0.0, 0.0, 0.0]
                continue

            # Huidige positie
            code, pos = arm.get_position()
            if code == 0 and pos is not None:
                x, y, z = pos[0], pos[1], pos[2]

            # Gewenste snelheid uit de toetsen
            sp = SPEEDS_MM_S[speed_idx]
            vx = vy = vz = 0.0
            if "up" in held:
                vx += DIR_X * sp
            if "down" in held:
                vx -= DIR_X * sp
            if "left" in held:
                vy += DIR_Y * sp
            if "right" in held:
                vy -= DIR_Y * sp
            if "shift" in held or "pgup" in held:
                vz += sp * Z_SPEED_FACTOR
            if "ctrl" in held or "pgdn" in held:
                vz -= sp * Z_SPEED_FACTOR

            # Zacht optrekken/afremmen (geen rukken)
            tv = [vx, vy, vz]
            acc = [ACCEL_MM_S2, ACCEL_MM_S2, ACCEL_MM_S2 * Z_SPEED_FACTOR]
            sm = [vprev[i] + clamp(tv[i] - vprev[i], -acc[i] * LOOP_DT, acc[i] * LOOP_DT)
                  for i in range(3)]

            # Binnen de bordgrenzen houden
            vx = limited_velocity(sm[0], x, LIM["x_min"], LIM["x_max"])
            vy = limited_velocity(sm[1], y, LIM["y_min"], LIM["y_max"])
            vz = limited_velocity(sm[2], z, LIM["z_min"], LIM["z_max"])
            vprev = [vx, vy, vz]

            try:
                arm.vc_set_cartesian_velocity([vx, vy, vz, 0, 0, 0],
                                              is_radian=False, duration=VC_DURATION)
            except Exception as err:
                print(f"\nSnelheidscommando mislukt: {err}")
                break

            # Grijper: soepel doorlopen zolang de toets ingedrukt is, stoppen bij loslaten
            gv = (1 if "+" in held else 0) - (1 if "-" in held else 0)
            now = time.time()
            if gv != gdir:
                if gv != 0:
                    arm.set_gripper_speed(GRIPPER_RATES[rate_idx])
                    arm.set_gripper_position(GRIPPER_OPEN if gv > 0 else GRIPPER_CLOSED,
                                             wait=False)
                else:
                    code, gpos = arm.get_gripper_position()
                    if code == 0 and gpos is not None:
                        grip = float(gpos)
                    arm.set_gripper_position(int(round(grip)), wait=False)   # hier stoppen
                gdir = gv
                last_grip_read = now
            elif gdir != 0 and now - last_grip_read > 0.2:
                code, gpos = arm.get_gripper_position()
                if code == 0 and gpos is not None:
                    grip = float(gpos)
                last_grip_read = now
            target = int(round(grip))

            if now - last_print > 0.25:
                print(f"\rX={x:7.1f} Y={y:7.1f} Z={z:6.1f} | grijper {target:3d} | "
                      f"{sp:.0f} mm/s (Z {sp * Z_SPEED_FACTOR:.1f})   ", end="", flush=True)
                last_print = now

            dt = time.time() - t0
            if dt < LOOP_DT:
                time.sleep(LOOP_DT - dt)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nStoppen...")
        stop_reader.set()
        if KITTY[0]:
            sys.stdout.write("\x1b[<u")               # normale toetsmodus herstellen
            sys.stdout.flush()
        if listener is not None:
            listener.stop()
        stop_motion(arm)
        time.sleep(0.2)
        try:
            arm.set_mode(0)
            arm.set_state(0)
        except Exception:
            pass
        arm.disconnect()
        if old_term is not None:
            import termios
            termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_term)
        elif os.name == "nt":
            import msvcrt
            while msvcrt.kbhit():
                msvcrt.getwch()


if __name__ == "__main__":
    main()
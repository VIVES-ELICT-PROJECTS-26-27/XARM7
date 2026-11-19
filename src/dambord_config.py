"""
Gedeelde instellingen voor dambord_setup.py en dambord_move.py.
Pas hier je bord en schijven aan; beide scripts gebruiken dit bestand.

Alle afmetingen in meters, in het basisframe van de robot (x vooruit, y naar links).
"""

import math

# ------------------------- Robot verbinding -------------------------
ROBOT_IP = "192.168.1.237"
AUTO_START_REALMOVE = True   # True = start zelf MoveIt met de echte robot als die nog niet draait

# ------------------------- Bord -------------------------
BASE_FRAME = "link_base"
BORD_BREEDTE = 0.45     # 45 cm, bord is vierkant
BORD_X_NABIJ = 0.18     # afstand robotbasis -> dichtstbijzijnde rand van het bord
BORD_Y_MIDDEN = 0.0     # y van het midden van het bord
BORD_HOOGTE = 0.032
BORD_DIKTE = 0.02       # dikte van het bord (alleen voor het vervangbord als er geen STL is)
BORDER = 0.0            # rand rond het speelveld (0 = vakjes beslaan het hele bord)
N = 10                  # 10x10 bord, 50 speelbare (donkere) vakjes

# Pas aan als A1 niet op de juiste hoek uitkomt
SPIEGEL_KOLOM = False   # spiegel links/rechts (A..J)
SPIEGEL_RIJ = False     # spiegel onder/boven (1..10)

# ----------------- Namen in de planning scene -----------------
BORD_ID = "dambord"
SCHIJF_PREFIX = "schijf_"          # schijf op startvakje 7 heet "schijf_7"
OUDE_OBJECTEN = ["dambord_volledig.stl_0", "Cilinder.stl_0"]

# ------------------- Bord uit STL (optioneel) -------------------
BORD_STL = "/home/ubuntu/src/dambord_volledig.stl"
BORD_STL_ROTATIE_GRADEN = 0

# ------------------------- Schijven -------------------------
SCHIJF_DIAMETER = 0.036   # 3,6 cm (vakje is 4,5 cm)
SCHIJF_HOOGTE = 0.012     # 1,2 cm
SCHIJVEN_A = list(range(1, 21))    # vakjes 1-20 (rijen 1-4): dichtst bij de robot (wit)
SCHIJVEN_B = list(range(31, 51))   # vakjes 31-50 (rijen 7-10): aan de andere kant (zwart)

KLEUR_BORD = (0.55, 0.40, 0.25, 1.0)
KLEUR_A = (0.95, 0.95, 0.95, 1.0)
KLEUR_B = (0.10, 0.10, 0.10, 1.0)

# Aflegplek rechts naast het bord voor geslagen schijven (binnen veiligheidszone)
AFLEG_POSITIE = (BORD_X_NABIJ + BORD_BREEDTE / 2, BORD_Y_MIDDEN - BORD_BREEDTE / 2 - 0.08)

# ------------------------- Gripper -------------------------
GRIPPER = True          # False = zonder gripper (virtueel vastmaken, zoals eerst)

GRIPPER_LINKS = [
    "xarm_gripper_base_link", "left_outer_knuckle", "left_finger", "left_inner_knuckle",
    "right_outer_knuckle", "right_finger", "right_inner_knuckle",
]
VINGER_LINKS = [
    "left_outer_knuckle", "left_finger", "left_inner_knuckle",
    "right_outer_knuckle", "right_finger", "right_inner_knuckle",
]

if GRIPPER:
    TCP_OFFSET_Z = 0.172        # flens -> gripperpunt (link_tcp), in meters
    EEF_FRAME = "link_tcp"      # het punt dat naar de vakjes beweegt en de schijf vasthoudt
    TOUCH_LINKS = ["link_tcp", "link_eef", "link7", "link6"] + GRIPPER_LINKS
else:
    TCP_OFFSET_Z = 0.0
    EEF_FRAME = "link_eef"
    TOUCH_LINKS = ["link_eef", "link7", "link6"]

# Stand van het gripper-gewricht (drive_joint, 0 tot 0,85 rad).
GRIPPER_OPEN = 0.0
GRIPPER_DICHT = 0.60

# Hoe diep het gripperpunt onder de bovenkant van de schijf komt bij het vastpakken (m)
GREEP_DIEPTE = 0.008

# Veilige klaarstand: gripperpunt hier boven het verhoogde bord (x, y, z in m).
KLAAR_TCP = (0.40, 0.0, 0.275)

# ------------------------- Berekeningen -------------------------
VAK = (BORD_BREEDTE - 2 * BORDER) / N
KOLOM_LETTERS = "ABCDEFGHIJ"


def vakje_naar_rij_kolom(nummer):
    """Zet speelvakje 1..50 om naar (rij 0..9, kolom 0..9). 1 = A1, 50 = J10."""
    idx = nummer - 1
    rij = idx // 5                                  # 0 = rij 1 (dichtst bij robot)
    kolom = 2 * (idx % 5) + (1 if rij % 2 else 0)   # 0 = kolom A (links)
    return rij, kolom


def vakje_naar_naam(nummer):
    """Geeft de dambordnaam (bijv. 'A1', 'B4', 'J10') voor speelvakje 1..50."""
    rij, kolom = vakje_naar_rij_kolom(nummer)
    return f"{KOLOM_LETTERS[kolom]}{rij + 1}"


def naam_naar_vakje(naam):
    """
    Zet een vaknaam zoals 'A1' of 'b4' om naar vakjesnummer 1..50.
    Geeft (nummer, None) bij een geldig donker speelvakje, of (None, foutmelding).
    """
    tekst = naam.strip().upper()
    if len(tekst) < 2 or tekst[0] not in KOLOM_LETTERS:
        return None, f"Ongeldige kolom in '{naam}'. Gebruik letters A t/m J (bijv. B4)."
    try:
        rij_num = int(tekst[1:])
    except ValueError:
        return None, f"Ongeldig rijnummer in '{naam}'. Gebruik cijfers 1 t/m 10 (bijv. B4)."
    if not 1 <= rij_num <= N:
        return None, f"Rij {rij_num} bestaat niet. Gebruik 1 t/m 10."

    kolom = KOLOM_LETTERS.index(tekst[0])
    rij = rij_num - 1

    if (rij + kolom) % 2 != 0:
        return None, f"{tekst} is een licht vakje; op een dambord speel je alleen op donkere vakjes."

    nummer = rij * 5 + (kolom // 2) + 1
    return nummer, None


def vakje_positie(nummer):
    """Midden (x, y) van speelvakje 1..50. 1 = A1 (links onder), 50 = J10 (rechts boven)."""
    rij, kolom = vakje_naar_rij_kolom(nummer)

    if SPIEGEL_KOLOM:
        kolom = N - 1 - kolom
    if SPIEGEL_RIJ:
        rij = N - 1 - rij

    x = BORD_X_NABIJ + BORDER + (rij + 0.5) * VAK
    y = BORD_Y_MIDDEN + BORD_BREEDTE / 2 - BORDER - (kolom + 0.5) * VAK
    return x, y


def dichtstbij(x, y):
    """Geeft (vakjesnummer, afstand in m) van het vakje het dichtst bij (x, y)."""
    beste, beste_afstand = None, 1e9
    for nummer in range(1, 51):
        vx, vy = vakje_positie(nummer)
        afstand = math.hypot(vx - x, vy - y)
        if afstand < beste_afstand:
            beste, beste_afstand = nummer, afstand
    return beste, beste_afstand
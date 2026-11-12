"""
Gedeelde instellingen voor dambord_setup.py en dambord_move.py.
Pas hier je bord en schijven aan; beide scripts gebruiken dit bestand.

Alle afmetingen in meters, in het basisframe van de robot (x vooruit, y naar links).
"""

import math

# ------------------------- Bord -------------------------
BASE_FRAME = "link_base"
BORD_BREEDTE = 0.45     # 45 cm, bord is vierkant
BORD_X_NABIJ = 0.18     # afstand robotbasis -> dichtstbijzijnde rand van het bord
BORD_Y_MIDDEN = 0.0     # y van het midden van het bord
BORD_HOOGTE = 0.02      # z van de BOVENKANT van het bord (meet dit na)
BORD_DIKTE = 0.02       # dikte van het bord (alleen voor het vervangbord als er geen STL is)
BORDER = 0.0            # rand rond het speelveld (0 = vakjes beslaan het hele bord)
N = 10                  # 10x10 bord, 50 speelbare (donkere) vakjes

# Pas aan als vakje 1 niet op de juiste hoek uitkomt
SPIEGEL_KOLOM = False   # spiegel links/rechts
SPIEGEL_RIJ = False     # spiegel onder/boven

# ----------------- Namen in de planning scene -----------------
BORD_ID = "dambord"
SCHIJF_PREFIX = "schijf_"          # schijf op startvakje 7 heet "schijf_7"
# Objecten die je eerder met de hand in RViz toevoegde; worden eerst verwijderd
OUDE_OBJECTEN = ["dambord_volledig.stl_0", "Cilinder.stl_0"]

# ------------------- Bord uit STL (optioneel) -------------------
# Bestaat dit bestand niet, dan maakt het setup-script een eenvoudig vlak bord.
BORD_STL = "/home/ubuntu/src/dambord_volledig.stl"
# Draai het STL kwartslagen (0, 90, 180, 270) als het patroon niet klopt met de nummering
BORD_STL_ROTATIE_GRADEN = 0

# ------------------------- Schijven -------------------------
SCHIJF_DIAMETER = 0.036   # 3,6 cm (vakje is 4,5 cm)
SCHIJF_HOOGTE = 0.012     # 1,2 cm
SCHIJVEN_A = list(range(1, 21))    # vakjes 1-20: dichtst bij de robot (wit)
SCHIJVEN_B = list(range(31, 51))   # vakjes 31-50: aan de andere kant (zwart)

KLEUR_BORD = (0.55, 0.40, 0.25, 1.0)
KLEUR_A = (0.95, 0.95, 0.95, 1.0)
KLEUR_B = (0.10, 0.10, 0.10, 1.0)

# Delen van de arm die een schijf mogen aanraken bij het oppakken
# Heb je een grijper? Voeg dan ook zijn linknamen toe.
TOUCH_LINKS = ["link_eef", "link7", "link6"]

# ------------------------- Berekeningen -------------------------
VAK = (BORD_BREEDTE - 2 * BORDER) / N


def vakje_positie(nummer):
    """Midden (x, y) van speelvakje 1..50. 1 = links onder, 50 = rechts boven."""
    idx = nummer - 1
    rij = idx // 5                                  # 0 = rij het dichtst bij de robot
    kolom = 2 * (idx % 5) + (1 if rij % 2 else 0)   # 0 = meest linkse kolom

    if SPIEGEL_KOLOM:
        kolom = N - 1 - kolom
    if SPIEGEL_RIJ:
        rij = N - 1 - rij

    # rij loopt van de robot af (x neemt toe), kolom van links naar rechts (y neemt af)
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

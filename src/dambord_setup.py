#!/usr/bin/env python3
"""
Zet het dambord en de damschijven in de MoveIt planning scene.
Draai dit script elke keer nadat je het xArm MoveIt-systeem hebt gestart.

  - Verwijdert eerst oude objecten (eerder toegevoegd bord/schijven), dus je mag het
    script vaak achter elkaar draaien.
  - Plaatst het bord (uit je STL, of anders een eenvoudig vlak bord) op de juiste plek.
  - Zet 40 schijven klaar: wit op vakjes 1-20, zwart op vakjes 31-50.
  - Staat de schijven en het bord toe elkaar te raken (nodig voor oppakken/neerzetten).

Gebruik:
    python3 dambord_setup.py                  # bord + schijven
    python3 dambord_setup.py --geen-schijven  # alleen het bord

Alle instellingen staan in dambord_config.py.
"""

import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    ObjectColor,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from std_msgs.msg import ColorRGBA

import dambord_config as cfg


def maak_pose(x, y, z):
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation.w = 1.0   # let op: standaard is w = 0, dat is geen geldige oriëntatie
    return pose


def maak_kleur(rgba):
    k = ColorRGBA()
    k.r, k.g, k.b, k.a = (float(v) for v in rgba)
    return k


class SceneClient:
    def __init__(self, node):
        self.node = node
        self.get = node.create_client(GetPlanningScene, "/get_planning_scene")
        self.apply = node.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def wacht(self, seconden=60.0):
        log = self.node.get_logger()
        eind = time.time() + seconden
        for client, naam in ((self.get, "/get_planning_scene"),
                             (self.apply, "/apply_planning_scene")):
            while not client.wait_for_service(timeout_sec=1.0):
                if time.time() > eind:
                    log.error(
                        f"{naam} niet gevonden. Draait het MoveIt-systeem? "
                        "Controleer met: ros2 service list | grep planning_scene"
                    )
                    return False
        return True

    def _wacht(self, future, naam):
        """Wacht op een serviceantwoord en meldt duidelijk wat er misgaat."""
        log = self.node.get_logger()
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=20.0)
        if not future.done():
            log.error(f"{naam}: geen antwoord binnen 20 seconden.")
            return None
        if future.exception() is not None:
            log.error(f"{naam}: serviceaanroep mislukt: {future.exception()}")
            return None
        return future.result()

    def lees(self, onderdelen):
        req = GetPlanningScene.Request()
        req.components.components = onderdelen
        result = self._wacht(self.get.call_async(req), "/get_planning_scene")
        return result.scene if result is not None else None

    def pas_toe(self, scene, naam="scene"):
        scene.is_diff = True
        scene.robot_state.is_diff = True   # anders worden vastgemaakte objecten gewist
        req = ApplyPlanningScene.Request()
        req.scene = scene
        result = self._wacht(self.apply.call_async(req), f"/apply_planning_scene ({naam})")
        if result is None:
            return False
        if not result.success:
            self.node.get_logger().error(
                f"/apply_planning_scene ({naam}) gaf success=False. Kijk in het "
                "terminalvenster van MoveIt (move_group) voor de reden."
            )
            return False
        return True


def verwijder_object(object_id):
    obj = CollisionObject()
    obj.header.frame_id = cfg.BASE_FRAME
    obj.id = object_id
    obj.operation = CollisionObject.REMOVE
    return obj


def lees_stl(pad):
    """Leest een binaire of ASCII STL zonder extra pakketten. Geeft (hoekpunten, driehoeken)."""
    with open(pad, "rb") as f:
        data = f.read()

    driehoeken = None
    if len(data) >= 84:
        aantal = int(np.frombuffer(data[80:84], dtype="<u4")[0])
        if len(data) == 84 + 50 * aantal:   # binaire STL
            dt = np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("a", "<u2")])
            driehoeken = np.frombuffer(data, dtype=dt, count=aantal, offset=84)["v"].astype(float)
    if driehoeken is None:                   # ASCII STL
        tekst = data.decode("ascii", errors="ignore").split()
        punten = [
            [float(tekst[i + 1]), float(tekst[i + 2]), float(tekst[i + 3])]
            for i, t in enumerate(tekst) if t == "vertex"
        ]
        if not punten or len(punten) % 3:
            raise ValueError("geen geldig STL-bestand")
        driehoeken = np.array(punten).reshape(-1, 3, 3)

    uniek, inverse = np.unique(np.round(driehoeken.reshape(-1, 3), 6),
                               axis=0, return_inverse=True)
    return uniek, np.asarray(inverse).reshape(-1, 3)


def maak_bord():
    """Geeft (CollisionObject, beschrijving) van het bord."""
    obj = CollisionObject()
    obj.header.frame_id = cfg.BASE_FRAME
    obj.id = cfg.BORD_ID
    obj.pose = maak_pose(0, 0, 0)
    obj.operation = CollisionObject.ADD

    x0 = cfg.BORD_X_NABIJ
    y0 = cfg.BORD_Y_MIDDEN - cfg.BORD_BREEDTE / 2

    if os.path.exists(cfg.BORD_STL):
        try:
            v, faces = lees_stl(cfg.BORD_STL)

            hoek = math.radians(cfg.BORD_STL_ROTATIE_GRADEN)
            c, s = math.cos(hoek), math.sin(hoek)
            v = np.column_stack([v[:, 0] * c - v[:, 1] * s,
                                 v[:, 0] * s + v[:, 1] * c,
                                 v[:, 2]])

            mn, mx = v.min(axis=0), v.max(axis=0)
            ex, ey = (mx - mn)[:2]
            schaal = cfg.BORD_BREEDTE / max(ex, ey)   # STL schalen naar 45 cm
            v = (v - mn) * schaal
            v[:, 0] += x0
            v[:, 1] += y0
            v[:, 2] += cfg.BORD_HOOGTE - v[:, 2].max()   # bovenkant op BORD_HOOGTE

            mesh = Mesh()
            mesh.vertices = [Point(x=float(a), y=float(b), z=float(d)) for a, b, d in v]
            mesh.triangles = []
            for a, b, d in faces:
                tri = MeshTriangle()
                tri.vertex_indices = [int(a), int(b), int(d)]
                mesh.triangles.append(tri)
            obj.meshes = [mesh]
            obj.mesh_poses = [maak_pose(0, 0, 0)]

            dikte = (mx - mn)[2] * schaal
            beschrijving = (
                f"bord uit {cfg.BORD_STL} (schaal {schaal:.4f}, "
                f"{ex * schaal * 100:.1f} x {ey * schaal * 100:.1f} cm, "
                f"dikte {dikte * 100:.1f} cm)"
            )
            return obj, beschrijving
        except Exception as e:
            print(f"STL laden mislukt ({e}): ik gebruik een eenvoudig vlak bord. LET OP: dit is NIET je STL.")
    else:
        print(f"STL niet gevonden op {cfg.BORD_STL}: ik gebruik een eenvoudig vlak bord.")

    doos = SolidPrimitive()
    doos.type = SolidPrimitive.BOX
    doos.dimensions = [cfg.BORD_BREEDTE, cfg.BORD_BREEDTE, cfg.BORD_DIKTE]
    obj.primitives = [doos]
    obj.primitive_poses = [maak_pose(
        x0 + cfg.BORD_BREEDTE / 2,
        cfg.BORD_Y_MIDDEN,
        cfg.BORD_HOOGTE - cfg.BORD_DIKTE / 2,
    )]
    return obj, "eenvoudig vlak bord (geen STL)"


def maak_schijf(nummer):
    obj = CollisionObject()
    obj.header.frame_id = cfg.BASE_FRAME
    obj.id = f"{cfg.SCHIJF_PREFIX}{nummer}"
    obj.pose = maak_pose(0, 0, 0)
    obj.operation = CollisionObject.ADD

    cilinder = SolidPrimitive()
    cilinder.type = SolidPrimitive.CYLINDER
    cilinder.dimensions = [cfg.SCHIJF_HOOGTE, cfg.SCHIJF_DIAMETER / 2]   # [hoogte, straal]
    obj.primitives = [cilinder]

    x, y = cfg.vakje_positie(nummer)
    obj.primitive_poses = [maak_pose(x, y, cfg.BORD_HOOGTE + cfg.SCHIJF_HOOGTE / 2)]
    return obj


def acm_toestaan(acm, a, b):
    """Staat botsingen toe tussen a en b in de Allowed Collision Matrix."""
    namen = list(acm.entry_names)
    rijen = [list(r.enabled) for r in acm.entry_values]
    for n in (a, b):
        if n not in namen:
            namen.append(n)
            for r in rijen:
                r.append(False)
            rijen.append([False] * len(namen))
    i, j = namen.index(a), namen.index(b)
    rijen[i][j] = True
    rijen[j][i] = True
    acm.entry_names = namen
    acm.entry_values = []
    for r in rijen:
        e = AllowedCollisionEntry()
        e.enabled = r
        acm.entry_values.append(e)


def main():
    geen_schijven = "--geen-schijven" in sys.argv

    rclpy.init()
    node = rclpy.create_node("dambord_setup")
    log = node.get_logger()
    client = SceneClient(node)

    try:
        if not client.wacht():
            return

        # 1. Opruimen: eerst vastgemaakte schijven loskoppelen, daarna alles verwijderen
        scene = client.lees(
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            | PlanningSceneComponents.WORLD_OBJECT_NAMES
        )
        if scene is None:
            log.error("Kon de planning scene niet lezen. Zie de melding hierboven.")
            return

        los = PlanningScene()
        for a in scene.robot_state.attached_collision_objects:
            if a.object.id.startswith(cfg.SCHIJF_PREFIX):
                aco = AttachedCollisionObject()
                aco.link_name = a.link_name
                aco.object.id = a.object.id
                aco.object.operation = CollisionObject.REMOVE
                los.robot_state.attached_collision_objects.append(aco)
        if los.robot_state.attached_collision_objects:
            if not client.pas_toe(los, "schijf loskoppelen"):
                log.warn("Loskoppelen van een vastgemaakte schijf mislukt; ik ga door.")

        # Alleen objecten verwijderen die er echt zijn. Objecten met dezelfde naam als
        # de nieuwe worden bij het toevoegen toch overschreven, dus mislukken is niet erg.
        ids = [cfg.BORD_ID] + cfg.OUDE_OBJECTEN + [
            f"{cfg.SCHIJF_PREFIX}{n}" for n in range(1, 51)
        ]
        bestaand = {o.id for o in scene.world.collision_objects}
        te_verwijderen = [i for i in ids if i in bestaand]
        if te_verwijderen:
            opruimen = PlanningScene()
            opruimen.world.collision_objects = [verwijder_object(i) for i in te_verwijderen]
            if not client.pas_toe(opruimen, "opruimen"):
                log.warn(
                    "Opruimen van de oude objecten mislukt; ik ga door. "
                    "Oude objecten met een andere naam kunnen in RViz blijven staan "
                    "(verwijder die dan met de hand)."
                )
            else:
                log.info(f"{len(te_verwijderen)} oude objecten verwijderd.")

        # 2. Bord en schijven toevoegen
        bord, beschrijving = maak_bord()
        schijven = [] if geen_schijven else [
            maak_schijf(n) for n in cfg.SCHIJVEN_A + cfg.SCHIJVEN_B
        ]

        nieuw = PlanningScene()
        nieuw.world.collision_objects = [bord] + schijven
        kleuren = {bord.id: cfg.KLEUR_BORD}
        for n in cfg.SCHIJVEN_A:
            kleuren[f"{cfg.SCHIJF_PREFIX}{n}"] = cfg.KLEUR_A
        for n in cfg.SCHIJVEN_B:
            kleuren[f"{cfg.SCHIJF_PREFIX}{n}"] = cfg.KLEUR_B
        for object_id, rgba in kleuren.items():
            if object_id == bord.id or not geen_schijven:
                kleur = ObjectColor()
                kleur.id = object_id
                kleur.color = maak_kleur(rgba)
                nieuw.object_colors.append(kleur)
        if not client.pas_toe(nieuw, "bord en schijven toevoegen"):
            log.error("Bord en schijven toevoegen mislukt.")
            return

        # 3. Botsingen schijf <-> bord en schijf <-> eindpunt van de arm toestaan
        if schijven:
            scene = client.lees(PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
            if scene is None:
                log.warn("Kon de collision matrix niet lezen; oppakken kan botsingen melden.")
            else:
                acm = scene.allowed_collision_matrix
                for s in schijven:
                    acm_toestaan(acm, s.id, cfg.BORD_ID)
                    for link in cfg.TOUCH_LINKS:
                        acm_toestaan(acm, s.id, link)
                diff = PlanningScene()
                diff.allowed_collision_matrix = acm
                if not client.pas_toe(diff, "collision matrix"):
                    log.warn("Collision matrix aanpassen mislukt.")

        # 4. Controle
        scene = client.lees(
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        if scene is None:
            log.warn("Kon de scene niet teruglezen.")
            return
        namen = [o.id for o in scene.world.collision_objects]
        n_schijven = sum(1 for n in namen if n.startswith(cfg.SCHIJF_PREFIX))
        log.info(f"Klaar: {beschrijving}")
        log.info(
            f"Bord in de scene: {'ja' if cfg.BORD_ID in namen else 'NEE'}, "
            f"schijven in de scene: {n_schijven}"
        )
        log.info(
            f"Bord: x {cfg.BORD_X_NABIJ:.2f} tot {cfg.BORD_X_NABIJ + cfg.BORD_BREEDTE:.2f} m, "
            f"y {cfg.BORD_Y_MIDDEN - cfg.BORD_BREEDTE / 2:.3f} tot "
            f"{cfg.BORD_Y_MIDDEN + cfg.BORD_BREEDTE / 2:.3f} m, "
            f"bovenkant z = {cfg.BORD_HOOGTE:.2f} m"
        )
        if not geen_schijven:
            log.info(
                f"Schijven: wit op vakjes {cfg.SCHIJVEN_A[0]}-{cfg.SCHIJVEN_A[-1]}, "
                f"zwart op vakjes {cfg.SCHIJVEN_B[0]}-{cfg.SCHIJVEN_B[-1]}"
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

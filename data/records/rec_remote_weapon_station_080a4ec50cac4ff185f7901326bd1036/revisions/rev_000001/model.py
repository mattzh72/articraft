from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _armored_housing_mesh() -> MeshGeometry:
    """Faceted truncated armored hull, authored in the turret frame."""

    geom = MeshGeometry()
    # Bottom rectangle, just above the rotating neck.
    verts = [
        (-0.32, -0.29, 0.20),
        (0.42, -0.29, 0.20),
        (0.42, 0.29, 0.20),
        (-0.32, 0.29, 0.20),
        # Top rectangle is slightly smaller to create sloped armor sides.
        (-0.25, -0.22, 0.62),
        (0.36, -0.22, 0.62),
        (0.39, 0.22, 0.62),
        (-0.25, 0.22, 0.62),
    ]
    for v in verts:
        geom.add_vertex(*v)

    def quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    quad(0, 1, 2, 3)  # bottom
    quad(4, 7, 6, 5)  # top
    quad(0, 4, 5, 1)  # -Y side
    quad(1, 5, 6, 2)  # front glacis
    quad(2, 6, 7, 3)  # +Y side
    quad(3, 7, 4, 0)  # rear plate
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="remote_weapon_station")

    olive = Material("mat_olive_drab", rgba=(0.22, 0.27, 0.18, 1.0))
    dark_olive = Material("mat_dark_olive", rgba=(0.12, 0.15, 0.10, 1.0))
    black = Material("mat_parkerized_black", rgba=(0.015, 0.017, 0.015, 1.0))
    gunmetal = Material("mat_gunmetal", rgba=(0.08, 0.085, 0.08, 1.0))
    worn_edge = Material("mat_worn_edges", rgba=(0.36, 0.38, 0.31, 1.0))
    glass = Material("mat_coated_glass", rgba=(0.02, 0.07, 0.09, 1.0))
    rubber = Material("mat_rubber", rgba=(0.025, 0.025, 0.022, 1.0))

    # Root: rigid deck plate, pedestal, and fixed lower bearing.
    base = model.part("base")
    base.visual(
        Box((1.15, 0.92, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_olive,
        name="deck_plate",
    )
    base.visual(
        Cylinder(radius=0.27, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=olive,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.39, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        material=gunmetal,
        name="lower_race",
    )
    # Cross-shaped welded webbing from the deck up to the pedestal.
    base.visual(Box((0.52, 0.045, 0.22)), origin=Origin(xyz=(0.0, 0.0, 0.19)), material=olive, name="web_x")
    base.visual(Box((0.045, 0.52, 0.22)), origin=Origin(xyz=(0.0, 0.0, 0.19)), material=olive, name="web_y")
    # Corner mounting lugs and bolts.
    for i, (x, y) in enumerate(((-0.46, -0.35), (-0.46, 0.35), (0.46, -0.35), (0.46, 0.35))):
        base.visual(
            Box((0.18, 0.12, 0.028)),
            origin=Origin(xyz=(x, y, 0.094)),
            material=olive,
            name=f"mount_lug_{i}",
        )
        base.visual(
            Cylinder(radius=0.032, length=0.020),
            origin=Origin(xyz=(x, y, 0.118)),
            material=worn_edge,
            name=f"anchor_bolt_{i}",
        )
    # Ring bolts around the slewing bearing.
    for i in range(12):
        a = 2.0 * math.pi * i / 12.0
        base.visual(
            Cylinder(radius=0.018, length=0.016),
            origin=Origin(xyz=(0.315 * math.cos(a), 0.315 * math.sin(a), 0.408)),
            material=worn_edge,
            name=f"race_bolt_{i}",
        )
    # Armored cable conduit clamped to the deck.
    base.visual(
        Cylinder(radius=0.026, length=0.36),
        origin=Origin(xyz=(-0.38, 0.31, 0.115), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="cable_conduit",
    )
    base.visual(
        Box((0.10, 0.055, 0.050)),
        origin=Origin(xyz=(-0.38, 0.15, 0.095)),
        material=olive,
        name="conduit_clamp",
    )

    # Panning assembly: rotating race, armored central housing, yoke cheeks,
    # sensor pod, and external appliqué armor.
    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.285, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=gunmetal,
        name="upper_race",
    )
    turret.visual(
        Cylinder(radius=0.31, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_olive,
        name="rotating_skirt",
    )
    turret.visual(
        Box((0.42, 0.38, 0.16)),
        origin=Origin(xyz=(0.02, 0.0, 0.16)),
        material=olive,
        name="neck_block",
    )
    turret.visual(
        mesh_from_geometry(_armored_housing_mesh(), "armored_housing"),
        material=olive,
        name="armored_housing",
    )
    turret.visual(
        Box((0.58, 0.42, 0.046)),
        origin=Origin(xyz=(0.05, 0.0, 0.643)),
        material=dark_olive,
        name="top_armor",
    )
    turret.visual(
        Box((0.050, 0.44, 0.30)),
        origin=Origin(xyz=(0.418, 0.0, 0.43), rpy=(0.0, -0.17, 0.0)),
        material=dark_olive,
        name="front_armor",
    )
    for i, y in enumerate((-0.303, 0.303)):
        turret.visual(
            Box((0.48, 0.060, 0.22)),
            origin=Origin(xyz=(0.04, -0.293 if y < 0 else 0.293, 0.43)),
            material=dark_olive,
            name=f"side_armor_{i}",
        )
        turret.visual(
            Box((0.28, 0.14, 0.12)),
            origin=Origin(xyz=(0.44, y, 0.40)),
            material=olive,
            name=f"lower_yoke_bridge_{i}",
        )
        turret.visual(
            Box((0.24, 0.13, 0.10)),
            origin=Origin(xyz=(0.46, y, 0.745)),
            material=olive,
            name=f"upper_yoke_bridge_{i}",
        )
    turret.visual(
        Box((0.24, 0.10, 0.40)),
        origin=Origin(xyz=(0.52, -0.40, 0.59)),
        material=dark_olive,
        name="cheek_0",
    )
    turret.visual(
        Cylinder(radius=0.112, length=0.025),
        origin=Origin(xyz=(0.52, -0.348, 0.59), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="bearing_face_0",
    )
    turret.visual(
        Box((0.24, 0.10, 0.40)),
        origin=Origin(xyz=(0.52, 0.40, 0.59)),
        material=dark_olive,
        name="cheek_1",
    )
    turret.visual(
        Cylinder(radius=0.112, length=0.025),
        origin=Origin(xyz=(0.52, 0.348, 0.59), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="bearing_face_1",
    )
    # Electro-optical sighting pod on an armored bracket, fixed to the panning head.
    turret.visual(
        Box((0.10, 0.16, 0.16)),
        origin=Origin(xyz=(0.08, -0.365, 0.55)),
        material=olive,
        name="sensor_bracket",
    )
    turret.visual(
        Box((0.26, 0.16, 0.18)),
        origin=Origin(xyz=(0.11, -0.52, 0.58)),
        material=dark_olive,
        name="sensor_pod",
    )
    turret.visual(
        Cylinder(radius=0.045, length=0.025),
        origin=Origin(xyz=(0.235, -0.55, 0.605), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="day_lens",
    )
    turret.visual(
        Cylinder(radius=0.030, length=0.025),
        origin=Origin(xyz=(0.235, -0.485, 0.555), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="thermal_lens",
    )
    turret.visual(
        Box((0.09, 0.12, 0.035)),
        origin=Origin(xyz=(-0.33, 0.0, 0.35)),
        material=black,
        name="rear_connector",
    )

    model.articulation(
        "pan",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=850.0, velocity=1.2),
        motion_properties=MotionProperties(damping=8.0, friction=1.2),
    )

    # Elevating weapon cradle: gun, trunnion journals, feed box, recoil rods,
    # and muzzle brake are a single rigid elevating group.
    cradle = model.part("cradle")
    cradle.visual(
        Box((0.40, 0.28, 0.16)),
        origin=Origin(xyz=(0.20, 0.0, 0.0)),
        material=gunmetal,
        name="receiver",
    )
    cradle.visual(
        Box((0.46, 0.050, 0.22)),
        origin=Origin(xyz=(0.20, -0.155, 0.0)),
        material=dark_olive,
        name="side_plate_0",
    )
    cradle.visual(
        Box((0.46, 0.050, 0.22)),
        origin=Origin(xyz=(0.20, 0.155, 0.0)),
        material=dark_olive,
        name="side_plate_1",
    )
    cradle.visual(
        Cylinder(radius=0.085, length=0.17),
        origin=Origin(xyz=(0.0, -0.265, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="trunnion_0",
    )
    cradle.visual(
        Cylinder(radius=0.085, length=0.17),
        origin=Origin(xyz=(0.0, 0.265, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="trunnion_1",
    )
    cradle.visual(
        Box((0.30, 0.18, 0.060)),
        origin=Origin(xyz=(0.21, 0.0, 0.105)),
        material=black,
        name="top_cover",
    )
    cradle.visual(
        Cylinder(radius=0.064, length=0.42),
        origin=Origin(xyz=(0.55, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="heat_shroud",
    )
    cradle.visual(
        Cylinder(radius=0.035, length=0.98),
        origin=Origin(xyz=(0.83, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="barrel",
    )
    for i, y in enumerate((-0.075, 0.075)):
        cradle.visual(
            Cylinder(radius=0.016, length=0.58),
            origin=Origin(xyz=(0.58, y, 0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=worn_edge,
            name=f"recoil_rod_{i}",
        )
    cradle.visual(
        Box((0.22, 0.035, 0.035)),
        origin=Origin(xyz=(0.47, 0.0, 0.115)),
        material=dark_olive,
        name="rod_clamp",
    )
    cradle.visual(
        Cylinder(radius=0.055, length=0.13),
        origin=Origin(xyz=(1.38, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="muzzle_brake",
    )
    cradle.visual(
        Box((0.065, 0.17, 0.032)),
        origin=Origin(xyz=(1.38, 0.0, 0.045)),
        material=black,
        name="brake_port_top",
    )
    cradle.visual(
        Box((0.065, 0.17, 0.032)),
        origin=Origin(xyz=(1.38, 0.0, -0.045)),
        material=black,
        name="brake_port_bottom",
    )
    cradle.visual(
        Box((0.24, 0.09, 0.20)),
        origin=Origin(xyz=(0.21, 0.550, 0.160)),
        material=olive,
        name="ammo_box",
    )
    cradle.visual(
        Box((0.20, 0.16, 0.090)),
        origin=Origin(xyz=(0.21, 0.200, 0.045)),
        material=dark_olive,
        name="feed_chute",
    )
    cradle.visual(
        Box((0.16, 0.09, 0.24)),
        origin=Origin(xyz=(0.21, 0.270, 0.165)),
        material=dark_olive,
        name="feed_riser",
    )
    cradle.visual(
        Box((0.18, 0.34, 0.050)),
        origin=Origin(xyz=(0.21, 0.400, 0.280)),
        material=dark_olive,
        name="feed_bridge",
    )
    cradle.visual(
        Box((0.56, 0.040, 0.040)),
        origin=Origin(xyz=(0.45, 0.0, -0.100)),
        material=dark_olive,
        name="under_rail",
    )

    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=cradle,
        origin=Origin(xyz=(0.52, 0.0, 0.59)),
        # The gun points along local +X at q=0; -Y makes positive q elevate it.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=520.0, velocity=0.9, lower=-0.25, upper=1.05),
        motion_properties=MotionProperties(damping=4.0, friction=0.8),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turret = object_model.get_part("turret")
    cradle = object_model.get_part("cradle")
    pan = object_model.get_articulation("pan")
    elevation = object_model.get_articulation("elevation")

    ctx.allow_overlap(
        cradle,
        turret,
        elem_a="trunnion_0",
        elem_b="bearing_face_0",
        reason="The elevating trunnion is intentionally captured inside the armored yoke bearing disk.",
    )
    ctx.allow_overlap(
        cradle,
        turret,
        elem_a="trunnion_1",
        elem_b="bearing_face_1",
        reason="The elevating trunnion is intentionally captured inside the armored yoke bearing disk.",
    )
    ctx.expect_gap(
        turret,
        base,
        axis="z",
        positive_elem="upper_race",
        negative_elem="lower_race",
        max_gap=0.002,
        max_penetration=0.0,
        name="rotating bearing sits on fixed race",
    )
    ctx.expect_contact(
        cradle,
        turret,
        elem_a="trunnion_0",
        elem_b="cheek_0",
        contact_tol=0.004,
        name="one trunnion is captured by the yoke cheek",
    )
    ctx.expect_contact(
        cradle,
        turret,
        elem_a="trunnion_1",
        elem_b="cheek_1",
        contact_tol=0.004,
        name="opposite trunnion is captured by the yoke cheek",
    )
    ctx.expect_within(
        cradle,
        turret,
        axes="xz",
        inner_elem="trunnion_0",
        outer_elem="bearing_face_0",
        margin=0.001,
        name="first trunnion is centered in its bearing disk",
    )
    ctx.expect_overlap(
        cradle,
        turret,
        axes="y",
        elem_a="trunnion_0",
        elem_b="bearing_face_0",
        min_overlap=0.010,
        name="first trunnion remains inserted in its bearing disk",
    )
    ctx.expect_within(
        cradle,
        turret,
        axes="xz",
        inner_elem="trunnion_1",
        outer_elem="bearing_face_1",
        margin=0.001,
        name="second trunnion is centered in its bearing disk",
    )
    ctx.expect_overlap(
        cradle,
        turret,
        axes="y",
        elem_a="trunnion_1",
        elem_b="bearing_face_1",
        min_overlap=0.010,
        name="second trunnion remains inserted in its bearing disk",
    )

    rest_cradle = ctx.part_world_position(cradle)
    with ctx.pose({pan: 0.85}):
        panned_cradle = ctx.part_world_position(cradle)
    ctx.check(
        "pan rotates the elevated assembly about the vertical axis",
        rest_cradle is not None
        and panned_cradle is not None
        and abs(panned_cradle[1] - rest_cradle[1]) > 0.20,
        details=f"rest={rest_cradle}, panned={panned_cradle}",
    )

    rest_barrel = ctx.part_element_world_aabb(cradle, elem="muzzle_brake")
    with ctx.pose({elevation: 0.75}):
        raised_barrel = ctx.part_element_world_aabb(cradle, elem="muzzle_brake")
    ctx.check(
        "positive elevation raises the muzzle",
        rest_barrel is not None
        and raised_barrel is not None
        and raised_barrel[1][2] > rest_barrel[1][2] + 0.35,
        details=f"rest={rest_barrel}, raised={raised_barrel}",
    )

    return ctx.report()


object_model = build_object_model()

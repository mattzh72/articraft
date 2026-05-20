from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


TRAVERSE_UPPER = 0.55
RECOIL_TRAVEL = 0.32
ELEVATION_UPPER = 0.32


def _barrel_body_shape() -> cq.Workplane:
    """Stubby cast-iron carronade tube, revolved about its local +X bore axis."""
    outer = (
        cq.Workplane("XZ")
        .moveTo(-0.22, 0.0)
        .lineTo(-0.22, 0.145)
        .lineTo(-0.14, 0.175)
        .lineTo(0.05, 0.168)
        .lineTo(0.18, 0.145)
        .lineTo(0.56, 0.118)
        .lineTo(0.70, 0.122)
        .lineTo(0.74, 0.168)
        .lineTo(0.84, 0.170)
        .lineTo(0.86, 0.145)
        .lineTo(0.86, 0.0)
        .close()
        .revolve(360.0, axisStart=(-0.30, 0.0, 0.0), axisEnd=(1.0, 0.0, 0.0))
    )

    bore = (
        cq.Workplane("YZ")
        .circle(0.064)
        .extrude(0.96)
        .translate((-0.03, 0.0, 0.0))
    )
    return outer.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="naval_carronade")

    model.material("dark_cast_iron", rgba=(0.015, 0.016, 0.015, 1.0))
    model.material("worn_iron", rgba=(0.08, 0.075, 0.065, 1.0))
    model.material("oiled_oak", rgba=(0.50, 0.31, 0.15, 1.0))
    model.material("end_grain", rgba=(0.34, 0.19, 0.09, 1.0))
    model.material("deck_oak", rgba=(0.58, 0.40, 0.22, 1.0))
    model.material("dark_groove", rgba=(0.045, 0.030, 0.018, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((2.45, 1.35, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="deck_oak",
        name="deck_planks",
    )
    for index, y in enumerate((-0.45, -0.225, 0.0, 0.225, 0.45)):
        deck.visual(
            Box((2.46, 0.010, 0.006)),
            origin=Origin(xyz=(0.0, y, 0.043)),
            material="dark_groove",
            name=f"plank_seam_{index}",
        )
    deck.visual(
        Cylinder(radius=0.30, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material="worn_iron",
        name="traverse_wear_plate",
    )

    slide_base = model.part("slide_base")
    slide_base.visual(
        Cylinder(radius=0.23, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material="worn_iron",
        name="pivot_plate",
    )
    slide_base.visual(
        Cylinder(radius=0.045, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.0875)),
        material="worn_iron",
        name="pivot_pin",
    )
    slide_base.visual(
        Box((1.55, 0.36, 0.10)),
        origin=Origin(xyz=(0.10, 0.0, 0.09)),
        material="oiled_oak",
        name="rail_plank",
    )
    slide_base.visual(
        Box((1.42, 0.045, 0.010)),
        origin=Origin(xyz=(0.11, -0.14, 0.145)),
        material="worn_iron",
        name="wear_strip_0",
    )
    slide_base.visual(
        Box((1.42, 0.045, 0.010)),
        origin=Origin(xyz=(0.11, 0.14, 0.145)),
        material="worn_iron",
        name="wear_strip_1",
    )
    slide_base.visual(
        Box((0.22, 0.42, 0.08)),
        origin=Origin(xyz=(-0.58, 0.0, 0.10)),
        material="end_grain",
        name="rear_stop",
    )
    slide_base.visual(
        Box((0.18, 0.42, 0.08)),
        origin=Origin(xyz=(0.78, 0.0, 0.10)),
        material="end_grain",
        name="front_stop",
    )

    carriage = model.part("carriage_bed")
    carriage.visual(
        Box((0.64, 0.56, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material="oiled_oak",
        name="sled",
    )
    carriage.visual(
        Box((0.10, 0.58, 0.20)),
        origin=Origin(xyz=(-0.25, 0.0, 0.22)),
        material="end_grain",
        name="rear_transom",
    )
    carriage.visual(
        Box((0.09, 0.58, 0.12)),
        origin=Origin(xyz=(0.27, 0.0, 0.16)),
        material="end_grain",
        name="front_transom",
    )
    for index, y in enumerate((-0.30, 0.30)):
        carriage.visual(
            Box((0.46, 0.08, 0.34)),
            origin=Origin(xyz=(0.04, y, 0.29)),
            material="oiled_oak",
            name=f"cheek_{index}",
        )
    carriage.visual(
        Cylinder(radius=0.085, length=0.050),
        origin=Origin(xyz=(0.05, -0.235, 0.40), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="worn_iron",
        name="bearing_boss_0",
    )
    carriage.visual(
        Cylinder(radius=0.085, length=0.050),
        origin=Origin(xyz=(0.05, 0.235, 0.40), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="worn_iron",
        name="bearing_boss_1",
    )
    carriage.visual(
        Box((0.10, 0.10, 0.08)),
        origin=Origin(xyz=(-0.18, 0.0, 0.175)),
        material="worn_iron",
        name="elevation_screw_block",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.12),
        origin=Origin(xyz=(-0.18, 0.0, 0.135)),
        material="worn_iron",
        name="elevation_screw",
    )

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_barrel_body_shape(), "barrel_body", tolerance=0.0012),
        material="dark_cast_iron",
        name="barrel_body",
    )
    barrel.visual(
        Cylinder(radius=0.055, length=0.48),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="dark_cast_iron",
        name="trunnion_axle",
    )
    barrel.visual(
        Cylinder(radius=0.060, length=0.14),
        origin=Origin(xyz=(-0.275, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_cast_iron",
        name="cascabel_neck",
    )
    barrel.visual(
        Sphere(radius=0.075),
        origin=Origin(xyz=(-0.36, 0.0, 0.0)),
        material="dark_cast_iron",
        name="cascabel_knob",
    )
    barrel.visual(
        Cylinder(radius=0.018, length=0.035),
        origin=Origin(xyz=(-0.09, 0.0, 0.175)),
        material="dark_cast_iron",
        name="touch_hole_boss",
    )

    model.articulation(
        "deck_to_slide_base",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=slide_base,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-TRAVERSE_UPPER, upper=TRAVERSE_UPPER, effort=400.0, velocity=0.35),
    )
    model.articulation(
        "slide_base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=slide_base,
        child=carriage,
        origin=Origin(xyz=(0.15, 0.0, 0.15)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=RECOIL_TRAVEL, effort=1200.0, velocity=0.45),
    )
    model.articulation(
        "carriage_to_barrel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(0.05, 0.0, 0.40)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.08, upper=ELEVATION_UPPER, effort=250.0, velocity=0.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    slide_base = object_model.get_part("slide_base")
    carriage = object_model.get_part("carriage_bed")
    barrel = object_model.get_part("barrel")
    deck = object_model.get_part("deck")

    traverse = object_model.get_articulation("deck_to_slide_base")
    recoil = object_model.get_articulation("slide_base_to_carriage")
    trunnion = object_model.get_articulation("carriage_to_barrel")

    for boss_name in ("bearing_boss_0", "bearing_boss_1"):
        ctx.allow_overlap(
            barrel,
            carriage,
            elem_a="trunnion_axle",
            elem_b=boss_name,
            reason="The trunnion cylinder is intentionally captured inside the carriage bearing boss.",
        )
        ctx.expect_overlap(
            barrel,
            carriage,
            axes="xy",
            elem_a="trunnion_axle",
            elem_b=boss_name,
            min_overlap=0.010,
            name=f"{boss_name} captures trunnion",
        )

    ctx.expect_contact(
        deck,
        slide_base,
        elem_a="traverse_wear_plate",
        elem_b="pivot_plate",
        contact_tol=0.001,
        name="traverse pivot plate sits on deck wear plate",
    )
    ctx.expect_contact(
        carriage,
        slide_base,
        elem_a="sled",
        elem_b="wear_strip_0",
        contact_tol=0.001,
        name="sliding bed bears on wooden rail wear strip",
    )
    ctx.expect_overlap(
        carriage,
        slide_base,
        axes="x",
        elem_a="sled",
        elem_b="rail_plank",
        min_overlap=0.45,
        name="carriage has retained slide length at rest",
    )

    rest_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_body")
    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({recoil: RECOIL_TRAVEL}):
        ctx.expect_overlap(
            carriage,
            slide_base,
            axes="x",
            elem_a="sled",
            elem_b="rail_plank",
            min_overlap=0.35,
            name="carriage remains on rail after recoil",
        )
        recoiled_pos = ctx.part_world_position(carriage)
    ctx.check(
        "recoil joint moves carriage aft",
        rest_carriage_pos is not None
        and recoiled_pos is not None
        and recoiled_pos[0] < rest_carriage_pos[0] - 0.25,
        details=f"rest={rest_carriage_pos}, recoiled={recoiled_pos}",
    )

    with ctx.pose({trunnion: ELEVATION_UPPER}):
        elevated_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_body")
    ctx.check(
        "trunnion joint elevates the muzzle",
        rest_aabb is not None
        and elevated_aabb is not None
        and elevated_aabb[1][2] > rest_aabb[1][2] + 0.08,
        details=f"rest_aabb={rest_aabb}, elevated_aabb={elevated_aabb}",
    )

    with ctx.pose({traverse: TRAVERSE_UPPER}):
        traversed_barrel = ctx.part_world_aabb(barrel)
    ctx.check(
        "slide base traverses around deck pivot",
        traversed_barrel is not None,
        details="barrel remains mounted while the slide base traverses",
    )

    return ctx.report()


object_model = build_object_model()

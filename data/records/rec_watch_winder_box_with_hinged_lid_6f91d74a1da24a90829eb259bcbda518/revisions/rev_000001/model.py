from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BOX_W = 0.300
BOX_D = 0.220
BOX_H = 0.100
WALL = 0.011
FLOOR = 0.014
HINGE_Y = BOX_D / 2.0 + 0.006
HINGE_Z = BOX_H + 0.010
SPINDLE_Y = 0.034
SPINDLE_Z = 0.061


def _tray_shell() -> cq.Workplane:
    """Open, light presentation-box lower shell with a single top frame/rim."""
    outer = cq.Workplane("XY").box(BOX_W, BOX_D, BOX_H).translate((0.0, 0.0, BOX_H / 2.0))
    cutter = (
        cq.Workplane("XY")
        .box(BOX_W - 2.0 * WALL, BOX_D - 2.0 * WALL, BOX_H + 0.030)
        .translate((0.0, 0.0, FLOOR + (BOX_H + 0.030) / 2.0))
    )
    return outer.cut(cutter).edges("|Z").fillet(0.004)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="watch_winder_presentation_box")

    shell_mat = model.material("warm_lacquer_shell", color=(0.78, 0.72, 0.61, 1.0))
    velvet_mat = model.material("dark_espresso_velvet", color=(0.035, 0.030, 0.026, 1.0))
    brass_mat = model.material("brushed_brass", color=(0.82, 0.62, 0.26, 1.0))
    steel_mat = model.material("satin_steel", color=(0.52, 0.54, 0.55, 1.0))
    glass_mat = model.material("smoked_glass", color=(0.48, 0.65, 0.74, 0.34))
    cushion_mat = model.material("cream_suede_cushion", color=(0.88, 0.82, 0.68, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_tray_shell(), "open_tray_shell", tolerance=0.0015),
        material=shell_mat,
        name="shell",
    )
    body.visual(
        Box((BOX_W - 2.0 * WALL - 0.012, BOX_D - 2.0 * WALL - 0.012, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR + 0.0015)),
        material=velvet_mat,
        name="floor_liner",
    )
    body.visual(
        Box((0.125, 0.012, 0.075)),
        origin=Origin(xyz=(0.0, BOX_D / 2.0 - WALL - 0.006, 0.055)),
        material=velvet_mat,
        name="rear_backplate",
    )
    body.visual(
        Cylinder(radius=0.034, length=0.018),
        origin=Origin(xyz=(0.0, 0.078, SPINDLE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="motor_boss",
    )
    body.visual(
        Cylinder(radius=0.0065, length=0.036),
        origin=Origin(
            xyz=(0.0, SPINDLE_Y + 0.018, SPINDLE_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass_mat,
        name="spindle_pin",
    )
    for idx, x in enumerate((-0.100, 0.100)):
        body.visual(
            Box((0.066, 0.010, 0.020)),
            origin=Origin(xyz=(x, BOX_D / 2.0 + 0.002, BOX_H + 0.001)),
            material=shell_mat,
            name=f"hinge_bracket_{idx}",
        )
        body.visual(
            Cylinder(radius=0.0062, length=0.064),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass_mat,
            name=f"hinge_barrel_{idx}",
        )

    lid = model.part("lid")
    lid_w = BOX_W + 0.010
    lid_d = BOX_D + 0.002
    rail = 0.018
    lid_z = 0.014
    lid.visual(
        Box((lid_w, 0.014, lid_z)),
        origin=Origin(xyz=(0.0, -0.021, 0.004)),
        material=shell_mat,
        name="rear_rail",
    )
    lid.visual(
        Box((lid_w, rail, lid_z)),
        origin=Origin(xyz=(0.0, -lid_d + rail / 2.0, 0.004)),
        material=shell_mat,
        name="front_rail",
    )
    for idx, x in enumerate((-(lid_w - rail) / 2.0, (lid_w - rail) / 2.0)):
        lid.visual(
            Box((rail, lid_d, lid_z)),
            origin=Origin(xyz=(x, -lid_d / 2.0, 0.004)),
            material=shell_mat,
            name=f"side_rail_{idx}",
        )
    lid.visual(
        Box((lid_w - 2.0 * rail + 0.010, lid_d - 2.0 * rail + 0.010, 0.003)),
        origin=Origin(xyz=(0.0, -lid_d / 2.0, 0.0085)),
        material=glass_mat,
        name="glass",
    )
    lid.visual(
        Box((0.090, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -0.008, 0.003)),
        material=shell_mat,
        name="hinge_tab",
    )
    lid.visual(
        Cylinder(radius=0.0060, length=0.136),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass_mat,
        name="hinge_knuckle",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.041, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="turntable",
    )
    cradle.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass_mat,
        name="hub",
    )
    cradle.visual(
        Box((0.074, 0.023, 0.036)),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material=cushion_mat,
        name="cushion",
    )
    cradle.visual(
        Box((0.088, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, -0.033, 0.0)),
        material=brass_mat,
        name="retaining_band",
    )
    for idx, x in enumerate((-0.043, 0.043)):
        cradle.visual(
            Box((0.007, 0.018, 0.042)),
            origin=Origin(xyz=(x, -0.019, 0.0)),
            material=steel_mat,
            name=f"side_lip_{idx}",
        )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.35),
        motion_properties=MotionProperties(damping=0.08, friction=0.02),
    )
    model.articulation(
        "cradle_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(0.0, SPINDLE_Y, SPINDLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.005),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("lid_hinge")
    cradle_spin = object_model.get_articulation("cradle_spin")

    ctx.check(
        "lid has a rear horizontal revolute hinge",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(lid_hinge.axis[0] + 1.0) < 1e-9
        and lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.lower == 0.0
        and lid_hinge.motion_limits.upper is not None
        and lid_hinge.motion_limits.upper > 1.2,
        details=f"hinge={lid_hinge}",
    )
    ctx.check(
        "cradle uses a continuous spindle joint",
        cradle_spin.articulation_type == ArticulationType.CONTINUOUS
        and cradle_spin.axis == (0.0, 1.0, 0.0),
        details=f"spin={cradle_spin}",
    )

    ctx.expect_contact(
        cradle,
        body,
        elem_a="hub",
        elem_b="spindle_pin",
        contact_tol=0.0015,
        name="cradle hub seats on spindle pin",
    )
    ctx.expect_within(
        cradle,
        body,
        axes="xz",
        inner_elem="turntable",
        outer_elem="shell",
        margin=0.004,
        name="rotating cradle fits inside box opening",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="glass",
        negative_elem="shell",
        min_gap=0.002,
        name="closed lid glass clears the tray rim",
    )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_rail")
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_front = ctx.part_element_world_aabb(lid, elem="front_rail")
    closed_z = closed_front[0][2] if closed_front is not None else None
    open_z = open_front[0][2] if open_front is not None else None
    ctx.check(
        "lid opens upward from rear hinge",
        closed_z is not None and open_z is not None and open_z > closed_z + 0.12,
        details=f"closed_front_min_z={closed_z}, open_front_min_z={open_z}",
    )

    with ctx.pose({cradle_spin: math.pi / 2.0}):
        ctx.expect_within(
            cradle,
            body,
            axes="xz",
            inner_elem="turntable",
            outer_elem="shell",
            margin=0.004,
            name="cradle remains inside during quarter turn",
        )

    return ctx.report()


object_model = build_object_model()

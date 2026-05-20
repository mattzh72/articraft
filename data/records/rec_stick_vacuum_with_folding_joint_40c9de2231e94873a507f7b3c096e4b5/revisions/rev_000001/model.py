from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_stick_vacuum")

    matte_graphite = Material("matte_graphite", rgba=(0.08, 0.085, 0.09, 1.0))
    charcoal = Material("charcoal_rubber", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_red = Material("satin_red", rgba=(0.75, 0.05, 0.035, 1.0))
    brushed_aluminum = Material("brushed_aluminum", rgba=(0.72, 0.72, 0.68, 1.0))
    smoky_clear = Material("smoky_clear_bin", rgba=(0.45, 0.62, 0.72, 0.42))
    dark_plastic = Material("dark_plastic", rgba=(0.035, 0.038, 0.045, 1.0))

    body = model.part("body")
    # The body is the dominant primary link: a long handle/spine with a compact
    # motor pod and dust bin above the fold knuckle.
    body.visual(
        Cylinder(radius=0.022, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        material=satin_red,
        name="upper_spine",
    )
    body.visual(
        Cylinder(radius=0.105, length=0.30),
        origin=Origin(xyz=(0.035, 0.0, 0.58), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_graphite,
        name="motor_pod",
    )
    body.visual(
        Cylinder(radius=0.072, length=0.24),
        origin=Origin(xyz=(0.055, 0.0, 0.36)),
        material=smoky_clear,
        name="dust_bin",
    )
    body.visual(
        Cylinder(radius=0.040, length=0.20),
        origin=Origin(xyz=(0.18, 0.0, 0.58), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="rear_filter_cap",
    )
    body.visual(
        Box((0.040, 0.052, 0.40)),
        origin=Origin(xyz=(-0.105, 0.0, 0.78)),
        material=charcoal,
        name="rear_grip",
    )
    body.visual(
        Box((0.140, 0.052, 0.040)),
        origin=Origin(xyz=(-0.048, 0.0, 0.975)),
        material=charcoal,
        name="top_handle_bridge",
    )
    body.visual(
        Box((0.135, 0.050, 0.036)),
        origin=Origin(xyz=(-0.045, 0.0, 0.635)),
        material=charcoal,
        name="lower_handle_bridge",
    )
    body.visual(
        Box((0.030, 0.046, 0.060)),
        origin=Origin(xyz=(-0.007, 0.0, 0.590)),
        material=dark_plastic,
        name="trigger_detail",
    )
    # Clevis cheeks and separated hinge barrels support the folding wand without
    # broad collision between the links.
    body.visual(
        Box((0.080, 0.030, 0.080)),
        origin=Origin(xyz=(0.0, -0.060, 0.040)),
        material=matte_graphite,
        name="fold_cheek_0",
    )
    body.visual(
        Cylinder(radius=0.036, length=0.032),
        origin=Origin(xyz=(0.0, -0.060, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_graphite,
        name="fold_outer_barrel_0",
    )
    body.visual(
        Box((0.080, 0.030, 0.080)),
        origin=Origin(xyz=(0.0, 0.060, 0.040)),
        material=matte_graphite,
        name="fold_cheek_1",
    )
    body.visual(
        Cylinder(radius=0.036, length=0.032),
        origin=Origin(xyz=(0.0, 0.060, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_graphite,
        name="fold_outer_barrel_1",
    )
    body.visual(
        Box((0.080, 0.150, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=matte_graphite,
        name="fold_top_bridge",
    )

    wand = model.part("wand")
    # The wand is intentionally the shorter secondary link.  Its part frame is
    # exactly on the fold hinge line, so folding motion is easy to read.
    wand.visual(
        Cylinder(radius=0.030, length=0.088),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_aluminum,
        name="fold_inner_barrel",
    )
    wand.visual(
        Box((0.046, 0.046, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=satin_red,
        name="upper_wand_neck",
    )
    wand.visual(
        Cylinder(radius=0.018, length=0.52),
        origin=Origin(xyz=(0.0, 0.0, -0.320)),
        material=brushed_aluminum,
        name="wand_tube",
    )
    wand.visual(
        Box((0.050, 0.050, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, -0.555)),
        material=satin_red,
        name="lower_wand_neck",
    )
    wand.visual(
        Box((0.056, 0.140, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, -0.570)),
        material=satin_red,
        name="head_yoke_bridge",
    )
    wand.visual(
        Box((0.056, 0.028, 0.100)),
        origin=Origin(xyz=(0.0, -0.052, -0.615)),
        material=satin_red,
        name="head_yoke_cheek_0",
    )
    wand.visual(
        Cylinder(radius=0.026, length=0.032),
        origin=Origin(xyz=(0.0, -0.052, -0.640), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_red,
        name="head_outer_barrel_0",
    )
    wand.visual(
        Box((0.056, 0.028, 0.100)),
        origin=Origin(xyz=(0.0, 0.052, -0.615)),
        material=satin_red,
        name="head_yoke_cheek_1",
    )
    wand.visual(
        Cylinder(radius=0.026, length=0.032),
        origin=Origin(xyz=(0.0, 0.052, -0.640), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_red,
        name="head_outer_barrel_1",
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.024, length=0.072),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="head_inner_barrel",
    )
    floor_head.visual(
        Box((0.065, 0.046, 0.044)),
        origin=Origin(xyz=(0.035, 0.0, -0.036)),
        material=dark_plastic,
        name="pitch_neck",
    )
    floor_head.visual(
        Box((0.065, 0.058, 0.056)),
        origin=Origin(xyz=(0.055, 0.0, -0.064)),
        material=dark_plastic,
        name="neck_stem",
    )
    floor_head.visual(
        Box((0.380, 0.280, 0.060)),
        origin=Origin(xyz=(0.120, 0.0, -0.105)),
        material=matte_graphite,
        name="nozzle_shell",
    )
    floor_head.visual(
        Box((0.260, 0.038, 0.014)),
        origin=Origin(xyz=(0.135, 0.0, -0.139)),
        material=charcoal,
        name="suction_slot",
    )
    floor_head.visual(
        Cylinder(radius=0.028, length=0.246),
        origin=Origin(xyz=(0.250, 0.0, -0.132), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="front_roller",
    )
    floor_head.visual(
        Box((0.030, 0.280, 0.070)),
        origin=Origin(xyz=(-0.070, 0.0, -0.100)),
        material=dark_plastic,
        name="rear_bumper",
    )

    model.articulation(
        "fold_joint",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.55),
    )
    model.articulation(
        "head_pitch",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, 0.0, -0.640)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.55, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fold = object_model.get_articulation("fold_joint")
    head_pitch = object_model.get_articulation("head_pitch")
    body = object_model.get_part("body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")

    ctx.check(
        "fold joint is horizontal revolute",
        fold.articulation_type == ArticulationType.REVOLUTE
        and abs(fold.axis[1]) > 0.99
        and abs(fold.axis[0]) < 0.01
        and abs(fold.axis[2]) < 0.01,
        details=f"type={fold.articulation_type}, axis={fold.axis}",
    )
    ctx.check(
        "head pitch is horizontal revolute",
        head_pitch.articulation_type == ArticulationType.REVOLUTE
        and abs(head_pitch.axis[1]) > 0.99
        and abs(head_pitch.axis[0]) < 0.01
        and abs(head_pitch.axis[2]) < 0.01,
        details=f"type={head_pitch.articulation_type}, axis={head_pitch.axis}",
    )
    ctx.expect_overlap(
        body,
        wand,
        axes="xz",
        elem_a="fold_outer_barrel_0",
        elem_b="fold_inner_barrel",
        min_overlap=0.030,
        name="fold barrels share hinge projection",
    )
    ctx.expect_overlap(
        wand,
        floor_head,
        axes="xz",
        elem_a="head_outer_barrel_0",
        elem_b="head_inner_barrel",
        min_overlap=0.020,
        name="head barrels share pitch projection",
    )

    straight_aabb = ctx.part_element_world_aabb(wand, elem="wand_tube")
    with ctx.pose({fold: 1.20}):
        folded_aabb = ctx.part_element_world_aabb(wand, elem="wand_tube")
    ctx.check(
        "folding moves wand forward and upward",
        straight_aabb is not None
        and folded_aabb is not None
        and folded_aabb[0][0] < straight_aabb[0][0] - 0.20
        and folded_aabb[0][2] > straight_aabb[0][2] + 0.15,
        details=f"straight={straight_aabb}, folded={folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

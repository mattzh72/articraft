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

    matte_black = Material("matte_black", rgba=(0.015, 0.015, 0.018, 1.0))
    dark_graphite = Material("dark_graphite", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_grey = Material("satin_grey", rgba=(0.42, 0.44, 0.46, 1.0))
    brushed_metal = Material("brushed_metal", rgba=(0.70, 0.72, 0.72, 1.0))
    accent_blue = Material("accent_blue", rgba=(0.02, 0.28, 0.75, 1.0))
    translucent_cup = Material("smoked_clear_bin", rgba=(0.72, 0.86, 0.95, 0.42))
    rubber = Material("black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    red = Material("red_release", rgba=(0.85, 0.03, 0.025, 1.0))

    fold_z = 0.86
    floor_pitch_z = -0.70

    body = model.part("motor_body")
    # A tall upper/root section keeps the folding joint well above the floor head.
    body.visual(
        Box((0.070, 0.080, 0.105)),
        origin=Origin(xyz=(0.0, 0.0, fold_z + 0.087)),
        material=dark_graphite,
        name="lower_socket",
    )
    body.visual(
        Box((0.054, 0.152, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, fold_z + 0.052)),
        material=satin_grey,
        name="fold_yoke_bridge",
    )
    body.visual(
        Box((0.155, 0.125, 0.335)),
        origin=Origin(xyz=(-0.005, 0.0, fold_z + 0.285)),
        material=matte_black,
        name="upright_shell",
    )
    body.visual(
        Cylinder(radius=0.118, length=0.270),
        origin=Origin(xyz=(0.012, 0.0, fold_z + 0.315), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_graphite,
        name="round_motor_pod",
    )
    body.visual(
        Cylinder(radius=0.064, length=0.205),
        origin=Origin(xyz=(0.117, 0.0, fold_z + 0.192)),
        material=translucent_cup,
        name="dust_cup",
    )
    body.visual(
        Box((0.170, 0.018, 0.022)),
        origin=Origin(xyz=(0.116, -0.065, fold_z + 0.190)),
        material=brushed_metal,
        name="cup_rib_0",
    )
    body.visual(
        Box((0.170, 0.018, 0.022)),
        origin=Origin(xyz=(0.116, 0.065, fold_z + 0.190)),
        material=brushed_metal,
        name="cup_rib_1",
    )
    body.visual(
        Box((0.210, 0.046, 0.050)),
        origin=Origin(xyz=(-0.094, 0.0, fold_z + 0.445)),
        material=matte_black,
        name="top_handle_rail",
    )
    body.visual(
        Box((0.185, 0.046, 0.050)),
        origin=Origin(xyz=(-0.110, 0.0, fold_z + 0.232)),
        material=matte_black,
        name="lower_handle_rail",
    )
    body.visual(
        Box((0.052, 0.046, 0.245)),
        origin=Origin(xyz=(-0.208, 0.0, fold_z + 0.335)),
        material=matte_black,
        name="rear_grip",
    )
    body.visual(
        Box((0.090, 0.100, 0.178)),
        origin=Origin(xyz=(-0.137, 0.0, fold_z + 0.125)),
        material=accent_blue,
        name="battery_pack",
    )
    body.visual(
        Box((0.042, 0.016, 0.028)),
        origin=Origin(xyz=(-0.225, -0.031, fold_z + 0.275)),
        material=red,
        name="trigger_mark",
    )
    body.visual(
        Box((0.052, 0.024, 0.115)),
        origin=Origin(xyz=(0.0, -0.075, fold_z)),
        material=satin_grey,
        name="fold_yoke_0",
    )
    body.visual(
        Box((0.052, 0.024, 0.115)),
        origin=Origin(xyz=(0.0, 0.075, fold_z)),
        material=satin_grey,
        name="fold_yoke_1",
    )
    body.visual(
        Cylinder(radius=0.040, length=0.042),
        origin=Origin(xyz=(0.0, -0.075, fold_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_grey,
        name="fold_outer_barrel_0",
    )
    body.visual(
        Cylinder(radius=0.040, length=0.042),
        origin=Origin(xyz=(0.0, 0.075, fold_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_grey,
        name="fold_outer_barrel_1",
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.035, length=0.088),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="fold_inner_barrel",
    )
    wand.visual(
        Box((0.058, 0.065, 0.135)),
        origin=Origin(xyz=(0.0, 0.0, -0.074)),
        material=dark_graphite,
        name="upper_collar",
    )
    wand.visual(
        Cylinder(radius=0.026, length=0.565),
        origin=Origin(xyz=(0.0, 0.0, -0.365)),
        material=brushed_metal,
        name="straight_wand_tube",
    )
    wand.visual(
        Box((0.054, 0.150, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, floor_pitch_z + 0.075)),
        material=dark_graphite,
        name="lower_cross_yoke",
    )
    wand.visual(
        Box((0.046, 0.024, 0.092)),
        origin=Origin(xyz=(0.0, -0.065, floor_pitch_z + 0.020)),
        material=dark_graphite,
        name="head_fork_0",
    )
    wand.visual(
        Box((0.046, 0.024, 0.092)),
        origin=Origin(xyz=(0.0, 0.065, floor_pitch_z + 0.020)),
        material=dark_graphite,
        name="head_fork_1",
    )
    wand.visual(
        Cylinder(radius=0.035, length=0.038),
        origin=Origin(xyz=(0.0, -0.065, floor_pitch_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_graphite,
        name="head_outer_barrel_0",
    )
    wand.visual(
        Cylinder(radius=0.035, length=0.038),
        origin=Origin(xyz=(0.0, 0.065, floor_pitch_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_graphite,
        name="head_outer_barrel_1",
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.032, length=0.092),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_grey,
        name="pitch_inner_barrel",
    )
    floor_head.visual(
        Box((0.105, 0.058, 0.070)),
        origin=Origin(xyz=(0.025, 0.0, -0.060)),
        material=dark_graphite,
        name="neck_block",
    )
    floor_head.visual(
        Box((0.355, 0.305, 0.100)),
        origin=Origin(xyz=(0.100, 0.0, -0.110)),
        material=dark_graphite,
        name="brush_housing",
    )
    floor_head.visual(
        Box((0.030, 0.320, 0.044)),
        origin=Origin(xyz=(0.290, 0.0, -0.092)),
        material=rubber,
        name="front_bumper",
    )
    floor_head.visual(
        Cylinder(radius=0.027, length=0.255),
        origin=Origin(xyz=(0.095, 0.0, -0.150), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent_blue,
        name="visible_brush_roll",
    )
    floor_head.visual(
        Cylinder(radius=0.034, length=0.026),
        origin=Origin(xyz=(-0.040, -0.164, -0.087), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rear_wheel_0",
    )
    floor_head.visual(
        Cylinder(radius=0.034, length=0.026),
        origin=Origin(xyz=(-0.040, 0.164, -0.087), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rear_wheel_1",
    )
    floor_head.visual(
        Box((0.170, 0.026, 0.012)),
        origin=Origin(xyz=(0.080, -0.152, -0.057)),
        material=brushed_metal,
        name="side_skid_0",
    )
    floor_head.visual(
        Box((0.170, 0.026, 0.012)),
        origin=Origin(xyz=(0.080, 0.152, -0.057)),
        material=brushed_metal,
        name="side_skid_1",
    )

    model.articulation(
        "fold_joint",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(xyz=(0.0, 0.0, fold_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.65),
    )
    model.articulation(
        "floor_pitch",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, 0.0, floor_pitch_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.55, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("motor_body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold = object_model.get_articulation("fold_joint")
    pitch = object_model.get_articulation("floor_pitch")

    floor_aabb = ctx.part_world_aabb(floor_head)
    fold_pos = ctx.part_world_position(wand)
    ctx.check(
        "fold joint is well above the floor line",
        floor_aabb is not None
        and fold_pos is not None
        and fold_pos[2] - floor_aabb[0][2] > 0.75,
        details=f"fold_pos={fold_pos}, floor_aabb={floor_aabb}",
    )
    ctx.check(
        "fold axis is horizontal",
        abs(fold.axis[0]) < 1e-6 and abs(abs(fold.axis[1]) - 1.0) < 1e-6 and abs(fold.axis[2]) < 1e-6,
        details=f"axis={fold.axis}",
    )
    ctx.check(
        "floor head pitch axis is horizontal",
        abs(pitch.axis[0]) < 1e-6 and abs(abs(pitch.axis[1]) - 1.0) < 1e-6 and abs(pitch.axis[2]) < 1e-6,
        details=f"axis={pitch.axis}",
    )
    ctx.expect_overlap(
        wand,
        body,
        axes="z",
        elem_a="fold_inner_barrel",
        elem_b="fold_outer_barrel_0",
        min_overlap=0.040,
        name="fold hinge barrels share the hinge height",
    )
    ctx.expect_overlap(
        floor_head,
        wand,
        axes="z",
        elem_a="pitch_inner_barrel",
        elem_b="head_outer_barrel_0",
        min_overlap=0.030,
        name="floor hinge barrels share the pitch height",
    )

    wand_rest = ctx.part_world_aabb(wand)
    with ctx.pose({fold: 1.20}):
        wand_folded = ctx.part_world_aabb(wand)
    ctx.check(
        "wand folds forward and upward about the high joint",
        wand_rest is not None
        and wand_folded is not None
        and wand_folded[0][2] > wand_rest[0][2] + 0.20
        and wand_folded[0][0] < wand_rest[0][0] - 0.25,
        details=f"rest={wand_rest}, folded={wand_folded}",
    )

    head_rest = ctx.part_world_aabb(floor_head)
    with ctx.pose({pitch: 0.35}):
        head_pitched = ctx.part_world_aabb(floor_head)
    ctx.check(
        "floor head visibly pitches on its hinge",
        head_rest is not None
        and head_pitched is not None
        and abs(head_pitched[0][2] - head_rest[0][2]) > 0.015,
        details=f"rest={head_rest}, pitched={head_pitched}",
    )

    return ctx.report()


object_model = build_object_model()

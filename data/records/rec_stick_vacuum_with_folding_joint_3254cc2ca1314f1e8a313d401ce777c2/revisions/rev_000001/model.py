from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_stick_vacuum")

    armor = model.material("powder_coated_armor", rgba=(0.18, 0.21, 0.23, 1.0))
    yellow = model.material("service_yellow", rgba=(0.95, 0.73, 0.12, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.016, 0.015, 1.0))
    steel = model.material("brushed_pin_steel", rgba=(0.63, 0.64, 0.62, 1.0))
    clear_smoke = model.material("smoked_clear_bin", rgba=(0.35, 0.52, 0.58, 0.42))
    wear_blue = model.material("replaceable_blue_wear", rgba=(0.05, 0.22, 0.80, 1.0))

    body = model.part("vacuum_body")
    # Slim workhorse spine, motor pod, dust cup, replaceable battery and rear handle.
    body.visual(
        Cylinder(radius=0.055, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 0.98)),
        material=armor,
        name="main_spine",
    )
    body.visual(
        Cylinder(radius=0.082, length=0.18),
        origin=Origin(xyz=(0.01, 0.0, 1.16), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=yellow,
        name="motor_pod",
    )
    body.visual(
        Cylinder(radius=0.065, length=0.25),
        origin=Origin(xyz=(0.045, 0.0, 0.93)),
        material=clear_smoke,
        name="dust_cup",
    )
    body.visual(
        Box((0.075, 0.13, 0.23)),
        origin=Origin(xyz=(-0.072, 0.0, 0.965)),
        material=armor,
        name="battery_pack",
    )
    body.visual(
        Box((0.035, 0.12, 0.24)),
        origin=Origin(xyz=(-0.16, 0.0, 1.08)),
        material=rubber,
        name="rear_grip",
    )
    body.visual(
        Box((0.16, 0.09, 0.035)),
        origin=Origin(xyz=(-0.105, 0.0, 1.205)),
        material=rubber,
        name="top_handle_strut",
    )
    body.visual(
        Box((0.13, 0.09, 0.035)),
        origin=Origin(xyz=(-0.115, 0.0, 0.97)),
        material=rubber,
        name="lower_handle_strut",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.81)),
        material=steel,
        name="upper_wand_tube",
    )
    body.visual(
        Box((0.085, 0.115, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.705)),
        material=yellow,
        name="fold_collar",
    )
    body.visual(
        Box((0.082, 0.026, 0.14)),
        origin=Origin(xyz=(0.0, 0.067, 0.62)),
        material=yellow,
        name="fold_yoke_0",
    )
    body.visual(
        Box((0.082, 0.026, 0.14)),
        origin=Origin(xyz=(0.0, -0.067, 0.62)),
        material=yellow,
        name="fold_yoke_1",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, 0.086, 0.62), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="fold_pin_cap_0",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, -0.086, 0.62), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="fold_pin_cap_1",
    )
    body.visual(
        Box((0.048, 0.020, 0.22)),
        origin=Origin(xyz=(0.080, -0.054, 0.93)),
        material=steel,
        name="hatch_hinge_leaf",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.22),
        origin=Origin(xyz=(0.106, -0.054, 0.93)),
        material=steel,
        name="hatch_hinge_barrel",
    )

    hatch = model.part("service_hatch")
    hatch.visual(
        Box((0.012, 0.155, 0.225)),
        origin=Origin(xyz=(0.0, 0.0775, 0.0)),
        material=yellow,
        name="hatch_panel",
    )
    hatch.visual(
        Cylinder(radius=0.010, length=0.225),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="hatch_barrel",
    )
    hatch.visual(
        Box((0.014, 0.040, 0.025)),
        origin=Origin(xyz=(0.008, 0.142, -0.076)),
        material=rubber,
        name="pull_tab",
    )

    lower = model.part("lower_wand")
    lower.visual(
        Cylinder(radius=0.050, length=0.108),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="fold_knuckle",
    )
    lower.visual(
        Cylinder(radius=0.023, length=0.43),
        origin=Origin(xyz=(0.0, 0.0, -0.235)),
        material=steel,
        name="lower_tube",
    )
    lower.visual(
        Box((0.070, 0.078, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        material=yellow,
        name="service_clamp",
    )
    lower.visual(
        Box((0.075, 0.120, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.450)),
        material=yellow,
        name="nozzle_socket",
    )
    lower.visual(
        Box((0.085, 0.026, 0.130)),
        origin=Origin(xyz=(0.0, 0.065, -0.520)),
        material=yellow,
        name="nozzle_fork_0",
    )
    lower.visual(
        Box((0.085, 0.026, 0.130)),
        origin=Origin(xyz=(0.0, -0.065, -0.520)),
        material=yellow,
        name="nozzle_fork_1",
    )
    lower.visual(
        Cylinder(radius=0.016, length=0.016),
        origin=Origin(xyz=(0.0, 0.086, -0.520), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="nozzle_pin_cap_0",
    )
    lower.visual(
        Cylinder(radius=0.016, length=0.016),
        origin=Origin(xyz=(0.0, -0.086, -0.520), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="nozzle_pin_cap_1",
    )

    head = model.part("floor_head")
    head.visual(
        Cylinder(radius=0.036, length=0.104),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="head_trunnion",
    )
    head.visual(
        Box((0.085, 0.075, 0.090)),
        origin=Origin(xyz=(0.030, 0.0, -0.045)),
        material=yellow,
        name="neck_block",
    )
    head.visual(
        Box((0.300, 0.430, 0.065)),
        origin=Origin(xyz=(0.080, 0.0, -0.108)),
        material=yellow,
        name="head_shell",
    )
    head.visual(
        Box((0.035, 0.430, 0.055)),
        origin=Origin(xyz=(0.246, 0.0, -0.105)),
        material=rubber,
        name="front_bumper",
    )
    head.visual(
        Box((0.060, 0.030, 0.165)),
        origin=Origin(xyz=(0.105, 0.195, -0.118)),
        material=armor,
        name="bearing_block_0",
    )
    head.visual(
        Box((0.060, 0.030, 0.165)),
        origin=Origin(xyz=(0.105, -0.195, -0.118)),
        material=armor,
        name="bearing_block_1",
    )
    head.visual(
        Box((0.230, 0.030, 0.018)),
        origin=Origin(xyz=(0.070, 0.195, -0.195)),
        material=wear_blue,
        name="skid_shoe_0",
    )
    head.visual(
        Box((0.230, 0.030, 0.018)),
        origin=Origin(xyz=(0.070, -0.195, -0.195)),
        material=wear_blue,
        name="skid_shoe_1",
    )
    head.visual(
        Box((0.135, 0.120, 0.010)),
        origin=Origin(xyz=(0.035, 0.0, -0.070)),
        material=clear_smoke,
        name="brush_access_window",
    )

    brush = model.part("brush_roll")
    brush.visual(
        Cylinder(radius=0.028, length=0.340),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="roller_core",
    )
    brush.visual(
        Cylinder(radius=0.008, length=0.360),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="brush_shaft",
    )
    brush.visual(
        Box((0.016, 0.320, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=wear_blue,
        name="bristle_strip_0",
    )
    brush.visual(
        Box((0.016, 0.320, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        material=wear_blue,
        name="bristle_strip_1",
    )
    brush.visual(
        Box((0.014, 0.320, 0.016)),
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
        material=wear_blue,
        name="bristle_strip_2",
    )
    brush.visual(
        Box((0.014, 0.320, 0.016)),
        origin=Origin(xyz=(-0.032, 0.0, 0.0)),
        material=wear_blue,
        name="bristle_strip_3",
    )

    model.articulation(
        "body_to_hatch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hatch,
        origin=Origin(xyz=(0.116, -0.078, 0.93)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=0.0, upper=1.25),
    )
    model.articulation(
        "fold_joint",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.2, lower=0.0, upper=1.55),
    )
    model.articulation(
        "nozzle_pitch",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, -0.520)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=-0.55, upper=0.70),
    )
    model.articulation(
        "brush_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=brush,
        origin=Origin(xyz=(0.105, 0.0, -0.180)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("vacuum_body")
    hatch = object_model.get_part("service_hatch")
    lower = object_model.get_part("lower_wand")
    head = object_model.get_part("floor_head")
    brush = object_model.get_part("brush_roll")
    hatch_joint = object_model.get_articulation("body_to_hatch")
    fold_joint = object_model.get_articulation("fold_joint")
    nozzle_joint = object_model.get_articulation("nozzle_pitch")

    ctx.expect_gap(
        hatch,
        body,
        axis="x",
        min_gap=0.0,
        max_gap=0.010,
        positive_elem="hatch_panel",
        negative_elem="dust_cup",
        name="service hatch is flush on dust cup",
    )
    ctx.expect_overlap(
        lower,
        body,
        axes="xz",
        min_overlap=0.05,
        elem_a="fold_knuckle",
        elem_b="fold_yoke_0",
        name="fold knuckle aligns inside yoke cheek profile",
    )
    ctx.expect_gap(
        body,
        lower,
        axis="y",
        min_gap=0.0,
        max_gap=0.008,
        positive_elem="fold_yoke_0",
        negative_elem="fold_knuckle",
        name="fold yoke keeps service clearance around knuckle",
    )
    ctx.expect_overlap(
        head,
        lower,
        axes="xz",
        min_overlap=0.045,
        elem_a="head_trunnion",
        elem_b="nozzle_fork_0",
        name="nozzle trunnion aligns inside fork cheek profile",
    )
    ctx.expect_gap(
        lower,
        head,
        axis="y",
        min_gap=0.0,
        max_gap=0.008,
        positive_elem="nozzle_fork_0",
        negative_elem="head_trunnion",
        name="nozzle fork keeps service clearance around trunnion",
    )
    ctx.expect_contact(
        brush,
        head,
        elem_a="brush_shaft",
        elem_b="bearing_block_0",
        contact_tol=0.002,
        name="brush shaft seats in replaceable bearing blocks",
    )

    rest_head_aabb = ctx.part_world_aabb(head)
    with ctx.pose({fold_joint: 1.20}):
        folded_head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "fold joint swings lower wand away from straight stick",
        rest_head_aabb is not None
        and folded_head_aabb is not None
        and folded_head_aabb[0][0] < rest_head_aabb[0][0] - 0.18,
        details=f"rest={rest_head_aabb}, folded={folded_head_aabb}",
    )

    rest_bumper = ctx.part_element_world_aabb(head, elem="front_bumper")
    with ctx.pose({nozzle_joint: 0.55}):
        pitched_bumper = ctx.part_element_world_aabb(head, elem="front_bumper")
    ctx.check(
        "nozzle pitch visibly changes floor head angle",
        rest_bumper is not None
        and pitched_bumper is not None
        and pitched_bumper[1][2] > rest_bumper[1][2] + 0.07,
        details=f"rest={rest_bumper}, pitched={pitched_bumper}",
    )

    rest_hatch = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    with ctx.pose({hatch_joint: 1.0}):
        open_hatch = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    ctx.check(
        "service hatch opens outward for filter access",
        rest_hatch is not None
        and open_hatch is not None
        and open_hatch[1][0] > rest_hatch[1][0] + 0.06,
        details=f"rest={rest_hatch}, open={open_hatch}",
    )

    return ctx.report()


object_model = build_object_model()

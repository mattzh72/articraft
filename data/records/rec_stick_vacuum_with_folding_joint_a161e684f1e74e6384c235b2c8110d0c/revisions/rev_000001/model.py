from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_stick_vacuum")

    dark = model.material("impact_black", rgba=(0.015, 0.017, 0.018, 1.0))
    graphite = model.material("graphite_composite", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    safety = model.material("safety_yellow", rgba=(1.0, 0.74, 0.03, 1.0))
    red = model.material("lockout_red", rgba=(0.85, 0.04, 0.025, 1.0))
    dust = model.material("smoked_clear_bin", rgba=(0.45, 0.56, 0.60, 0.42))

    body = model.part("body")
    # Root frame is the folding-hinge pin center.  The vacuum body extends upward
    # from it, keeping the high-stress fold bracket visually dominant.
    body.visual(
        Box((0.118, 0.122, 0.56)),
        origin=Origin(xyz=(0.018, 0.0, 0.326)),
        material=graphite,
        name="slim_spine",
    )
    body.visual(
        Cylinder(radius=0.067, length=0.34),
        origin=Origin(xyz=(0.048, 0.0, 0.260)),
        material=dust,
        name="dust_bin",
    )
    body.visual(
        Box((0.120, 0.130, 0.175)),
        origin=Origin(xyz=(0.010, 0.0, 0.585)),
        material=dark,
        name="motor_housing",
    )
    body.visual(
        Box((0.075, 0.118, 0.245)),
        origin=Origin(xyz=(-0.076, 0.0, 0.435)),
        material=dark,
        name="battery_pack",
    )
    body.visual(
        Box((0.035, 0.045, 0.285)),
        origin=Origin(xyz=(-0.168, 0.0, 0.500)),
        material=dark,
        name="rear_handle_post",
    )
    body.visual(
        Box((0.160, 0.046, 0.038)),
        origin=Origin(xyz=(-0.096, 0.0, 0.660)),
        material=dark,
        name="top_grip",
    )
    body.visual(
        Box((0.115, 0.042, 0.040)),
        origin=Origin(xyz=(-0.107, 0.0, 0.365)),
        material=dark,
        name="lower_handle_bridge",
    )
    body.visual(
        Box((0.108, 0.162, 0.026)),
        origin=Origin(xyz=(-0.006, 0.0, 0.082)),
        material=steel,
        name="fold_upper_crossplate",
    )
    for y in (-0.078, 0.078):
        body.visual(
            Box((0.136, 0.014, 0.170)),
            origin=Origin(xyz=(0.000, y, 0.000)),
            material=steel,
            name=f"fold_cheek_{'neg' if y < 0 else 'pos'}",
        )
        body.visual(
            Box((0.034, 0.018, 0.068)),
            origin=Origin(xyz=(0.060, y, 0.036)),
            material=safety,
            name=f"fold_stop_{'neg' if y < 0 else 'pos'}",
        )
        body.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(-0.041, y * 1.095, 0.038), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"fold_bolt_upper_{'neg' if y < 0 else 'pos'}",
        )
        body.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(0.043, y * 1.095, -0.037), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"fold_bolt_lower_{'neg' if y < 0 else 'pos'}",
        )
    body.visual(
        Cylinder(radius=0.014, length=0.188),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="fold_pin",
    )
    body.visual(
        Cylinder(radius=0.025, length=0.010),
        origin=Origin(xyz=(0.0, -0.099, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="fold_pin_head_0",
    )
    body.visual(
        Cylinder(radius=0.025, length=0.010),
        origin=Origin(xyz=(0.0, 0.099, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="fold_pin_head_1",
    )
    body.visual(
        Box((0.036, 0.162, 0.035)),
        origin=Origin(xyz=(0.048, 0.0, 0.095)),
        material=safety,
        name="fold_guard_bridge",
    )
    body.visual(
        Box((0.080, 0.010, 0.040)),
        origin=Origin(xyz=(0.034, 0.092, 0.072)),
        material=steel,
        name="lock_slider_rail",
    )

    fold_lock = model.part("fold_lock")
    fold_lock.visual(
        Box((0.060, 0.014, 0.030)),
        origin=Origin(xyz=(0.000, 0.0, 0.000)),
        material=red,
        name="lock_slider",
    )
    fold_lock.visual(
        Box((0.025, 0.020, 0.046)),
        origin=Origin(xyz=(-0.026, 0.003, 0.000)),
        material=red,
        name="thumb_tab",
    )
    fold_lock.visual(
        Box((0.020, 0.010, 0.022)),
        origin=Origin(xyz=(0.040, -0.002, 0.000)),
        material=steel,
        name="lock_nose",
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.036, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="fold_barrel",
    )
    wand.visual(
        Box((0.064, 0.080, 0.085)),
        origin=Origin(xyz=(0.0, 0.0, -0.056)),
        material=steel,
        name="fold_tang",
    )
    wand.visual(
        Cylinder(radius=0.029, length=0.805),
        origin=Origin(xyz=(0.0, 0.0, -0.472)),
        material=graphite,
        name="main_tube",
    )
    wand.visual(
        Cylinder(radius=0.036, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.116)),
        material=safety,
        name="upper_reinforced_collar",
    )
    wand.visual(
        Cylinder(radius=0.034, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.840)),
        material=safety,
        name="lower_reinforced_collar",
    )
    for y in (-0.033, 0.033):
        wand.visual(
            Box((0.018, 0.012, 0.650)),
            origin=Origin(xyz=(0.023, y * 0.61, -0.484)),
            material=steel,
            name=f"front_strap_{'neg' if y < 0 else 'pos'}",
        )
        wand.visual(
            Box((0.018, 0.012, 0.650)),
            origin=Origin(xyz=(-0.023, y * 0.61, -0.484)),
            material=steel,
            name=f"rear_strap_{'neg' if y < 0 else 'pos'}",
        )
    wand.visual(
        Box((0.047, 0.065, 0.040)),
        origin=Origin(xyz=(0.054, 0.0, -0.035)),
        material=safety,
        name="fold_travel_tab",
    )
    # Pinned bracket at the nozzle end, connected to the tube by the lower collar.
    wand.visual(
        Box((0.070, 0.138, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.877)),
        material=steel,
        name="nozzle_upper_crossplate",
    )
    for y in (-0.076, 0.076):
        wand.visual(
            Box((0.078, 0.014, 0.116)),
            origin=Origin(xyz=(0.0, y, -0.920)),
            material=steel,
            name=f"nozzle_cheek_{'neg' if y < 0 else 'pos'}",
        )
        wand.visual(
            Box((0.027, 0.017, 0.048)),
            origin=Origin(xyz=(-0.046, y, -0.875)),
            material=safety,
            name=f"nozzle_stop_{'neg' if y < 0 else 'pos'}",
        )
        wand.visual(
            Cylinder(radius=0.0065, length=0.006),
            origin=Origin(xyz=(0.026, y * 1.095, -0.887), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"nozzle_bolt_upper_{'neg' if y < 0 else 'pos'}",
        )
        wand.visual(
            Cylinder(radius=0.0065, length=0.006),
            origin=Origin(xyz=(-0.025, y * 1.095, -0.952), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"nozzle_bolt_lower_{'neg' if y < 0 else 'pos'}",
        )
    wand.visual(
        Cylinder(radius=0.012, length=0.184),
        origin=Origin(xyz=(0.0, 0.0, -0.920), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="nozzle_pin",
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.030, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="nozzle_barrel",
    )
    floor_head.visual(
        Box((0.064, 0.094, 0.072)),
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
        material=steel,
        name="nozzle_lug",
    )
    floor_head.visual(
        Box((0.330, 0.690, 0.060)),
        origin=Origin(xyz=(0.062, 0.0, -0.118)),
        material=dark,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.038, 0.700, 0.050)),
        origin=Origin(xyz=(0.228, 0.0, -0.095)),
        material=rubber,
        name="front_bumper",
    )
    floor_head.visual(
        Box((0.025, 0.690, 0.030)),
        origin=Origin(xyz=(-0.105, 0.0, -0.093)),
        material=rubber,
        name="rear_skid",
    )
    floor_head.visual(
        Box((0.270, 0.510, 0.012)),
        origin=Origin(xyz=(0.062, 0.0, -0.151)),
        material=steel,
        name="wear_plate",
    )
    for y in (-0.358, 0.358):
        floor_head.visual(
            Box((0.332, 0.030, 0.070)),
            origin=Origin(xyz=(0.064, y, -0.111)),
            material=safety,
            name=f"side_guard_{'neg' if y < 0 else 'pos'}",
        )
        floor_head.visual(
            Cylinder(radius=0.007, length=0.006),
            origin=Origin(xyz=(0.160, y * 0.975, -0.073), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"head_guard_bolt_front_{'neg' if y < 0 else 'pos'}",
        )
        floor_head.visual(
            Cylinder(radius=0.007, length=0.006),
            origin=Origin(xyz=(-0.055, y * 0.975, -0.073), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"head_guard_bolt_rear_{'neg' if y < 0 else 'pos'}",
        )
    floor_head.visual(
        Box((0.050, 0.160, 0.030)),
        origin=Origin(xyz=(-0.052, 0.0, -0.080)),
        material=safety,
        name="nozzle_travel_stop",
    )

    fold_joint = model.articulation(
        "body_to_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=1.75),
        motion_properties=MotionProperties(damping=1.2, friction=0.25),
    )
    fold_joint.meta["qc_samples"] = [0.0, 0.6, 1.2]

    model.articulation(
        "body_to_fold_lock",
        ArticulationType.PRISMATIC,
        parent=body,
        child=fold_lock,
        origin=Origin(xyz=(0.034, 0.104, 0.072)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.08, lower=0.0, upper=0.026),
        motion_properties=MotionProperties(damping=0.8, friction=0.35),
    )

    nozzle_joint = model.articulation(
        "wand_to_floor_head",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, 0.0, -0.920)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.4, lower=-0.55, upper=0.55),
        motion_properties=MotionProperties(damping=0.9, friction=0.12),
    )
    nozzle_joint.meta["qc_samples"] = [-0.45, 0.0, 0.45]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold_lock = object_model.get_part("fold_lock")
    fold_joint = object_model.get_articulation("body_to_wand")
    lock_joint = object_model.get_articulation("body_to_fold_lock")
    nozzle_joint = object_model.get_articulation("wand_to_floor_head")

    ctx.allow_overlap(
        body,
        wand,
        elem_a="fold_pin",
        elem_b="fold_barrel",
        reason="The visible fold hinge pin is intentionally captured through the wand barrel.",
    )
    ctx.expect_within(
        body,
        wand,
        axes="xz",
        inner_elem="fold_pin",
        outer_elem="fold_barrel",
        margin=0.003,
        name="fold pin centered in barrel",
    )
    ctx.expect_overlap(
        body,
        wand,
        axes="y",
        elem_a="fold_pin",
        elem_b="fold_barrel",
        min_overlap=0.090,
        name="fold pin spans wand barrel",
    )

    ctx.allow_overlap(
        wand,
        floor_head,
        elem_a="nozzle_pin",
        elem_b="nozzle_barrel",
        reason="The nozzle pitch pin is intentionally captured through the floor-head barrel.",
    )
    ctx.expect_within(
        wand,
        floor_head,
        axes="xz",
        inner_elem="nozzle_pin",
        outer_elem="nozzle_barrel",
        margin=0.003,
        name="nozzle pin centered in barrel",
    )
    ctx.expect_overlap(
        wand,
        floor_head,
        axes="y",
        elem_a="nozzle_pin",
        elem_b="nozzle_barrel",
        min_overlap=0.090,
        name="nozzle pin spans floor-head barrel",
    )

    rest_head = ctx.part_world_position(floor_head)
    with ctx.pose({fold_joint: 1.20}):
        folded_head = ctx.part_world_position(floor_head)
    ctx.check(
        "fold joint swings wand away from upright",
        rest_head is not None
        and folded_head is not None
        and folded_head[0] < rest_head[0] - 0.45
        and folded_head[2] > rest_head[2] + 0.25,
        details=f"rest={rest_head}, folded={folded_head}",
    )

    rest_lock = ctx.part_world_position(fold_lock)
    with ctx.pose({lock_joint: 0.026}):
        shifted_lock = ctx.part_world_position(fold_lock)
    ctx.check(
        "fold lockout slider has short positive travel",
        rest_lock is not None and shifted_lock is not None and shifted_lock[0] > rest_lock[0] + 0.020,
        details=f"rest={rest_lock}, shifted={shifted_lock}",
    )

    with ctx.pose({nozzle_joint: 0.45}):
        pitched_aabb = ctx.part_element_world_aabb(floor_head, elem="head_shell")
    with ctx.pose({nozzle_joint: 0.0}):
        flat_aabb = ctx.part_element_world_aabb(floor_head, elem="head_shell")
    ctx.check(
        "nozzle articulation pitches the wide floor head",
        flat_aabb is not None
        and pitched_aabb is not None
        and pitched_aabb[1][2] > flat_aabb[1][2] + 0.030,
        details=f"flat={flat_aabb}, pitched={pitched_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

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
    model = ArticulatedObject(name="compact_travel_metronome")

    body_mat = model.material("satin_black_plastic", rgba=(0.015, 0.018, 0.022, 1.0))
    trim_mat = model.material("brushed_brass", rgba=(0.86, 0.64, 0.27, 1.0))
    cream_mat = model.material("ivory_scale", rgba=(0.88, 0.82, 0.68, 1.0))
    mark_mat = model.material("black_print", rgba=(0.02, 0.018, 0.014, 1.0))
    steel_mat = model.material("dark_blued_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    red_mat = model.material("red_weight_insert", rgba=(0.62, 0.05, 0.035, 1.0))

    housing = model.part("housing")

    # Compact rectangular case: open at the front, solid back face, and a raised
    # front bezel.  The separate members intentionally overlap slightly inside
    # the same root link so the shell reads as one manufactured frame.
    housing.visual(
        Box((0.080, 0.004, 0.145)),
        origin=Origin(xyz=(0.0, 0.016, 0.0725)),
        material=body_mat,
        name="back_plate",
    )
    housing.visual(
        Box((0.007, 0.038, 0.145)),
        origin=Origin(xyz=(-0.037, 0.0, 0.0725)),
        material=body_mat,
        name="side_wall_0",
    )
    housing.visual(
        Box((0.007, 0.038, 0.145)),
        origin=Origin(xyz=(0.037, 0.0, 0.0725)),
        material=body_mat,
        name="side_wall_1",
    )
    housing.visual(
        Box((0.080, 0.038, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=body_mat,
        name="base_rail",
    )
    housing.visual(
        Box((0.080, 0.038, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=body_mat,
        name="top_rail",
    )

    housing.visual(
        Box((0.066, 0.003, 0.005)),
        origin=Origin(xyz=(0.0, -0.019, 0.132)),
        material=trim_mat,
        name="front_bezel_top",
    )
    housing.visual(
        Box((0.066, 0.003, 0.005)),
        origin=Origin(xyz=(0.0, -0.019, 0.016)),
        material=trim_mat,
        name="front_bezel_bottom",
    )
    housing.visual(
        Box((0.005, 0.003, 0.121)),
        origin=Origin(xyz=(-0.033, -0.019, 0.074)),
        material=trim_mat,
        name="front_bezel_side_0",
    )
    housing.visual(
        Box((0.005, 0.003, 0.121)),
        origin=Origin(xyz=(0.033, -0.019, 0.074)),
        material=trim_mat,
        name="front_bezel_side_1",
    )

    # Interior tempo scale on the back plate, visible behind the pendulum.
    housing.visual(
        Box((0.016, 0.0015, 0.095)),
        origin=Origin(xyz=(0.0, 0.0135, 0.074)),
        material=cream_mat,
        name="tempo_scale",
    )
    for i, z in enumerate([0.034, 0.046, 0.058, 0.070, 0.082, 0.094, 0.106, 0.118]):
        width = 0.012 if i % 2 == 0 else 0.008
        housing.visual(
            Box((width, 0.0012, 0.0015)),
            origin=Origin(xyz=(0.0, 0.0124, z)),
            material=mark_mat,
            name=f"tempo_tick_{i}",
        )

    # Yoke-style top bearing that visually supports the pendulum shaft.
    housing.visual(
        Box((0.026, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -0.009, 0.134)),
        material=steel_mat,
        name="pivot_bridge",
    )
    housing.visual(
        Box((0.004, 0.010, 0.020)),
        origin=Origin(xyz=(-0.005, -0.009, 0.123)),
        material=steel_mat,
        name="pivot_cheek_0",
    )
    housing.visual(
        Box((0.004, 0.010, 0.020)),
        origin=Origin(xyz=(0.005, -0.009, 0.123)),
        material=steel_mat,
        name="pivot_cheek_1",
    )

    # External hinge lugs for the fold-flat base stand.
    housing.visual(
        Box((0.014, 0.006, 0.010)),
        origin=Origin(xyz=(-0.031, -0.021, -0.001)),
        material=steel_mat,
        name="stand_hinge_lug_0",
    )
    housing.visual(
        Box((0.014, 0.006, 0.010)),
        origin=Origin(xyz=(0.031, -0.021, -0.001)),
        material=steel_mat,
        name="stand_hinge_lug_1",
    )

    pendulum_rod = model.part("pendulum_rod")
    pendulum_rod.visual(
        Cylinder(radius=0.0012, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, -0.054)),
        material=trim_mat,
        name="rod",
    )
    pendulum_rod.visual(
        Cylinder(radius=0.0030, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="pivot_hub",
    )

    pendulum_weight = model.part("pendulum_weight")
    pendulum_weight.visual(
        Cylinder(radius=0.013, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_mat,
        name="sliding_weight",
    )
    pendulum_weight.visual(
        Cylinder(radius=0.007, length=0.0068),
        origin=Origin(xyz=(0.0, -0.0005, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=red_mat,
        name="weight_insert",
    )

    stand = model.part("foot_stand")
    stand.visual(
        Cylinder(radius=0.0030, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mat,
        name="stand_barrel",
    )
    stand.visual(
        Box((0.052, 0.038, 0.004)),
        origin=Origin(xyz=(0.0, 0.017, -0.0038)),
        material=body_mat,
        name="stand_plate",
    )
    stand.visual(
        Box((0.040, 0.007, 0.0025)),
        origin=Origin(xyz=(0.0, 0.035, -0.0068)),
        material=trim_mat,
        name="rubber_foot",
    )

    winding_key = model.part("winding_key")
    winding_key.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="key_disk",
    )
    winding_key.visual(
        Box((0.022, 0.0030, 0.0045)),
        origin=Origin(xyz=(0.0, 0.0072, 0.0)),
        material=trim_mat,
        name="key_grip",
    )

    model.articulation(
        "housing_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum_rod,
        origin=Origin(xyz=(0.0, -0.010, 0.123)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=5.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "rod_to_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum_rod,
        child=pendulum_weight,
        origin=Origin(xyz=(0.0, 0.0, -0.043)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.045),
    )
    model.articulation(
        "housing_to_stand",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=stand,
        origin=Origin(xyz=(0.0, -0.021, -0.003)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0, lower=0.0, upper=1.10),
    )
    model.articulation(
        "housing_to_key",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=winding_key,
        origin=Origin(xyz=(0.0, 0.018, 0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rod = object_model.get_part("pendulum_rod")
    weight = object_model.get_part("pendulum_weight")
    stand = object_model.get_part("foot_stand")
    key = object_model.get_part("winding_key")
    swing = object_model.get_articulation("housing_to_pendulum")
    slide = object_model.get_articulation("rod_to_weight")
    stand_hinge = object_model.get_articulation("housing_to_stand")
    key_turn = object_model.get_articulation("housing_to_key")

    ctx.check(
        "primary metronome mechanisms are articulated",
        all((swing, slide, stand_hinge, key_turn)),
        details="Expected pendulum swing, sliding weight, hinged stand, and continuous winding key.",
    )

    ctx.allow_overlap(
        rod,
        weight,
        elem_a="rod",
        elem_b="sliding_weight",
        reason="The sliding weight is intentionally captured around the thin pendulum rod.",
    )
    ctx.expect_overlap(
        rod,
        weight,
        axes="xyz",
        elem_a="rod",
        elem_b="sliding_weight",
        min_overlap=0.001,
        name="sliding weight captures rod",
    )
    ctx.expect_within(
        weight,
        "housing",
        axes="xz",
        margin=0.004,
        name="sliding weight stays inside front opening silhouette",
    )
    ctx.expect_contact(
        key,
        "housing",
        elem_a="key_disk",
        elem_b="back_plate",
        contact_tol=0.001,
        name="winding key disk sits flush on back face",
    )

    rest_weight = ctx.part_world_position(weight)
    with ctx.pose({slide: 0.045}):
        low_weight = ctx.part_world_position(weight)
    ctx.check(
        "weight slides down the pendulum rod",
        rest_weight is not None
        and low_weight is not None
        and low_weight[2] < rest_weight[2] - 0.035,
        details=f"rest={rest_weight}, lower={low_weight}",
    )

    rest_rod_aabb = ctx.part_world_aabb(rod)
    with ctx.pose({swing: 0.40}):
        swung_rod_aabb = ctx.part_world_aabb(rod)
    if rest_rod_aabb is not None and swung_rod_aabb is not None:
        rest_center_x = (rest_rod_aabb[0][0] + rest_rod_aabb[1][0]) / 2.0
        swung_center_x = (swung_rod_aabb[0][0] + swung_rod_aabb[1][0]) / 2.0
        rod_sweeps = abs(swung_center_x - rest_center_x) > 0.010
    else:
        rod_sweeps = False
        rest_center_x = swung_center_x = None
    ctx.check(
        "pendulum swings laterally from top pivot",
        rod_sweeps,
        details=f"rest_x={rest_center_x}, swung_x={swung_center_x}",
    )

    folded_stand_aabb = ctx.part_world_aabb(stand)
    with ctx.pose({stand_hinge: 1.10}):
        deployed_stand_aabb = ctx.part_world_aabb(stand)
    if folded_stand_aabb is not None and deployed_stand_aabb is not None:
        stand_drops = deployed_stand_aabb[0][2] < folded_stand_aabb[0][2] - 0.018
    else:
        stand_drops = False
    ctx.check(
        "foot stand rotates down from folded base position",
        stand_drops,
        details=f"folded={folded_stand_aabb}, deployed={deployed_stand_aabb}",
    )

    key_grip_rest = ctx.part_element_world_aabb(key, elem="key_grip")
    with ctx.pose({key_turn: math.pi / 2.0}):
        key_grip_turned = ctx.part_element_world_aabb(key, elem="key_grip")
    if key_grip_rest is not None and key_grip_turned is not None:
        rest_x = key_grip_rest[1][0] - key_grip_rest[0][0]
        rest_z = key_grip_rest[1][2] - key_grip_rest[0][2]
        turned_x = key_grip_turned[1][0] - key_grip_turned[0][0]
        turned_z = key_grip_turned[1][2] - key_grip_turned[0][2]
        key_rotates = rest_x > rest_z and turned_z > turned_x
    else:
        key_rotates = False
        rest_x = rest_z = turned_x = turned_z = None
    ctx.check(
        "continuous winding key visibly turns",
        key_rotates,
        details=f"rest_xz=({rest_x}, {rest_z}), turned_xz=({turned_x}, {turned_z})",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="folding_stick_vacuum")

    charcoal = model.material("charcoal_plastic", rgba=(0.03, 0.035, 0.04, 1.0))
    dark = model.material("dark_rubber", rgba=(0.005, 0.006, 0.007, 1.0))
    red = model.material("red_accent", rgba=(0.75, 0.04, 0.025, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.75, 1.0))
    translucent = model.material("smoked_clear_bin", rgba=(0.62, 0.78, 0.88, 0.42))

    motor_body = model.part("motor_body")
    motor_body.visual(
        Box((0.34, 0.16, 0.18)),
        origin=Origin(xyz=(-0.17, 0.0, 0.68)),
        material=charcoal,
        name="motor_housing",
    )
    motor_body.visual(
        Cylinder(radius=0.075, length=0.30),
        origin=Origin(xyz=(-0.13, 0.0, 0.56), rpy=(0.0, pi / 2.0, 0.0)),
        material=translucent,
        name="dust_bin",
    )
    motor_body.visual(
        Box((0.07, 0.09, 0.36)),
        origin=Origin(xyz=(-0.44, 0.0, 0.62)),
        material=dark,
        name="rear_grip",
    )
    motor_body.visual(
        Box((0.24, 0.08, 0.055)),
        origin=Origin(xyz=(-0.32, 0.0, 0.785)),
        material=dark,
        name="upper_handle_strut",
    )
    motor_body.visual(
        Box((0.24, 0.08, 0.055)),
        origin=Origin(xyz=(-0.31, 0.0, 0.510)),
        material=dark,
        name="lower_handle_strut",
    )
    motor_body.visual(
        Box((0.065, 0.07, 0.095)),
        origin=Origin(xyz=(0.025, 0.0, 0.68)),
        material=red,
        name="fold_socket",
    )
    motor_body.visual(
        Cylinder(radius=0.045, length=0.09),
        origin=Origin(xyz=(0.055, 0.0, 0.68), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="fold_center_barrel",
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.024, length=0.92),
        origin=Origin(xyz=(0.62, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
        name="straight_tube",
    )
    wand.visual(
        Box((0.09, 0.17, 0.060)),
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        material=red,
        name="fold_collar",
    )
    wand.visual(
        Box((0.145, 0.035, 0.070)),
        origin=Origin(xyz=(0.087, -0.0725, 0.0)),
        material=red,
        name="fold_cheek_0",
    )
    wand.visual(
        Cylinder(radius=0.045, length=0.055),
        origin=Origin(xyz=(0.0, -0.0725, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="fold_outer_barrel_0",
    )
    wand.visual(
        Box((0.145, 0.035, 0.070)),
        origin=Origin(xyz=(0.087, 0.0725, 0.0)),
        material=red,
        name="fold_cheek_1",
    )
    wand.visual(
        Cylinder(radius=0.045, length=0.055),
        origin=Origin(xyz=(0.0, 0.0725, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="fold_outer_barrel_1",
    )
    wand.visual(
        Box((0.10, 0.15, 0.055)),
        origin=Origin(xyz=(1.095, 0.0, 0.0)),
        material=red,
        name="head_hinge_collar",
    )
    wand.visual(
        Box((0.115, 0.035, 0.062)),
        origin=Origin(xyz=(1.125, -0.0725, 0.0)),
        material=red,
        name="head_hinge_cheek_0",
    )
    wand.visual(
        Cylinder(radius=0.035, length=0.055),
        origin=Origin(xyz=(1.18, -0.0725, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="head_outer_barrel_0",
    )
    wand.visual(
        Box((0.115, 0.035, 0.062)),
        origin=Origin(xyz=(1.125, 0.0725, 0.0)),
        material=red,
        name="head_hinge_cheek_1",
    )
    wand.visual(
        Cylinder(radius=0.035, length=0.055),
        origin=Origin(xyz=(1.18, 0.0725, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="head_outer_barrel_1",
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.033, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="pitch_center_barrel",
    )
    floor_head.visual(
        Box((0.014, 0.038, 0.040)),
        origin=Origin(xyz=(0.034, 0.0, -0.016)),
        material=red,
        name="pitch_barrel_web",
    )
    floor_head.visual(
        Box((0.11, 0.08, 0.055)),
        origin=Origin(xyz=(0.09, 0.0, -0.022)),
        material=red,
        name="pitch_neck",
    )
    floor_head.visual(
        Box((0.40, 0.28, 0.070)),
        origin=Origin(xyz=(0.17, 0.0, -0.070)),
        material=charcoal,
        name="nozzle_shell",
    )
    floor_head.visual(
        Box((0.42, 0.30, 0.020)),
        origin=Origin(xyz=(0.17, 0.0, -0.115)),
        material=dark,
        name="rubber_sole",
    )
    floor_head.visual(
        Box((0.17, 0.20, 0.012)),
        origin=Origin(xyz=(0.18, 0.0, -0.030)),
        material=dark,
        name="intake_slot",
    )
    floor_head.visual(
        Cylinder(radius=0.035, length=0.29),
        origin=Origin(xyz=(0.33, 0.0, -0.080), rpy=(pi / 2.0, 0.0, 0.0)),
        material=red,
        name="front_brush_roll",
    )

    model.articulation(
        "fold_joint",
        ArticulationType.REVOLUTE,
        parent=motor_body,
        child=wand,
        origin=Origin(xyz=(0.055, 0.0, 0.68)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=0.0, upper=1.35),
    )
    model.articulation(
        "floor_pitch",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(1.18, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-0.65, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    motor_body = object_model.get_part("motor_body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold_joint = object_model.get_articulation("fold_joint")
    floor_pitch = object_model.get_articulation("floor_pitch")

    ctx.check(
        "fold joint is a horizontal revolute",
        fold_joint.articulation_type == ArticulationType.REVOLUTE
        and abs(fold_joint.axis[1]) > 0.99
        and abs(fold_joint.axis[0]) < 0.01
        and abs(fold_joint.axis[2]) < 0.01,
        details=f"type={fold_joint.articulation_type}, axis={fold_joint.axis}",
    )
    ctx.check(
        "floor head pitch is a horizontal revolute",
        floor_pitch.articulation_type == ArticulationType.REVOLUTE
        and abs(floor_pitch.axis[1]) > 0.99
        and abs(floor_pitch.axis[0]) < 0.01
        and abs(floor_pitch.axis[2]) < 0.01,
        details=f"type={floor_pitch.articulation_type}, axis={floor_pitch.axis}",
    )
    ctx.expect_origin_distance(
        wand,
        floor_head,
        axes="x",
        min_dist=1.0,
        name="fold and floor-pitch hinges are well separated by the long wand",
    )
    ctx.expect_overlap(
        motor_body,
        wand,
        axes="xz",
        elem_a="fold_center_barrel",
        elem_b="fold_outer_barrel_0",
        min_overlap=0.030,
        name="fold hinge barrels share the same horizontal hinge line",
    )
    ctx.expect_overlap(
        wand,
        floor_head,
        axes="xz",
        elem_a="head_outer_barrel_0",
        elem_b="pitch_center_barrel",
        min_overlap=0.025,
        name="floor-head hinge barrels share the same pitch line",
    )

    rest_head_pos = ctx.part_world_position(floor_head)
    with ctx.pose({fold_joint: 1.0}):
        folded_head_pos = ctx.part_world_position(floor_head)
    ctx.check(
        "fold joint lifts the long wand chain",
        rest_head_pos is not None
        and folded_head_pos is not None
        and folded_head_pos[2] > rest_head_pos[2] + 0.45,
        details=f"rest={rest_head_pos}, folded={folded_head_pos}",
    )

    with ctx.pose({floor_pitch: 0.45}):
        pitched_aabb = ctx.part_element_world_aabb(floor_head, elem="front_brush_roll")
    level_aabb = ctx.part_element_world_aabb(floor_head, elem="front_brush_roll")
    ctx.check(
        "floor head pitch visibly moves the nozzle front",
        level_aabb is not None
        and pitched_aabb is not None
        and pitched_aabb[0][2] < level_aabb[0][2] - 0.03,
        details=f"level={level_aabb}, pitched={pitched_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

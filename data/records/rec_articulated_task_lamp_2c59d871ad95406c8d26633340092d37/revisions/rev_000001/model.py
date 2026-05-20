from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RADIUS = 0.115
BASE_THICKNESS = 0.024
COLUMN_HEIGHT = 0.138
SHOULDER_Z = BASE_THICKNESS + COLUMN_HEIGHT + 0.003

LOWER_ARM_LENGTH = 0.315
UPPER_ARM_LENGTH = 0.285

ARM_RAIL_HEIGHT = 0.010
ARM_RAIL_WIDTH = 0.012
ARM_RAIL_SPACING = 0.042

HEAD_OUTER_RADIUS = 0.088
HEAD_INNER_RADIUS = 0.060
HEAD_DEPTH = 0.018
HEAD_CENTER_X = 0.060

COVER_OUTER_RADIUS = 0.083
COVER_INNER_RADIUS = 0.061
COVER_DEPTH = 0.006


def _arm_frame(
    *,
    length: float,
    body_width: float,
    body_height: float,
    end_block_width: float,
    end_block_height: float,
    name: str,
) -> tuple[object, str]:
    root_block = (
        cq.Workplane("XY")
        .box(0.016, body_width + 0.004, body_height + 0.004, centered=(False, True, True))
        .translate((0.0, 0.0, 0.0))
    )

    main_beam = (
        cq.Workplane("XY")
        .box(length - 0.010, body_width, body_height, centered=(False, True, True))
        .translate((0.010, 0.0, 0.0))
    )

    mid_rib = (
        cq.Workplane("XY")
        .box(0.020, body_width + 0.010, body_height + 0.004)
        .translate((length * 0.55, 0.0, 0.0))
    )

    end_block = (
        cq.Workplane("XY")
        .box(0.018, end_block_width, end_block_height, centered=(False, True, True))
        .translate((length - 0.018, 0.0, 0.0))
    )

    arm_shape = (
        root_block.union(main_beam)
        .union(mid_rib)
        .union(end_block)
    )
    return arm_shape, name


def _head_housing() -> object:
    ring = (
        cq.Workplane("YZ", origin=(HEAD_CENTER_X - HEAD_DEPTH / 2.0, 0.0, 0.0))
        .circle(HEAD_OUTER_RADIUS)
        .circle(HEAD_INNER_RADIUS)
        .extrude(HEAD_DEPTH)
    )

    lower_neck = (
        cq.Workplane("XY")
        .box(HEAD_CENTER_X - 0.004, 0.018, 0.018, centered=(False, True, True))
        .translate((0.0, 0.0, -0.068))
    )
    rear_post = (
        cq.Workplane("XY")
        .box(0.016, 0.016, 0.038, centered=(False, True, True))
        .translate((0.0, 0.0, -0.055))
    )
    root_pad = (
        cq.Workplane("XY")
        .box(0.012, 0.016, 0.030, centered=(False, True, True))
        .translate((0.0, 0.0, -0.028))
    )

    top_bridge = (
        cq.Workplane("XY")
        .box(0.026, 0.016, 0.014)
        .translate((HEAD_CENTER_X - 0.004, 0.0, HEAD_OUTER_RADIUS - 0.008))
    )

    knuckle_left = (
        cq.Workplane("XZ", origin=(HEAD_CENTER_X + HEAD_DEPTH / 2.0 - 0.006, 0.015, HEAD_OUTER_RADIUS - 0.002))
        .circle(0.004)
        .extrude(0.0045, both=True)
    )
    knuckle_right = (
        cq.Workplane("XZ", origin=(HEAD_CENTER_X + HEAD_DEPTH / 2.0 - 0.006, -0.015, HEAD_OUTER_RADIUS - 0.002))
        .circle(0.004)
        .extrude(0.0045, both=True)
    )

    housing = (
        ring.union(lower_neck)
        .union(rear_post)
        .union(root_pad)
        .union(top_bridge)
        .union(knuckle_left)
        .union(knuckle_right)
    )
    return housing


def _cover_frame() -> object:
    frame = (
        cq.Workplane("YZ", origin=(0.0, 0.0, -0.073))
        .circle(COVER_OUTER_RADIUS)
        .circle(COVER_INNER_RADIUS)
        .extrude(COVER_DEPTH)
    )

    top_stem = (
        cq.Workplane("XY")
        .box(0.028, 0.018, 0.014)
        .translate((-0.004, 0.0, -0.002))
    )

    return frame.union(top_stem)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((lo + hi) / 2.0 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="magnifier_task_lamp")

    base_finish = model.material("base_finish", rgba=(0.14, 0.15, 0.17, 1.0))
    arm_finish = model.material("arm_finish", rgba=(0.66, 0.68, 0.71, 1.0))
    housing_finish = model.material("housing_finish", rgba=(0.90, 0.91, 0.89, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.23, 0.24, 0.26, 1.0))
    glass = model.material("glass", rgba=(0.82, 0.90, 0.95, 0.40))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=base_finish,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=0.018, length=COLUMN_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + COLUMN_HEIGHT / 2.0 - 0.001)),
        material=base_finish,
        name="column",
    )
    base.visual(
        Box((0.030, 0.060, 0.016)),
        origin=Origin(xyz=(0.028, 0.0, SHOULDER_Z - 0.020)),
        material=base_finish,
        name="fork_bridge",
    )
    base.visual(
        Box((0.022, 0.010, 0.048)),
        origin=Origin(xyz=(0.028, 0.021, SHOULDER_Z)),
        material=base_finish,
        name="fork_left",
    )
    base.visual(
        Box((0.022, 0.010, 0.048)),
        origin=Origin(xyz=(0.028, -0.021, SHOULDER_Z)),
        material=base_finish,
        name="fork_right",
    )
    base.visual(
        Box((0.014, 0.018, 0.024)),
        origin=Origin(xyz=(0.036, 0.0, SHOULDER_Z)),
        material=base_finish,
        name="pivot_block",
    )

    lower_arm_shape, lower_name = _arm_frame(
        length=LOWER_ARM_LENGTH,
        body_width=0.014,
        body_height=0.011,
        end_block_width=0.022,
        end_block_height=0.022,
        name="lower_frame",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        mesh_from_cadquery(lower_arm_shape, lower_name),
        material=arm_finish,
        name="lower_frame",
    )

    upper_arm_shape, upper_name = _arm_frame(
        length=UPPER_ARM_LENGTH,
        body_width=0.013,
        body_height=0.010,
        end_block_width=0.020,
        end_block_height=0.020,
        name="upper_frame",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(upper_arm_shape, upper_name),
        material=arm_finish,
        name="upper_frame",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_housing(), "head_housing"),
        material=housing_finish,
        name="housing",
    )
    head.visual(
        Cylinder(radius=0.061, length=0.005),
        origin=Origin(
            xyz=(HEAD_CENTER_X + HEAD_DEPTH / 2.0 - 0.003, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=glass,
        name="lens",
    )
    head.visual(
        Cylinder(radius=0.071, length=0.004),
        origin=Origin(
            xyz=(HEAD_CENTER_X + HEAD_DEPTH / 2.0 - 0.001, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_finish,
        name="bezel",
    )

    cover = model.part("cover")
    cover.visual(
        mesh_from_cadquery(_cover_frame(), "cover_frame"),
        material=housing_finish,
        name="cover_frame",
    )
    cover.visual(
        Cylinder(radius=0.062, length=0.003),
        origin=Origin(
            xyz=(0.002, 0.0, -0.073),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=glass,
        name="cover_lens",
    )

    model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(0.039, 0.0, SHOULDER_Z), rpy=(0.0, math.radians(-58.0), 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.0,
            lower=math.radians(-42.0),
            upper=math.radians(34.0),
        ),
    )

    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0), rpy=(0.0, math.radians(22.0), 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=math.radians(-70.0),
            upper=math.radians(55.0),
        ),
    )

    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=head,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0), rpy=(0.0, math.radians(78.0), 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=math.radians(-20.0),
            upper=math.radians(85.0),
        ),
    )

    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=head,
        child=cover,
        origin=Origin(
            xyz=(HEAD_CENTER_X + HEAD_DEPTH / 2.0 + 0.017, 0.0, HEAD_OUTER_RADIUS - 0.002),
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(135.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    head = object_model.get_part("head")
    cover = object_model.get_part("cover")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")

    base_hinge = object_model.get_articulation("base_hinge")
    elbow_hinge = object_model.get_articulation("elbow_hinge")
    head_tilt = object_model.get_articulation("head_tilt")
    cover_hinge = object_model.get_articulation("cover_hinge")

    ctx.allow_overlap(
        "base",
        "lower_arm",
        reason="The base shoulder is simplified as a compact pivot block that slightly embeds the lower arm root.",
    )
    ctx.allow_overlap(
        "lower_arm",
        "upper_arm",
        reason="The elbow is represented with simplified overlapping hinge solids at the arm tips.",
    )
    ctx.allow_overlap(
        "head",
        "cover",
        reason="The closed magnifier cover is simplified as a nested front cap seated directly against the head ring.",
    )
    ctx.allow_overlap(
        "upper_arm",
        "head",
        reason="The head tilt joint uses a compact integrated yoke proxy at the arm tip.",
    )

    ctx.expect_overlap(
        cover,
        head,
        axes="yz",
        elem_a="cover_frame",
        elem_b="housing",
        min_overlap=0.120,
        name="cover aligns over the circular head",
    )

    head_rest = ctx.part_world_position(head)
    with ctx.pose({base_hinge: math.radians(20.0)}):
        head_raised = ctx.part_world_position(head)
    ctx.check(
        "base hinge raises the lamp head",
        head_rest is not None
        and head_raised is not None
        and head_raised[2] > head_rest[2] + 0.05,
        details=f"rest={head_rest}, raised={head_raised}",
    )

    head_rest = ctx.part_world_position(head)
    with ctx.pose({elbow_hinge: math.radians(32.0)}):
        elbow_pose = ctx.part_world_position(head)
    ctx.check(
        "elbow hinge lifts the head assembly",
        head_rest is not None
        and elbow_pose is not None
        and elbow_pose[2] > head_rest[2] + 0.03,
        details=f"rest={head_rest}, elbow_pose={elbow_pose}",
    )

    lens_rest = _aabb_center(ctx.part_element_world_aabb(head, elem="lens"))
    with ctx.pose({head_tilt: math.radians(28.0)}):
        lens_tilted = _aabb_center(ctx.part_element_world_aabb(head, elem="lens"))
    ctx.check(
        "head tilt swings the lens upward",
        lens_rest is not None
        and lens_tilted is not None
        and lens_tilted[2] > lens_rest[2] + 0.02,
        details=f"rest={lens_rest}, tilted={lens_tilted}",
    )

    cover_closed = ctx.part_element_world_aabb(cover, elem="cover_frame")
    with ctx.pose({cover_hinge: math.radians(100.0)}):
        cover_open = ctx.part_element_world_aabb(cover, elem="cover_frame")
    ctx.check(
        "cover flips away from the lens opening",
        cover_closed is not None
        and cover_open is not None
        and cover_open[1][0] > cover_closed[1][0] + 0.06,
        details=f"closed={cover_closed}, open={cover_open}",
    )

    return ctx.report()


object_model = build_object_model()

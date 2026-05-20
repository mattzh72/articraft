from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RADIUS = 0.19
BASE_THICKNESS = 0.055
SOCKET_RADIUS = 0.041
SOCKET_HEIGHT = 0.030
OUTER_SLEEVE_RADIUS = 0.021
OUTER_SLEEVE_INNER_RADIUS = 0.0156
OUTER_SLEEVE_HEIGHT = 0.510
TOP_COLLAR_HEIGHT = 0.040

INNER_MAST_RADIUS = 0.0138
INNER_MAST_LENGTH = 1.020
INNER_MAST_CENTER_Z = 0.130
MAST_SLIDE_MAX = 0.280

HEAD_AXIS_Z = 0.660
DESK_REST_TILT = math.radians(16.0)
DESK_WIDTH = 0.520
DESK_HEIGHT = 0.340


def _center_from_aabb(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def _build_base_body_shape() -> cq.Workplane:
    base_lower = cq.Workplane("XY").circle(BASE_RADIUS).extrude(0.030)
    base_upper = cq.Workplane("XY").circle(0.160).extrude(BASE_THICKNESS - 0.030).translate((0.0, 0.0, 0.030))
    socket = cq.Workplane("XY").circle(SOCKET_RADIUS).extrude(SOCKET_HEIGHT).translate((0.0, 0.0, BASE_THICKNESS))
    return base_lower.union(base_upper).union(socket)


def _build_outer_sleeve_shape() -> cq.Workplane:
    sleeve = (
        cq.Workplane("XY")
        .circle(OUTER_SLEEVE_RADIUS)
        .circle(OUTER_SLEEVE_INNER_RADIUS)
        .extrude(OUTER_SLEEVE_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS + SOCKET_HEIGHT))
    )
    top_collar = (
        cq.Workplane("XY")
        .circle(0.029)
        .circle(OUTER_SLEEVE_INNER_RADIUS)
        .extrude(TOP_COLLAR_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS + SOCKET_HEIGHT + OUTER_SLEEVE_HEIGHT - TOP_COLLAR_HEIGHT))
    )
    return sleeve.union(top_collar)


def _build_clamp_handle_shape() -> cq.Workplane:
    clamp_z = BASE_THICKNESS + SOCKET_HEIGHT + OUTER_SLEEVE_HEIGHT - 0.070
    stem = cq.Workplane("YZ").circle(0.0045).extrude(0.028).translate((0.021, 0.0, clamp_z))
    knob = cq.Workplane("YZ").circle(0.010).extrude(0.014).translate((0.042, 0.0, clamp_z))
    grip = cq.Workplane("YZ").box(0.010, 0.018, 0.018).translate((0.050, 0.0, clamp_z))
    return stem.union(knob).union(grip)


def _build_head_shape() -> cq.Workplane:
    collar = cq.Workplane("XY").circle(0.019).extrude(0.032).translate((0.0, 0.0, HEAD_AXIS_Z - 0.070))
    stem = cq.Workplane("XY").box(0.034, 0.034, 0.072).translate((0.0, 0.004, HEAD_AXIS_Z - 0.032))
    crossbar = cq.Workplane("XY").box(0.088, 0.010, 0.018).translate((0.0, 0.010, HEAD_AXIS_Z - 0.004))
    left_ear = cq.Workplane("XY").box(0.010, 0.026, 0.044).translate((-0.033, 0.032, HEAD_AXIS_Z))
    right_ear = cq.Workplane("XY").box(0.010, 0.026, 0.044).translate((0.033, 0.032, HEAD_AXIS_Z))
    left_strut = cq.Workplane("XY").box(0.010, 0.022, 0.018).translate((-0.033, 0.021, HEAD_AXIS_Z - 0.017))
    right_strut = cq.Workplane("XY").box(0.010, 0.022, 0.018).translate((0.033, 0.021, HEAD_AXIS_Z - 0.017))
    return collar.union(stem).union(crossbar).union(left_ear).union(right_ear).union(left_strut).union(right_strut)


def _build_desk_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(DESK_WIDTH, 0.006, DESK_HEIGHT).translate((0.0, 0.003, 0.085))
    side_left = cq.Workplane("XY").box(0.014, 0.020, DESK_HEIGHT).translate((-(DESK_WIDTH * 0.5) + 0.007, 0.010, 0.085))
    side_right = cq.Workplane("XY").box(0.014, 0.020, DESK_HEIGHT).translate(((DESK_WIDTH * 0.5) - 0.007, 0.010, 0.085))
    top_rim = cq.Workplane("XY").box(DESK_WIDTH, 0.020, 0.014).translate((0.0, 0.010, 0.248))
    lower_return = cq.Workplane("XY").box(0.480, 0.020, 0.018).translate((0.0, 0.010, -0.076))
    score_shelf = cq.Workplane("XY").box(0.472, 0.058, 0.006).translate((0.0, 0.035, -0.085))
    score_fence = cq.Workplane("XY").box(0.472, 0.010, 0.018).translate((0.0, 0.060, -0.073))
    trunnion = cq.Workplane("YZ").circle(0.0065).extrude(0.026, both=True)
    bracket = cq.Workplane("XY").box(0.082, 0.020, 0.050).translate((0.0, -0.010, 0.000))
    brace = cq.Workplane("XY").box(0.086, 0.018, 0.120).translate((0.0, -0.006, 0.040))
    clip_pad_left = cq.Workplane("XY").box(0.026, 0.014, 0.012).translate((-0.235, 0.010, 0.244))
    clip_pad_right = cq.Workplane("XY").box(0.026, 0.014, 0.012).translate((0.235, 0.010, 0.244))
    return (
        panel.union(side_left)
        .union(side_right)
        .union(top_rim)
        .union(lower_return)
        .union(score_shelf)
        .union(score_fence)
        .union(trunnion)
        .union(bracket)
        .union(brace)
        .union(clip_pad_left)
        .union(clip_pad_right)
    )


def _build_clip_shape() -> cq.Workplane:
    barrel = cq.Workplane("YZ").circle(0.0042).extrude(0.008, both=True)
    arm = cq.Workplane("XY").box(0.012, 0.004, 0.056).translate((0.0, 0.004, -0.030))
    finger = cq.Workplane("XY").box(0.022, 0.006, 0.010).translate((0.0, 0.006, -0.055))
    return barrel.union(arm).union(finger)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="symphony_music_stand")

    cast_black = model.material("cast_black", rgba=(0.12, 0.12, 0.13, 1.0))
    tube_black = model.material("tube_black", rgba=(0.18, 0.18, 0.19, 1.0))
    desk_gray = model.material("desk_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    clip_gray = model.material("clip_gray", rgba=(0.21, 0.22, 0.24, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_body_shape(), "stand_base_body"),
        material=cast_black,
        name="base_body",
    )
    base.visual(
        mesh_from_cadquery(_build_outer_sleeve_shape(), "stand_outer_sleeve"),
        material=tube_black,
        name="outer_sleeve",
    )
    base.visual(
        mesh_from_cadquery(_build_clamp_handle_shape(), "stand_clamp_handle"),
        material=cast_black,
        name="clamp_handle",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.380, 0.380, 0.635)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.317)),
    )

    upper_mast = model.part("upper_mast")
    upper_mast.visual(
        Cylinder(radius=INNER_MAST_RADIUS, length=INNER_MAST_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, INNER_MAST_CENTER_Z)),
        material=tube_black,
        name="inner_mast",
    )
    upper_mast.visual(
        Cylinder(radius=0.024, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=tube_black,
        name="stop_collar",
    )
    upper_mast.visual(
        mesh_from_cadquery(_build_head_shape(), "stand_head_yoke"),
        material=tube_black,
        name="head_yoke",
    )
    upper_mast.inertial = Inertial.from_geometry(
        Box((0.100, 0.060, 1.050)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
    )

    model.articulation(
        "base_to_upper_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_mast,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + SOCKET_HEIGHT + OUTER_SLEEVE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.18,
            lower=0.0,
            upper=MAST_SLIDE_MAX,
        ),
    )

    desk = model.part("desk")
    desk.visual(
        mesh_from_cadquery(_build_desk_shape(), "stand_desk_panel"),
        material=desk_gray,
        name="desk_panel",
    )
    desk.inertial = Inertial.from_geometry(
        Box((0.540, 0.100, 0.360)),
        mass=2.3,
        origin=Origin(xyz=(0.0, 0.010, 0.080)),
    )

    model.articulation(
        "mast_to_desk",
        ArticulationType.REVOLUTE,
        parent=upper_mast,
        child=desk,
        origin=Origin(xyz=(0.0, 0.032, HEAD_AXIS_Z), rpy=(DESK_REST_TILT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.1,
            lower=math.radians(-10.0),
            upper=math.radians(32.0),
        ),
    )

    for index, x_pos in enumerate((-0.235, 0.235)):
        clip = model.part(f"clip_{index}")
        clip.visual(
            mesh_from_cadquery(_build_clip_shape(), f"stand_clip_{index}"),
            material=clip_gray,
            name="clip_arm",
        )
        clip.inertial = Inertial.from_geometry(
            Box((0.024, 0.012, 0.060)),
            mass=0.04,
            origin=Origin(xyz=(0.0, 0.004, -0.028)),
        )
        model.articulation(
            f"desk_to_clip_{index}",
            ArticulationType.REVOLUTE,
            parent=desk,
            child=clip,
            origin=Origin(xyz=(x_pos, 0.021, 0.244)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.2,
                velocity=2.5,
                lower=math.radians(-6.0),
                upper=math.radians(55.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    upper_mast = object_model.get_part("upper_mast")
    desk = object_model.get_part("desk")
    clip_0 = object_model.get_part("clip_0")
    clip_1 = object_model.get_part("clip_1")

    mast_slide = object_model.get_articulation("base_to_upper_mast")
    desk_tilt = object_model.get_articulation("mast_to_desk")
    clip_joint_0 = object_model.get_articulation("desk_to_clip_0")
    clip_joint_1 = object_model.get_articulation("desk_to_clip_1")

    ctx.allow_overlap(
        desk,
        upper_mast,
        elem_a="desk_panel",
        elem_b="head_yoke",
        reason="The desk trunnion is represented as a solid tilt pivot captured inside the mast yoke.",
    )
    ctx.allow_overlap(
        clip_0,
        desk,
        elem_a="clip_arm",
        elem_b="desk_panel",
        reason="The page-retainer clip barrel is modeled as a simplified pivot sitting in the desk's corner mount.",
    )
    ctx.allow_overlap(
        clip_1,
        desk,
        elem_a="clip_arm",
        elem_b="desk_panel",
        reason="The page-retainer clip barrel is modeled as a simplified pivot sitting in the desk's corner mount.",
    )

    mast_limits = mast_slide.motion_limits
    if mast_limits is not None and mast_limits.upper is not None:
        ctx.expect_within(
            upper_mast,
            base,
            axes="xy",
            inner_elem="inner_mast",
            outer_elem="outer_sleeve",
            margin=0.0015,
            name="inner mast stays centered in the lower sleeve at rest",
        )
        ctx.expect_overlap(
            upper_mast,
            base,
            axes="z",
            elem_a="inner_mast",
            elem_b="outer_sleeve",
            min_overlap=0.300,
            name="inner mast remains deeply inserted at rest",
        )

        rest_pos = ctx.part_world_position(upper_mast)
        with ctx.pose({mast_slide: mast_limits.upper}):
            ctx.expect_within(
                upper_mast,
                base,
                axes="xy",
                inner_elem="inner_mast",
                outer_elem="outer_sleeve",
                margin=0.0015,
                name="inner mast stays centered in the lower sleeve when extended",
            )
            ctx.expect_overlap(
                upper_mast,
                base,
                axes="z",
                elem_a="inner_mast",
                elem_b="outer_sleeve",
                min_overlap=0.095,
                name="inner mast retains insertion at maximum extension",
            )
            extended_pos = ctx.part_world_position(upper_mast)

        ctx.check(
            "mast extends upward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[2] > rest_pos[2] + 0.20,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    tilt_limits = desk_tilt.motion_limits
    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        with ctx.pose({desk_tilt: tilt_limits.lower}):
            upright_aabb = ctx.part_element_world_aabb(desk, elem="desk_panel")
        with ctx.pose({desk_tilt: tilt_limits.upper}):
            tilted_aabb = ctx.part_element_world_aabb(desk, elem="desk_panel")

        upright_center = _center_from_aabb(upright_aabb)
        tilted_center = _center_from_aabb(tilted_aabb)
        ctx.check(
            "desk tilts backward at upper limit",
            upright_center is not None
            and tilted_center is not None
            and tilted_center[1] < upright_center[1] - 0.025,
            details=f"upright={upright_center}, tilted={tilted_center}",
        )

    for clip_part, clip_joint, index in (
        (clip_0, clip_joint_0, 0),
        (clip_1, clip_joint_1, 1),
    ):
        ctx.expect_contact(
            clip_part,
            desk,
            contact_tol=0.0015,
            name=f"clip {index} sits on the desk rim",
        )

        limits = clip_joint.motion_limits
        if limits is not None and limits.upper is not None:
            rest_aabb = ctx.part_element_world_aabb(clip_part, elem="clip_arm")
            with ctx.pose({clip_joint: limits.upper}):
                open_aabb = ctx.part_element_world_aabb(clip_part, elem="clip_arm")

            rest_center = _center_from_aabb(rest_aabb)
            open_center = _center_from_aabb(open_aabb)
            ctx.check(
                f"clip {index} pivots forward when opened",
                rest_center is not None
                and open_center is not None
                and open_center[1] > rest_center[1] + 0.008,
                details=f"rest={rest_center}, open={open_center}",
            )

    return ctx.report()


object_model = build_object_model()

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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


DESK_ROLL = 0.18
CLIP_POSITIONS = (-0.165, 0.165)


def _hub_shell():
    sleeve = cq.Workplane("XY").circle(0.0175).circle(0.0135).extrude(0.38)
    lower_collar = cq.Workplane("XY").circle(0.030).circle(0.0135).extrude(0.07)
    top_socket = (
        cq.Workplane("XY")
        .circle(0.0215)
        .circle(0.0135)
        .extrude(0.08)
        .translate((0.0, 0.0, 0.30))
    )
    return sleeve.union(lower_collar).union(top_socket)


def _tripod_leg_mesh(angle: float):
    c = math.cos(angle)
    s = math.sin(angle)
    return tube_from_spline_points(
        [
            (0.028 * c, 0.028 * s, 0.145),
            (0.165 * c, 0.165 * s, 0.085),
            (0.350 * c, 0.350 * s, 0.018),
        ],
        radius=0.010,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )


def _support_arm_mesh():
    return tube_from_spline_points(
        [
            (0.0, 0.0, 0.016),
            (0.0, -0.020, 0.040),
            (0.0, -0.050, 0.074),
            (0.0, -0.070, 0.104),
        ],
        radius=0.008,
        samples_per_segment=18,
        radial_segments=16,
        cap_ends=True,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="school_orchestra_stand")

    powder_black = model.material("powder_black", rgba=(0.14, 0.14, 0.15, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.64, 0.67, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_hub_shell(), "tripod_hub_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=powder_black,
        name="hub_sleeve",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(xyz=(0.031, 0.0, 0.405), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="clamp_stem",
    )
    base.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.047, 0.0, 0.405)),
        material=rubber,
        name="clamp_knob",
    )

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        base.visual(
            mesh_from_geometry(_tripod_leg_mesh(angle), f"tripod_leg_{index}"),
            material=powder_black,
            name=f"leg_{index}",
        )
        base.visual(
            Sphere(radius=0.012),
            origin=Origin(xyz=(0.350 * c, 0.350 * s, 0.018)),
            material=rubber,
            name=f"foot_{index}",
        )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.0115, length=0.88),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=graphite,
        name="mast_tube",
    )
    mast.visual(
        Cylinder(radius=0.0155, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.535)),
        material=powder_black,
        name="mast_collar",
    )
    mast.visual(
        Cylinder(radius=0.020, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=powder_black,
        name="stop_collar",
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=0.0,
            upper=0.22,
        ),
    )

    head = model.part("head")
    head.visual(
        Box((0.060, 0.048, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=powder_black,
        name="receiver_block",
    )
    head.visual(
        mesh_from_geometry(_support_arm_mesh(), "tilt_head_arm"),
        material=powder_black,
        name="support_arm",
    )
    head.visual(
        Box((0.084, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, -0.065, 0.104)),
        material=powder_black,
        name="yoke_crossbar",
    )
    head.visual(
        Box((0.010, 0.020, 0.054)),
        origin=Origin(xyz=(-0.032, -0.050, 0.130)),
        material=powder_black,
        name="yoke_ear_0",
    )
    head.visual(
        Box((0.010, 0.020, 0.054)),
        origin=Origin(xyz=(0.032, -0.050, 0.130)),
        material=powder_black,
        name="yoke_ear_1",
    )
    head.visual(
        Cylinder(radius=0.011, length=0.020),
        origin=Origin(xyz=(0.042, -0.050, 0.130), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="tilt_knob",
    )
    head.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(-0.042, -0.050, 0.130), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="tilt_washer",
    )

    model.articulation(
        "mast_to_head",
        ArticulationType.FIXED,
        parent=mast,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
    )

    desk = model.part("desk")
    desk.visual(
        Cylinder(radius=0.009, length=0.054),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_black,
        name="hinge_barrel",
    )
    desk.visual(
        Box((0.050, 0.030, 0.060)),
        origin=Origin(xyz=(0.0, 0.010, 0.030)),
        material=powder_black,
        name="hinge_block",
    )
    desk.visual(
        Box((0.540, 0.006, 0.400)),
        origin=Origin(xyz=(0.0, 0.010, 0.206), rpy=(DESK_ROLL, 0.0, 0.0)),
        material=graphite,
        name="panel",
    )
    desk.visual(
        Box((0.100, 0.020, 0.240)),
        origin=Origin(xyz=(0.0, 0.016, 0.182), rpy=(DESK_ROLL, 0.0, 0.0)),
        material=powder_black,
        name="center_reinforcement",
    )
    desk.visual(
        Box((0.014, 0.022, 0.400)),
        origin=Origin(xyz=(-0.263, 0.011, 0.206), rpy=(DESK_ROLL, 0.0, 0.0)),
        material=graphite,
        name="side_flange_0",
    )
    desk.visual(
        Box((0.014, 0.022, 0.400)),
        origin=Origin(xyz=(0.263, 0.011, 0.206), rpy=(DESK_ROLL, 0.0, 0.0)),
        material=graphite,
        name="side_flange_1",
    )
    desk.visual(
        Box((0.540, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.028, 0.401), rpy=(DESK_ROLL, 0.0, 0.0)),
        material=graphite,
        name="top_rim",
    )
    desk.visual(
        Box((0.500, 0.022, 0.036)),
        origin=Origin(xyz=(0.0, 0.034, 0.062), rpy=(0.10, 0.0, 0.0)),
        material=powder_black,
        name="lower_rail",
    )
    desk.visual(
        Box((0.500, 0.060, 0.012)),
        origin=Origin(xyz=(0.0, 0.060, 0.050), rpy=(0.04, 0.0, 0.0)),
        material=graphite,
        name="shelf",
    )
    desk.visual(
        Box((0.500, 0.008, 0.024)),
        origin=Origin(xyz=(0.0, 0.086, 0.060), rpy=(0.04, 0.0, 0.0)),
        material=graphite,
        name="shelf_fence",
    )

    for index, clip_x in enumerate(CLIP_POSITIONS):
        desk.visual(
            Box((0.030, 0.012, 0.010)),
            origin=Origin(xyz=(clip_x, -0.031, 0.403), rpy=(DESK_ROLL, 0.0, 0.0)),
            material=powder_black,
            name=f"clip_mount_{index}",
        )
        desk.visual(
            Box((0.020, 0.030, 0.010)),
            origin=Origin(xyz=(clip_x, -0.022, 0.4025), rpy=(DESK_ROLL, 0.0, 0.0)),
            material=powder_black,
            name=f"clip_bracket_{index}",
        )

    model.articulation(
        "head_to_desk",
        ArticulationType.REVOLUTE,
        parent=head,
        child=desk,
        origin=Origin(xyz=(0.0, -0.050, 0.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=-0.30,
            upper=0.55,
        ),
    )

    for index, clip_x in enumerate(CLIP_POSITIONS):
        clip = model.part(f"clip_{index}")
        clip.visual(
            Cylinder(radius=0.0045, length=0.016),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="pivot_barrel",
        )
        clip.visual(
            Box((0.018, 0.020, 0.014)),
            origin=Origin(xyz=(0.0, 0.010, -0.007)),
            material=steel,
            name="bridge",
        )
        clip.visual(
            Box((0.024, 0.010, 0.080)),
            origin=Origin(xyz=(0.0, 0.020, -0.040)),
            material=steel,
            name="leaf",
        )
        clip.visual(
            Box((0.018, 0.010, 0.010)),
            origin=Origin(xyz=(0.0, 0.023, -0.079)),
            material=steel,
            name="tip",
        )

        model.articulation(
            f"desk_to_clip_{index}",
            ArticulationType.REVOLUTE,
            parent=desk,
            child=clip,
            origin=Origin(xyz=(clip_x, -0.011, 0.412), rpy=(DESK_ROLL, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=3.0,
                lower=0.0,
                upper=0.95,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    head = object_model.get_part("head")
    desk = object_model.get_part("desk")
    clip_0 = object_model.get_part("clip_0")
    clip_1 = object_model.get_part("clip_1")

    mast_joint = object_model.get_articulation("base_to_mast")
    desk_joint = object_model.get_articulation("head_to_desk")
    clip_0_joint = object_model.get_articulation("desk_to_clip_0")

    mast_limits = mast_joint.motion_limits
    if mast_limits is not None and mast_limits.upper is not None:
        ctx.expect_within(
            mast,
            base,
            axes="xy",
            inner_elem="mast_tube",
            outer_elem="hub_sleeve",
            margin=0.002,
            name="mast stays centered in the tripod sleeve at rest",
        )
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="mast_tube",
            elem_b="hub_sleeve",
            min_overlap=0.30,
            name="mast remains deeply inserted at rest",
        )

        mast_rest = ctx.part_world_position(mast)
        with ctx.pose({mast_joint: mast_limits.upper}):
            ctx.expect_within(
                mast,
                base,
                axes="xy",
                inner_elem="mast_tube",
                outer_elem="hub_sleeve",
                margin=0.002,
                name="mast stays centered when extended",
            )
            ctx.expect_overlap(
                mast,
                base,
                axes="z",
                elem_a="mast_tube",
                elem_b="hub_sleeve",
                min_overlap=0.10,
                name="mast retains insertion at full height",
            )
            mast_extended = ctx.part_world_position(mast)

        ctx.check(
            "mast extends upward",
            mast_rest is not None
            and mast_extended is not None
            and mast_extended[2] > mast_rest[2] + 0.18,
            details=f"rest={mast_rest}, extended={mast_extended}",
        )

    panel_aabb = ctx.part_element_world_aabb(desk, elem="panel")
    crossbar_aabb = ctx.part_element_world_aabb(head, elem="yoke_crossbar")
    panel_center = _aabb_center(panel_aabb)
    crossbar_center = _aabb_center(crossbar_aabb)
    ctx.check(
        "tilt head stays visibly behind the desk",
        panel_center is not None
        and crossbar_center is not None
        and crossbar_center[1] < panel_center[1] - 0.02,
        details=f"panel_center={panel_center}, crossbar_center={crossbar_center}",
    )

    desk_limits = desk_joint.motion_limits
    if desk_limits is not None and desk_limits.upper is not None:
        rest_panel = ctx.part_element_world_aabb(desk, elem="panel")
        with ctx.pose({desk_joint: desk_limits.upper}):
            tilted_panel = ctx.part_element_world_aabb(desk, elem="panel")

        rest_center = _aabb_center(rest_panel)
        tilted_center = _aabb_center(tilted_panel)
        ctx.check(
            "desk tilts through a visible range",
            rest_center is not None
            and tilted_center is not None
            and tilted_center[1] < rest_center[1] - 0.04,
            details=f"rest_center={rest_center}, tilted_center={tilted_center}",
        )

    ctx.expect_origin_distance(
        clip_0,
        clip_1,
        axes="x",
        min_dist=0.28,
        name="page clips stay distinct across the top edge",
    )
    ctx.expect_within(
        clip_0,
        desk,
        axes="x",
        inner_elem="leaf",
        outer_elem="panel",
        margin=0.03,
        name="left clip stays within the desk width",
    )
    ctx.expect_within(
        clip_1,
        desk,
        axes="x",
        inner_elem="leaf",
        outer_elem="panel",
        margin=0.03,
        name="right clip stays within the desk width",
    )

    clip_limits = clip_0_joint.motion_limits
    if clip_limits is not None and clip_limits.upper is not None:
        clip_rest = ctx.part_element_world_aabb(clip_0, elem="leaf")
        with ctx.pose({clip_0_joint: clip_limits.upper}):
            clip_open = ctx.part_element_world_aabb(clip_0, elem="leaf")

        clip_rest_center = _aabb_center(clip_rest)
        clip_open_center = _aabb_center(clip_open)
        ctx.check(
            "page clip pivots outward",
            clip_rest_center is not None
            and clip_open_center is not None
            and clip_open_center[1] > clip_rest_center[1] + 0.015,
            details=f"rest_center={clip_rest_center}, open_center={clip_open_center}",
        )

    return ctx.report()


object_model = build_object_model()

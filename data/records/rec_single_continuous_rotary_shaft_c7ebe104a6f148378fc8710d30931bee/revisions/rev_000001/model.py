from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


FRAME_LENGTH = 0.62
RAIL_CENTER_Y = 0.11
RAIL_WIDTH = 0.042
RAIL_HEIGHT = 0.038
CROSS_TIE_WIDTH = 0.23
CROSS_TIE_HEIGHT = 0.024

AXIS_HEIGHT = 0.165
SUPPORT_BASE_Z = RAIL_HEIGHT + CROSS_TIE_HEIGHT - 0.004

GUIDE_STATION_X = 0.23
GUIDE_THICKNESS = 0.032
GUIDE_WIDTH = 0.15
GUIDE_HEIGHT = 0.20
GUIDE_WINDOW_WIDTH = 0.082
GUIDE_WINDOW_HEIGHT = 0.10

CENTER_THICKNESS = 0.084
CENTER_WIDTH = 0.178
CENTER_BLOCK_HEIGHT = 0.212
CENTER_BOSS_RADIUS = 0.055
CENTER_WINDOW_WIDTH = 0.072
CENTER_WINDOW_HEIGHT = 0.082

SHAFT_RADIUS = 0.014
GUIDE_BORE_RADIUS = SHAFT_RADIUS + 0.0025
CENTER_BORE_RADIUS = SHAFT_RADIUS + 0.0032
SHAFT_LENGTH = 0.74

COLLAR_RADIUS = 0.021
COLLAR_THICKNESS = 0.008
COLLAR_OFFSET = GUIDE_STATION_X + GUIDE_THICKNESS / 2.0 + COLLAR_THICKNESS / 2.0 + 0.004

FLAG_HUB_X = 0.305
FLAG_HUB_RADIUS = 0.024
FLAG_HUB_THICKNESS = 0.016
FLAG_STEM_X = 0.012
FLAG_STEM_Y = 0.012
FLAG_STEM_HEIGHT = 0.068
FLAG_PADDLE_X = 0.012
FLAG_PADDLE_Y = 0.044
FLAG_PADDLE_HEIGHT = 0.052


def _build_base_frame_shape() -> cq.Workplane:
    frame = cq.Workplane("XY")

    for rail_y in (-RAIL_CENTER_Y, RAIL_CENTER_Y):
        rail = cq.Workplane("XY").center(0.0, rail_y).box(
            FRAME_LENGTH,
            RAIL_WIDTH,
            RAIL_HEIGHT,
            centered=(True, True, False),
        )
        frame = frame.union(rail)

    for tie_x, tie_thickness in (
        (-0.285, 0.05),
        (-GUIDE_STATION_X, 0.055),
        (0.0, 0.095),
        (GUIDE_STATION_X, 0.055),
        (0.285, 0.05),
    ):
        tie = (
            cq.Workplane("XY")
            .center(tie_x, 0.0)
            .box(
                tie_thickness,
                CROSS_TIE_WIDTH,
                CROSS_TIE_HEIGHT,
                centered=(True, True, False),
            )
            .translate((0.0, 0.0, RAIL_HEIGHT - 0.004))
        )
        frame = frame.union(tie)

    return frame


def _build_end_guide_shape(x_station: float) -> cq.Workplane:
    profile_plane = cq.Workplane("YZ", origin=(x_station, 0.0, 0.0))

    guide = profile_plane.center(0.0, SUPPORT_BASE_Z + GUIDE_HEIGHT / 2.0).rect(
        GUIDE_WIDTH,
        GUIDE_HEIGHT,
    ).extrude(GUIDE_THICKNESS / 2.0, both=True)

    window_center_z = SUPPORT_BASE_Z + 0.024 + GUIDE_WINDOW_HEIGHT / 2.0
    guide = guide.cut(
        profile_plane.center(0.0, window_center_z).rect(
            GUIDE_WINDOW_WIDTH,
            GUIDE_WINDOW_HEIGHT,
        ).extrude(GUIDE_THICKNESS, both=True)
    )

    guide = guide.cut(
        profile_plane.center(0.0, AXIS_HEIGHT)
        .circle(GUIDE_BORE_RADIUS)
        .extrude(GUIDE_THICKNESS, both=True)
    )

    return guide


def _build_center_bearing_shape() -> cq.Workplane:
    profile_plane = cq.Workplane("YZ", origin=(0.0, 0.0, 0.0))

    center_body = profile_plane.center(
        0.0,
        SUPPORT_BASE_Z + CENTER_BLOCK_HEIGHT / 2.0,
    ).rect(
        CENTER_WIDTH,
        CENTER_BLOCK_HEIGHT,
    ).extrude(
        CENTER_THICKNESS / 2.0,
        both=True,
    )

    boss = profile_plane.center(0.0, AXIS_HEIGHT).circle(CENTER_BOSS_RADIUS).extrude(
        CENTER_THICKNESS / 2.0,
        both=True,
    )
    center_body = center_body.union(boss)

    window_center_z = SUPPORT_BASE_Z + 0.026 + CENTER_WINDOW_HEIGHT / 2.0
    center_body = center_body.cut(
        profile_plane.center(0.0, window_center_z).rect(
            CENTER_WINDOW_WIDTH,
            CENTER_WINDOW_HEIGHT,
        ).extrude(CENTER_THICKNESS, both=True)
    )

    center_body = center_body.cut(
        profile_plane.center(0.0, AXIS_HEIGHT)
        .circle(CENTER_BORE_RADIUS)
        .extrude(CENTER_THICKNESS, both=True)
    )

    return center_body


def _build_shaft_shape() -> cq.Workplane:
    return cq.Workplane("YZ").circle(SHAFT_RADIUS).extrude(SHAFT_LENGTH / 2.0, both=True)


def _build_collar_shape(x_center: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ", origin=(x_center, 0.0, 0.0))
        .circle(COLLAR_RADIUS)
        .extrude(COLLAR_THICKNESS / 2.0, both=True)
    )


def _build_phase_flag_shape() -> cq.Workplane:
    hub = (
        cq.Workplane("YZ", origin=(FLAG_HUB_X, 0.0, 0.0))
        .circle(FLAG_HUB_RADIUS)
        .extrude(FLAG_HUB_THICKNESS / 2.0, both=True)
    )
    stem = (
        cq.Workplane("XY")
        .center(FLAG_HUB_X, 0.0)
        .box(
            FLAG_STEM_X,
            FLAG_STEM_Y,
            FLAG_STEM_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, SHAFT_RADIUS - 0.001))
    )
    paddle = (
        cq.Workplane("XY")
        .center(FLAG_HUB_X, 0.0)
        .box(
            FLAG_PADDLE_X,
            FLAG_PADDLE_Y,
            FLAG_PADDLE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, SHAFT_RADIUS + FLAG_STEM_HEIGHT - 0.004))
    )
    return hub.union(stem).union(paddle)


def _center_from_aabb(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="center_bearing_line_shaft_module")

    model.material("frame_paint", rgba=(0.18, 0.21, 0.24, 1.0))
    model.material("guide_casting", rgba=(0.56, 0.59, 0.62, 1.0))
    model.material("bearing_housing", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("shaft_steel", rgba=(0.79, 0.80, 0.82, 1.0))
    model.material("collar_steel", rgba=(0.46, 0.48, 0.50, 1.0))
    model.material("indicator_paint", rgba=(0.80, 0.38, 0.14, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_base_frame_shape(), "base_frame"),
        material="frame_paint",
        name="base_frame",
    )
    frame.visual(
        mesh_from_cadquery(_build_end_guide_shape(-GUIDE_STATION_X), "left_guide"),
        material="guide_casting",
        name="left_guide",
    )
    frame.visual(
        mesh_from_cadquery(_build_center_bearing_shape(), "center_bearing"),
        material="bearing_housing",
        name="center_bearing",
    )
    frame.visual(
        mesh_from_cadquery(_build_end_guide_shape(GUIDE_STATION_X), "right_guide"),
        material="guide_casting",
        name="right_guide",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_LENGTH, CROSS_TIE_WIDTH, AXIS_HEIGHT + 0.10)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, (AXIS_HEIGHT + 0.10) / 2.0)),
    )

    shaft = model.part("shaft")
    shaft.visual(
        mesh_from_cadquery(_build_shaft_shape(), "shaft_body"),
        material="shaft_steel",
        name="shaft_body",
    )
    shaft.visual(
        mesh_from_cadquery(_build_collar_shape(-COLLAR_OFFSET), "left_collar"),
        material="collar_steel",
        name="left_collar",
    )
    shaft.visual(
        mesh_from_cadquery(_build_collar_shape(COLLAR_OFFSET), "right_collar"),
        material="collar_steel",
        name="right_collar",
    )
    shaft.visual(
        mesh_from_cadquery(_build_phase_flag_shape(), "phase_flag"),
        material="indicator_paint",
        name="phase_flag",
    )
    shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        mass=1.1,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "shaft_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, AXIS_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    shaft = object_model.get_part("shaft")
    shaft_spin = object_model.get_articulation("shaft_spin")

    base_frame = frame.get_visual("base_frame")
    left_guide = frame.get_visual("left_guide")
    center_bearing = frame.get_visual("center_bearing")
    right_guide = frame.get_visual("right_guide")
    shaft_body = shaft.get_visual("shaft_body")
    phase_flag = shaft.get_visual("phase_flag")

    ctx.allow_isolated_part(
        shaft,
        reason="The shaft is intentionally free to rotate inside the end guides and center bearing housing with bearing clearance rather than a fused support.",
    )

    limits = shaft_spin.motion_limits
    ctx.check(
        "shaft uses a continuous x-axis articulation",
        shaft_spin.axis == (1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=f"axis={shaft_spin.axis}, limits={limits}",
    )

    ctx.expect_gap(
        shaft,
        frame,
        axis="z",
        positive_elem=shaft_body,
        negative_elem=base_frame,
        min_gap=0.08,
        name="shaft clears the grounded ladder frame",
    )

    ctx.expect_within(
        shaft,
        frame,
        axes="yz",
        inner_elem=shaft_body,
        outer_elem=left_guide,
        margin=0.0,
        name="shaft stays centered inside the left guide envelope",
    )
    ctx.expect_within(
        shaft,
        frame,
        axes="yz",
        inner_elem=shaft_body,
        outer_elem=center_bearing,
        margin=0.0,
        name="shaft stays centered inside the center bearing envelope",
    )
    ctx.expect_within(
        shaft,
        frame,
        axes="yz",
        inner_elem=shaft_body,
        outer_elem=right_guide,
        margin=0.0,
        name="shaft stays centered inside the right guide envelope",
    )

    ctx.expect_overlap(
        shaft,
        frame,
        axes="x",
        elem_a=shaft_body,
        elem_b=left_guide,
        min_overlap=GUIDE_THICKNESS - 0.002,
        name="shaft spans the left guide station",
    )
    ctx.expect_overlap(
        shaft,
        frame,
        axes="x",
        elem_a=shaft_body,
        elem_b=center_bearing,
        min_overlap=CENTER_THICKNESS - 0.002,
        name="shaft spans the deeper center bearing station",
    )
    ctx.expect_overlap(
        shaft,
        frame,
        axes="x",
        elem_a=shaft_body,
        elem_b=right_guide,
        min_overlap=GUIDE_THICKNESS - 0.002,
        name="shaft spans the right guide station",
    )

    center_aabb = ctx.part_element_world_aabb(frame, elem=center_bearing)
    left_aabb = ctx.part_element_world_aabb(frame, elem=left_guide)
    right_aabb = ctx.part_element_world_aabb(frame, elem=right_guide)
    center_depth = None if center_aabb is None else center_aabb[1][0] - center_aabb[0][0]
    left_depth = None if left_aabb is None else left_aabb[1][0] - left_aabb[0][0]
    right_depth = None if right_aabb is None else right_aabb[1][0] - right_aabb[0][0]
    ctx.check(
        "center bearing body is visibly deeper than the end guides",
        center_depth is not None
        and left_depth is not None
        and right_depth is not None
        and center_depth > left_depth + 0.04
        and center_depth > right_depth + 0.04,
        details=f"center_depth={center_depth}, left_depth={left_depth}, right_depth={right_depth}",
    )

    rest_flag_center = _center_from_aabb(ctx.part_element_world_aabb(shaft, elem=phase_flag))
    with ctx.pose({shaft_spin: pi / 2.0}):
        turned_flag_center = _center_from_aabb(ctx.part_element_world_aabb(shaft, elem=phase_flag))
    ctx.check(
        "indicator flag orbits around the shaft axis when the joint turns",
        rest_flag_center is not None
        and turned_flag_center is not None
        and turned_flag_center[1] < rest_flag_center[1] - 0.03
        and turned_flag_center[2] < rest_flag_center[2] - 0.03,
        details=f"rest_flag_center={rest_flag_center}, turned_flag_center={turned_flag_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

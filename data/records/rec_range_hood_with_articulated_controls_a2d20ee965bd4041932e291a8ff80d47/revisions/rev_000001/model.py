from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

HOOD_WIDTH = 0.90
HOOD_DEPTH = 0.56
BOTTOM_RIM = 0.040
CANOPY_HEIGHT = 0.30
CANOPY_TOP_WIDTH = 0.43
CANOPY_TOP_DEPTH = 0.29
INTAKE_WIDTH = 0.82
INTAKE_DEPTH = 0.46
SHELL_THICKNESS = 0.012

CHIMNEY_WIDTH = 0.36
CHIMNEY_DEPTH = 0.26
CHIMNEY_HEIGHT = 0.60
CHIMNEY_WALL = 0.008

CONTROL_Z = 0.122
CONTROL_FACE_Y = -0.270
CONTROL_PANEL_DEPTH = 0.026
BUTTON_TRAVEL = 0.007

FILTER_GAP = 0.024
FILTER_WIDTH = (INTAKE_WIDTH - FILTER_GAP) / 2.0
FILTER_DEPTH = 0.38
FILTER_THICKNESS = 0.010
FILTER_HINGE_Y = 0.19
FILTER_HINGE_Z = 0.022
FILTER_OPEN = math.radians(75.0)
FILTER_CENTER_X = FILTER_WIDTH / 2.0 + FILTER_GAP / 2.0

FACE_FRAME_RPY = (math.pi / 2.0, 0.0, 0.0)


def _hood_shell_mesh():
    ring_outer = cq.Workplane("XY").box(HOOD_WIDTH, HOOD_DEPTH, BOTTOM_RIM).translate((0.0, 0.0, BOTTOM_RIM / 2.0))
    ring_inner = (
        cq.Workplane("XY")
        .box(INTAKE_WIDTH, INTAKE_DEPTH, BOTTOM_RIM + 0.010)
        .translate((0.0, 0.0, BOTTOM_RIM / 2.0))
    )
    intake_ring = ring_outer.cut(ring_inner)

    outer_loft = (
        cq.Workplane("XY")
        .rect(HOOD_WIDTH, HOOD_DEPTH)
        .workplane(offset=CANOPY_HEIGHT - BOTTOM_RIM)
        .rect(CANOPY_TOP_WIDTH, CANOPY_TOP_DEPTH)
        .loft()
        .translate((0.0, 0.0, BOTTOM_RIM))
    )
    inner_loft = (
        cq.Workplane("XY")
        .rect(INTAKE_WIDTH, INTAKE_DEPTH)
        .workplane(offset=CANOPY_HEIGHT - BOTTOM_RIM)
        .rect(CHIMNEY_WIDTH - 0.050, CHIMNEY_DEPTH - 0.070)
        .loft()
        .translate((0.0, 0.0, BOTTOM_RIM))
    )
    canopy_shell = outer_loft.cut(inner_loft)

    chimney_outer = (
        cq.Workplane("XY")
        .box(CHIMNEY_WIDTH, CHIMNEY_DEPTH, CHIMNEY_HEIGHT)
        .translate((0.0, 0.0, CANOPY_HEIGHT + CHIMNEY_HEIGHT / 2.0))
    )
    chimney_inner = (
        cq.Workplane("XY")
        .box(
            CHIMNEY_WIDTH - 2.0 * CHIMNEY_WALL,
            CHIMNEY_DEPTH - 2.0 * CHIMNEY_WALL,
            CHIMNEY_HEIGHT + 0.050,
        )
        .translate((0.0, 0.0, CANOPY_HEIGHT + CHIMNEY_HEIGHT / 2.0))
    )
    chimney_shell = chimney_outer.cut(chimney_inner)

    shell = intake_ring.union(canopy_shell).union(chimney_shell)
    return mesh_from_cadquery(shell, "hood_shell")


def _build_filter(part, *, mesh_name: str) -> None:
    part.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (FILTER_WIDTH - 0.018, FILTER_DEPTH - 0.020),
                FILTER_THICKNESS,
                slot_size=(0.026, 0.005),
                pitch=(0.034, 0.016),
                frame=0.012,
                corner_radius=0.006,
                slot_angle_deg=18.0,
                stagger=True,
            ),
            mesh_name,
        ),
        origin=Origin(xyz=(0.0, -FILTER_DEPTH / 2.0, -0.010)),
        material="filter_metal",
        name="panel",
    )
    part.visual(
        Box((FILTER_WIDTH - 0.010, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, -0.008, -0.010)),
        material="filter_metal",
        name="rear_rail",
    )
    part.visual(
        Cylinder(radius=0.005, length=FILTER_WIDTH - 0.014),
        origin=Origin(xyz=(0.0, -0.004, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="filter_metal",
        name="hinge_barrel",
    )
    part.visual(
        Box((0.054, 0.012, 0.015)),
        origin=Origin(xyz=(0.0, -FILTER_DEPTH + 0.008, -0.014)),
        material="filter_metal",
        name="pull_tab",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pro_canopy_range_hood")

    model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("trim_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("control_black", rgba=(0.14, 0.14, 0.15, 1.0))
    model.material("filter_metal", rgba=(0.70, 0.72, 0.74, 1.0))

    hood = model.part("hood")
    hood.visual(_hood_shell_mesh(), material="stainless", name="shell")
    hood.visual(
        Box((0.35, CONTROL_PANEL_DEPTH, 0.056)),
        origin=Origin(xyz=(0.0, CONTROL_FACE_Y + CONTROL_PANEL_DEPTH / 2.0, CONTROL_Z)),
        material="trim_black",
        name="control_strip",
    )
    hood.visual(
        Box((HOOD_WIDTH - 0.10, 0.042, 0.018)),
        origin=Origin(xyz=(0.0, -0.229, 0.009)),
        material="stainless",
        name="front_rail",
    )
    hood.visual(
        Box((0.018, FILTER_DEPTH + 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material="stainless",
        name="center_divider",
    )
    hood.visual(
        Box((FILTER_WIDTH + 0.024, 0.018, 0.020)),
        origin=Origin(xyz=(-FILTER_CENTER_X, FILTER_HINGE_Y + 0.009, 0.016)),
        material="stainless",
        name="left_hinge_beam",
    )
    hood.visual(
        Box((FILTER_WIDTH + 0.024, 0.018, 0.020)),
        origin=Origin(xyz=(FILTER_CENTER_X, FILTER_HINGE_Y + 0.009, 0.016)),
        material="stainless",
        name="right_hinge_beam",
    )

    knob = model.part("knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.060,
                0.034,
                body_style="skirted",
                top_diameter=0.049,
                skirt=KnobSkirt(0.072, 0.008, flare=0.05),
                grip=KnobGrip(style="fluted", count=24, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
                center=False,
            ),
            "hood_knob",
        ),
        material="control_black",
        name="knob_shell",
    )

    left_button = model.part("left_button")
    left_button.visual(
        Box((0.026, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material="control_black",
        name="button_cap",
    )

    right_button = model.part("right_button")
    right_button.visual(
        Box((0.026, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material="control_black",
        name="button_cap",
    )

    left_filter = model.part("left_filter")
    right_filter = model.part("right_filter")
    _build_filter(left_filter, mesh_name="left_filter_panel")
    _build_filter(right_filter, mesh_name="right_filter_panel")

    model.articulation(
        "hood_to_knob",
        ArticulationType.CONTINUOUS,
        parent=hood,
        child=knob,
        origin=Origin(xyz=(0.0, CONTROL_FACE_Y, CONTROL_Z), rpy=FACE_FRAME_RPY),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=12.0),
    )
    model.articulation(
        "hood_to_left_button",
        ArticulationType.PRISMATIC,
        parent=hood,
        child=left_button,
        origin=Origin(xyz=(-0.112, CONTROL_FACE_Y, CONTROL_Z), rpy=FACE_FRAME_RPY),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.05, lower=0.0, upper=BUTTON_TRAVEL),
    )
    model.articulation(
        "hood_to_right_button",
        ArticulationType.PRISMATIC,
        parent=hood,
        child=right_button,
        origin=Origin(xyz=(0.112, CONTROL_FACE_Y, CONTROL_Z), rpy=FACE_FRAME_RPY),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.05, lower=0.0, upper=BUTTON_TRAVEL),
    )
    model.articulation(
        "hood_to_left_filter",
        ArticulationType.REVOLUTE,
        parent=hood,
        child=left_filter,
        origin=Origin(xyz=(-FILTER_CENTER_X, FILTER_HINGE_Y, FILTER_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.0, lower=0.0, upper=FILTER_OPEN),
    )
    model.articulation(
        "hood_to_right_filter",
        ArticulationType.REVOLUTE,
        parent=hood,
        child=right_filter,
        origin=Origin(xyz=(FILTER_CENTER_X, FILTER_HINGE_Y, FILTER_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.0, lower=0.0, upper=FILTER_OPEN),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((float(mins[i]) + float(maxs[i])) / 2.0 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hood = object_model.get_part("hood")
    knob = object_model.get_part("knob")
    left_button = object_model.get_part("left_button")
    right_button = object_model.get_part("right_button")
    left_filter = object_model.get_part("left_filter")
    right_filter = object_model.get_part("right_filter")

    knob_joint = object_model.get_articulation("hood_to_knob")
    left_button_joint = object_model.get_articulation("hood_to_left_button")
    right_button_joint = object_model.get_articulation("hood_to_right_button")
    left_filter_joint = object_model.get_articulation("hood_to_left_filter")
    right_filter_joint = object_model.get_articulation("hood_to_right_filter")

    hood_aabb = ctx.part_world_aabb(hood)
    ctx.check("hood_has_world_aabb", hood_aabb is not None, "Expected a world AABB for the hood.")
    if hood_aabb is not None:
        mins, maxs = hood_aabb
        width = float(maxs[0] - mins[0])
        depth = float(maxs[1] - mins[1])
        height = float(maxs[2] - mins[2])
        ctx.check("hood_residential_width", 0.86 <= width <= 0.94, f"width={width:.4f}")
        ctx.check("hood_residential_depth", 0.53 <= depth <= 0.59, f"depth={depth:.4f}")
        ctx.check("hood_residential_height", 0.86 <= height <= 0.93, f"height={height:.4f}")

    knob_limits = knob_joint.motion_limits
    ctx.check(
        "knob_is_continuous_control",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_limits is not None
        and knob_limits.lower is None
        and knob_limits.upper is None,
        f"joint_type={knob_joint.articulation_type}, limits={knob_limits!r}",
    )

    ctx.expect_origin_distance(left_button, knob, axes="x", min_dist=0.09, max_dist=0.13, name="left button flanks knob")
    ctx.expect_origin_distance(right_button, knob, axes="x", min_dist=0.09, max_dist=0.13, name="right button flanks knob")
    ctx.expect_origin_distance(left_filter, right_filter, axes="x", min_dist=0.40, max_dist=0.44, name="filters span intake width")

    left_rest = ctx.part_world_position(left_button)
    right_rest = ctx.part_world_position(right_button)
    with ctx.pose({left_button_joint: BUTTON_TRAVEL}):
        left_pressed = ctx.part_world_position(left_button)
        right_while_left = ctx.part_world_position(right_button)
    with ctx.pose({right_button_joint: BUTTON_TRAVEL}):
        right_pressed = ctx.part_world_position(right_button)
        left_while_right = ctx.part_world_position(left_button)

    ctx.check(
        "left button depresses inward",
        left_rest is not None and left_pressed is not None and left_pressed[1] > left_rest[1] + 0.0045,
        f"rest={left_rest}, pressed={left_pressed}",
    )
    ctx.check(
        "right button depresses inward",
        right_rest is not None and right_pressed is not None and right_pressed[1] > right_rest[1] + 0.0045,
        f"rest={right_rest}, pressed={right_pressed}",
    )
    ctx.check(
        "left button motion is independent",
        right_rest is not None
        and right_while_left is not None
        and abs(right_while_left[1] - right_rest[1]) < 1e-6,
        f"right_rest={right_rest}, right_while_left={right_while_left}",
    )
    ctx.check(
        "right button motion is independent",
        left_rest is not None
        and left_while_right is not None
        and abs(left_while_right[1] - left_rest[1]) < 1e-6,
        f"left_rest={left_rest}, left_while_right={left_while_right}",
    )

    left_closed_center = _aabb_center(ctx.part_world_aabb(left_filter))
    right_closed_center = _aabb_center(ctx.part_world_aabb(right_filter))
    with ctx.pose({left_filter_joint: FILTER_OPEN}):
        left_open_center = _aabb_center(ctx.part_world_aabb(left_filter))
        right_during_left = _aabb_center(ctx.part_world_aabb(right_filter))
    with ctx.pose({right_filter_joint: FILTER_OPEN}):
        right_open_center = _aabb_center(ctx.part_world_aabb(right_filter))
        left_during_right = _aabb_center(ctx.part_world_aabb(left_filter))

    ctx.check(
        "left filter swings downward",
        left_closed_center is not None
        and left_open_center is not None
        and left_open_center[2] < left_closed_center[2] - 0.08
        and left_open_center[1] > left_closed_center[1] + 0.08,
        f"closed={left_closed_center}, open={left_open_center}",
    )
    ctx.check(
        "right filter swings downward",
        right_closed_center is not None
        and right_open_center is not None
        and right_open_center[2] < right_closed_center[2] - 0.08
        and right_open_center[1] > right_closed_center[1] + 0.08,
        f"closed={right_closed_center}, open={right_open_center}",
    )
    ctx.check(
        "left filter motion is independent",
        right_closed_center is not None
        and right_during_left is not None
        and abs(right_during_left[2] - right_closed_center[2]) < 1e-6,
        f"right_closed={right_closed_center}, right_during_left={right_during_left}",
    )
    ctx.check(
        "right filter motion is independent",
        left_closed_center is not None
        and left_during_right is not None
        and abs(left_during_right[2] - left_closed_center[2]) < 1e-6,
        f"left_closed={left_closed_center}, left_during_right={left_during_right}",
    )

    return ctx.report()


object_model = build_object_model()

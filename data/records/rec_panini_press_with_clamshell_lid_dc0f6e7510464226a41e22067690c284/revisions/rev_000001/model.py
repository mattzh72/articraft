from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_DEPTH = 0.250
BODY_WIDTH = 0.282
BODY_HEIGHT = 0.055

LOWER_PLATE_SIZE = (0.188, 0.208, 0.009)
LOWER_PLATE_CENTER_X = 0.006

HINGE_BLOCK_SIZE = (0.024, 0.086, 0.040)
HINGE_BLOCK_CENTER = (-0.131, 0.0, 0.073)

RECESS_DEPTH = 0.018
RECESS_WIDTH = 0.070
RECESS_HEIGHT = 0.044
RECESS_CENTER_Z = 0.024

LID_LENGTH = 0.245
LID_WIDTH = 0.274
LID_INNER_WIDTH = 0.248
HINGE_ORIGIN = (-0.118, 0.0, 0.050)
UPPER_PLATE_SIZE = (0.186, 0.206, 0.010)
UPPER_PLATE_ORIGIN = (0.124, 0.0, 0.001)

HANDLE_SUPPORT_SIZE = (0.036, 0.018, 0.028)
HANDLE_SUPPORT_X = 0.210
HANDLE_SUPPORT_Y = 0.044
HANDLE_SUPPORT_Z = 0.026
HANDLE_BAR_SIZE = (0.034, 0.126, 0.014)
HANDLE_BAR_X = 0.233
HANDLE_BAR_Z = 0.041

KNOB_DIAMETER = 0.034
KNOB_HEIGHT = 0.018
KNOB_SHAFT_RADIUS = 0.0035
KNOB_SHAFT_LENGTH = 0.008
KNOB_ORIGIN = (BODY_DEPTH / 2.0 - (RECESS_DEPTH + 0.002) + 0.001, 0.0, RECESS_CENTER_Z)


def _make_body_shell() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT)
        .translate((0.0, 0.0, BODY_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.017)
    )

    cooking_cavity = (
        cq.Workplane("XY")
        .box(0.206, 0.226, 0.018)
        .translate((LOWER_PLATE_CENTER_X, 0.0, BODY_HEIGHT - 0.009))
    )

    front_recess = (
        cq.Workplane("XY")
        .box(RECESS_DEPTH + 0.002, RECESS_WIDTH, RECESS_HEIGHT)
        .translate((BODY_DEPTH / 2.0 - (RECESS_DEPTH + 0.002) / 2.0 + 0.001, 0.0, RECESS_CENTER_Z))
    )

    return shell.cut(cooking_cavity).cut(front_recess)


def _lid_profile(bottom_z: float, rear_top_z: float, front_top_z: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .moveTo(0.0, bottom_z)
        .lineTo(0.0, rear_top_z)
        .threePointArc((0.095, 0.078), (0.210, 0.040))
        .threePointArc((0.232, 0.030), (LID_LENGTH, front_top_z))
        .lineTo(LID_LENGTH, bottom_z)
        .close()
    )


def _make_lid_shell() -> cq.Workplane:
    outer = _lid_profile(bottom_z=0.006, rear_top_z=0.024, front_top_z=0.020)
    inner = _lid_profile(bottom_z=0.010, rear_top_z=0.018, front_top_z=0.018)

    lid_shell = (
        outer.extrude(LID_WIDTH).translate((0.0, LID_WIDTH / 2.0, 0.0)).cut(
            inner.extrude(LID_INNER_WIDTH).translate((0.0, LID_INNER_WIDTH / 2.0, 0.0))
        )
    )

    support_0 = (
        cq.Workplane("XY")
        .box(*HANDLE_SUPPORT_SIZE)
        .translate((HANDLE_SUPPORT_X, HANDLE_SUPPORT_Y, HANDLE_SUPPORT_Z))
    )
    support_1 = (
        cq.Workplane("XY")
        .box(*HANDLE_SUPPORT_SIZE)
        .translate((HANDLE_SUPPORT_X, -HANDLE_SUPPORT_Y, HANDLE_SUPPORT_Z))
    )
    bar = (
        cq.Workplane("XY")
        .box(*HANDLE_BAR_SIZE)
        .translate((HANDLE_BAR_X, 0.0, HANDLE_BAR_Z))
    )
    base_pad = (
        cq.Workplane("XY")
        .box(0.078, 0.110, 0.012)
        .translate((0.223, 0.0, 0.020))
    )

    return lid_shell.union(base_pad).union(support_0).union(support_1).union(bar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sandwich_press")

    shell_black = model.material("shell_black", rgba=(0.11, 0.11, 0.12, 1.0))
    plate_metal = model.material("plate_metal", rgba=(0.42, 0.43, 0.45, 1.0))
    control_black = model.material("control_black", rgba=(0.15, 0.15, 0.16, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shell(), "sandwich_press_body_shell"),
        material=shell_black,
        name="body_shell",
    )
    body.visual(
        Box(LOWER_PLATE_SIZE),
        origin=Origin(xyz=(LOWER_PLATE_CENTER_X, 0.0, 0.0415)),
        material=plate_metal,
        name="lower_plate",
    )
    body.visual(
        Box(HINGE_BLOCK_SIZE),
        origin=Origin(xyz=HINGE_BLOCK_CENTER),
        material=shell_black,
        name="hinge_block",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_make_lid_shell(), "sandwich_press_lid_shell"),
        material=shell_black,
        name="lid_shell",
    )
    lid.visual(
        Box(UPPER_PLATE_SIZE),
        origin=Origin(xyz=UPPER_PLATE_ORIGIN),
        material=plate_metal,
        name="upper_plate",
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                KNOB_DIAMETER,
                KNOB_HEIGHT,
                body_style="skirted",
                top_diameter=0.028,
                skirt=KnobSkirt(0.060, 0.004, flare=0.06),
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
                center=False,
            ),
            "sandwich_press_timer_knob",
        ),
        origin=Origin(xyz=(0.019, 0.0, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=control_black,
        name="knob_cap",
    )
    timer_knob.visual(
        Box((KNOB_SHAFT_LENGTH, KNOB_SHAFT_RADIUS * 2.0, KNOB_SHAFT_RADIUS * 2.0)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=Material(name="shaft_metal", rgba=(0.55, 0.56, 0.58, 1.0)),
        name="shaft",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=HINGE_ORIGIN),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=0.0, upper=1.2),
    )
    model.articulation(
        "timer_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_knob,
        origin=Origin(xyz=KNOB_ORIGIN),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    timer_knob = object_model.get_part("timer_knob")
    lid_hinge = object_model.get_articulation("lid_hinge")
    knob_joint = object_model.get_articulation("timer_knob_spin")

    ctx.check(
        "lid_hinge_limits",
        lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.lower == 0.0
        and lid_hinge.motion_limits.upper is not None
        and 1.0 <= lid_hinge.motion_limits.upper <= 1.3,
        details=f"limits={lid_hinge.motion_limits!r}",
    )
    ctx.check(
        "timer_knob_is_continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(knob_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={knob_joint.articulation_type!r}, axis={knob_joint.axis!r}",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="upper_plate",
            negative_elem="lower_plate",
            min_gap=0.0,
            max_gap=0.002,
            name="closed_plates_hold_small_cooking_gap",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="upper_plate",
            elem_b="lower_plate",
            min_overlap=0.175,
            name="closed_plates_cover_same_cooking_area",
        )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    upper_limit = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else None
    opened_lid_aabb = None
    if upper_limit is not None:
        with ctx.pose({lid_hinge: upper_limit}):
            opened_lid_aabb = ctx.part_world_aabb(lid)

    lid_rises = (
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and float(opened_lid_aabb[1][2]) > float(closed_lid_aabb[1][2]) + 0.09
    )
    ctx.check(
        "lid_opens_upward",
        lid_rises,
        details=f"closed={closed_lid_aabb!r}, opened={opened_lid_aabb!r}",
    )

    body_shell_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    hinge_block_aabb = ctx.part_element_world_aabb(body, elem="hinge_block")
    hinge_visible = (
        body_shell_aabb is not None
        and hinge_block_aabb is not None
        and float(hinge_block_aabb[1][2]) > float(body_shell_aabb[1][2]) + 0.02
        and float(hinge_block_aabb[0][0]) < float(body_shell_aabb[0][0]) + 0.005
    )
    ctx.check(
        "hinge_block_stands_above_back_edge",
        hinge_visible,
        details=f"body_shell={body_shell_aabb!r}, hinge_block={hinge_block_aabb!r}",
    )

    knob_origin = ctx.part_world_position(timer_knob)
    knob_aabb = ctx.part_world_aabb(timer_knob)
    recessed_mount = (
        body_shell_aabb is not None
        and knob_origin is not None
        and 0.015 <= float(body_shell_aabb[1][0]) - float(knob_origin[0]) <= 0.022
    )
    knob_projects_forward = (
        body_shell_aabb is not None
        and knob_aabb is not None
        and float(knob_aabb[1][0]) > float(body_shell_aabb[1][0]) + 0.010
    )
    ctx.check(
        "timer_knob_mounts_from_front_recess",
        recessed_mount and knob_projects_forward,
        details=f"body_shell={body_shell_aabb!r}, knob_origin={knob_origin!r}, knob_aabb={knob_aabb!r}",
    )

    return ctx.report()


object_model = build_object_model()

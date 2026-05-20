from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_RADIUS = 0.135
BASE_THICK = 0.018
HUB_RADIUS = 0.028
HUB_HEIGHT = 0.048

SLEEVE_Z0 = BASE_THICK + HUB_HEIGHT
SLEEVE_OUTER_RADIUS = 0.021
SLEEVE_INNER_RADIUS = 0.0175
SLEEVE_HEIGHT = 0.190
COLLAR_OUTER_RADIUS = 0.026
COLLAR_HEIGHT = 0.052
LOWER_TOP = SLEEVE_Z0 + SLEEVE_HEIGHT + COLLAR_HEIGHT

SLOT_LENGTH = 0.040
SLOT_WIDTH = 0.014
SLOT_HEIGHT = COLLAR_HEIGHT + 0.006

SCREW_Z = SLEEVE_Z0 + SLEEVE_HEIGHT + COLLAR_HEIGHT * 0.52
BOSS_LENGTH = 0.024
BOSS_WIDTH = 0.016
BOSS_HEIGHT = 0.018
BOSS_CENTER_X = COLLAR_OUTER_RADIUS + BOSS_LENGTH / 2.0 - 0.001
SCREW_AXIS_X = COLLAR_OUTER_RADIUS + 0.017
SHAFT_RADIUS = 0.004

INNER_TUBE_RADIUS = 0.015
INNER_TUBE_LENGTH = 0.510
INNER_TUBE_CENTER_Z = 0.115
MAST_TRAVEL = 0.095
HINGE_Z = 0.392

DESK_WIDTH = 0.420
DESK_HEIGHT = 0.220
DESK_TILT_REST = 0.22


def _sleeve_shell_shape():
    lower_sleeve = (
        cq.Workplane("XY")
        .workplane(offset=SLEEVE_Z0)
        .circle(SLEEVE_OUTER_RADIUS)
        .circle(SLEEVE_INNER_RADIUS)
        .extrude(SLEEVE_HEIGHT)
    )
    collar = (
        cq.Workplane("XY")
        .workplane(offset=SLEEVE_Z0 + SLEEVE_HEIGHT)
        .circle(COLLAR_OUTER_RADIUS)
        .circle(SLEEVE_INNER_RADIUS)
        .extrude(COLLAR_HEIGHT)
    )
    slot = (
        cq.Workplane("XY")
        .box(SLOT_LENGTH, SLOT_WIDTH, SLOT_HEIGHT)
        .translate(
            (
                COLLAR_OUTER_RADIUS + SLOT_LENGTH / 2.0 - 0.006,
                0.0,
                SLEEVE_Z0 + SLEEVE_HEIGHT + COLLAR_HEIGHT / 2.0,
            )
        )
    )
    return lower_sleeve.union(collar).cut(slot)


def _clamp_boss_shape():
    boss = (
        cq.Workplane("XY")
        .box(BOSS_LENGTH, BOSS_WIDTH, BOSS_HEIGHT)
        .faces(">X")
        .workplane()
        .hole(2.0 * (SHAFT_RADIUS + 0.0008), depth=BOSS_LENGTH + 0.002)
        .translate((BOSS_CENTER_X, 0.0, SCREW_Z))
    )
    return boss


def _aabb_center_axis(aabb, axis_index: int) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][axis_index] + aabb[1][axis_index])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pit_orchestra_stand")

    cast_iron = model.material("cast_iron", rgba=(0.15, 0.15, 0.16, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.30, 0.31, 0.33, 1.0))
    graphite = model.material("graphite", rgba=(0.21, 0.22, 0.24, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICK),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICK / 2.0)),
        material=cast_iron,
        name="weighted_base",
    )
    base.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICK + HUB_HEIGHT / 2.0)),
        material=cast_iron,
        name="hub",
    )
    base.visual(
        mesh_from_cadquery(_sleeve_shell_shape(), "sleeve_shell"),
        material=satin_steel,
        name="sleeve_shell",
    )
    base.visual(
        mesh_from_cadquery(_clamp_boss_shape(), "clamp_boss"),
        material=graphite,
        name="clamp_boss",
    )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=INNER_TUBE_RADIUS, length=INNER_TUBE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, INNER_TUBE_CENTER_Z)),
        material=graphite,
        name="inner_tube",
    )
    mast.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_steel,
        name="stop_collar",
    )
    mast.visual(
        Box((0.026, 0.018, 0.036)),
        origin=Origin(xyz=(0.0, -0.010, 0.356)),
        material=satin_steel,
        name="head_neck",
    )
    mast.visual(
        Box((0.070, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.014, 0.380)),
        material=satin_steel,
        name="head_web",
    )
    mast.visual(
        Box((0.010, 0.016, 0.026)),
        origin=Origin(xyz=(-0.026, -0.008, HINGE_Z)),
        material=satin_steel,
        name="ear_0",
    )
    mast.visual(
        Box((0.010, 0.016, 0.026)),
        origin=Origin(xyz=(0.026, -0.008, HINGE_Z)),
        material=satin_steel,
        name="ear_1",
    )

    desk = model.part("desk")
    desk.visual(
        Cylinder(radius=0.0075, length=0.040),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="hinge_barrel",
    )
    desk.visual(
        Box((0.040, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, -0.004, 0.010), rpy=(DESK_TILT_REST, 0.0, 0.0)),
        material=satin_steel,
        name="hinge_leaf",
    )
    desk.visual(
        Box((DESK_WIDTH, 0.006, DESK_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.010, 0.118), rpy=(DESK_TILT_REST, 0.0, 0.0)),
        material=graphite,
        name="panel",
    )
    desk.visual(
        Box((0.010, 0.026, DESK_HEIGHT)),
        origin=Origin(xyz=(-0.205, 0.000, 0.118), rpy=(DESK_TILT_REST, 0.0, 0.0)),
        material=graphite,
        name="flange_0",
    )
    desk.visual(
        Box((0.010, 0.026, DESK_HEIGHT)),
        origin=Origin(xyz=(0.205, 0.000, 0.118), rpy=(DESK_TILT_REST, 0.0, 0.0)),
        material=graphite,
        name="flange_1",
    )
    desk.visual(
        Box((0.380, 0.055, 0.010)),
        origin=Origin(xyz=(0.0, 0.020, 0.020), rpy=(DESK_TILT_REST, 0.0, 0.0)),
        material=graphite,
        name="shelf",
    )
    desk.visual(
        Box((0.360, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.046, 0.024), rpy=(DESK_TILT_REST, 0.0, 0.0)),
        material=graphite,
        name="lip",
    )
    desk.visual(
        Box((0.090, 0.016, 0.170)),
        origin=Origin(xyz=(0.0, -0.002, 0.108), rpy=(DESK_TILT_REST, 0.0, 0.0)),
        material=satin_steel,
        name="stiffener",
    )
    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=SHAFT_RADIUS, length=0.016),
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="screw_shaft",
    )
    knob.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="knob_hub",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.034,
                0.016,
                body_style="mushroom",
                top_diameter=0.024,
                edge_radius=0.0015,
            ),
            "clamp_knob",
        ),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="knob_cap",
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, LOWER_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.12,
            lower=0.0,
            upper=MAST_TRAVEL,
        ),
    )
    model.articulation(
        "mast_to_desk",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=-0.25,
            upper=0.50,
        ),
    )
    model.articulation(
        "base_to_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=knob,
        origin=Origin(xyz=(BOSS_CENTER_X + BOSS_LENGTH / 2.0, 0.0, SCREW_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    desk = object_model.get_part("desk")
    knob = object_model.get_part("knob")

    slide = object_model.get_articulation("base_to_mast")
    tilt = object_model.get_articulation("mast_to_desk")

    ctx.expect_origin_distance(
        mast,
        base,
        axes="xy",
        max_dist=0.001,
        name="mast stays centered over the weighted base",
    )
    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="sleeve_shell",
        margin=0.0005,
        name="mast tube stays within the sleeve footprint",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="sleeve_shell",
        min_overlap=0.13,
        name="collapsed mast remains inserted in the sleeve",
    )

    slide_limits = slide.motion_limits
    if slide_limits is not None and slide_limits.upper is not None:
        rest_pos = ctx.part_world_position(mast)
        with ctx.pose({slide: slide_limits.upper}):
            ctx.expect_within(
                mast,
                base,
                axes="xy",
                inner_elem="inner_tube",
                outer_elem="sleeve_shell",
                margin=0.0005,
                name="extended mast stays concentric in the sleeve",
            )
            ctx.expect_overlap(
                mast,
                base,
                axes="z",
                elem_a="inner_tube",
                elem_b="sleeve_shell",
                min_overlap=0.04,
                name="extended mast keeps retained insertion",
            )
            extended_pos = ctx.part_world_position(mast)
        ctx.check(
            "mast extends upward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[2] > rest_pos[2] + 0.08,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    tilt_limits = tilt.motion_limits
    if (
        tilt_limits is not None
        and tilt_limits.lower is not None
        and tilt_limits.upper is not None
    ):
        with ctx.pose({tilt: tilt_limits.lower}):
            lower_panel = ctx.part_element_world_aabb(desk, elem="panel")
        with ctx.pose({tilt: tilt_limits.upper}):
            upper_panel = ctx.part_element_world_aabb(desk, elem="panel")
        lower_center_y = _aabb_center_axis(lower_panel, 1)
        upper_center_y = _aabb_center_axis(upper_panel, 1)
        ctx.check(
            "desk tilts rearward with positive hinge motion",
            lower_center_y is not None
            and upper_center_y is not None
            and upper_center_y < lower_center_y - 0.035,
            details=f"lower_center_y={lower_center_y}, upper_center_y={upper_center_y}",
        )

    ctx.expect_origin_gap(
        knob,
        base,
        axis="x",
        min_gap=0.047,
        max_gap=0.052,
        name="clamp knob sits on the collar side",
    )
    ctx.expect_origin_gap(
        knob,
        base,
        axis="z",
        min_gap=0.275,
        max_gap=0.292,
        name="clamp knob sits at collar height",
    )

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


BASE_WIDTH = 0.34
BASE_DEPTH = 0.23
BASE_THICKNESS = 0.014
BASE_CROWN_WIDTH = 0.19
BASE_CROWN_DEPTH = 0.13
BASE_CROWN_THICKNESS = 0.009

SLEEVE_INNER_RADIUS = 0.0255
SLEEVE_OUTER_RADIUS = 0.031
SLEEVE_LOWER_RADIUS = 0.040
SLEEVE_UPPER_RADIUS = 0.036
SLEEVE_LOWER_HEIGHT = 0.022
SLEEVE_TUBE_HEIGHT = 0.128
SLEEVE_UPPER_HEIGHT = 0.014
PEDESTAL_ENTRY_Z = (
    BASE_THICKNESS + SLEEVE_LOWER_HEIGHT + SLEEVE_TUBE_HEIGHT + SLEEVE_UPPER_HEIGHT
)

MAST_RADIUS = 0.022
MAST_LENGTH = 0.29
MAST_CENTER_Z = 0.015
MAST_TOP_COLLAR_RADIUS = 0.028
MAST_TOP_COLLAR_HEIGHT = 0.016
MAST_TOP_Z = MAST_CENTER_Z + (MAST_LENGTH * 0.5)

LIFT_LOWER = 0.0
LIFT_UPPER = 0.09
SWIVEL_LOWER = -1.8
SWIVEL_UPPER = 1.8
TILT_LOWER = -0.45
TILT_UPPER = 0.65

TILT_AXIS_X = 0.018
TILT_AXIS_Z = 0.038

TRAY_DEPTH = 0.30
TRAY_WIDTH = 0.25
TRAY_THICKNESS = 0.010
TRAY_CENTER_X = 0.130
TRAY_CENTER_Z = 0.016


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _ring_band(
    outer_radius: float,
    inner_radius: float,
    height: float,
    z_center: float,
):
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -height * 0.5), (outer_radius, height * 0.5)],
        [(inner_radius, -height * 0.5), (inner_radius, height * 0.5)],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    ).translate(0.0, 0.0, z_center)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laptop_stand")

    satin_aluminum = model.material("satin_aluminum", rgba=(0.72, 0.74, 0.78, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.30, 0.32, 0.35, 1.0))
    textured_black = model.material("textured_black", rgba=(0.12, 0.12, 0.13, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        _mesh(
            "base_plate",
            ExtrudeGeometry.from_z0(
                superellipse_profile(BASE_WIDTH, BASE_DEPTH, exponent=2.1, segments=72),
                BASE_THICKNESS,
            ),
        ),
        material=satin_aluminum,
        name="base_plate",
    )
    base.visual(
        _mesh(
            "base_crown",
            ExtrudeGeometry.from_z0(
                superellipse_profile(
                    BASE_CROWN_WIDTH,
                    BASE_CROWN_DEPTH,
                    exponent=2.3,
                    segments=64,
                ),
                BASE_CROWN_THICKNESS,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS)),
        material=satin_aluminum,
        name="base_crown",
    )
    base.visual(
        _mesh(
            "sleeve_lower",
            _ring_band(
                SLEEVE_LOWER_RADIUS,
                SLEEVE_INNER_RADIUS,
                SLEEVE_LOWER_HEIGHT,
                BASE_THICKNESS + (SLEEVE_LOWER_HEIGHT * 0.5),
            ),
        ),
        material=dark_aluminum,
        name="sleeve_lower",
    )
    base.visual(
        _mesh(
            "sleeve_tube",
            _ring_band(
                SLEEVE_OUTER_RADIUS,
                SLEEVE_INNER_RADIUS,
                SLEEVE_TUBE_HEIGHT,
                BASE_THICKNESS
                + SLEEVE_LOWER_HEIGHT
                + (SLEEVE_TUBE_HEIGHT * 0.5),
            ),
        ),
        material=dark_aluminum,
        name="sleeve_tube",
    )
    base.visual(
        _mesh(
            "sleeve_upper",
            _ring_band(
                SLEEVE_UPPER_RADIUS,
                SLEEVE_INNER_RADIUS,
                SLEEVE_UPPER_HEIGHT,
                BASE_THICKNESS
                + SLEEVE_LOWER_HEIGHT
                + SLEEVE_TUBE_HEIGHT
                + (SLEEVE_UPPER_HEIGHT * 0.5),
            ),
        ),
        material=dark_aluminum,
        name="sleeve_upper",
    )
    base.visual(
        Cylinder(radius=0.0055, length=0.020),
        origin=Origin(
            xyz=(SLEEVE_OUTER_RADIUS + 0.008, 0.0, BASE_THICKNESS + 0.095),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=textured_black,
        name="height_knob_stem",
    )
    base.visual(
        Cylinder(radius=0.011, length=0.008),
        origin=Origin(
            xyz=(SLEEVE_OUTER_RADIUS + 0.019, 0.0, BASE_THICKNESS + 0.095),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=textured_black,
        name="height_knob_cap",
    )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=MAST_RADIUS, length=MAST_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, MAST_CENTER_Z)),
        material=dark_aluminum,
        name="inner_column",
    )
    mast.visual(
        Cylinder(radius=0.034, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_aluminum,
        name="stop_collar",
    )
    mast.visual(
        Cylinder(radius=MAST_TOP_COLLAR_RADIUS, length=MAST_TOP_COLLAR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, MAST_TOP_Z - (MAST_TOP_COLLAR_HEIGHT * 0.5))),
        material=dark_aluminum,
        name="head_spigot",
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_ENTRY_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.12,
            lower=LIFT_LOWER,
            upper=LIFT_UPPER,
        ),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.031, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=textured_black,
        name="swivel_disk",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=textured_black,
        name="swivel_neck",
    )
    head.visual(
        Box((0.028, 0.060, 0.012)),
        origin=Origin(xyz=(0.012, 0.0, 0.024)),
        material=textured_black,
        name="tilt_bridge",
    )
    head.visual(
        Box((0.014, 0.042, 0.012)),
        origin=Origin(xyz=(0.028, 0.0, 0.022)),
        material=textured_black,
        name="front_block",
    )
    head.visual(
        Box((0.030, 0.010, 0.034)),
        origin=Origin(xyz=(0.018, 0.030, 0.035)),
        material=textured_black,
        name="cheek_0",
    )
    head.visual(
        Box((0.030, 0.010, 0.034)),
        origin=Origin(xyz=(0.018, -0.030, 0.035)),
        material=textured_black,
        name="cheek_1",
    )

    model.articulation(
        "mast_to_head",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, MAST_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=SWIVEL_LOWER,
            upper=SWIVEL_UPPER,
        ),
    )

    tray = model.part("tray")
    tray.visual(
        _mesh(
            "tray_panel",
            ExtrudeGeometry.centered(
                rounded_rect_profile(TRAY_DEPTH, TRAY_WIDTH, 0.018, corner_segments=10),
                TRAY_THICKNESS,
            ),
        ),
        origin=Origin(xyz=(TRAY_CENTER_X, 0.0, TRAY_CENTER_Z)),
        material=textured_black,
        name="tray_panel",
    )
    tray.visual(
        Box((0.090, 0.042, 0.016)),
        origin=Origin(xyz=(0.020, 0.0, 0.004)),
        material=textured_black,
        name="mount_block",
    )
    tray.visual(
        Box((0.030, 0.220, 0.010)),
        origin=Origin(xyz=(-0.032, 0.0, 0.008)),
        material=textured_black,
        name="rear_spine",
    )
    tray.visual(
        Box((0.024, 0.060, 0.018)),
        origin=Origin(xyz=(0.262, 0.076, 0.024)),
        material=textured_black,
        name="front_lip_0",
    )
    tray.visual(
        Box((0.024, 0.060, 0.018)),
        origin=Origin(xyz=(0.262, -0.076, 0.024)),
        material=textured_black,
        name="front_lip_1",
    )
    tray.visual(
        Box((0.078, 0.028, 0.0025)),
        origin=Origin(xyz=(0.105, 0.070, 0.02025)),
        material=rubber,
        name="pad_0",
    )
    tray.visual(
        Box((0.078, 0.028, 0.0025)),
        origin=Origin(xyz=(0.105, -0.070, 0.02025)),
        material=rubber,
        name="pad_1",
    )
    tray.visual(
        Box((0.068, 0.040, 0.0025)),
        origin=Origin(xyz=(0.052, 0.0, 0.02025)),
        material=rubber,
        name="pad_2",
    )

    model.articulation(
        "head_to_tray",
        ArticulationType.REVOLUTE,
        parent=head,
        child=tray,
        origin=Origin(xyz=(TILT_AXIS_X, 0.0, TILT_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=TILT_LOWER,
            upper=TILT_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    tray = object_model.get_part("tray")
    lift = object_model.get_articulation("base_to_mast")
    swivel = object_model.get_articulation("mast_to_head")
    tilt = object_model.get_articulation("head_to_tray")

    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_column",
        outer_elem="sleeve_tube",
        margin=0.001,
        name="mast stays centered in the sleeve",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_column",
        elem_b="sleeve_tube",
        min_overlap=0.11,
        name="mast remains inserted at rest",
    )

    rest_mast_pos = ctx.part_world_position(mast)
    with ctx.pose({lift: LIFT_UPPER}):
        ctx.expect_within(
            mast,
            base,
            axes="xy",
            inner_elem="inner_column",
            outer_elem="sleeve_tube",
            margin=0.001,
            name="mast stays centered at full height",
        )
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_column",
            elem_b="sleeve_tube",
            min_overlap=0.025,
            name="mast keeps retained insertion at full height",
        )
        extended_mast_pos = ctx.part_world_position(mast)

    ctx.check(
        "mast extends upward",
        rest_mast_pos is not None
        and extended_mast_pos is not None
        and extended_mast_pos[2] > rest_mast_pos[2] + 0.05,
        details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
    )

    panel_aabb = ctx.part_element_world_aabb(tray, elem="tray_panel")
    lip_aabb = ctx.part_element_world_aabb(tray, elem="front_lip_0")
    ctx.check(
        "front lip stands above the tray surface",
        panel_aabb is not None
        and lip_aabb is not None
        and lip_aabb[1][2] > panel_aabb[1][2] + 0.010,
        details=f"panel={panel_aabb}, lip={lip_aabb}",
    )

    rest_lip_center = _aabb_center(ctx.part_element_world_aabb(tray, elem="front_lip_0"))
    with ctx.pose({tilt: TILT_UPPER}):
        tilted_lip_center = _aabb_center(ctx.part_element_world_aabb(tray, elem="front_lip_0"))
    ctx.check(
        "tilt raises the tray front edge",
        rest_lip_center is not None
        and tilted_lip_center is not None
        and tilted_lip_center[2] > rest_lip_center[2] + 0.10,
        details=f"rest={rest_lip_center}, tilted={tilted_lip_center}",
    )

    with ctx.pose({swivel: 1.0}):
        swiveled_lip_center = _aabb_center(ctx.part_element_world_aabb(tray, elem="front_lip_0"))
    ctx.check(
        "swivel changes tray azimuth",
        rest_lip_center is not None
        and swiveled_lip_center is not None
        and abs(swiveled_lip_center[1] - rest_lip_center[1]) > 0.12,
        details=f"rest={rest_lip_center}, swiveled={swiveled_lip_center}",
    )

    return ctx.report()


object_model = build_object_model()

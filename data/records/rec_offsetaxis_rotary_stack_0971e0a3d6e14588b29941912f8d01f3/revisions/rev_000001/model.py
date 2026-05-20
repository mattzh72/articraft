from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.42
BASE_WIDTH = 0.34
BASE_HEIGHT = 0.045
BASE_FILLET = 0.018

LOWER_DRUM_RADIUS = 0.115
LOWER_DRUM_HEIGHT = 0.025
LOWER_SPIGOT_RADIUS = 0.080
LOWER_SPIGOT_HEIGHT = 0.010

LOWER_AXIS_Z = BASE_HEIGHT + LOWER_DRUM_HEIGHT + LOWER_SPIGOT_HEIGHT

LOWER_PLATTER_RADIUS = 0.185
LOWER_PLATTER_THICKNESS = 0.018
LOWER_HUB_RADIUS = 0.090
LOWER_HUB_HEIGHT = 0.012

UPPER_AXIS_OFFSET_X = 0.128
CHEEK_THICKNESS = 0.038
CHEEK_WIDTH = 0.098
CHEEK_HEIGHT = 0.155
WEB_WIDTH = 0.074
UPPER_HOUSING_RADIUS = 0.056
UPPER_HOUSING_HEIGHT = 0.028

UPPER_AXIS_Z = LOWER_PLATTER_THICKNESS + CHEEK_HEIGHT + UPPER_HOUSING_HEIGHT

UPPER_FLANGE_RADIUS = 0.055
UPPER_FLANGE_HEIGHT = 0.012
UPPER_BODY_RADIUS = 0.045
UPPER_BODY_HEIGHT = 0.045
UPPER_CAP_RADIUS = 0.062
UPPER_CAP_HEIGHT = 0.008
UPPER_POD_LENGTH = 0.060
UPPER_POD_WIDTH = 0.034
UPPER_POD_HEIGHT = 0.026
UPPER_TOTAL_HEIGHT = UPPER_FLANGE_HEIGHT + UPPER_BODY_HEIGHT + UPPER_CAP_HEIGHT


def _base_shape() -> cq.Workplane:
    plinth = (
        cq.Workplane("XY")
        .rect(BASE_LENGTH, BASE_WIDTH)
        .extrude(BASE_HEIGHT)
        .edges("|Z")
        .fillet(BASE_FILLET)
    )

    lower_drum = (
        cq.Workplane("XY")
        .circle(LOWER_DRUM_RADIUS)
        .extrude(LOWER_DRUM_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT))
    )
    lower_spigot = (
        cq.Workplane("XY")
        .circle(LOWER_SPIGOT_RADIUS)
        .extrude(LOWER_SPIGOT_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT + LOWER_DRUM_HEIGHT))
    )
    feet = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.145, -0.105),
                (-0.145, 0.105),
                (0.145, -0.105),
                (0.145, 0.105),
            ]
        )
        .rect(0.052, 0.040)
        .extrude(0.010)
    )

    return plinth.union(lower_drum).union(lower_spigot).union(feet)


def _lower_stage_shape() -> cq.Workplane:
    platter = (
        cq.Workplane("XY")
        .circle(LOWER_PLATTER_RADIUS)
        .extrude(LOWER_PLATTER_THICKNESS)
    )
    hub = (
        cq.Workplane("XY")
        .circle(LOWER_HUB_RADIUS)
        .extrude(LOWER_HUB_HEIGHT)
        .translate((0.0, 0.0, LOWER_PLATTER_THICKNESS))
    )

    cheek = (
        cq.Workplane("XY")
        .rect(CHEEK_THICKNESS, CHEEK_WIDTH)
        .extrude(CHEEK_HEIGHT)
        .translate((UPPER_AXIS_OFFSET_X, 0.0, LOWER_PLATTER_THICKNESS))
        .edges("|Z")
        .fillet(0.010)
    )

    cheek_top_z = LOWER_PLATTER_THICKNESS + CHEEK_HEIGHT
    support_web = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.040, LOWER_PLATTER_THICKNESS),
                (UPPER_AXIS_OFFSET_X - 0.024, LOWER_PLATTER_THICKNESS),
                (UPPER_AXIS_OFFSET_X - 0.010, cheek_top_z - 0.036),
                (0.070, LOWER_PLATTER_THICKNESS + 0.030),
            ]
        )
        .close()
        .extrude(WEB_WIDTH / 2.0, both=True)
    )

    upper_housing = (
        cq.Workplane("XY")
        .circle(UPPER_HOUSING_RADIUS)
        .extrude(UPPER_HOUSING_HEIGHT)
        .translate((UPPER_AXIS_OFFSET_X, 0.0, cheek_top_z))
    )
    housing_backer = (
        cq.Workplane("XY")
        .rect(0.070, 0.078)
        .extrude(0.020)
        .translate((UPPER_AXIS_OFFSET_X - 0.016, 0.0, cheek_top_z - 0.012))
        .edges("|Z")
        .fillet(0.006)
    )

    return (
        platter.union(hub)
        .union(cheek)
        .union(support_web)
        .union(housing_backer)
        .union(upper_housing)
    )


def _upper_head_shape() -> cq.Workplane:
    flange = cq.Workplane("XY").circle(UPPER_FLANGE_RADIUS).extrude(UPPER_FLANGE_HEIGHT)
    body = (
        cq.Workplane("XY")
        .circle(UPPER_BODY_RADIUS)
        .extrude(UPPER_BODY_HEIGHT)
        .translate((0.0, 0.0, UPPER_FLANGE_HEIGHT))
    )
    cap = (
        cq.Workplane("XY")
        .circle(UPPER_CAP_RADIUS)
        .extrude(UPPER_CAP_HEIGHT)
        .translate((0.0, 0.0, UPPER_FLANGE_HEIGHT + UPPER_BODY_HEIGHT))
    )

    pod = (
        cq.Workplane("XY")
        .rect(UPPER_POD_LENGTH, UPPER_POD_WIDTH)
        .extrude(UPPER_POD_HEIGHT)
        .translate(
            (
                UPPER_BODY_RADIUS + 0.5 * UPPER_POD_LENGTH - 0.010,
                0.0,
                0.022,
            )
        )
        .edges("|Z")
        .fillet(0.005)
    )
    pod_cap = (
        cq.Workplane("XY")
        .circle(0.014)
        .extrude(0.012)
        .translate(
            (
                UPPER_BODY_RADIUS + UPPER_POD_LENGTH - 0.004,
                0.0,
                0.029,
            )
        )
    )
    top_boss = (
        cq.Workplane("XY")
        .circle(0.020)
        .extrude(0.010)
        .translate((0.0, 0.0, UPPER_TOTAL_HEIGHT))
    )

    return flange.union(body).union(cap).union(pod).union(pod_cap).union(top_boss)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_offset_dual_rotary_stack")

    model.material("base_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("stage_gray", rgba=(0.60, 0.63, 0.66, 1.0))
    model.material("head_blue", rgba=(0.24, 0.41, 0.72, 1.0))

    base = model.part("base_pedestal")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base_pedestal"),
        material="base_dark",
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, LOWER_AXIS_Z)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, LOWER_AXIS_Z / 2.0)),
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        mesh_from_cadquery(_lower_stage_shape(), "lower_stage"),
        material="stage_gray",
        name="lower_stage_shell",
    )
    lower_stage.inertial = Inertial.from_geometry(
        Box((0.39, 0.37, UPPER_AXIS_Z)),
        mass=8.5,
        origin=Origin(xyz=(0.03, 0.0, UPPER_AXIS_Z / 2.0)),
    )

    upper_head = model.part("upper_head")
    upper_head.visual(
        mesh_from_cadquery(_upper_head_shape(), "upper_head"),
        material="head_blue",
        name="upper_head_shell",
    )
    upper_head.inertial = Inertial.from_geometry(
        Box((0.150, 0.124, UPPER_TOTAL_HEIGHT + 0.010)),
        mass=2.4,
        origin=Origin(xyz=(0.020, 0.0, (UPPER_TOTAL_HEIGHT + 0.010) / 2.0)),
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-3.0,
            upper=3.0,
            effort=35.0,
            velocity=1.8,
        ),
    )
    model.articulation(
        "lower_to_upper",
        ArticulationType.REVOLUTE,
        parent=lower_stage,
        child=upper_head,
        origin=Origin(xyz=(UPPER_AXIS_OFFSET_X, 0.0, UPPER_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-3.1,
            upper=3.1,
            effort=16.0,
            velocity=2.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_pedestal")
    lower_stage = object_model.get_part("lower_stage")
    upper_head = object_model.get_part("upper_head")
    base_to_lower = object_model.get_articulation("base_to_lower")
    lower_to_upper = object_model.get_articulation("lower_to_upper")

    ctx.check(
        "lower rotary axis is vertical",
        tuple(base_to_lower.axis) == (0.0, 0.0, 1.0),
        details=f"axis={base_to_lower.axis}",
    )
    ctx.check(
        "upper rotary axis is vertical",
        tuple(lower_to_upper.axis) == (0.0, 0.0, 1.0),
        details=f"axis={lower_to_upper.axis}",
    )
    ctx.expect_contact(
        lower_stage,
        base,
        name="lower turntable is seated on the base bearing",
    )
    ctx.expect_contact(
        upper_head,
        lower_stage,
        name="upper rotary head is supported by the raised cheek",
    )

    rest_upper_origin = ctx.part_world_position(upper_head)
    with ctx.pose({base_to_lower: 1.0}):
        swung_upper_origin = ctx.part_world_position(upper_head)

    offset_ok = False
    if rest_upper_origin is not None:
        offset_ok = math.hypot(rest_upper_origin[0], rest_upper_origin[1]) > 0.10
    ctx.check(
        "upper axis is displaced from the base centerline",
        offset_ok,
        details=f"rest_upper_origin={rest_upper_origin}",
    )

    lower_motion_ok = False
    if rest_upper_origin is not None and swung_upper_origin is not None:
        xy_shift = math.hypot(
            swung_upper_origin[0] - rest_upper_origin[0],
            swung_upper_origin[1] - rest_upper_origin[1],
        )
        z_shift = abs(swung_upper_origin[2] - rest_upper_origin[2])
        lower_motion_ok = xy_shift > 0.10 and z_shift < 0.002
    ctx.check(
        "lower stage carries the offset upper axis around the base",
        lower_motion_ok,
        details=f"rest={rest_upper_origin}, swung={swung_upper_origin}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    rest_upper_shell = _aabb_center(
        ctx.part_element_world_aabb(upper_head, elem="upper_head_shell")
    )
    with ctx.pose({lower_to_upper: 1.2}):
        rotated_upper_shell = _aabb_center(
            ctx.part_element_world_aabb(upper_head, elem="upper_head_shell")
        )

    upper_motion_ok = False
    if rest_upper_shell is not None and rotated_upper_shell is not None:
        upper_motion_ok = (
            math.hypot(
                rotated_upper_shell[0] - rest_upper_shell[0],
                rotated_upper_shell[1] - rest_upper_shell[1],
            )
            > 0.008
        )
    ctx.check(
        "upper head rotates independently on its own offset axis",
        upper_motion_ok,
        details=f"rest={rest_upper_shell}, rotated={rotated_upper_shell}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import pi

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


BASE_W = 0.160
BASE_D = 0.160
BASE_H = 0.052
BASE_WALL = 0.006
BASE_BOTTOM = 0.008

SUPPORT_CENTER_X = 0.056
SUPPORT_PEDESTAL_X = 0.020
SUPPORT_PEDESTAL_Y = 0.036
SUPPORT_PEDESTAL_Z = 0.038
SUPPORT_CHEEK_X = 0.020
SUPPORT_CHEEK_Y = 0.010
SUPPORT_CHEEK_Z = 0.040
SUPPORT_CHEEK_OFFSET_Y = 0.0092
SUPPORT_CHEEK_CENTER_Z = 0.058

PITCH_AXIS_Z = 0.064
PITCH_LIMIT = 0.42

OUTER_SHAFT_RADIUS = 0.0042
OUTER_SHAFT_LENGTH = 0.124
OUTER_BRIDGE_X = 0.026
OUTER_BRIDGE_Y = 0.056
OUTER_BRIDGE_Z = 0.010
OUTER_BRIDGE_CENTER_Z = 0.006
OUTER_UPRIGHT_X = 0.006
OUTER_UPRIGHT_Y = 0.010
OUTER_UPRIGHT_Z = 0.040
OUTER_UPRIGHT_X_OFFSET = 0.0065
OUTER_UPRIGHT_Y_OFFSET = 0.028
OUTER_UPRIGHT_CENTER_Z = 0.026

ROLL_AXIS_Z = 0.024
ROLL_LIMIT = 0.38

INNER_SHAFT_RADIUS = 0.0035
INNER_SHAFT_LENGTH = 0.066
INNER_BRIDGE_X = 0.050
INNER_BRIDGE_Y = 0.008
INNER_BRIDGE_Z = 0.010
INNER_BRIDGE_CENTER_Z = 0.004
INNER_HUB_X = 0.018
INNER_HUB_Y = 0.018
INNER_HUB_Z = 0.016
INNER_HUB_CENTER_Z = 0.004
INNER_CHEEK_X = 0.010
INNER_CHEEK_Y = 0.008
INNER_CHEEK_Z = 0.030
INNER_CHEEK_X_OFFSET = 0.020
INNER_CHEEK_CENTER_Z = 0.018

LEVER_BASE = 0.024
LEVER_BASE_H = 0.008
LEVER_SHAFT = 0.011
LEVER_SHAFT_H = 0.046
LEVER_CAP = 0.024
LEVER_CAP_H = 0.018


def _make_housing_shell_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BASE_W, BASE_D, BASE_H).translate((0.0, 0.0, BASE_H / 2.0))
    cavity = (
        cq.Workplane("XY")
        .box(BASE_W - 2.0 * BASE_WALL, BASE_D - 2.0 * BASE_WALL, BASE_H - BASE_BOTTOM + 0.004)
        .translate((0.0, 0.0, BASE_BOTTOM + (BASE_H - BASE_BOTTOM + 0.004) / 2.0))
    )
    return outer.cut(cavity)


def _make_support_shape() -> cq.Workplane:
    pedestal = (
        cq.Workplane("XY")
        .box(SUPPORT_PEDESTAL_X, SUPPORT_PEDESTAL_Y, SUPPORT_PEDESTAL_Z)
        .translate((0.0, 0.0, SUPPORT_PEDESTAL_Z / 2.0))
    )
    support = pedestal
    for sign in (-1.0, 1.0):
        cheek = (
            cq.Workplane("XY")
            .box(SUPPORT_CHEEK_X, SUPPORT_CHEEK_Y, SUPPORT_CHEEK_Z)
            .translate((0.0, sign * SUPPORT_CHEEK_OFFSET_Y, SUPPORT_CHEEK_CENTER_Z))
        )
        support = support.union(cheek)
    return support


def _make_outer_yoke_frame_shape() -> cq.Workplane:
    frame = (
        cq.Workplane("XY")
        .box(OUTER_BRIDGE_X, OUTER_BRIDGE_Y, OUTER_BRIDGE_Z)
        .translate((0.0, 0.0, OUTER_BRIDGE_CENTER_Z))
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            upright = (
                cq.Workplane("XY")
                .box(OUTER_UPRIGHT_X, OUTER_UPRIGHT_Y, OUTER_UPRIGHT_Z)
                .translate(
                    (
                        x_sign * OUTER_UPRIGHT_X_OFFSET,
                        y_sign * OUTER_UPRIGHT_Y_OFFSET,
                        OUTER_UPRIGHT_CENTER_Z,
                    )
                )
            )
            frame = frame.union(upright)
    return frame


def _make_inner_cradle_frame_shape() -> cq.Workplane:
    bridge = (
        cq.Workplane("XY")
        .box(INNER_BRIDGE_X, INNER_BRIDGE_Y, INNER_BRIDGE_Z)
        .translate((0.0, 0.0, INNER_BRIDGE_CENTER_Z))
    )
    hub = (
        cq.Workplane("XY")
        .box(INNER_HUB_X, INNER_HUB_Y, INNER_HUB_Z)
        .translate((0.0, 0.0, INNER_HUB_CENTER_Z))
    )
    frame = bridge.union(hub)
    for sign in (-1.0, 1.0):
        cheek = (
            cq.Workplane("XY")
            .box(INNER_CHEEK_X, INNER_CHEEK_Y, INNER_CHEEK_Z)
            .translate((sign * INNER_CHEEK_X_OFFSET, 0.0, INNER_CHEEK_CENTER_Z))
        )
        frame = frame.union(cheek)
    return frame


def _make_lever_shape() -> cq.Workplane:
    base = cq.Workplane("XY").rect(LEVER_BASE, LEVER_BASE).extrude(LEVER_BASE_H)
    shaft = cq.Workplane("XY").circle(LEVER_SHAFT / 2.0).extrude(LEVER_SHAFT_H).translate(
        (0.0, 0.0, LEVER_BASE_H)
    )
    cap = cq.Workplane("XY").rect(LEVER_CAP, LEVER_CAP).extrude(LEVER_CAP_H).translate(
        (0.0, 0.0, LEVER_BASE_H + LEVER_SHAFT_H)
    )
    return base.union(shaft).union(cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_gimbal_joystick")
    model.material("housing_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("frame_metal", rgba=(0.68, 0.70, 0.73, 1.0))
    model.material("lever_dark", rgba=(0.12, 0.12, 0.14, 1.0))

    housing = model.part("base_housing")
    housing.visual(
        mesh_from_cadquery(_make_housing_shell_shape(), "housing_shell"),
        material="housing_dark",
        name="housing_shell",
    )
    for side_name, x_sign in (("left_support", -1.0), ("right_support", 1.0)):
        housing.visual(
            mesh_from_cadquery(_make_support_shape(), side_name),
            origin=Origin(xyz=(x_sign * SUPPORT_CENTER_X, 0.0, 0.0)),
            material="housing_dark",
            name=side_name,
        )
    housing.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, SUPPORT_CHEEK_CENTER_Z + SUPPORT_CHEEK_Z / 2.0)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, (SUPPORT_CHEEK_CENTER_Z + SUPPORT_CHEEK_Z / 2.0) / 2.0)),
    )

    outer_yoke = model.part("outer_pitch_yoke")
    outer_yoke.visual(
        mesh_from_cadquery(_make_outer_yoke_frame_shape(), "outer_pitch_yoke"),
        material="frame_metal",
        name="yoke_frame",
    )
    outer_yoke.visual(
        Cylinder(radius=OUTER_SHAFT_RADIUS, length=OUTER_SHAFT_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="frame_metal",
        name="pitch_axle",
    )
    outer_yoke.inertial = Inertial.from_geometry(
        Box((OUTER_SHAFT_LENGTH, 0.066, 0.052)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    inner_cradle = model.part("inner_roll_cradle")
    inner_cradle.visual(
        mesh_from_cadquery(_make_inner_cradle_frame_shape(), "inner_roll_cradle"),
        material="frame_metal",
        name="cradle_frame",
    )
    inner_cradle.visual(
        Cylinder(radius=INNER_SHAFT_RADIUS, length=INNER_SHAFT_LENGTH),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="frame_metal",
        name="roll_journal",
    )
    inner_cradle.visual(
        mesh_from_cadquery(_make_lever_shape(), "lever_assembly"),
        material="lever_dark",
        name="lever_assembly",
    )
    inner_cradle.inertial = Inertial.from_geometry(
        Box((INNER_BRIDGE_X, INNER_SHAFT_LENGTH, LEVER_BASE_H + LEVER_SHAFT_H + LEVER_CAP_H)),
        mass=0.26,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
    )

    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=outer_yoke,
        origin=Origin(xyz=(0.0, 0.0, PITCH_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.2,
            lower=-PITCH_LIMIT,
            upper=PITCH_LIMIT,
        ),
    )
    model.articulation(
        "roll_joint",
        ArticulationType.REVOLUTE,
        parent=outer_yoke,
        child=inner_cradle,
        origin=Origin(xyz=(0.0, 0.0, ROLL_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.4,
            lower=-ROLL_LIMIT,
            upper=ROLL_LIMIT,
        ),
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

    housing = object_model.get_part("base_housing")
    outer_yoke = object_model.get_part("outer_pitch_yoke")
    inner_cradle = object_model.get_part("inner_roll_cradle")
    pitch_joint = object_model.get_articulation("pitch_joint")
    roll_joint = object_model.get_articulation("roll_joint")
    housing_shell = housing.get_visual("housing_shell")
    lever_visual = inner_cradle.get_visual("lever_assembly")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((low + high) / 2.0 for low, high in zip(min_corner, max_corner))

    ctx.check(
        "pitch joint uses x-axis",
        tuple(round(v, 6) for v in pitch_joint.axis) == (-1.0, 0.0, 0.0),
        details=f"axis={pitch_joint.axis}",
    )
    ctx.check(
        "roll joint uses y-axis",
        tuple(round(v, 6) for v in roll_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={roll_joint.axis}",
    )
    ctx.expect_origin_distance(
        inner_cradle,
        housing,
        axes="xy",
        max_dist=0.001,
        name="gimbal stays centered in housing plan",
    )
    ctx.expect_gap(
        inner_cradle,
        housing,
        axis="z",
        positive_elem=lever_visual,
        negative_elem=housing_shell,
        min_gap=0.006,
        name="lever sits proud of housing rim",
    )

    neutral_center = _aabb_center(ctx.part_element_world_aabb(inner_cradle, elem=lever_visual))
    with ctx.pose({pitch_joint: 0.30}):
        pitched_center = _aabb_center(ctx.part_element_world_aabb(inner_cradle, elem=lever_visual))
    ctx.check(
        "positive pitch tips lever forward",
        neutral_center is not None
        and pitched_center is not None
        and pitched_center[1] > neutral_center[1] + 0.010
        and pitched_center[2] < neutral_center[2] - 0.002,
        details=f"neutral={neutral_center}, pitched={pitched_center}",
    )

    with ctx.pose({roll_joint: 0.28}):
        rolled_center = _aabb_center(ctx.part_element_world_aabb(inner_cradle, elem=lever_visual))
    ctx.check(
        "positive roll tips lever to the right",
        neutral_center is not None
        and rolled_center is not None
        and rolled_center[0] > neutral_center[0] + 0.008,
        details=f"neutral={neutral_center}, rolled={rolled_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

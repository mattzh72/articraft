from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
import pathlib

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
)

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        return "/"


os.getcwd = _safe_getcwd
pathlib.os.getcwd = _safe_getcwd

FOOT_SIZE = (0.028, 0.014, 0.010)

FRONT_RAIL_LENGTH = 0.255
FRONT_RAIL_TILT = math.radians(23.0)
FRONT_RAIL_SIZE = (0.014, 0.006, FRONT_RAIL_LENGTH)
FRONT_RUN_X = FRONT_RAIL_LENGTH * math.sin(FRONT_RAIL_TILT)
FRONT_RUN_Z = FRONT_RAIL_LENGTH * math.cos(FRONT_RAIL_TILT)
TOP_Z = FOOT_SIZE[2] / 2.0 + FRONT_RUN_Z

TOP_HINGE_RADIUS = 0.0035
TOP_HINGE_KNUCKLE_LENGTH = 0.003
TOP_HINGE_BARREL_LENGTH = 0.006
TOP_KNUCKLE_OFFSET = 0.00375
RIGHT_RAIL_Y = 0.010
LEFT_RAIL_Y = -0.010

REAR_HINGE_RADIUS = 0.0032
REAR_HINGE_KNUCKLE_LENGTH = 0.003
REAR_HINGE_BARREL_LENGTH = 0.006
REAR_KNUCKLE_OFFSET = 0.0045

REAR_JOINT_Y = -0.016
REAR_JOINT_Z = TOP_Z - 0.0003
REAR_LEG_LENGTH = 0.253
REAR_LEG_TILT = math.radians(24.0)
REAR_LEG_SIZE = (0.006, 0.008, REAR_LEG_LENGTH)
REAR_RUN_Y = REAR_LEG_LENGTH * math.sin(REAR_LEG_TILT)
REAR_RUN_Z = REAR_LEG_LENGTH * math.cos(REAR_LEG_TILT)

LEFT_RAIL_CENTER = (-0.5 * FRONT_RUN_X, LEFT_RAIL_Y, TOP_Z - 0.5 * FRONT_RUN_Z)
LEFT_FOOT_CENTER = (-FRONT_RUN_X, LEFT_RAIL_Y, FOOT_SIZE[2] / 2.0)
REAR_MOUNT_CENTER = (0.0, -0.008, REAR_JOINT_Z - 0.0045)
REAR_MOUNT_SIZE = (0.016, 0.016, 0.009)

RIGHT_RAIL_CENTER_LOCAL = (0.5 * FRONT_RUN_X, 0.0, -0.5 * FRONT_RUN_Z)
RIGHT_RAIL_CENTER_LOCAL = (0.5 * FRONT_RUN_X, RIGHT_RAIL_Y, -0.5 * FRONT_RUN_Z)
RIGHT_MOUNT_CENTER_LOCAL = (0.0, 0.0065, -0.004)
RIGHT_MOUNT_SIZE = (0.008, 0.007, 0.010)
RIGHT_FOOT_CENTER_LOCAL = (FRONT_RUN_X, RIGHT_RAIL_Y, FOOT_SIZE[2] / 2.0 - TOP_Z)

REAR_LEG_CENTER_LOCAL = (0.0, -0.5 * REAR_RUN_Y, -0.5 * REAR_RUN_Z)
REAR_FOOT_CENTER_LOCAL = (0.0, -REAR_RUN_Y, FOOT_SIZE[2] / 2.0 - REAR_JOINT_Z)


def _origin_y_axis(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0))


def _origin_x_axis(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_display_easel")

    ash = model.material("ash_wood", rgba=(0.78, 0.67, 0.48, 1.0))
    hardware = model.material("charcoal_hardware", rgba=(0.24, 0.24, 0.25, 1.0))
    rubber = model.material("rubber_tip", rgba=(0.12, 0.12, 0.13, 1.0))

    front_tab_size = (0.008, 0.003, 0.012)
    front_tab_x = -0.004
    front_tab_z = TOP_Z - 0.006
    rear_tab_size = (0.003, 0.010, 0.012)
    rear_tab_y = -0.008
    rear_tab_z = REAR_JOINT_Z - 0.006
    right_mount_size = (0.007, 0.004, 0.010)
    rear_strap_size = (0.012, 0.012, 0.012)

    left_rail = model.part("left_rail")
    left_rail.visual(
        Box(FRONT_RAIL_SIZE),
        origin=Origin(xyz=LEFT_RAIL_CENTER, rpy=(0.0, FRONT_RAIL_TILT, 0.0)),
        material=ash,
        name="side_rail",
    )
    left_rail.visual(
        Box(front_tab_size),
        origin=Origin(xyz=(front_tab_x, -TOP_KNUCKLE_OFFSET - 0.5 * LEFT_RAIL_Y, front_tab_z)),
        material=hardware,
        name="hinge_back_tab",
    )
    left_rail.visual(
        Box(front_tab_size),
        origin=Origin(xyz=(front_tab_x, TOP_KNUCKLE_OFFSET - 0.5 * LEFT_RAIL_Y, front_tab_z)),
        material=hardware,
        name="front_hinge_tab",
    )
    left_rail.visual(
        Box(FOOT_SIZE),
        origin=Origin(xyz=LEFT_FOOT_CENTER),
        material=rubber,
        name="left_foot",
    )
    left_rail.visual(
        Cylinder(radius=TOP_HINGE_RADIUS, length=TOP_HINGE_KNUCKLE_LENGTH),
        origin=_origin_y_axis((0.0, -TOP_KNUCKLE_OFFSET, TOP_Z)),
        material=hardware,
        name="hinge_knuckle_back",
    )
    left_rail.visual(
        Cylinder(radius=TOP_HINGE_RADIUS, length=TOP_HINGE_KNUCKLE_LENGTH),
        origin=_origin_y_axis((0.0, TOP_KNUCKLE_OFFSET, TOP_Z)),
        material=hardware,
        name="hinge_knuckle_front",
    )
    left_rail.visual(
        Box(rear_tab_size),
        origin=Origin(xyz=(-REAR_KNUCKLE_OFFSET, rear_tab_y - 0.5 * LEFT_RAIL_Y, rear_tab_z)),
        material=hardware,
        name="rear_hinge_tab_left",
    )
    left_rail.visual(
        Box(rear_tab_size),
        origin=Origin(xyz=(REAR_KNUCKLE_OFFSET, rear_tab_y - 0.5 * LEFT_RAIL_Y, rear_tab_z)),
        material=hardware,
        name="rear_hinge_tab_right",
    )
    left_rail.visual(
        Cylinder(radius=REAR_HINGE_RADIUS, length=REAR_HINGE_KNUCKLE_LENGTH),
        origin=_origin_x_axis((-REAR_KNUCKLE_OFFSET, REAR_JOINT_Y, REAR_JOINT_Z)),
        material=hardware,
        name="rear_hinge_knuckle_left",
    )
    left_rail.visual(
        Cylinder(radius=REAR_HINGE_RADIUS, length=REAR_HINGE_KNUCKLE_LENGTH),
        origin=_origin_x_axis((REAR_KNUCKLE_OFFSET, REAR_JOINT_Y, REAR_JOINT_Z)),
        material=hardware,
        name="rear_hinge_knuckle_right",
    )
    left_rail.inertial = Inertial.from_geometry(
        Box((0.12, 0.03, 0.26)),
        mass=0.12,
        origin=Origin(xyz=(-0.060, 0.0, 0.125)),
    )

    right_rail = model.part("right_rail")
    right_rail.visual(
        Box(FRONT_RAIL_SIZE),
        origin=Origin(xyz=RIGHT_RAIL_CENTER_LOCAL, rpy=(0.0, -FRONT_RAIL_TILT, 0.0)),
        material=ash,
        name="side_rail",
    )
    right_rail.visual(
        Box(right_mount_size),
        origin=Origin(xyz=(0.0025, 0.0, -0.005)),
        material=hardware,
        name="right_hinge_mount",
    )
    right_rail.visual(
        Cylinder(radius=TOP_HINGE_RADIUS, length=TOP_HINGE_BARREL_LENGTH),
        origin=_origin_y_axis((0.0, 0.0, 0.0)),
        material=hardware,
        name="hinge_barrel",
    )
    right_rail.visual(
        Box(FOOT_SIZE),
        origin=Origin(xyz=RIGHT_FOOT_CENTER_LOCAL),
        material=rubber,
        name="right_foot",
    )
    right_rail.inertial = Inertial.from_geometry(
        Box((0.12, 0.03, 0.26)),
        mass=0.12,
        origin=Origin(xyz=(0.060, 0.0, -0.125)),
    )

    rear_support = model.part("rear_support")
    rear_support.visual(
        Cylinder(radius=REAR_HINGE_RADIUS, length=REAR_HINGE_BARREL_LENGTH),
        origin=_origin_x_axis((0.0, 0.0, 0.0)),
        material=hardware,
        name="rear_hinge_barrel",
    )
    rear_support.visual(
        Box(rear_strap_size),
        origin=Origin(xyz=(0.0, -0.006, -0.006)),
        material=hardware,
        name="rear_hinge_strap",
    )
    rear_support.visual(
        Box(REAR_LEG_SIZE),
        origin=Origin(xyz=REAR_LEG_CENTER_LOCAL, rpy=(-REAR_LEG_TILT, 0.0, 0.0)),
        material=ash,
        name="support_leg",
    )
    rear_support.visual(
        Box((0.024, 0.016, FOOT_SIZE[2])),
        origin=Origin(xyz=REAR_FOOT_CENTER_LOCAL),
        material=rubber,
        name="rear_foot",
    )
    rear_support.inertial = Inertial.from_geometry(
        Box((0.04, 0.14, 0.25)),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.060, -0.120)),
    )

    model.articulation(
        "left_to_right_rail",
        ArticulationType.REVOLUTE,
        parent=left_rail,
        child=right_rail,
        origin=Origin(xyz=(0.0, 0.0, TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(38.0),
        ),
    )
    model.articulation(
        "left_rail_to_rear_support",
        ArticulationType.REVOLUTE,
        parent=left_rail,
        child=rear_support,
        origin=Origin(xyz=(0.0, REAR_JOINT_Y, REAR_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(20.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)
    left_rail = object_model.get_part("left_rail")
    right_rail = object_model.get_part("right_rail")
    rear_support = object_model.get_part("rear_support")
    rail_hinge = object_model.get_articulation("left_to_right_rail")
    rear_hinge = object_model.get_articulation("left_rail_to_rear_support")

    left_back_knuckle = left_rail.get_visual("hinge_knuckle_back")
    left_front_knuckle = left_rail.get_visual("hinge_knuckle_front")
    rear_left_knuckle = left_rail.get_visual("rear_hinge_knuckle_left")
    rear_right_knuckle = left_rail.get_visual("rear_hinge_knuckle_right")
    left_foot = left_rail.get_visual("left_foot")
    right_foot = right_rail.get_visual("right_foot")
    rear_foot = rear_support.get_visual("rear_foot")
    left_side_rail = left_rail.get_visual("side_rail")
    right_side_rail = right_rail.get_visual("side_rail")
    right_mount = right_rail.get_visual("right_hinge_mount")
    right_barrel = right_rail.get_visual("hinge_barrel")
    rear_barrel = rear_support.get_visual("rear_hinge_barrel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()

    ctx.expect_contact(right_rail, right_rail, elem_a=right_mount, elem_b=right_barrel, contact_tol=0.002)
    ctx.expect_contact(right_rail, left_rail, elem_a=right_barrel, elem_b=left_front_knuckle)
    ctx.expect_contact(right_rail, left_rail, elem_a=right_barrel, elem_b=left_back_knuckle)
    ctx.expect_contact(rear_support, left_rail, elem_a=rear_barrel, elem_b=rear_left_knuckle)
    ctx.expect_contact(rear_support, left_rail, elem_a=rear_barrel, elem_b=rear_right_knuckle)
    ctx.expect_gap(
        right_rail,
        left_rail,
        axis="x",
        min_gap=0.17,
        positive_elem=right_foot,
        negative_elem=left_foot,
        name="front_feet_form_a_stable_display_span",
    )
    ctx.expect_gap(
        left_rail,
        rear_support,
        axis="y",
        min_gap=0.08,
        positive_elem=left_foot,
        negative_elem=rear_foot,
        name="rear_support_leg_sits_behind_the_front_rails",
    )
    with ctx.pose({rail_hinge: rail_hinge.motion_limits.upper}):
        ctx.fail_if_isolated_parts(name="right_rail_folded_no_floating")
        ctx.expect_gap(
            right_rail,
            left_rail,
            axis="x",
            max_gap=0.040,
            max_penetration=0.005,
            positive_elem=right_foot,
            negative_elem=left_foot,
            name="right_rail_can_fold_nearly_flat_against_left_rail",
        )
    with ctx.pose({rail_hinge: rail_hinge.motion_limits.lower}):
        ctx.fail_if_isolated_parts(name="right_rail_wide_open_no_floating")
        ctx.expect_gap(
            right_rail,
            left_rail,
            axis="x",
            min_gap=0.17,
            positive_elem=right_foot,
            negative_elem=left_foot,
            name="right_rail_can_open_wider_for_a_broader_stance",
        )
    with ctx.pose({rear_hinge: rear_hinge.motion_limits.upper}):
        ctx.fail_if_isolated_parts(name="rear_leg_folded_no_floating")
        ctx.expect_gap(
            left_rail,
            rear_support,
            axis="y",
            max_gap=0.045,
            max_penetration=0.0,
            positive_elem=left_foot,
            negative_elem=rear_foot,
            name="rear_leg_can_fold_close_to_the_front_frame",
        )
    with ctx.pose({rear_hinge: rear_hinge.motion_limits.lower}):
        ctx.fail_if_isolated_parts(name="rear_leg_spread_no_floating")
        ctx.expect_gap(
            left_rail,
            rear_support,
            axis="y",
            min_gap=0.08,
            positive_elem=left_foot,
            negative_elem=rear_foot,
            name="rear_leg_can_open_for_a_deeper_stance",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

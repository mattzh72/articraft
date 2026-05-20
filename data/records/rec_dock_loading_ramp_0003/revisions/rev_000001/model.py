from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os
import pathlib


def _repair_cwd() -> str:
    for candidate in ("/tmp", "/"):
        try:
            os.chdir(candidate)
            return candidate
        except FileNotFoundError:
            continue
    return "/tmp"


_ORIG_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIG_GETCWD()
    except FileNotFoundError:
        _repair_cwd()
        try:
            return _ORIG_GETCWD()
        except FileNotFoundError:
            return "/tmp"


os.getcwd = _safe_getcwd
_safe_getcwd()

_ORIG_PATH_CWD = pathlib.Path.cwd.__func__


def _safe_path_cwd(cls) -> pathlib.Path:
    try:
        return _ORIG_PATH_CWD(cls)
    except FileNotFoundError:
        return cls(_safe_getcwd())


pathlib.Path.cwd = classmethod(_safe_path_cwd)

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
)

DECK_LENGTH = 2.05
DECK_WIDTH = 1.83
DECK_PLATE_THICK = 0.012
DECK_TOP_LOCAL_Z = 0.05
DECK_UNDERSIDE_LOCAL_Z = DECK_TOP_LOCAL_Z - DECK_PLATE_THICK
FRONT_HINGE_Z = 0.012

REAR_HINGE_Z = -0.025
REAR_HINGE_RADIUS = 0.025
FRONT_HINGE_RADIUS = 0.022

REAR_PLATE_START_X = REAR_HINGE_RADIUS

LIP_LENGTH = 0.41
LIP_WIDTH = 1.77
LIP_THICK = 0.016
LIP_TOP_LOCAL_Z = DECK_TOP_LOCAL_Z - FRONT_HINGE_Z
LIP_PLATE_START_X = FRONT_HINGE_RADIUS + 0.004

FRAME_OUTER_WIDTH = 2.05
FRAME_SIDE_WIDTH = 0.11


def _add_y_axis_knuckle(
    part,
    *,
    name: str,
    radius: float,
    length: float,
    center_x: float,
    center_y: float,
    center_z: float,
    material,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(center_x, center_y, center_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_dock_leveler")

    frame_steel = model.material("frame_steel", rgba=(0.20, 0.22, 0.25, 1.0))
    deck_steel = model.material("deck_steel", rgba=(0.43, 0.45, 0.48, 1.0))
    lip_steel = model.material("lip_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.92, 0.76, 0.14, 1.0))

    base = model.part("base_frame")
    base.visual(
        Box((0.175, FRAME_OUTER_WIDTH, 0.30)),
        origin=Origin(xyz=(-0.1125, 0.0, -0.18)),
        material=frame_steel,
        name="rear_header",
    )
    base.visual(
        Box((2.20, FRAME_SIDE_WIDTH, 0.24)),
        origin=Origin(xyz=(1.10, 0.97, -0.23)),
        material=frame_steel,
        name="left_pit_angle",
    )
    base.visual(
        Box((2.20, FRAME_SIDE_WIDTH, 0.24)),
        origin=Origin(xyz=(1.10, -0.97, -0.23)),
        material=frame_steel,
        name="right_pit_angle",
    )
    base.visual(
        Box((0.16, FRAME_OUTER_WIDTH, 0.18)),
        origin=Origin(xyz=(2.13, 0.0, -0.25)),
        material=frame_steel,
        name="front_sill",
    )
    base.visual(
        Box((2.20, 1.95, 0.05)),
        origin=Origin(xyz=(1.10, 0.0, -0.325)),
        material=frame_steel,
        name="support_floor",
    )
    base.visual(
        Box((0.12, 1.60, 0.05)),
        origin=Origin(xyz=(-0.02, 0.0, -0.325)),
        material=frame_steel,
        name="rear_floor_bridge",
    )
    _add_y_axis_knuckle(
        base,
        name="rear_hinge_knuckle_left",
        radius=REAR_HINGE_RADIUS,
        length=0.24,
        center_x=0.0,
        center_y=-0.72,
        center_z=REAR_HINGE_Z,
        material=frame_steel,
    )
    _add_y_axis_knuckle(
        base,
        name="rear_hinge_knuckle_center",
        radius=REAR_HINGE_RADIUS,
        length=0.24,
        center_x=0.0,
        center_y=0.0,
        center_z=REAR_HINGE_Z,
        material=frame_steel,
    )
    _add_y_axis_knuckle(
        base,
        name="rear_hinge_knuckle_right",
        radius=REAR_HINGE_RADIUS,
        length=0.24,
        center_x=0.0,
        center_y=0.72,
        center_z=REAR_HINGE_Z,
        material=frame_steel,
    )
    for name, y in (
        ("rear_hinge_web_left", -0.72),
        ("rear_hinge_web_center", 0.0),
        ("rear_hinge_web_right", 0.72),
    ):
        base.visual(
            Box((0.10, 0.24, 0.05)),
            origin=Origin(xyz=(-0.05, y, -0.025)),
            material=frame_steel,
            name=name,
        )
    base.inertial = Inertial.from_geometry(
        Box((2.40, FRAME_OUTER_WIDTH, 0.33)),
        mass=900.0,
        origin=Origin(xyz=(1.00, 0.0, -0.18)),
    )

    deck = model.part("deck_plate")
    deck.visual(
        Box((DECK_LENGTH - REAR_PLATE_START_X, DECK_WIDTH, DECK_PLATE_THICK)),
        origin=Origin(
            xyz=(
                REAR_PLATE_START_X + ((DECK_LENGTH - REAR_PLATE_START_X) / 2.0),
                0.0,
                DECK_TOP_LOCAL_Z - (DECK_PLATE_THICK / 2.0),
            )
        ),
        material=deck_steel,
        name="deck_surface",
    )
    deck.visual(
        Box((1.72, 0.14, 0.148)),
        origin=Origin(xyz=(1.06, 0.60, DECK_UNDERSIDE_LOCAL_Z - 0.074)),
        material=deck_steel,
        name="left_channel",
    )
    deck.visual(
        Box((1.72, 0.14, 0.148)),
        origin=Origin(xyz=(1.06, -0.60, DECK_UNDERSIDE_LOCAL_Z - 0.074)),
        material=deck_steel,
        name="right_channel",
    )
    deck.visual(
        Box((1.46, 0.20, 0.128)),
        origin=Origin(xyz=(0.89, 0.0, DECK_UNDERSIDE_LOCAL_Z - 0.064)),
        material=deck_steel,
        name="center_box_beam",
    )
    for name, y in (
        ("rear_hinge_leaf_left", -0.72),
        ("rear_hinge_leaf_center", 0.0),
        ("rear_hinge_leaf_right", 0.72),
    ):
        deck.visual(
            Box((0.10, 0.24, 0.056)),
            origin=Origin(xyz=(0.061, y, 0.012)),
            material=deck_steel,
            name=name,
        )
    _add_y_axis_knuckle(
        deck,
        name="front_hinge_knuckle_outer_left",
        radius=FRONT_HINGE_RADIUS,
        length=0.32,
        center_x=DECK_LENGTH,
        center_y=-0.72,
        center_z=FRONT_HINGE_Z,
        material=deck_steel,
    )
    _add_y_axis_knuckle(
        deck,
        name="front_hinge_knuckle_center",
        radius=FRONT_HINGE_RADIUS,
        length=0.32,
        center_x=DECK_LENGTH,
        center_y=0.0,
        center_z=FRONT_HINGE_Z,
        material=deck_steel,
    )
    _add_y_axis_knuckle(
        deck,
        name="front_hinge_knuckle_outer_right",
        radius=FRONT_HINGE_RADIUS,
        length=0.32,
        center_x=DECK_LENGTH,
        center_y=0.72,
        center_z=FRONT_HINGE_Z,
        material=deck_steel,
    )
    deck.visual(
        Box((0.048, 1.58, 0.048)),
        origin=Origin(xyz=(DECK_LENGTH - 0.046, 0.0, 0.014)),
        material=deck_steel,
        name="front_hinge_carrier",
    )
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_WIDTH, 0.19)),
        mass=430.0,
        origin=Origin(xyz=(DECK_LENGTH / 2.0, 0.0, -0.07)),
    )

    lip = model.part("dock_lip")
    lip.visual(
        Box((LIP_LENGTH - LIP_PLATE_START_X, LIP_WIDTH, LIP_THICK)),
        origin=Origin(
            xyz=(
                LIP_PLATE_START_X + ((LIP_LENGTH - LIP_PLATE_START_X) / 2.0),
                0.0,
                LIP_TOP_LOCAL_Z - (LIP_THICK / 2.0),
            )
        ),
        material=lip_steel,
        name="lip_surface",
    )
    _add_y_axis_knuckle(
        lip,
        name="lip_hinge_knuckle_left",
        radius=FRONT_HINGE_RADIUS,
        length=0.32,
        center_x=0.0,
        center_y=-0.72,
        center_z=0.0,
        material=lip_steel,
    )
    _add_y_axis_knuckle(
        lip,
        name="lip_hinge_knuckle_center",
        radius=FRONT_HINGE_RADIUS,
        length=0.32,
        center_x=0.0,
        center_y=0.0,
        center_z=0.0,
        material=lip_steel,
    )
    _add_y_axis_knuckle(
        lip,
        name="lip_hinge_knuckle_right",
        radius=FRONT_HINGE_RADIUS,
        length=0.32,
        center_x=0.0,
        center_y=0.72,
        center_z=0.0,
        material=lip_steel,
    )
    lip.visual(
        Box((0.028, 1.58, 0.044)),
        origin=Origin(xyz=(0.036, 0.0, 0.0)),
        material=lip_steel,
        name="lip_hinge_carrier",
    )
    lip.visual(
        Box((0.055, LIP_WIDTH, 0.050)),
        origin=Origin(xyz=(0.3825, 0.0, -0.003)),
        material=lip_steel,
        name="lip_nose_bar",
    )
    lip.visual(
        Box((0.060, LIP_WIDTH, 0.003)),
        origin=Origin(xyz=(0.380, 0.0, LIP_TOP_LOCAL_Z + 0.0015)),
        material=safety_yellow,
        name="warning_stripe",
    )
    lip.inertial = Inertial.from_geometry(
        Box((LIP_LENGTH, LIP_WIDTH, 0.08)),
        mass=115.0,
        origin=Origin(xyz=(LIP_LENGTH / 2.0, 0.0, -0.01)),
    )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=deck,
        origin=Origin(xyz=(0.0, 0.0, REAR_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30000.0,
            velocity=0.8,
            lower=-0.45,
            upper=0.0,
        ),
    )
    model.articulation(
        "front_lip_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lip,
        origin=Origin(xyz=(DECK_LENGTH, 0.0, FRONT_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12000.0,
            velocity=1.2,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)
    base = object_model.get_part("base_frame")
    deck = object_model.get_part("deck_plate")
    lip = object_model.get_part("dock_lip")

    rear_hinge = object_model.get_articulation("rear_hinge")
    front_hinge = object_model.get_articulation("front_lip_hinge")

    deck_surface = deck.get_visual("deck_surface")
    rear_header = base.get_visual("rear_header")
    front_sill = base.get_visual("front_sill")
    rear_base_left_knuckle = base.get_visual("rear_hinge_knuckle_left")
    rear_base_center_knuckle = base.get_visual("rear_hinge_knuckle_center")
    rear_base_right_knuckle = base.get_visual("rear_hinge_knuckle_right")
    rear_deck_left_leaf = deck.get_visual("rear_hinge_leaf_left")
    rear_deck_center_leaf = deck.get_visual("rear_hinge_leaf_center")
    rear_deck_right_leaf = deck.get_visual("rear_hinge_leaf_right")
    front_deck_left_knuckle = deck.get_visual("front_hinge_knuckle_outer_left")
    front_deck_center_knuckle = deck.get_visual("front_hinge_knuckle_center")
    front_deck_right_knuckle = deck.get_visual("front_hinge_knuckle_outer_right")
    lip_surface = lip.get_visual("lip_surface")
    lip_left_knuckle = lip.get_visual("lip_hinge_knuckle_left")
    lip_center_knuckle = lip.get_visual("lip_hinge_knuckle_center")
    lip_right_knuckle = lip.get_visual("lip_hinge_knuckle_right")
    lip_nose_bar = lip.get_visual("lip_nose_bar")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        base,
        deck,
        elem_a=rear_base_left_knuckle,
        elem_b=rear_deck_left_leaf,
        reason="rear hinge pin barrel is captured inside the left deck hinge leaf",
    )
    ctx.allow_overlap(
        base,
        deck,
        elem_a=rear_base_center_knuckle,
        elem_b=rear_deck_center_leaf,
        reason="rear hinge pin barrel is captured inside the center deck hinge leaf",
    )
    ctx.allow_overlap(
        base,
        deck,
        elem_a=rear_base_right_knuckle,
        elem_b=rear_deck_right_leaf,
        reason="rear hinge pin barrel is captured inside the right deck hinge leaf",
    )
    ctx.allow_overlap(
        deck,
        lip,
        elem_a=front_deck_left_knuckle,
        elem_b=lip_left_knuckle,
        reason="front lip hinge uses an interleaved left barrel around a shared pin axis",
    )
    ctx.allow_overlap(
        deck,
        lip,
        elem_a=front_deck_center_knuckle,
        elem_b=lip_center_knuckle,
        reason="front lip hinge uses an interleaved center barrel around a shared pin axis",
    )
    ctx.allow_overlap(
        deck,
        lip,
        elem_a=front_deck_right_knuckle,
        elem_b=lip_right_knuckle,
        reason="front lip hinge uses an interleaved right barrel around a shared pin axis",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    rear_limits = rear_hinge.motion_limits
    front_limits = front_hinge.motion_limits
    ctx.check(
        "rear_hinge_axis_and_limits",
        rear_hinge.axis == (0.0, 1.0, 0.0)
        and rear_limits is not None
        and rear_limits.lower == -0.45
        and rear_limits.upper == 0.0,
        f"rear hinge should rotate about +Y from -0.45 to 0.0 rad, got axis={rear_hinge.axis}, limits={rear_limits}",
    )
    ctx.check(
        "front_lip_hinge_axis_and_limits",
        front_hinge.axis == (0.0, 1.0, 0.0)
        and front_limits is not None
        and front_limits.lower == 0.0
        and front_limits.upper == 1.35,
        f"front lip hinge should rotate about +Y from 0.0 to 1.35 rad, got axis={front_hinge.axis}, limits={front_limits}",
    )

    ctx.expect_contact(
        deck,
        base,
        elem_a=rear_deck_center_leaf,
        elem_b=rear_base_center_knuckle,
        name="rear hinge center barrel keeps the deck mounted to the frame",
    )
    ctx.expect_contact(
        deck,
        base,
        elem_a=rear_deck_right_leaf,
        elem_b=rear_base_right_knuckle,
        name="rear hinge engages on the right side as well",
    )
    ctx.expect_contact(
        lip,
        deck,
        elem_a=lip_center_knuckle,
        elem_b=front_deck_center_knuckle,
        name="front lip hinge barrel keeps the lip mounted to the deck",
    )
    ctx.expect_gap(
        deck,
        base,
        axis="z",
        min_gap=0.040,
        max_gap=0.045,
        positive_elem=deck_surface,
        negative_elem=rear_header,
        name="closed deck plate sits just above the rear pit header",
    )
    ctx.expect_gap(
        lip,
        deck,
        axis="x",
        min_gap=0.0,
        max_gap=0.01,
        max_penetration=0.0,
        positive_elem=lip_surface,
        negative_elem=front_deck_center_knuckle,
        name="lip plate begins just beyond the front hinge barrel",
    )
    ctx.expect_overlap(
        lip,
        deck,
        axes="y",
        min_overlap=1.75,
        name="lip spans nearly the full usable deck width",
    )
    ctx.expect_within(
        lip,
        deck,
        axes="y",
        margin=0.03,
        inner_elem=lip_surface,
        outer_elem=deck_surface,
        name="lip stays inside the deck side edges",
    )

    with ctx.pose({rear_hinge: -0.45, front_hinge: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="rear_hinge_upper_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="rear_hinge_upper_pose_no_floating")
        ctx.expect_gap(
            deck,
            base,
            axis="z",
            min_gap=0.80,
            positive_elem=front_deck_center_knuckle,
            negative_elem=front_sill,
            name="raised deck front clears the front pit sill",
        )
        ctx.expect_contact(
            deck,
            base,
            elem_a=rear_deck_center_leaf,
            elem_b=rear_base_center_knuckle,
            name="rear hinge remains engaged while the deck is raised",
        )

    with ctx.pose({rear_hinge: -0.20, front_hinge: 1.35}):
        ctx.fail_if_parts_overlap_in_current_pose(name="front_lip_hinge_upper_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="front_lip_hinge_upper_pose_no_floating")
        ctx.expect_overlap(
            lip,
            deck,
            axes="y",
            min_overlap=1.75,
            name="deployed lip stays centered under the deck width",
        )
        ctx.expect_contact(
            lip,
            deck,
            elem_a=lip_center_knuckle,
            elem_b=front_deck_center_knuckle,
            name="lip hinge remains engaged when the lip is deployed",
        )
        ctx.expect_gap(
            deck,
            lip,
            axis="z",
            min_gap=0.02,
            positive_elem=front_deck_center_knuckle,
            negative_elem=lip_nose_bar,
            name="lip nose drops below the front hinge line when deployed",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

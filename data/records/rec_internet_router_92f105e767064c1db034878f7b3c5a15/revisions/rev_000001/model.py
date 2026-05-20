from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BODY_WIDTH = 0.168
BODY_DEPTH = 0.046
BODY_HEIGHT = 0.214
SHELL_THICKNESS = 0.0032

ANTENNA_HINGE_X = BODY_WIDTH / 2.0 - 0.020
ANTENNA_HINGE_Y = -BODY_DEPTH / 2.0 + 0.006
ANTENNA_HINGE_Z = BODY_HEIGHT + 0.007
ANTENNA_BARREL_RADIUS = 0.0045
ANTENNA_BARREL_LENGTH = 0.014
ANTENNA_EAR_THICKNESS = 0.003
ANTENNA_BLADE_HEIGHT = 0.084
ANTENNA_BLADE_WIDTH = 0.014
ANTENNA_BLADE_THICKNESS = 0.006
ANTENNA_NECK_HEIGHT = 0.022

KICKSTAND_HINGE_Y = -BODY_DEPTH / 2.0 - 0.0045
KICKSTAND_HINGE_Z = 0.026
KICKSTAND_BARREL_RADIUS = 0.0045
KICKSTAND_BARREL_LENGTH = 0.036
KICKSTAND_EAR_THICKNESS = 0.004
KICKSTAND_PANEL_THICKNESS = 0.0055
KICKSTAND_OPEN_ANGLE = 0.93
KICKSTAND_PANEL_HEIGHT = 0.106
KICKSTAND_PANEL_WIDTH = 0.048
KICKSTAND_PANEL_ROOT_HEIGHT = 0.044
KICKSTAND_PANEL_TIP_HEIGHT = KICKSTAND_PANEL_HEIGHT - KICKSTAND_PANEL_ROOT_HEIGHT

LEFT_ANTENNA_NAME = "left_antenna"
RIGHT_ANTENNA_NAME = "right_antenna"
KICKSTAND_NAME = "kickstand"


def _axis_matches(joint_axis: tuple[float, float, float], expected: tuple[float, float, float]) -> bool:
    return all(abs(a - b) < 1e-9 for a, b in zip(joint_axis, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upright_home_router")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    graphite = model.material("graphite", rgba=(0.14, 0.15, 0.17, 1.0))
    charcoal = model.material("charcoal", rgba=(0.22, 0.23, 0.25, 1.0))
    led_smoke = model.material("led_smoke", rgba=(0.18, 0.20, 0.24, 1.0))

    body = model.part(
        "body",
        inertial=Inertial.from_geometry(
            Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
            mass=0.82,
            origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
        ),
    )

    front_y = BODY_DEPTH / 2.0 - SHELL_THICKNESS / 2.0
    rear_y = -BODY_DEPTH / 2.0 + SHELL_THICKNESS / 2.0
    corner_radius = 0.010
    corner_column_height = BODY_HEIGHT - 0.016
    slot_width = 0.062
    rear_strip_width = 0.050

    body.visual(
        Box((BODY_WIDTH - 0.020, SHELL_THICKNESS, BODY_HEIGHT - 0.020)),
        origin=Origin(xyz=(0.0, front_y, BODY_HEIGHT / 2.0)),
        material=shell_white,
        name="front_shell",
    )
    body.visual(
        Box((BODY_WIDTH - 0.024, BODY_DEPTH - 0.014, SHELL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - SHELL_THICKNESS / 2.0)),
        material=shell_white,
        name="top_shell",
    )
    body.visual(
        Box((BODY_WIDTH - 0.024, BODY_DEPTH - 0.014, SHELL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, SHELL_THICKNESS / 2.0)),
        material=charcoal,
        name="bottom_edge",
    )
    body.visual(
        Box((SHELL_THICKNESS, BODY_DEPTH - 0.010, BODY_HEIGHT - 0.020)),
        origin=Origin(xyz=(-BODY_WIDTH / 2.0 + SHELL_THICKNESS / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=shell_white,
        name="left_shell_side",
    )
    body.visual(
        Box((SHELL_THICKNESS, BODY_DEPTH - 0.010, BODY_HEIGHT - 0.020)),
        origin=Origin(xyz=(BODY_WIDTH / 2.0 - SHELL_THICKNESS / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=shell_white,
        name="right_shell_side",
    )
    body.visual(
        Box((rear_strip_width, SHELL_THICKNESS, BODY_HEIGHT - 0.020)),
        origin=Origin(xyz=(-(slot_width / 2.0 + rear_strip_width / 2.0), rear_y, BODY_HEIGHT / 2.0)),
        material=shell_white,
        name="rear_left_strip",
    )
    body.visual(
        Box((rear_strip_width, SHELL_THICKNESS, BODY_HEIGHT - 0.020)),
        origin=Origin(xyz=((slot_width / 2.0 + rear_strip_width / 2.0), rear_y, BODY_HEIGHT / 2.0)),
        material=shell_white,
        name="rear_right_strip",
    )
    body.visual(
        Box((slot_width, SHELL_THICKNESS, 0.044)),
        origin=Origin(xyz=(0.0, rear_y, BODY_HEIGHT - 0.022)),
        material=shell_white,
        name="rear_upper_bridge",
    )
    body.visual(
        Box((0.062, SHELL_THICKNESS, 0.020)),
        origin=Origin(xyz=(0.0, rear_y, 0.010)),
        material=shell_white,
        name="rear_lower_bridge",
    )
    body.visual(
        Box((0.060, 0.0016, 0.010)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 + 0.0008, 0.034)),
        material=led_smoke,
        name="status_window",
    )
    body.visual(
        Box((0.016, BODY_DEPTH - 2.0 * SHELL_THICKNESS, BODY_HEIGHT - 2.0 * SHELL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
        material=graphite,
        name="internal_spine",
    )

    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            body.visual(
                Cylinder(radius=corner_radius, length=corner_column_height),
                origin=Origin(
                    xyz=(
                        sx * (BODY_WIDTH / 2.0 - corner_radius),
                        sy * (BODY_DEPTH / 2.0 - corner_radius),
                        BODY_HEIGHT / 2.0,
                    )
                ),
                material=shell_white,
                name=f"corner_{'l' if sx < 0 else 'r'}_{'rear' if sy < 0 else 'front'}",
            )

    for side_name, hinge_x in (("left", -ANTENNA_HINGE_X), ("right", ANTENNA_HINGE_X)):
        ear_offset = ANTENNA_BARREL_LENGTH / 2.0 + ANTENNA_EAR_THICKNESS / 2.0
        for idx, ear_sign in enumerate((-1.0, 1.0), start=1):
            body.visual(
                Box((ANTENNA_EAR_THICKNESS, 0.010, 0.014)),
                origin=Origin(
                    xyz=(
                        hinge_x + ear_sign * ear_offset,
                        ANTENNA_HINGE_Y,
                        BODY_HEIGHT + 0.004,
                    )
                ),
                material=graphite,
                name=f"{side_name}_antenna_ear_{idx}",
            )

    ear_offset = KICKSTAND_BARREL_LENGTH / 2.0 + KICKSTAND_EAR_THICKNESS / 2.0
    for idx, ear_sign in enumerate((-1.0, 1.0), start=1):
        body.visual(
            Box((KICKSTAND_EAR_THICKNESS, 0.010, 0.012)),
            origin=Origin(
                xyz=(
                    ear_sign * ear_offset,
                    KICKSTAND_HINGE_Y,
                    KICKSTAND_HINGE_Z,
                )
            ),
            material=graphite,
            name=f"kickstand_ear_{idx}",
        )

    left_antenna = model.part(
        LEFT_ANTENNA_NAME,
        inertial=Inertial.from_geometry(
            Box((0.016, 0.010, 0.095)),
            mass=0.055,
            origin=Origin(xyz=(0.0, -0.003, 0.047)),
        ),
    )
    left_antenna.visual(
        Cylinder(radius=ANTENNA_BARREL_RADIUS, length=ANTENNA_BARREL_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="root_barrel",
    )
    left_antenna.visual(
        Box((0.010, ANTENNA_BLADE_THICKNESS, ANTENNA_NECK_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.001, ANTENNA_NECK_HEIGHT / 2.0)),
        material=graphite,
        name="root_neck",
    )
    left_antenna.visual(
        Box((ANTENNA_BLADE_WIDTH, ANTENNA_BLADE_THICKNESS, ANTENNA_BLADE_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.001, ANTENNA_NECK_HEIGHT + ANTENNA_BLADE_HEIGHT / 2.0)),
        material=charcoal,
        name="antenna_blade",
    )

    right_antenna = model.part(
        RIGHT_ANTENNA_NAME,
        inertial=Inertial.from_geometry(
            Box((0.016, 0.010, 0.095)),
            mass=0.055,
            origin=Origin(xyz=(0.0, -0.003, 0.047)),
        ),
    )
    right_antenna.visual(
        Cylinder(radius=ANTENNA_BARREL_RADIUS, length=ANTENNA_BARREL_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="root_barrel",
    )
    right_antenna.visual(
        Box((0.010, ANTENNA_BLADE_THICKNESS, ANTENNA_NECK_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.001, ANTENNA_NECK_HEIGHT / 2.0)),
        material=graphite,
        name="root_neck",
    )
    right_antenna.visual(
        Box((ANTENNA_BLADE_WIDTH, ANTENNA_BLADE_THICKNESS, ANTENNA_BLADE_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.001, ANTENNA_NECK_HEIGHT + ANTENNA_BLADE_HEIGHT / 2.0)),
        material=charcoal,
        name="antenna_blade",
    )

    kickstand = model.part(
        KICKSTAND_NAME,
        inertial=Inertial.from_geometry(
            Box((0.052, 0.010, 0.114)),
            mass=0.072,
            origin=Origin(xyz=(0.0, -0.004, 0.056)),
        ),
    )
    kickstand.visual(
        Cylinder(radius=KICKSTAND_BARREL_RADIUS, length=KICKSTAND_BARREL_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="root_barrel",
    )
    kickstand.visual(
        Box((0.008, 0.010, 0.030)),
        origin=Origin(xyz=(-0.016, -0.006, 0.015)),
        material=graphite,
        name="left_arm",
    )
    kickstand.visual(
        Box((0.008, 0.010, 0.030)),
        origin=Origin(xyz=(0.016, -0.006, 0.015)),
        material=graphite,
        name="right_arm",
    )
    kickstand.visual(
        Box((KICKSTAND_PANEL_WIDTH, KICKSTAND_PANEL_THICKNESS, KICKSTAND_PANEL_ROOT_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.0105, KICKSTAND_PANEL_ROOT_HEIGHT / 2.0)),
        material=charcoal,
        name="stand_panel",
    )
    kickstand.visual(
        Box((0.040, KICKSTAND_PANEL_THICKNESS, KICKSTAND_PANEL_TIP_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -0.0105,
                KICKSTAND_PANEL_ROOT_HEIGHT + KICKSTAND_PANEL_TIP_HEIGHT / 2.0,
            )
        ),
        material=charcoal,
        name="stand_tip",
    )

    model.articulation(
        "left_antenna_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_antenna,
        origin=Origin(xyz=(-ANTENNA_HINGE_X, ANTENNA_HINGE_Y, ANTENNA_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=-0.20,
            upper=1.35,
        ),
    )
    model.articulation(
        "right_antenna_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_antenna,
        origin=Origin(xyz=(ANTENNA_HINGE_X, ANTENNA_HINGE_Y, ANTENNA_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=-0.20,
            upper=1.35,
        ),
    )
    model.articulation(
        "kickstand_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=kickstand,
        origin=Origin(xyz=(0.0, KICKSTAND_HINGE_Y, KICKSTAND_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=KICKSTAND_OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_antenna = object_model.get_part(LEFT_ANTENNA_NAME)
    right_antenna = object_model.get_part(RIGHT_ANTENNA_NAME)
    kickstand = object_model.get_part(KICKSTAND_NAME)

    left_hinge = object_model.get_articulation("left_antenna_hinge")
    right_hinge = object_model.get_articulation("right_antenna_hinge")
    kickstand_hinge = object_model.get_articulation("kickstand_hinge")

    left_blade = left_antenna.get_visual("antenna_blade")
    right_blade = right_antenna.get_visual("antenna_blade")
    stand_panel = kickstand.get_visual("stand_panel")
    stand_tip = kickstand.get_visual("stand_tip")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(left_antenna, body, name="left_antenna_contacts_router_body")
    ctx.expect_contact(right_antenna, body, name="right_antenna_contacts_router_body")
    ctx.expect_contact(kickstand, body, name="kickstand_contacts_router_body")

    ctx.expect_origin_distance(
        left_antenna,
        right_antenna,
        axes="x",
        min_dist=0.11,
        max_dist=0.14,
        name="antennas_span_the_top_corners",
    )
    ctx.expect_origin_gap(
        left_antenna,
        body,
        axis="z",
        min_gap=BODY_HEIGHT,
        max_gap=BODY_HEIGHT + 0.02,
        name="left_antenna_hinge_sits_above_top_edge",
    )
    ctx.expect_origin_gap(
        right_antenna,
        body,
        axis="z",
        min_gap=BODY_HEIGHT,
        max_gap=BODY_HEIGHT + 0.02,
        name="right_antenna_hinge_sits_above_top_edge",
    )
    ctx.expect_origin_gap(
        body,
        kickstand,
        axis="y",
        min_gap=0.025,
        max_gap=0.032,
        name="kickstand_hinge_is_mounted_on_the_rear",
    )
    ctx.expect_gap(
        body,
        kickstand,
        axis="y",
        negative_elem=stand_panel,
        min_gap=0.001,
        max_gap=0.005,
        name="kickstand_panel_stows_close_to_the_back_slot",
    )
    ctx.expect_within(
        kickstand,
        body,
        axes="xz",
        inner_elem=stand_panel,
        margin=0.006,
        name="kickstand_panel_fits_the_rear_slot_footprint",
    )

    ctx.check(
        "antenna_hinges_rotate_about_the_top_edge_axis",
        _axis_matches(left_hinge.axis, (1.0, 0.0, 0.0)) and _axis_matches(right_hinge.axis, (1.0, 0.0, 0.0)),
        details=f"left={left_hinge.axis}, right={right_hinge.axis}",
    )
    ctx.check(
        "kickstand_hinge_rotates_about_the_lower_rear_axis",
        _axis_matches(kickstand_hinge.axis, (1.0, 0.0, 0.0)),
        details=f"kickstand axis={kickstand_hinge.axis}",
    )
    ctx.check(
        "antenna_motion_limits_allow_upright_and_folded_positions",
        (
            left_hinge.motion_limits is not None
            and right_hinge.motion_limits is not None
            and left_hinge.motion_limits.lower is not None
            and left_hinge.motion_limits.upper is not None
            and right_hinge.motion_limits.lower is not None
            and right_hinge.motion_limits.upper is not None
            and left_hinge.motion_limits.lower <= 0.0 <= left_hinge.motion_limits.upper
            and right_hinge.motion_limits.lower <= 0.0 <= right_hinge.motion_limits.upper
            and left_hinge.motion_limits.upper >= 1.2
            and right_hinge.motion_limits.upper >= 1.2
        ),
        details="Antenna hinges should rest upright at zero and fold back substantially.",
    )
    ctx.check(
        "kickstand_motion_limits_allow_deployed_and_stowed_positions",
        (
            kickstand_hinge.motion_limits is not None
            and kickstand_hinge.motion_limits.lower is not None
            and kickstand_hinge.motion_limits.upper is not None
            and kickstand_hinge.motion_limits.lower <= 0.0 <= kickstand_hinge.motion_limits.upper
            and kickstand_hinge.motion_limits.upper >= 0.85
        ),
        details="Kickstand should rest stowed at zero and rotate rearward to a stable deployed pose.",
    )

    with ctx.pose({kickstand_hinge: KICKSTAND_OPEN_ANGLE}):
        ctx.expect_gap(
            body,
            kickstand,
            axis="y",
            negative_elem=stand_tip,
            min_gap=0.025,
            name="kickstand_tip_deploys_behind_router_body",
        )

    with ctx.pose({"left_antenna_hinge": 1.20, "right_antenna_hinge": 1.20}):
        ctx.expect_gap(
            body,
            left_antenna,
            axis="y",
            negative_elem=left_blade,
            min_gap=0.001,
            max_gap=0.020,
            name="left_antenna_can_fold_rearward",
        )
        ctx.expect_gap(
            body,
            right_antenna,
            axis="y",
            negative_elem=right_blade,
            min_gap=0.001,
            max_gap=0.020,
            name="right_antenna_can_fold_rearward",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


RAIL_LENGTH = 0.72
RAIL_WIDTH = 0.09
RAIL_BASE_HEIGHT = 0.016
RAIL_SHOULDER_WIDTH = 0.06
RAIL_SHOULDER_HEIGHT = 0.016
RAIL_GUIDE_WIDTH = 0.032
RAIL_GUIDE_HEIGHT = 0.018
RAIL_END_STOP_LENGTH = 0.028
RAIL_END_STOP_WIDTH = 0.07
RAIL_END_STOP_HEIGHT = 0.028

RAIL_SHOULDER_TOP_Z = RAIL_BASE_HEIGHT + RAIL_SHOULDER_HEIGHT
RAIL_GUIDE_TOP_Z = RAIL_SHOULDER_TOP_Z + RAIL_GUIDE_HEIGHT

CARRIAGE_LENGTH = 0.13
CARRIAGE_WIDTH = 0.12
CARRIAGE_RUNNER_LENGTH = 0.118
CARRIAGE_RUNNER_WIDTH = 0.024
CARRIAGE_RUNNER_HEIGHT = 0.022
CARRIAGE_RUNNER_OFFSET_Y = 0.028
CARRIAGE_BODY_HEIGHT = 0.026
CARRIAGE_PEDESTAL_LENGTH = 0.065
CARRIAGE_PEDESTAL_WIDTH = 0.07
CARRIAGE_PEDESTAL_HEIGHT = 0.02

PIVOT_GAP = 0.05
PIVOT_EAR_THICKNESS = 0.016
PIVOT_EAR_LENGTH = 0.04
PIVOT_EAR_HEIGHT = 0.052
PIVOT_BOSS_LENGTH = 0.008
PIVOT_BOSS_RADIUS = 0.024

PIVOT_CENTER_Z = (
    CARRIAGE_RUNNER_HEIGHT
    + CARRIAGE_BODY_HEIGHT
    + CARRIAGE_PEDESTAL_HEIGHT
    + PIVOT_EAR_HEIGHT / 2.0
)

BRACKET_HUB_RADIUS = 0.022
BRACKET_ARM_WIDTH = 0.042
BRACKET_SLEEVE_LENGTH = 0.07
BRACKET_SLEEVE_WIDTH = 0.07
BRACKET_SLEEVE_HEIGHT = 0.065
BRACKET_SLEEVE_CENTER_X = 0.145
BRACKET_TOOL_CHANNEL = 0.033
BRACKET_TOOL_FACE_X = BRACKET_SLEEVE_CENTER_X + BRACKET_SLEEVE_LENGTH / 2.0

TOOL_RAM_LENGTH = 0.19
TOOL_RAM_SECTION = 0.028
TOOL_SHOULDER_LENGTH = 0.016
TOOL_SHOULDER_SECTION = 0.05
TOOL_NOSE_RADIUS = 0.011
TOOL_NOSE_LENGTH = 0.055


def make_rail() -> cq.Workplane:
    base = cq.Workplane("XY").box(
        RAIL_LENGTH,
        RAIL_WIDTH,
        RAIL_BASE_HEIGHT,
        centered=(True, True, False),
    )
    shoulder = cq.Workplane("XY").box(
        RAIL_LENGTH * 0.96,
        RAIL_SHOULDER_WIDTH,
        RAIL_SHOULDER_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, 0.0, RAIL_BASE_HEIGHT))
    guide = cq.Workplane("XY").box(
        RAIL_LENGTH * 0.9,
        RAIL_GUIDE_WIDTH,
        RAIL_GUIDE_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, 0.0, RAIL_SHOULDER_TOP_Z))

    stop_z = RAIL_SHOULDER_TOP_Z
    stop_x = RAIL_LENGTH / 2.0 - 0.055
    stop_left = cq.Workplane("XY").box(
        RAIL_END_STOP_LENGTH,
        RAIL_END_STOP_WIDTH,
        RAIL_END_STOP_HEIGHT,
        centered=(True, True, False),
    ).translate((-stop_x, 0.0, stop_z))
    stop_right = cq.Workplane("XY").box(
        RAIL_END_STOP_LENGTH,
        RAIL_END_STOP_WIDTH,
        RAIL_END_STOP_HEIGHT,
        centered=(True, True, False),
    ).translate((stop_x, 0.0, stop_z))

    return base.union(shoulder).union(guide).union(stop_left).union(stop_right)


def make_carriage() -> cq.Workplane:
    runner_left = cq.Workplane("XY").box(
        CARRIAGE_RUNNER_LENGTH,
        CARRIAGE_RUNNER_WIDTH,
        CARRIAGE_RUNNER_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, CARRIAGE_RUNNER_OFFSET_Y, 0.0))
    runner_right = cq.Workplane("XY").box(
        CARRIAGE_RUNNER_LENGTH,
        CARRIAGE_RUNNER_WIDTH,
        CARRIAGE_RUNNER_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, -CARRIAGE_RUNNER_OFFSET_Y, 0.0))

    body = cq.Workplane("XY").box(
        CARRIAGE_LENGTH,
        CARRIAGE_WIDTH,
        CARRIAGE_BODY_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, 0.0, CARRIAGE_RUNNER_HEIGHT))
    pedestal = cq.Workplane("XY").box(
        CARRIAGE_PEDESTAL_LENGTH,
        CARRIAGE_PEDESTAL_WIDTH,
        CARRIAGE_PEDESTAL_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, 0.0, CARRIAGE_RUNNER_HEIGHT + CARRIAGE_BODY_HEIGHT))

    ear_z = CARRIAGE_RUNNER_HEIGHT + CARRIAGE_BODY_HEIGHT + CARRIAGE_PEDESTAL_HEIGHT
    ear_offset_y = PIVOT_GAP / 2.0 + PIVOT_EAR_THICKNESS / 2.0
    ear_left = cq.Workplane("XY").box(
        PIVOT_EAR_LENGTH,
        PIVOT_EAR_THICKNESS,
        PIVOT_EAR_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, ear_offset_y, ear_z))
    ear_right = cq.Workplane("XY").box(
        PIVOT_EAR_LENGTH,
        PIVOT_EAR_THICKNESS,
        PIVOT_EAR_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, -ear_offset_y, ear_z))

    rear_bridge = cq.Workplane("XY").box(
        0.022,
        PIVOT_GAP + 2.0 * PIVOT_EAR_THICKNESS + 0.018,
        0.016,
        centered=(True, True, False),
    ).translate((-0.024, 0.0, PIVOT_CENTER_Z - 0.008))

    boss_y = PIVOT_GAP / 2.0 + PIVOT_EAR_THICKNESS + PIVOT_BOSS_LENGTH / 2.0
    boss_left = cq.Workplane("XZ").circle(PIVOT_BOSS_RADIUS).extrude(
        PIVOT_BOSS_LENGTH,
        both=True,
    ).translate((0.0, boss_y, PIVOT_CENTER_Z))
    boss_right = cq.Workplane("XZ").circle(PIVOT_BOSS_RADIUS).extrude(
        PIVOT_BOSS_LENGTH,
        both=True,
    ).translate((0.0, -boss_y, PIVOT_CENTER_Z))

    clevis_slot = cq.Workplane("XY").box(
        0.095,
        0.05,
        0.07,
        centered=(True, True, True),
    ).translate((0.015, 0.0, 0.08))

    return (
        runner_left.union(runner_right)
        .union(body)
        .union(pedestal)
        .union(ear_left)
        .union(ear_right)
        .union(rear_bridge)
        .union(boss_left)
        .union(boss_right)
        .cut(clevis_slot)
    )


def make_middle_bracket() -> cq.Workplane:
    hub = cq.Workplane("XZ").circle(BRACKET_HUB_RADIUS).extrude(PIVOT_GAP / 2.0, both=True)

    arm = (
        cq.Workplane("XZ")
        .moveTo(0.0, -0.018)
        .lineTo(0.03, -0.018)
        .lineTo(0.09, -0.016)
        .lineTo(0.16, -0.016)
        .lineTo(0.16, 0.03)
        .lineTo(0.115, 0.04)
        .lineTo(0.06, 0.05)
        .lineTo(0.0, 0.034)
        .close()
        .extrude(BRACKET_ARM_WIDTH / 2.0, both=True)
    )

    sleeve = cq.Workplane("XY").box(
        BRACKET_SLEEVE_LENGTH,
        BRACKET_SLEEVE_WIDTH,
        BRACKET_SLEEVE_HEIGHT,
        centered=(True, True, True),
    ).translate((BRACKET_SLEEVE_CENTER_X, 0.0, 0.0))

    channel = cq.Workplane("XY").box(
        0.1,
        BRACKET_TOOL_CHANNEL,
        BRACKET_TOOL_CHANNEL,
        centered=(True, True, True),
    ).translate((BRACKET_SLEEVE_CENTER_X, 0.0, 0.0))
    top_window = cq.Workplane("XY").box(
        0.05,
        0.02,
        0.018,
        centered=(True, True, True),
    ).translate((BRACKET_SLEEVE_CENTER_X, 0.0, 0.02))

    return hub.union(arm).union(sleeve).cut(channel).cut(top_window)


def make_tool_nose() -> cq.Workplane:
    ram = cq.Workplane("XY").box(
        TOOL_RAM_LENGTH,
        TOOL_RAM_SECTION,
        TOOL_RAM_SECTION,
        centered=(True, True, True),
    ).translate((TOOL_RAM_LENGTH / 2.0, 0.0, 0.0))

    shoulder = cq.Workplane("XY").box(
        TOOL_SHOULDER_LENGTH,
        TOOL_SHOULDER_SECTION,
        TOOL_SHOULDER_SECTION,
        centered=(True, True, True),
    ).translate((TOOL_SHOULDER_LENGTH / 2.0, 0.0, 0.0))

    nose = cq.Workplane("YZ").circle(TOOL_NOSE_RADIUS).extrude(TOOL_NOSE_LENGTH).translate(
        (TOOL_RAM_LENGTH, 0.0, 0.0)
    )

    return ram.union(shoulder).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_positioning_module")

    rail_mat = model.material("rail_steel", color=(0.22, 0.23, 0.25))
    carriage_mat = model.material("carriage_gray", color=(0.48, 0.50, 0.53))
    bracket_mat = model.material("bracket_orange", color=(0.86, 0.44, 0.12))
    tool_mat = model.material("tool_steel", color=(0.74, 0.76, 0.78))

    rail = model.part("rail")
    rail.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, RAIL_BASE_HEIGHT / 2.0)),
        material=rail_mat,
        name="rail_base",
    )
    rail.visual(
        Box((RAIL_LENGTH * 0.94, 0.018, RAIL_SHOULDER_HEIGHT)),
        origin=Origin(
            xyz=(0.0, CARRIAGE_RUNNER_OFFSET_Y, RAIL_BASE_HEIGHT + RAIL_SHOULDER_HEIGHT / 2.0)
        ),
        material=rail_mat,
        name="left_bearing_land",
    )
    rail.visual(
        Box((RAIL_LENGTH * 0.94, 0.018, RAIL_SHOULDER_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -CARRIAGE_RUNNER_OFFSET_Y, RAIL_BASE_HEIGHT + RAIL_SHOULDER_HEIGHT / 2.0)
        ),
        material=rail_mat,
        name="right_bearing_land",
    )
    rail.visual(
        Box((RAIL_LENGTH * 0.94, 0.022, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, RAIL_BASE_HEIGHT + 0.006)),
        material=rail_mat,
        name="rail_web",
    )
    rail.visual(
        Box((RAIL_END_STOP_LENGTH, RAIL_END_STOP_WIDTH, 0.016)),
        origin=Origin(
            xyz=(-RAIL_LENGTH / 2.0 + 0.05, 0.0, RAIL_BASE_HEIGHT + 0.008)
        ),
        material=rail_mat,
        name="left_stop",
    )
    rail.visual(
        Box((RAIL_END_STOP_LENGTH, RAIL_END_STOP_WIDTH, 0.016)),
        origin=Origin(
            xyz=(RAIL_LENGTH / 2.0 - 0.05, 0.0, RAIL_BASE_HEIGHT + 0.008)
        ),
        material=rail_mat,
        name="right_stop",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.116, 0.016, 0.008)),
        origin=Origin(xyz=(0.0, CARRIAGE_RUNNER_OFFSET_Y, 0.004)),
        material=carriage_mat,
        name="left_pad",
    )
    carriage.visual(
        Box((0.116, 0.016, 0.008)),
        origin=Origin(xyz=(0.0, -CARRIAGE_RUNNER_OFFSET_Y, 0.004)),
        material=carriage_mat,
        name="right_pad",
    )
    carriage.visual(
        Box((0.12, 0.02, 0.045)),
        origin=Origin(xyz=(0.0, 0.036, 0.0305)),
        material=carriage_mat,
        name="left_side_cheek",
    )
    carriage.visual(
        Box((0.12, 0.02, 0.045)),
        origin=Origin(xyz=(0.0, -0.036, 0.0305)),
        material=carriage_mat,
        name="right_side_cheek",
    )
    carriage.visual(
        Box((0.044, 0.072, 0.038)),
        origin=Origin(xyz=(-0.048, 0.0, 0.027)),
        material=carriage_mat,
        name="rear_bridge_block",
    )
    carriage.visual(
        Box((0.028, 0.018, 0.026)),
        origin=Origin(xyz=(-0.008, 0.033, 0.056)),
        material=carriage_mat,
        name="left_upright",
    )
    carriage.visual(
        Box((0.028, 0.018, 0.026)),
        origin=Origin(xyz=(-0.008, -0.033, 0.056)),
        material=carriage_mat,
        name="right_upright",
    )
    carriage.visual(
        Box((0.022, PIVOT_EAR_THICKNESS, 0.05)),
        origin=Origin(
            xyz=(0.0, PIVOT_GAP / 2.0 + PIVOT_EAR_THICKNESS / 2.0, PIVOT_CENTER_Z)
        ),
        material=carriage_mat,
        name="left_ear",
    )
    carriage.visual(
        Box((0.022, PIVOT_EAR_THICKNESS, 0.05)),
        origin=Origin(
            xyz=(0.0, -PIVOT_GAP / 2.0 - PIVOT_EAR_THICKNESS / 2.0, PIVOT_CENTER_Z)
        ),
        material=carriage_mat,
        name="right_ear",
    )

    middle_bracket = model.part("middle_bracket")
    middle_bracket.visual(
        Cylinder(radius=0.021, length=PIVOT_GAP),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=bracket_mat,
        name="hub",
    )
    middle_bracket.visual(
        Box((0.09, 0.026, 0.018)),
        origin=Origin(xyz=(0.06, 0.0, 0.0)),
        material=bracket_mat,
        name="lower_arm",
    )
    middle_bracket.visual(
        Box((0.065, 0.024, 0.018)),
        origin=Origin(xyz=(0.095, 0.0, 0.022)),
        material=bracket_mat,
        name="upper_arm",
    )
    middle_bracket.visual(
        Box((0.06, 0.022, 0.026)),
        origin=Origin(xyz=(0.085, 0.0, 0.012)),
        material=bracket_mat,
        name="arm_gusset",
    )
    middle_bracket.visual(
        Box((BRACKET_SLEEVE_LENGTH, 0.06, 0.013)),
        origin=Origin(xyz=(BRACKET_SLEEVE_CENTER_X, 0.0, 0.0235)),
        material=bracket_mat,
        name="sleeve_top",
    )
    middle_bracket.visual(
        Box((BRACKET_SLEEVE_LENGTH, 0.06, 0.013)),
        origin=Origin(xyz=(BRACKET_SLEEVE_CENTER_X, 0.0, -0.0235)),
        material=bracket_mat,
        name="sleeve_bottom",
    )
    middle_bracket.visual(
        Box((BRACKET_SLEEVE_LENGTH, 0.013, 0.034)),
        origin=Origin(xyz=(BRACKET_SLEEVE_CENTER_X, 0.0235, 0.0)),
        material=bracket_mat,
        name="sleeve_left",
    )
    middle_bracket.visual(
        Box((BRACKET_SLEEVE_LENGTH, 0.013, 0.034)),
        origin=Origin(xyz=(BRACKET_SLEEVE_CENTER_X, -0.0235, 0.0)),
        material=bracket_mat,
        name="sleeve_right",
    )

    tool_nose = model.part("tool_nose")
    tool_nose.visual(
        Box((0.18, TOOL_RAM_SECTION, TOOL_RAM_SECTION)),
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
        material=tool_mat,
        name="tool_ram",
    )
    tool_nose.visual(
        Box((TOOL_SHOULDER_LENGTH, TOOL_SHOULDER_SECTION, TOOL_SHOULDER_SECTION)),
        origin=Origin(xyz=(TOOL_SHOULDER_LENGTH / 2.0, 0.0, 0.0)),
        material=tool_mat,
        name="tool_shoulder",
    )
    tool_nose.visual(
        Cylinder(radius=TOOL_NOSE_RADIUS, length=TOOL_NOSE_LENGTH),
        origin=Origin(xyz=(0.1575, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=tool_mat,
        name="tool_tip",
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(-0.18, 0.0, RAIL_SHOULDER_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.25,
            lower=0.0,
            upper=0.36,
        ),
    )

    model.articulation(
        "carriage_to_bracket",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=middle_bracket,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.5,
            lower=-1.05,
            upper=1.05,
        ),
    )

    model.articulation(
        "bracket_to_tool",
        ArticulationType.PRISMATIC,
        parent=middle_bracket,
        child=tool_nose,
        origin=Origin(xyz=(BRACKET_TOOL_FACE_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.22,
            lower=0.0,
            upper=0.14,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    middle_bracket = object_model.get_part("middle_bracket")
    tool_nose = object_model.get_part("tool_nose")

    rail_slide = object_model.get_articulation("rail_to_carriage")
    elbow = object_model.get_articulation("carriage_to_bracket")
    tool_slide = object_model.get_articulation("bracket_to_tool")

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

    ctx.check(
        "parts_present",
        all(part is not None for part in (rail, carriage, middle_bracket, tool_nose)),
        "one or more required parts are missing",
    )
    ctx.check(
        "rail_slide_is_prismatic",
        rail_slide.articulation_type == ArticulationType.PRISMATIC,
        f"expected PRISMATIC, got {rail_slide.articulation_type}",
    )
    ctx.check(
        "elbow_is_revolute",
        elbow.articulation_type == ArticulationType.REVOLUTE,
        f"expected REVOLUTE, got {elbow.articulation_type}",
    )
    ctx.check(
        "tool_slide_is_prismatic",
        tool_slide.articulation_type == ArticulationType.PRISMATIC,
        f"expected PRISMATIC, got {tool_slide.articulation_type}",
    )
    ctx.check(
        "rail_slide_axis",
        rail_slide.axis == (1.0, 0.0, 0.0),
        f"unexpected rail slide axis {rail_slide.axis}",
    )
    ctx.check(
        "elbow_axis",
        elbow.axis == (0.0, 1.0, 0.0),
        f"unexpected elbow axis {elbow.axis}",
    )
    ctx.check(
        "tool_slide_axis",
        tool_slide.axis == (1.0, 0.0, 0.0),
        f"unexpected tool slide axis {tool_slide.axis}",
    )
    ctx.check(
        "rail_slide_limits",
        (
            rail_slide.motion_limits is not None
            and isclose(rail_slide.motion_limits.lower or 0.0, 0.0, abs_tol=1e-9)
            and isclose(rail_slide.motion_limits.upper or 0.0, 0.36, abs_tol=1e-9)
        ),
        f"unexpected rail slide limits {rail_slide.motion_limits}",
    )
    ctx.check(
        "elbow_limits",
        (
            elbow.motion_limits is not None
            and isclose(elbow.motion_limits.lower or 0.0, -1.05, abs_tol=1e-9)
            and isclose(elbow.motion_limits.upper or 0.0, 1.05, abs_tol=1e-9)
        ),
        f"unexpected elbow limits {elbow.motion_limits}",
    )
    ctx.check(
        "tool_slide_limits",
        (
            tool_slide.motion_limits is not None
            and isclose(tool_slide.motion_limits.lower or 0.0, 0.0, abs_tol=1e-9)
            and isclose(tool_slide.motion_limits.upper or 0.0, 0.14, abs_tol=1e-9)
        ),
        f"unexpected tool slide limits {tool_slide.motion_limits}",
    )

    ctx.expect_contact(
        carriage,
        rail,
        contact_tol=5e-4,
        name="carriage_bears_on_rail",
    )
    ctx.expect_contact(
        middle_bracket,
        carriage,
        contact_tol=5e-4,
        name="bracket_supported_by_carriage",
    )
    ctx.expect_contact(
        tool_nose,
        middle_bracket,
        contact_tol=5e-4,
        name="tool_seated_against_sleeve",
    )
    ctx.expect_overlap(
        carriage,
        rail,
        axes="xy",
        min_overlap=0.08,
        name="carriage_tracks_over_rail_plan",
    )
    ctx.expect_within(
        tool_nose,
        middle_bracket,
        axes="yz",
        margin=0.0,
        name="tool_aligned_in_bracket_section",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

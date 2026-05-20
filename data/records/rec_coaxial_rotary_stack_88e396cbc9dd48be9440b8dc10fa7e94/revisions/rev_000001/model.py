from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FOOT_LENGTH = 0.34
FOOT_WIDTH = 0.24
FOOT_HEIGHT = 0.024
FOOT_CORNER_RADIUS = 0.012
FOOT_BOLT_D = 0.012
FOOT_BOLT_X = 0.115
FOOT_BOLT_Y = 0.075

PLINTH_RADIUS = 0.070
PLINTH_HEIGHT = 0.016
SHOULDER_RADIUS = 0.040
SHOULDER_HEIGHT = 0.012
POST_RADIUS = 0.025
POST_SEGMENT_HEIGHT = 0.058
TOP_POST_HEIGHT = 0.060

LOWER_Z = FOOT_HEIGHT + PLINTH_HEIGHT + SHOULDER_HEIGHT
MIDDLE_Z = LOWER_Z + POST_SEGMENT_HEIGHT + SHOULDER_HEIGHT
UPPER_Z = MIDDLE_Z + POST_SEGMENT_HEIGHT + SHOULDER_HEIGHT

CARRIER_BORE_RADIUS = 0.026
CARRIER_HUB_RADIUS = 0.055
CARRIER_HUB_HEIGHT = 0.018
CARRIER_ARM_THICKNESS = 0.010
CARRIER_PAD_THICKNESS = 0.012
CARRIER_LUG_HEIGHT = 0.028


def make_base_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .rect(FOOT_LENGTH, FOOT_WIDTH)
        .extrude(FOOT_HEIGHT)
        .edges("|Z")
        .fillet(FOOT_CORNER_RADIUS)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-FOOT_BOLT_X, -FOOT_BOLT_Y),
                (-FOOT_BOLT_X, FOOT_BOLT_Y),
                (FOOT_BOLT_X, -FOOT_BOLT_Y),
                (FOOT_BOLT_X, FOOT_BOLT_Y),
            ]
        )
        .hole(FOOT_BOLT_D)
    )

    tower = foot
    for radius, height in (
        (PLINTH_RADIUS, PLINTH_HEIGHT),
        (SHOULDER_RADIUS, SHOULDER_HEIGHT),
        (POST_RADIUS, POST_SEGMENT_HEIGHT),
        (SHOULDER_RADIUS, SHOULDER_HEIGHT),
        (POST_RADIUS, POST_SEGMENT_HEIGHT),
        (SHOULDER_RADIUS, SHOULDER_HEIGHT),
        (POST_RADIUS, TOP_POST_HEIGHT),
    ):
        tower = (
            tower.faces(">Z")
            .workplane(centerOption="CenterOfMass")
            .circle(radius)
            .extrude(height)
        )
    return tower


def make_carrier_shape(reach: float, angle_deg: float) -> cq.Workplane:
    bore_diameter = 2.0 * CARRIER_BORE_RADIUS
    pad_length = 0.070
    pad_width = 0.092
    arm_width = 0.050
    arm_x0 = 0.028
    arm_x1 = reach - pad_length + 0.020
    arm_length = arm_x1 - arm_x0
    pad_center_x = reach - pad_length / 2.0
    lug_x = reach - 0.020
    lug_y = 0.028
    lug_length = 0.018
    lug_width = 0.016
    bridge_length = 0.012
    bridge_width = 0.070
    bridge_height = 0.012

    hub = cq.Workplane("XY").circle(CARRIER_HUB_RADIUS).extrude(CARRIER_HUB_HEIGHT)
    hub = hub.faces(">Z").workplane(centerOption="CenterOfMass").hole(bore_diameter)

    arm = (
        cq.Workplane("XY")
        .center((arm_x0 + arm_x1) / 2.0, 0.0)
        .rect(arm_length, arm_width)
        .extrude(CARRIER_ARM_THICKNESS)
        .translate((0.0, 0.0, CARRIER_HUB_HEIGHT))
    )

    pad = (
        cq.Workplane("XY")
        .center(pad_center_x, 0.0)
        .rect(pad_length, pad_width)
        .extrude(CARRIER_PAD_THICKNESS)
        .translate((0.0, 0.0, CARRIER_HUB_HEIGHT))
    )

    left_lug = (
        cq.Workplane("XY")
        .center(lug_x, lug_y)
        .rect(lug_length, lug_width)
        .extrude(CARRIER_LUG_HEIGHT)
        .translate((0.0, 0.0, CARRIER_HUB_HEIGHT + CARRIER_PAD_THICKNESS))
    )

    right_lug = (
        cq.Workplane("XY")
        .center(lug_x, -lug_y)
        .rect(lug_length, lug_width)
        .extrude(CARRIER_LUG_HEIGHT)
        .translate((0.0, 0.0, CARRIER_HUB_HEIGHT + CARRIER_PAD_THICKNESS))
    )

    bridge = (
        cq.Workplane("XY")
        .center(reach - 0.030, 0.0)
        .rect(bridge_length, bridge_width)
        .extrude(bridge_height)
        .translate(
            (
                0.0,
                0.0,
                CARRIER_HUB_HEIGHT
                + CARRIER_PAD_THICKNESS
                + CARRIER_LUG_HEIGHT
                - bridge_height,
            )
        )
    )

    backstop = (
        cq.Workplane("XY")
        .center(reach - 0.058, 0.0)
        .rect(0.010, 0.070)
        .extrude(0.020)
        .translate((0.0, 0.0, CARRIER_HUB_HEIGHT + CARRIER_PAD_THICKNESS))
    )

    carrier = hub.union(arm).union(pad).union(left_lug).union(right_lug).union(bridge).union(backstop)
    if angle_deg:
        carrier = carrier.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
    return carrier


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coaxial_rotary_fixture")

    base_color = model.material("base_gray", color=(0.30, 0.32, 0.35))
    lower_color = model.material("lower_blue", color=(0.47, 0.54, 0.60))
    middle_color = model.material("middle_bronze", color=(0.58, 0.50, 0.36))
    upper_color = model.material("upper_olive", color=(0.45, 0.50, 0.40))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_shape(), "fixture_base"),
        material=base_color,
        name="base_shell",
    )

    lower_carrier = model.part("lower_carrier")
    lower_carrier.visual(
        mesh_from_cadquery(make_carrier_shape(0.185, 0.0), "lower_carrier"),
        material=lower_color,
        name="lower_shell",
    )

    middle_carrier = model.part("middle_carrier")
    middle_carrier.visual(
        mesh_from_cadquery(make_carrier_shape(0.160, 120.0), "middle_carrier"),
        material=middle_color,
        name="middle_shell",
    )

    upper_carrier = model.part("upper_carrier")
    upper_carrier.visual(
        mesh_from_cadquery(make_carrier_shape(0.140, -120.0), "upper_carrier"),
        material=upper_color,
        name="upper_shell",
    )

    common_limits = MotionLimits(
        effort=12.0,
        velocity=2.5,
        lower=-pi,
        upper=pi,
    )

    model.articulation(
        "base_to_lower_carrier",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_carrier,
        origin=Origin(xyz=(0.0, 0.0, LOWER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=common_limits,
    )
    model.articulation(
        "base_to_middle_carrier",
        ArticulationType.REVOLUTE,
        parent=base,
        child=middle_carrier,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=common_limits,
    )
    model.articulation(
        "base_to_upper_carrier",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_carrier,
        origin=Origin(xyz=(0.0, 0.0, UPPER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=common_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_carrier = object_model.get_part("lower_carrier")
    middle_carrier = object_model.get_part("middle_carrier")
    upper_carrier = object_model.get_part("upper_carrier")
    lower_joint = object_model.get_articulation("base_to_lower_carrier")
    middle_joint = object_model.get_articulation("base_to_middle_carrier")
    upper_joint = object_model.get_articulation("base_to_upper_carrier")

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

    ctx.expect_contact(lower_carrier, base, name="lower carrier supported by lower shoulder")
    ctx.expect_contact(middle_carrier, base, name="middle carrier supported by middle shoulder")
    ctx.expect_contact(upper_carrier, base, name="upper carrier supported by upper shoulder")

    ctx.expect_origin_distance(base, lower_carrier, axes="xy", max_dist=1e-6, name="lower carrier coaxial with tower")
    ctx.expect_origin_distance(base, middle_carrier, axes="xy", max_dist=1e-6, name="middle carrier coaxial with tower")
    ctx.expect_origin_distance(base, upper_carrier, axes="xy", max_dist=1e-6, name="upper carrier coaxial with tower")

    ctx.expect_gap(
        middle_carrier,
        lower_carrier,
        axis="z",
        min_gap=0.010,
        name="middle carrier clears lower carrier vertically",
    )
    ctx.expect_gap(
        upper_carrier,
        middle_carrier,
        axis="z",
        min_gap=0.010,
        name="upper carrier clears middle carrier vertically",
    )

    joints_vertical = all(
        tuple(round(value, 6) for value in joint.axis) == (0.0, 0.0, 1.0)
        for joint in (lower_joint, middle_joint, upper_joint)
    )
    ctx.check(
        "all carriers revolve about common vertical axis",
        joints_vertical,
        details=(
            f"axes were {lower_joint.axis}, {middle_joint.axis}, {upper_joint.axis}"
        ),
    )

    with ctx.pose(
        {
            lower_joint: 0.85,
            middle_joint: -1.05,
            upper_joint: 1.20,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no interference in staggered rotated pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

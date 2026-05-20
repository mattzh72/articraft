from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


TOP_PLATE_X = 0.220
TOP_PLATE_Y = 0.140
TOP_PLATE_T = 0.016
TOP_PLATE_BOTTOM_Z = 0.036

SIDE_CHEEK_X = 0.018
SIDE_CHEEK_Y = 0.102
SIDE_CHEEK_H = 0.046
SIDE_CHEEK_CENTER_X = 0.082

BEARING_HOUSING_R = 0.046
BEARING_HOUSING_H = TOP_PLATE_BOTTOM_Z + 0.006
UPPER_CARTRIDGE_R = 0.032
UPPER_CARTRIDGE_H = 0.020

THRUST_RING_R = 0.043
THRUST_RING_H = 0.014
ROTOR_BODY_R = 0.055
ROTOR_BODY_H = 0.048
OUTPUT_FLANGE_R = 0.075
OUTPUT_FLANGE_H = 0.012
OUTPUT_PILOT_R = 0.024
OUTPUT_PILOT_H = 0.008

CONNECTOR_BLOCK_X = 0.028
CONNECTOR_BLOCK_Y = 0.024
CONNECTOR_BLOCK_Z = 0.020
CONNECTOR_BLOCK_CENTER_Y = 0.065
CONNECTOR_BLOCK_CENTER_Z = -0.040

YAW_LIMIT = pi


def _mount_hole_points() -> list[tuple[float, float]]:
    return [
        (-0.074, -0.042),
        (-0.074, 0.042),
        (0.074, -0.042),
        (0.074, 0.042),
    ]


def _make_support_bracket() -> cq.Workplane:
    top_plate = (
        cq.Workplane("XY")
        .box(TOP_PLATE_X, TOP_PLATE_Y, TOP_PLATE_T, centered=(True, True, False))
        .translate((0.0, 0.0, TOP_PLATE_BOTTOM_Z))
    )
    left_cheek = (
        cq.Workplane("XY")
        .box(SIDE_CHEEK_X, SIDE_CHEEK_Y, SIDE_CHEEK_H, centered=(True, True, False))
        .translate((-SIDE_CHEEK_CENTER_X, 0.0, 0.0))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(SIDE_CHEEK_X, SIDE_CHEEK_Y, SIDE_CHEEK_H, centered=(True, True, False))
        .translate((SIDE_CHEEK_CENTER_X, 0.0, 0.0))
    )
    bearing_housing = cq.Workplane("XY").circle(BEARING_HOUSING_R).extrude(BEARING_HOUSING_H)
    upper_cartridge = (
        cq.Workplane("XY")
        .circle(UPPER_CARTRIDGE_R)
        .extrude(UPPER_CARTRIDGE_H)
        .translate((0.0, 0.0, TOP_PLATE_BOTTOM_Z + TOP_PLATE_T))
    )

    bracket = (
        top_plate.union(left_cheek)
        .union(right_cheek)
        .union(bearing_housing)
        .union(upper_cartridge)
    )

    window_cut = (
        cq.Workplane("YZ")
        .moveTo(0.006, -0.034)
        .lineTo(0.028, -0.010)
        .lineTo(0.028, 0.010)
        .lineTo(0.006, 0.034)
        .close()
        .extrude(SIDE_CHEEK_X + 0.004, both=True)
    )
    bracket = bracket.cut(window_cut.translate((-SIDE_CHEEK_CENTER_X, 0.0, 0.0)))
    bracket = bracket.cut(window_cut.translate((SIDE_CHEEK_CENTER_X, 0.0, 0.0)))

    bracket = (
        bracket.faces(">Z")
        .workplane()
        .pushPoints(_mount_hole_points())
        .hole(0.010)
    )
    return bracket


def _make_rotary_member() -> cq.Workplane:
    thrust_ring = cq.Workplane("XY").circle(THRUST_RING_R).extrude(THRUST_RING_H).translate(
        (0.0, 0.0, -THRUST_RING_H)
    )
    rotor_body = cq.Workplane("XY").circle(ROTOR_BODY_R).extrude(ROTOR_BODY_H).translate(
        (0.0, 0.0, -(THRUST_RING_H + ROTOR_BODY_H))
    )
    output_flange = (
        cq.Workplane("XY")
        .sketch()
        .circle(OUTPUT_FLANGE_R)
        .circle(0.018, mode="s")
        .finalize()
        .extrude(OUTPUT_FLANGE_H)
        .translate((0.0, 0.0, -(THRUST_RING_H + ROTOR_BODY_H + OUTPUT_FLANGE_H)))
    )
    output_pilot = (
        cq.Workplane("XY")
        .sketch()
        .circle(OUTPUT_PILOT_R)
        .circle(0.018, mode="s")
        .finalize()
        .extrude(OUTPUT_PILOT_H)
        .translate(
            (
                0.0,
                0.0,
                -(THRUST_RING_H + ROTOR_BODY_H + OUTPUT_FLANGE_H + OUTPUT_PILOT_H),
            )
        )
    )
    connector_block = (
        cq.Workplane("XY")
        .box(CONNECTOR_BLOCK_X, CONNECTOR_BLOCK_Y, CONNECTOR_BLOCK_Z)
        .translate((0.0, CONNECTOR_BLOCK_CENTER_Y, CONNECTOR_BLOCK_CENTER_Z))
    )

    member = (
        thrust_ring.union(rotor_body)
        .union(output_flange)
        .union(output_pilot)
        .union(connector_block)
    )

    bolt_circle = (
        cq.Workplane("XY")
        .polarArray(0.048, 0.0, 360.0, 6)
        .circle(0.004)
        .extrude(OUTPUT_FLANGE_H + OUTPUT_PILOT_H + 0.002)
        .translate(
            (
                0.0,
                0.0,
                -(THRUST_RING_H + ROTOR_BODY_H + OUTPUT_FLANGE_H + OUTPUT_PILOT_H + 0.001),
            )
        )
    )
    member = member.cut(bolt_circle)
    return member


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_yaw_rotary_module")

    model.material("bracket_paint", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("rotor_metal", rgba=(0.70, 0.72, 0.75, 1.0))

    support_bracket = model.part("support_bracket")
    support_bracket.visual(
        mesh_from_cadquery(_make_support_bracket(), "support_bracket"),
        material="bracket_paint",
        name="support_bracket_body",
    )
    support_bracket.inertial = Inertial.from_geometry(
        Box((TOP_PLATE_X, TOP_PLATE_Y, TOP_PLATE_BOTTOM_Z + TOP_PLATE_T + UPPER_CARTRIDGE_H)),
        mass=5.6,
        origin=Origin(
            xyz=(0.0, 0.0, (TOP_PLATE_BOTTOM_Z + TOP_PLATE_T + UPPER_CARTRIDGE_H) / 2.0)
        ),
    )

    rotary_member = model.part("rotary_member")
    rotary_member.visual(
        mesh_from_cadquery(_make_rotary_member(), "rotary_member"),
        material="rotor_metal",
        name="rotary_member_body",
    )
    rotary_member.inertial = Inertial.from_geometry(
        Box(
            (
                2.0 * OUTPUT_FLANGE_R,
                2.0 * OUTPUT_FLANGE_R,
                THRUST_RING_H + ROTOR_BODY_H + OUTPUT_FLANGE_H + OUTPUT_PILOT_H,
            )
        ),
        mass=3.8,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                -0.5 * (THRUST_RING_H + ROTOR_BODY_H + OUTPUT_FLANGE_H + OUTPUT_PILOT_H),
            )
        ),
    )

    model.articulation(
        "support_to_rotary",
        ArticulationType.REVOLUTE,
        parent=support_bracket,
        child=rotary_member,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.8,
            lower=-YAW_LIMIT,
            upper=YAW_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_bracket = object_model.get_part("support_bracket")
    rotary_member = object_model.get_part("rotary_member")
    yaw_joint = object_model.get_articulation("support_to_rotary")

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

    limits = yaw_joint.motion_limits
    ctx.check(
        "vertical_single_axis_yaw_joint",
        yaw_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in yaw_joint.axis) == (0.0, 0.0, 1.0),
        details=f"expected a single vertical revolute axis, got type={yaw_joint.articulation_type} axis={yaw_joint.axis}",
    )
    ctx.check(
        "yaw_joint_has_bidirectional_travel",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper
        and limits.upper - limits.lower >= 2.5,
        details=f"expected broad bidirectional yaw travel, got limits={limits}",
    )
    ctx.expect_contact(
        rotary_member,
        support_bracket,
        name="underslung_member_is_carried_by_bracket",
    )
    ctx.expect_gap(
        support_bracket,
        rotary_member,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        name="moving_member_hangs_directly_below_top_support",
    )
    ctx.expect_overlap(
        support_bracket,
        rotary_member,
        axes="xy",
        min_overlap=0.086,
        name="rotary_axis_stays_centered_under_bracket",
    )

    with ctx.pose({yaw_joint: 1.2}):
        ctx.expect_contact(
            rotary_member,
            support_bracket,
            name="bearing_support_contact_persists_when_rotated",
        )
        ctx.expect_gap(
            support_bracket,
            rotary_member,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            name="rotary_member_remains_underslung_at_yaw_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_L = 0.46
BASE_W = 0.16
BASE_T = 0.018
RAIL_L = 0.38
RAIL_W = 0.022
RAIL_H = 0.024
RAIL_Y = 0.045
END_STOP_L = 0.012
END_STOP_H = 0.012

CARR_L = 0.12
CARR_W = 0.13
CARR_T = 0.014
BRACKET_GAP = 0.060
CHEEK_T = 0.010
CHEEK_L = 0.080
CHEEK_H = 0.100
PIVOT_X = 0.045
PIVOT_Z = 0.085

HUB_R = 0.024
AXLE_R = 0.010
PIVOT_BORE_R = 0.0096
ARM_L = 0.30
ARM_W = 0.022
ARM_H = 0.020
GUIDE_START_X = 0.080
GUIDE_L = 0.22
GUIDE_W = 0.020
GUIDE_H = 0.010
GUIDE_TOP_Z = ARM_H / 2.0 + GUIDE_H

DISTAL_JOINT_X = 0.10
SLIDE_L = 0.085
SLIDE_W = 0.050
SLIDE_SHOE_T = 0.008
SLIDE_BODY_L = 0.050
SLIDE_BODY_H = 0.026
SLIDE_SIDE_T = 0.008
SLIDE_SIDE_DROP = 0.010

ROOT_SLIDE_X = -0.10


def _make_base_shape() -> cq.Workplane:
    deck = (
        cq.Workplane("XY")
        .box(BASE_L, BASE_W, BASE_T)
        .translate((0.0, 0.0, BASE_T / 2.0))
    )
    rail_left = (
        cq.Workplane("XY")
        .box(RAIL_L, RAIL_W, RAIL_H)
        .translate((0.0, RAIL_Y, BASE_T + RAIL_H / 2.0))
    )
    rail_right = (
        cq.Workplane("XY")
        .box(RAIL_L, RAIL_W, RAIL_H)
        .translate((0.0, -RAIL_Y, BASE_T + RAIL_H / 2.0))
    )
    foot_left = (
        cq.Workplane("XY")
        .box(BASE_L * 0.72, 0.028, 0.010)
        .translate((0.0, 0.046, 0.005))
    )
    foot_right = (
        cq.Workplane("XY")
        .box(BASE_L * 0.72, 0.028, 0.010)
        .translate((0.0, -0.046, 0.005))
    )
    stop_front = (
        cq.Workplane("XY")
        .box(END_STOP_L, BASE_W * 0.70, END_STOP_H)
        .translate((RAIL_L / 2.0 - END_STOP_L / 2.0, 0.0, BASE_T + END_STOP_H / 2.0))
    )
    stop_back = (
        cq.Workplane("XY")
        .box(END_STOP_L, BASE_W * 0.70, END_STOP_H)
        .translate((-RAIL_L / 2.0 + END_STOP_L / 2.0, 0.0, BASE_T + END_STOP_H / 2.0))
    )

    return (
        deck.union(rail_left)
        .union(rail_right)
        .union(foot_left)
        .union(foot_right)
        .union(stop_front)
        .union(stop_back)
    )


def _make_carriage_bracket_shape() -> cq.Workplane:
    cheek_y = BRACKET_GAP / 2.0 + CHEEK_T / 2.0
    plate = (
        cq.Workplane("XY")
        .box(CARR_L, 0.104, CARR_T)
        .translate((0.0, 0.0, CARR_T / 2.0))
    )
    skid_left = (
        cq.Workplane("XY")
        .box(CARR_L * 0.70, RAIL_W, 0.010)
        .translate((0.0, RAIL_Y, 0.005))
    )
    skid_right = (
        cq.Workplane("XY")
        .box(CARR_L * 0.70, RAIL_W, 0.010)
        .translate((0.0, -RAIL_Y, 0.005))
    )
    cheek_left = (
        cq.Workplane("XY")
        .box(0.060, CHEEK_T, 0.094)
        .translate((PIVOT_X - 0.008, cheek_y, CARR_T + 0.047))
    )
    cheek_right = (
        cq.Workplane("XY")
        .box(0.060, CHEEK_T, 0.094)
        .translate((PIVOT_X - 0.008, -cheek_y, CARR_T + 0.047))
    )
    rear_tie = (
        cq.Workplane("XY")
        .box(0.018, BRACKET_GAP, 0.034)
        .translate((PIVOT_X - 0.032, 0.0, CARR_T + 0.017))
    )
    left_gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.024, CARR_T),
                (PIVOT_X - 0.026, CARR_T),
                (PIVOT_X - 0.004, PIVOT_Z - 0.022),
                (-0.024, PIVOT_Z - 0.012),
            ]
        )
        .close()
        .extrude(CHEEK_T)
        .translate((0.0, BRACKET_GAP / 2.0, 0.0))
    )
    right_gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.024, CARR_T),
                (PIVOT_X - 0.026, CARR_T),
                (PIVOT_X - 0.004, PIVOT_Z - 0.022),
                (-0.024, PIVOT_Z - 0.012),
            ]
        )
        .close()
        .extrude(CHEEK_T)
        .translate((0.0, -BRACKET_GAP / 2.0 - CHEEK_T, 0.0))
    )
    left_bore = (
        cq.Workplane("XZ")
        .circle(PIVOT_BORE_R)
        .extrude(CHEEK_T + 0.006)
        .translate((PIVOT_X, BRACKET_GAP / 2.0 - 0.003, PIVOT_Z))
    )
    right_bore = (
        cq.Workplane("XZ")
        .circle(PIVOT_BORE_R)
        .extrude(CHEEK_T + 0.006)
        .translate((PIVOT_X, -BRACKET_GAP / 2.0 - CHEEK_T - 0.003, PIVOT_Z))
    )

    return (
        plate.union(skid_left)
        .union(skid_right)
        .union(cheek_left)
        .union(cheek_right)
        .union(rear_tie)
        .union(left_gusset)
        .union(right_gusset)
        .cut(left_bore)
        .cut(right_bore)
    )


def _make_arm_shape() -> cq.Workplane:
    shoulder_len = 0.032
    shoulder_start = 0.022
    shoulder = (
        cq.Workplane("XY")
        .box(shoulder_len, BRACKET_GAP, 0.030)
        .translate((shoulder_start + shoulder_len / 2.0, 0.0, 0.0))
    )
    beam_start = shoulder_start + shoulder_len
    beam = (
        cq.Workplane("XY")
        .box(ARM_L - beam_start, ARM_W, ARM_H)
        .translate((beam_start + (ARM_L - beam_start) / 2.0, 0.0, 0.0))
    )
    neck = (
        cq.Workplane("XY")
        .box(0.050, 0.036, 0.022)
        .translate((0.050, 0.0, 0.0))
    )
    guide = (
        cq.Workplane("XY")
        .box(GUIDE_L, GUIDE_W, GUIDE_H)
        .translate((GUIDE_START_X + GUIDE_L / 2.0, 0.0, ARM_H / 2.0 + GUIDE_H / 2.0))
    )
    lower_rib = (
        cq.Workplane("XY")
        .box(0.090, ARM_W * 0.82, 0.014)
        .translate((0.065, 0.0, -ARM_H / 2.0))
    )
    side_boss_left = (
        cq.Workplane("XZ")
        .circle(0.013)
        .extrude(0.004)
        .translate((0.010, 0.025, 0.0))
    )
    side_boss_right = (
        cq.Workplane("XZ")
        .circle(0.013)
        .extrude(0.004)
        .translate((0.010, -0.029, 0.0))
    )

    return (
        shoulder.union(neck)
        .union(beam)
        .union(guide)
        .union(lower_rib)
    )


def _make_distal_slider_shape() -> cq.Workplane:
    shoe = (
        cq.Workplane("XY")
        .box(SLIDE_L, SLIDE_W, SLIDE_SHOE_T)
        .translate((SLIDE_L / 2.0, 0.0, SLIDE_SHOE_T / 2.0))
    )
    body = (
        cq.Workplane("XY")
        .box(SLIDE_BODY_L, SLIDE_W, SLIDE_BODY_H)
        .translate((SLIDE_L * 0.60, 0.0, SLIDE_SHOE_T + SLIDE_BODY_H / 2.0))
    )
    side_y = GUIDE_W / 2.0 + SLIDE_SIDE_T / 2.0 + 0.003
    side_left = (
        cq.Workplane("XY")
        .box(SLIDE_L * 0.70, SLIDE_SIDE_T, SLIDE_SIDE_DROP)
        .translate((SLIDE_L * 0.42, side_y, SLIDE_SHOE_T - SLIDE_SIDE_DROP / 2.0))
    )
    side_right = (
        cq.Workplane("XY")
        .box(SLIDE_L * 0.70, SLIDE_SIDE_T, SLIDE_SIDE_DROP)
        .translate((SLIDE_L * 0.42, -side_y, SLIDE_SHOE_T - SLIDE_SIDE_DROP / 2.0))
    )
    nose = (
        cq.Workplane("XY")
        .box(0.012, SLIDE_W * 0.90, 0.018)
        .translate((SLIDE_L - 0.006, 0.0, SLIDE_SHOE_T + 0.009))
    )

    return shoe.union(body).union(side_left).union(side_right).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prismatic_revolute_prismatic_stage")

    model.material("base_steel", rgba=(0.26, 0.28, 0.30, 1.0))
    model.material("carriage_aluminum", rgba=(0.73, 0.76, 0.80, 1.0))
    model.material("arm_orange", rgba=(0.85, 0.46, 0.16, 1.0))
    model.material("slider_graphite", rgba=(0.22, 0.24, 0.27, 1.0))

    base = model.part("base_slide")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base_slide"),
        name="base_slide_visual",
        material="base_steel",
    )

    carriage = model.part("carriage_bracket")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_bracket_shape(), "carriage_bracket"),
        name="carriage_bracket_visual",
        material="carriage_aluminum",
    )

    arm = model.part("end_member")
    arm.visual(
        mesh_from_cadquery(_make_arm_shape(), "end_member"),
        name="end_member_visual",
        material="arm_orange",
    )

    distal = model.part("distal_slide")
    distal.visual(
        mesh_from_cadquery(_make_distal_slider_shape(), "distal_slide"),
        name="distal_slide_visual",
        material="slider_graphite",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(ROOT_SLIDE_X, 0.0, BASE_T + RAIL_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.30,
            lower=0.0,
            upper=0.18,
        ),
    )

    model.articulation(
        "carriage_to_end_member",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=arm,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.5,
            lower=-0.25,
            upper=1.20,
        ),
    )

    model.articulation(
        "end_member_to_distal_slide",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=distal,
        origin=Origin(xyz=(DISTAL_JOINT_X, 0.0, GUIDE_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.18,
            lower=0.0,
            upper=0.08,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_slide")
    carriage = object_model.get_part("carriage_bracket")
    arm = object_model.get_part("end_member")
    distal = object_model.get_part("distal_slide")

    root_slide = object_model.get_articulation("base_to_carriage")
    pivot = object_model.get_articulation("carriage_to_end_member")
    distal_slide = object_model.get_articulation("end_member_to_distal_slide")

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

    ctx.expect_contact(
        carriage,
        base,
        contact_tol=1e-6,
        name="carriage_supported_on_base_rails",
    )
    ctx.expect_contact(
        arm,
        carriage,
        contact_tol=1e-6,
        name="arm_hub_bears_in_pivot_bracket",
    )
    ctx.expect_contact(
        distal,
        arm,
        contact_tol=1e-6,
        name="distal_slide_supported_on_local_guide",
    )

    with ctx.pose({root_slide: 0.0, pivot: 0.0, distal_slide: 0.0}):
        carriage_x0, _, carriage_z0 = ctx.part_world_position(carriage)
    with ctx.pose({root_slide: 0.12, pivot: 0.0, distal_slide: 0.0}):
        carriage_x1, _, carriage_z1 = ctx.part_world_position(carriage)
    ctx.check(
        "root_prismatic_moves_carriage_along_base",
        carriage_x1 > carriage_x0 + 0.10 and abs(carriage_z1 - carriage_z0) < 1e-6,
        details=(
            f"expected carriage x to increase by > 0.10 m with stable z, "
            f"got x0={carriage_x0:.4f}, x1={carriage_x1:.4f}, "
            f"z0={carriage_z0:.4f}, z1={carriage_z1:.4f}"
        ),
    )

    with ctx.pose({root_slide: 0.0, pivot: 0.0, distal_slide: 0.0}):
        distal_rest = ctx.part_world_position(distal)
    with ctx.pose({root_slide: 0.0, pivot: 0.90, distal_slide: 0.0}):
        distal_raised = ctx.part_world_position(distal)
    ctx.check(
        "revolute_joint_lifts_end_member",
        distal_raised[2] > distal_rest[2] + 0.07,
        details=(
            f"expected distal slide z to rise with positive pivot motion, "
            f"got z_rest={distal_rest[2]:.4f}, z_raised={distal_raised[2]:.4f}"
        ),
    )

    with ctx.pose({root_slide: 0.0, pivot: 0.80, distal_slide: 0.0}):
        distal_retracted = ctx.part_world_position(distal)
    with ctx.pose({root_slide: 0.0, pivot: 0.80, distal_slide: 0.06}):
        distal_extended = ctx.part_world_position(distal)
    ctx.check(
        "distal_prismatic_follows_rotated_local_guide",
        distal_extended[0] > distal_retracted[0] + 0.03
        and distal_extended[2] > distal_retracted[2] + 0.03,
        details=(
            f"expected distal extension in a raised pose to advance in both x and z, "
            f"got retracted={distal_retracted}, extended={distal_extended}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

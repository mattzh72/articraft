from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


FOOT_LEN = 0.160
FOOT_W = 0.080
FOOT_T = 0.012
FOOT_SLOT_LEN = 0.030
FOOT_SLOT_W = 0.010
FOOT_SLOT_X = 0.045

AXIS_HEIGHT = 0.043

HOUSING_W = 0.064
HOUSING_H = 0.044
BEARING_BLOCK_LEN = 0.028
BEARING_BLOCK_W = 0.058
BEARING_BLOCK_X = 0.031
END_BOSS_T = 0.008
END_BOSS_R = 0.024
HOUSING_TOTAL_LEN = 2.0 * (BEARING_BLOCK_X + BEARING_BLOCK_LEN / 2.0 + END_BOSS_T)
TOP_BRIDGE_LEN = 0.040
TOP_BRIDGE_W = 0.058
TOP_BRIDGE_T = 0.010
TOP_BRIDGE_Z = 0.022

LEG_LEN = 0.018
LEG_W = 0.048
LEG_H = AXIS_HEIGHT - HOUSING_H / 2.0
LEG_X = 0.026

SHAFT_R = 0.0110
REAR_FLANGE_R = 0.020
REAR_FLANGE_X0 = -0.065
REAR_FLANGE_X1 = -0.053

SHAFT_X0 = -0.054
SHAFT_X1 = 0.058

FACEPLATE_R = 0.035
FACEPLATE_X0 = 0.058
FACEPLATE_X1 = 0.070
PILOT_R = 0.015
PILOT_X1 = 0.074
BOLT_CIRCLE_R = 0.021
BOLT_HOLE_R = 0.003
INDEX_PIN_R = 0.003
INDEX_PIN_LEN = 0.010
INDEX_PIN_Z = 0.020


def _x_cylinder(radius: float, x0: float, x1: float, *, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .center(y, z)
        .circle(radius)
        .extrude(x1 - x0)
        .translate((x0, 0.0, 0.0))
    )


def make_mounting_foot() -> cq.Workplane:
    foot = cq.Workplane("XY").box(FOOT_LEN, FOOT_W, FOOT_T).translate((0.0, 0.0, -FOOT_T / 2.0))
    foot = foot.edges("|Z").fillet(0.003)

    slot_cutter = (
        cq.Workplane("XY")
        .slot2D(FOOT_SLOT_LEN, FOOT_SLOT_W)
        .extrude(FOOT_T + 0.002)
        .translate((0.0, 0.0, -FOOT_T - 0.001))
    )
    foot = foot.cut(slot_cutter.translate((FOOT_SLOT_X, 0.0, 0.0)))
    foot = foot.cut(slot_cutter.translate((-FOOT_SLOT_X, 0.0, 0.0)))
    return foot


def make_bearing_housing() -> cq.Workplane:
    left_block = (
        cq.Workplane("XY")
        .box(BEARING_BLOCK_LEN, BEARING_BLOCK_W, HOUSING_H)
        .translate((-BEARING_BLOCK_X, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.004)
    )
    right_block = (
        cq.Workplane("XY")
        .box(BEARING_BLOCK_LEN, BEARING_BLOCK_W, HOUSING_H)
        .translate((BEARING_BLOCK_X, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.004)
    )

    left_boss = _x_cylinder(
        radius=END_BOSS_R,
        x0=-(BEARING_BLOCK_X + BEARING_BLOCK_LEN / 2.0 + END_BOSS_T),
        x1=-(BEARING_BLOCK_X + BEARING_BLOCK_LEN / 2.0),
    )
    right_boss = _x_cylinder(
        radius=END_BOSS_R,
        x0=(BEARING_BLOCK_X + BEARING_BLOCK_LEN / 2.0),
        x1=(BEARING_BLOCK_X + BEARING_BLOCK_LEN / 2.0 + END_BOSS_T),
    )

    left_carrier = left_block.union(left_boss)
    right_carrier = right_block.union(right_boss)

    left_bore = _x_cylinder(
        0.0175,
        -(BEARING_BLOCK_X + BEARING_BLOCK_LEN / 2.0 + END_BOSS_T + 0.001),
        -(BEARING_BLOCK_X - BEARING_BLOCK_LEN / 2.0 - 0.001),
    )
    right_bore = _x_cylinder(
        0.0175,
        (BEARING_BLOCK_X - BEARING_BLOCK_LEN / 2.0 - 0.001),
        (BEARING_BLOCK_X + BEARING_BLOCK_LEN / 2.0 + END_BOSS_T + 0.001),
    )
    left_carrier = left_carrier.cut(left_bore)
    right_carrier = right_carrier.cut(right_bore)

    top_bridge = cq.Workplane("XY").box(TOP_BRIDGE_LEN, TOP_BRIDGE_W, TOP_BRIDGE_T).translate((0.0, 0.0, TOP_BRIDGE_Z))

    leg_z = -(HOUSING_H / 2.0 + LEG_H / 2.0)
    left_leg = cq.Workplane("XY").box(LEG_LEN, LEG_W, LEG_H).translate((-LEG_X, 0.0, leg_z))
    right_leg = cq.Workplane("XY").box(LEG_LEN, LEG_W, LEG_H).translate((LEG_X, 0.0, leg_z))

    return left_carrier.union(right_carrier).union(top_bridge).union(left_leg).union(right_leg)


def make_spindle_journal() -> cq.Workplane:
    return _x_cylinder(SHAFT_R, SHAFT_X0, SHAFT_X1)


def make_rear_flange() -> cq.Workplane:
    return _x_cylinder(REAR_FLANGE_R, REAR_FLANGE_X0, REAR_FLANGE_X1)


def make_faceplate() -> cq.Workplane:
    faceplate = _x_cylinder(FACEPLATE_R, FACEPLATE_X0, FACEPLATE_X1)
    pilot = _x_cylinder(PILOT_R, FACEPLATE_X1, PILOT_X1)
    faceplate = faceplate.union(pilot)

    bolt_points = [
        (
            BOLT_CIRCLE_R * math.cos(angle),
            BOLT_CIRCLE_R * math.sin(angle),
        )
        for angle in (math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)
    ]
    bolt_cutter = (
        cq.Workplane("YZ")
        .pushPoints(bolt_points)
        .circle(BOLT_HOLE_R)
        .extrude(PILOT_X1 - FACEPLATE_X0 + 0.002)
        .translate((FACEPLATE_X0 - 0.001, 0.0, 0.0))
    )
    return faceplate.cut(bolt_cutter)


def make_index_pin() -> cq.Workplane:
    return _x_cylinder(
        INDEX_PIN_R,
        FACEPLATE_X1,
        FACEPLATE_X1 + INDEX_PIN_LEN,
        z=INDEX_PIN_Z,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_axis_roll_stage")

    foot_material = model.material("painted_steel", rgba=(0.20, 0.22, 0.25, 1.0))
    housing_material = model.material("housing_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    spindle_material = model.material("ground_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    indicator_material = model.material("indicator_pin", rgba=(0.84, 0.18, 0.12, 1.0))

    foot = model.part("mounting_foot")
    foot.visual(
        mesh_from_cadquery(make_mounting_foot(), "mounting_foot_v2"),
        origin=Origin(),
        material=foot_material,
        name="foot",
    )

    housing = model.part("bearing_housing")
    housing.visual(
        mesh_from_cadquery(make_bearing_housing(), "bearing_housing_v5"),
        origin=Origin(),
        material=housing_material,
        name="housing_body",
    )

    spindle = model.part("spindle")
    spindle.visual(
        mesh_from_cadquery(make_spindle_journal(), "spindle_journal_v3"),
        origin=Origin(),
        material=spindle_material,
        name="journal",
    )
    spindle.visual(
        mesh_from_cadquery(make_rear_flange(), "spindle_rear_flange_v1"),
        origin=Origin(),
        material=spindle_material,
        name="rear_flange",
    )
    spindle.visual(
        mesh_from_cadquery(make_faceplate(), "spindle_faceplate_v3"),
        origin=Origin(),
        material=spindle_material,
        name="faceplate",
    )
    spindle.visual(
        mesh_from_cadquery(make_index_pin(), "spindle_index_pin_v3"),
        origin=Origin(),
        material=indicator_material,
        name="index_pin",
    )

    model.articulation(
        "foot_to_housing",
        ArticulationType.FIXED,
        parent=foot,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, AXIS_HEIGHT)),
    )
    model.articulation(
        "housing_to_spindle",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=spindle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=6.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)
    foot = object_model.get_part("mounting_foot")
    housing = object_model.get_part("bearing_housing")
    spindle = object_model.get_part("spindle")
    roll = object_model.get_articulation("housing_to_spindle")

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
        "spindle_joint_type",
        roll.articulation_type == ArticulationType.REVOLUTE,
        details=f"expected REVOLUTE, got {roll.articulation_type}",
    )
    ctx.check(
        "spindle_joint_axis",
        all(math.isclose(v, t, abs_tol=1e-9) for v, t in zip(roll.axis, (1.0, 0.0, 0.0))),
        details=f"axis={roll.axis}",
    )
    limits = roll.motion_limits
    ctx.check(
        "spindle_joint_limits",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and math.isclose(limits.lower, -math.pi, abs_tol=1e-9)
        and math.isclose(limits.upper, math.pi, abs_tol=1e-9),
        details=f"limits={limits}",
    )

    ctx.expect_contact(
        housing,
        foot,
        elem_a="housing_body",
        elem_b="foot",
        name="housing_supported_by_foot",
    )
    ctx.expect_overlap(
        housing,
        foot,
        axes="xy",
        min_overlap=0.055,
        elem_a="housing_body",
        elem_b="foot",
        name="housing_centered_over_foot",
    )
    ctx.expect_origin_gap(
        housing,
        foot,
        axis="z",
        min_gap=0.0425,
        max_gap=0.0435,
        name="housing_axis_height",
    )

    ctx.expect_contact(
        spindle,
        housing,
        elem_a="rear_flange",
        elem_b="housing_body",
        name="spindle_retained_at_rear_face",
    )
    ctx.expect_origin_distance(
        spindle,
        housing,
        axes="yz",
        max_dist=1e-6,
        name="spindle_coaxial_with_housing",
    )
    ctx.expect_overlap(
        spindle,
        housing,
        axes="yz",
        min_overlap=0.020,
        elem_a="journal",
        elem_b="housing_body",
        name="journal_aligned_with_bearing_seats",
    )
    ctx.expect_gap(
        spindle,
        housing,
        axis="x",
        positive_elem="faceplate",
        negative_elem="housing_body",
        min_gap=0.0045,
        max_gap=0.0055,
        name="faceplate_clears_housing_front",
    )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({roll: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="spindle_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="spindle_lower_no_floating")
            ctx.expect_contact(
                spindle,
                housing,
                elem_a="rear_flange",
                elem_b="housing_body",
                name="spindle_lower_rear_retention",
            )
            ctx.expect_gap(
                spindle,
                housing,
                axis="x",
                positive_elem="faceplate",
                negative_elem="housing_body",
                min_gap=0.0045,
                max_gap=0.0055,
                name="spindle_lower_faceplate_clearance",
            )
        with ctx.pose({roll: math.pi / 2.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="spindle_mid_no_overlap")
            ctx.fail_if_isolated_parts(name="spindle_mid_no_floating")
            ctx.expect_contact(
                spindle,
                housing,
                elem_a="rear_flange",
                elem_b="housing_body",
                name="spindle_mid_rear_retention",
            )
        with ctx.pose({roll: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="spindle_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="spindle_upper_no_floating")
            ctx.expect_contact(
                spindle,
                housing,
                elem_a="rear_flange",
                elem_b="housing_body",
                name="spindle_upper_rear_retention",
            )
            ctx.expect_gap(
                spindle,
                housing,
                axis="x",
                positive_elem="faceplate",
                negative_elem="housing_body",
                min_gap=0.0045,
                max_gap=0.0055,
                name="spindle_upper_faceplate_clearance",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

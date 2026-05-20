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


BODY_L = 0.28
BODY_W = 0.20
BODY_BASE_H = 0.052
BODY_DECK_H = 0.014
BODY_POD_H = 0.012
BODY_POD_D = 0.108
BODY_TOTAL_H = BODY_BASE_H + BODY_DECK_H + BODY_POD_H

STAGE_D = 0.132
STAGE_H = 0.018
TOWER_Y = 0.018
LOWER_BRIDGE_W = 0.102
LOWER_BRIDGE_D = 0.044
LOWER_BRIDGE_H = 0.022
RAIL_SPACING = 0.060
RAIL_R = 0.008
RAIL_H = 0.160
TOP_BRIDGE_W = 0.098
TOP_BRIDGE_D = 0.038
TOP_BRIDGE_H = 0.014

CARR_W = 0.094
CARR_D = 0.050
CARR_H = 0.048
CARR_NOSE_W = 0.054
CARR_NOSE_D = 0.012
CARR_NOSE_H = 0.028
CARR_BORE_R = 0.0096
CARR_HOME_Z = STAGE_H + LOWER_BRIDGE_H + (CARR_H / 2.0)
LIFT_STROKE = 0.090


def _make_body_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .rect(BODY_L, BODY_W)
        .extrude(BODY_BASE_H)
        .edges("|Z")
        .fillet(0.018)
        .faces(">Z")
        .workplane()
        .rect(BODY_L - 0.040, BODY_W - 0.040)
        .extrude(BODY_DECK_H)
        .faces(">Z")
        .workplane()
        .circle(BODY_POD_D / 2.0)
        .extrude(BODY_POD_H)
    )


def _make_stage_shape() -> cq.Workplane:
    disc = (
        cq.Workplane("XY")
        .circle(STAGE_D / 2.0)
        .extrude(STAGE_H)
        .edges("%CIRCLE")
        .fillet(0.003)
    )

    lower_bridge = (
        cq.Workplane("XY")
        .rect(LOWER_BRIDGE_W, LOWER_BRIDGE_D)
        .extrude(LOWER_BRIDGE_H)
        .translate((0.0, TOWER_Y, STAGE_H))
        .edges("|Z")
        .fillet(0.005)
    )

    rails = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-RAIL_SPACING / 2.0, TOWER_Y),
                (RAIL_SPACING / 2.0, TOWER_Y),
            ]
        )
        .circle(RAIL_R)
        .extrude(RAIL_H)
        .translate((0.0, 0.0, STAGE_H + LOWER_BRIDGE_H))
    )

    top_bridge = (
        cq.Workplane("XY")
        .rect(TOP_BRIDGE_W, TOP_BRIDGE_D)
        .extrude(TOP_BRIDGE_H)
        .translate((0.0, TOWER_Y, STAGE_H + LOWER_BRIDGE_H + RAIL_H))
        .edges("|Z")
        .fillet(0.004)
    )

    return disc.union(lower_bridge).union(rails).union(top_bridge)


def _make_carriage_shape() -> cq.Workplane:
    carriage = (
        cq.Workplane("XY")
        .box(CARR_W, CARR_D, CARR_H)
        .edges("|Z")
        .fillet(0.006)
    )

    nose = cq.Workplane("XY").box(CARR_NOSE_W, CARR_NOSE_D, CARR_NOSE_H).translate(
        (0.0, (CARR_D / 2.0) + (CARR_NOSE_D / 2.0) - 0.004, -0.002)
    )
    carriage = carriage.union(nose)

    return (
        carriage.faces(">Z")
        .workplane()
        .pushPoints([(-RAIL_SPACING / 2.0, 0.0), (RAIL_SPACING / 2.0, 0.0)])
        .circle(CARR_BORE_R)
        .cutThruAll()
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_pan_lift_unit")

    body_paint = model.material("body_paint", rgba=(0.20, 0.21, 0.23, 1.0))
    stage_metal = model.material("stage_metal", rgba=(0.63, 0.66, 0.70, 1.0))
    carriage_finish = model.material("carriage_finish", rgba=(0.38, 0.43, 0.49, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shape(), "body_shell"),
        material=body_paint,
        name="body_shell",
    )

    stage = model.part("rotary_stage")
    stage.visual(
        mesh_from_cadquery(_make_stage_shape(), "rotary_stage"),
        material=stage_metal,
        name="rotary_stage",
    )

    carriage = model.part("vertical_carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_shape(), "vertical_carriage"),
        material=carriage_finish,
        name="vertical_carriage",
    )

    model.articulation(
        "body_to_stage",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stage,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOTAL_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=-2.35,
            upper=2.35,
        ),
    )

    model.articulation(
        "stage_to_carriage",
        ArticulationType.PRISMATIC,
        parent=stage,
        child=carriage,
        origin=Origin(xyz=(0.0, TOWER_Y, CARR_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.20,
            lower=0.0,
            upper=LIFT_STROKE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    stage = object_model.get_part("rotary_stage")
    carriage = object_model.get_part("vertical_carriage")
    pan = object_model.get_articulation("body_to_stage")
    lift = object_model.get_articulation("stage_to_carriage")

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

    ctx.expect_gap(
        stage,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="rotary_stage_seated_on_body",
    )
    ctx.expect_contact(
        carriage,
        stage,
        contact_tol=0.001,
        name="carriage_supported_on_lower_stage_at_home",
    )
    ctx.expect_overlap(
        stage,
        body,
        axes="xy",
        min_overlap=0.09,
        name="body_supports_broad_rotary_stage_footprint",
    )
    ctx.expect_overlap(
        carriage,
        stage,
        axes="xy",
        min_overlap=0.05,
        name="carriage_stays_over_rotary_stage",
    )
    ctx.expect_gap(
        carriage,
        body,
        axis="z",
        min_gap=0.035,
        name="carriage_clears_body",
    )

    ctx.check(
        "pan_joint_is_vertical_revolute",
        pan.articulation_type == ArticulationType.REVOLUTE and pan.axis == (0.0, 0.0, 1.0),
        details=f"type={pan.articulation_type}, axis={pan.axis}",
    )
    ctx.check(
        "lift_joint_is_vertical_prismatic",
        lift.articulation_type == ArticulationType.PRISMATIC and lift.axis == (0.0, 0.0, 1.0),
        details=f"type={lift.articulation_type}, axis={lift.axis}",
    )

    with ctx.pose({pan: 0.0, lift: 0.0}):
        home_pos = ctx.part_world_position(carriage)
    with ctx.pose({pan: 1.2, lift: 0.0}):
        panned_pos = ctx.part_world_position(carriage)
    ctx.check(
        "pan_stage_swings_offset_carriage_in_xy",
        home_pos is not None
        and panned_pos is not None
        and abs(panned_pos[0] - home_pos[0]) > 0.010
        and abs(panned_pos[2] - home_pos[2]) < 1e-4,
        details=f"home={home_pos}, panned={panned_pos}",
    )

    upper_lift = lift.motion_limits.upper if lift.motion_limits is not None else None
    with ctx.pose({pan: 0.0, lift: 0.0}):
        low_pos = ctx.part_world_position(carriage)
    with ctx.pose({pan: 0.0, lift: upper_lift or 0.0}):
        high_pos = ctx.part_world_position(carriage)
    ctx.check(
        "carriage_lifts_upward",
        low_pos is not None
        and high_pos is not None
        and upper_lift is not None
        and (high_pos[2] - low_pos[2]) > 0.080,
        details=f"low={low_pos}, high={high_pos}, stroke={upper_lift}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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


BASE_LENGTH = 0.34
BASE_WIDTH = 0.28
BASE_HEIGHT = 0.074
BASE_FOOT_THICKNESS = 0.016
BASE_PLINTH_RADIUS = 0.088

STAGE_RADIUS = 0.11
STAGE_THICKNESS = 0.018
GUIDE_Y = -0.055
GUIDE_HEIGHT = 0.31
ROD_SPACING = 0.06
ROD_RADIUS = 0.009
ROD_LENGTH = 0.282
ROD_START_Z = STAGE_THICKNESS

CARRIAGE_WIDTH = 0.116
CARRIAGE_BODY_DEPTH = 0.038
CARRIAGE_HEIGHT = 0.070
CARRIAGE_ENVELOPE_HEIGHT = CARRIAGE_HEIGHT
CARRIAGE_HOME_STOP_TOP_Z = 0.040
CARRIAGE_HOME_CENTER_Z = CARRIAGE_HOME_STOP_TOP_Z + CARRIAGE_ENVELOPE_HEIGHT / 2.0
CARRIAGE_TRAVEL = 0.16


def make_base_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_FOOT_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.022)
    )

    plinth = (
        cq.Workplane("XY")
        .circle(BASE_PLINTH_RADIUS)
        .extrude(BASE_HEIGHT - BASE_FOOT_THICKNESS)
        .translate((0.0, 0.0, BASE_FOOT_THICKNESS))
    )

    drive_housing = (
        cq.Workplane("XY")
        .box(0.118, 0.094, 0.030, centered=(True, True, False))
        .translate((0.0, -0.070, BASE_FOOT_THICKNESS))
        .edges("|Z")
        .fillet(0.010)
    )

    front_rib = (
        cq.Workplane("XY")
        .box(0.150, 0.070, 0.018, centered=(True, True, False))
        .translate((0.0, 0.062, BASE_FOOT_THICKNESS))
        .edges("|Z")
        .fillet(0.008)
    )

    return foot.union(plinth).union(drive_housing).union(front_rib)


def make_turntable_deck_shape() -> cq.Workplane:
    deck = cq.Workplane("XY").circle(STAGE_RADIUS).extrude(STAGE_THICKNESS)

    center_hub = (
        cq.Workplane("XY")
        .circle(0.046)
        .extrude(0.012)
        .translate((0.0, 0.0, STAGE_THICKNESS))
    )

    return deck.union(center_hub)


def make_guide_frame_shape() -> cq.Workplane:
    lower_saddle = (
        cq.Workplane("XY")
        .box(0.104, 0.032, 0.028, centered=(True, True, False))
        .translate((0.0, GUIDE_Y - 0.040, STAGE_THICKNESS))
        .edges("|Z")
        .fillet(0.005)
    )

    lower_rod_block = (
        cq.Workplane("XY")
        .box(0.088, 0.026, 0.022, centered=(True, True, False))
        .translate((0.0, GUIDE_Y - 0.012, STAGE_THICKNESS))
        .edges("|Z")
        .fillet(0.004)
    )

    back_spine = (
        cq.Workplane("XY")
        .box(0.094, 0.016, GUIDE_HEIGHT - 0.068, centered=(True, True, False))
        .translate((0.0, GUIDE_Y - 0.040, STAGE_THICKNESS + 0.028))
        .edges("|Z")
        .fillet(0.004)
    )

    top_head = (
        cq.Workplane("XY")
        .box(0.120, 0.022, 0.022, centered=(True, True, False))
        .translate((0.0, GUIDE_Y - 0.030, GUIDE_HEIGHT - 0.022))
        .edges("|Z")
        .fillet(0.005)
    )

    upper_bridge = (
        cq.Workplane("XY")
        .box(0.080, 0.028, 0.016, centered=(True, True, False))
        .translate((0.0, GUIDE_Y - 0.012, GUIDE_HEIGHT - 0.040))
        .edges("|Z")
        .fillet(0.004)
    )

    return lower_saddle.union(lower_rod_block).union(back_spine).union(top_head).union(upper_bridge)


def make_home_stop_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.100, 0.036, 0.014, centered=(True, True, False))
        .translate((0.0, GUIDE_Y + 0.028, CARRIAGE_HOME_STOP_TOP_Z - 0.014))
        .edges("|Z")
        .fillet(0.004)
    )


def make_guide_rods_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-ROD_SPACING / 2.0, GUIDE_Y),
                (ROD_SPACING / 2.0, GUIDE_Y),
            ]
        )
        .circle(ROD_RADIUS)
        .extrude(ROD_LENGTH)
        .translate((0.0, 0.0, ROD_START_Z))
    )


def make_carriage_shape() -> cq.Workplane:
    left_bearing = (
        cq.Workplane("XY")
        .circle(0.017)
        .extrude(CARRIAGE_HEIGHT)
        .translate((-ROD_SPACING / 2.0, 0.0, -CARRIAGE_HEIGHT / 2.0))
    )

    right_bearing = (
        cq.Workplane("XY")
        .circle(0.017)
        .extrude(CARRIAGE_HEIGHT)
        .translate((ROD_SPACING / 2.0, 0.0, -CARRIAGE_HEIGHT / 2.0))
    )

    center_bridge = (
        cq.Workplane("XY")
        .box(0.086, 0.032, 0.044)
        .translate((0.0, 0.022, 0.0))
        .edges("|Z")
        .fillet(0.005)
    )

    nut_block = (
        cq.Workplane("XY")
        .box(0.028, 0.024, 0.032)
        .translate((0.0, 0.022, 0.0))
        .edges("|Z")
        .fillet(0.003)
    )

    face_plate = (
        cq.Workplane("XY")
        .box(0.094, 0.010, CARRIAGE_HEIGHT)
        .translate((0.0, 0.040, 0.0))
        .edges("|Z")
        .fillet(0.004)
    )

    rod_bores = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-ROD_SPACING / 2.0, 0.0),
                (ROD_SPACING / 2.0, 0.0),
            ]
        )
        .circle(ROD_RADIUS + 0.0012)
        .extrude(CARRIAGE_HEIGHT + 0.016, both=True)
    )

    center_relief = cq.Workplane("XY").box(0.040, 0.056, 0.034).translate(
        (0.0, 0.012, 0.0)
    )

    return (
        left_bearing.union(right_bearing)
        .union(center_bridge)
        .union(nut_block)
        .union(face_plate)
        .cut(rod_bores)
        .cut(center_relief)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turntable_lift_module")

    dark_base = model.material("dark_base", color=(0.17, 0.18, 0.20))
    machine_gray = model.material("machine_gray", color=(0.70, 0.72, 0.75))
    guide_steel = model.material("guide_steel", color=(0.82, 0.84, 0.87))
    carriage_orange = model.material("carriage_orange", color=(0.86, 0.42, 0.16))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_shape(), "base_body"),
        material=dark_base,
        name="base_body",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    rotary_stage = model.part("rotary_stage")
    rotary_stage.visual(
        mesh_from_cadquery(make_turntable_deck_shape(), "turntable_deck"),
        material=machine_gray,
        name="turntable_deck",
    )
    rotary_stage.visual(
        mesh_from_cadquery(make_guide_frame_shape(), "guide_frame"),
        material=machine_gray,
        name="guide_frame",
    )
    rotary_stage.visual(
        mesh_from_cadquery(make_home_stop_shape(), "home_stop"),
        material=machine_gray,
        name="home_stop",
    )
    rotary_stage.visual(
        mesh_from_cadquery(make_guide_rods_shape(), "guide_rods"),
        material=guide_steel,
        name="guide_rods",
    )
    rotary_stage.inertial = Inertial.from_geometry(
        Box((0.24, 0.18, GUIDE_HEIGHT)),
        mass=4.6,
        origin=Origin(xyz=(0.0, -0.02, GUIDE_HEIGHT / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(make_carriage_shape(), "carriage_body"),
        material=carriage_orange,
        name="carriage_body",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.12, 0.09, CARRIAGE_HEIGHT)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.022, 0.0)),
    )

    model.articulation(
        "base_to_stage",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rotary_stage,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.4,
            lower=-pi,
            upper=pi,
        ),
    )

    model.articulation(
        "stage_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rotary_stage,
        child=carriage,
        origin=Origin(xyz=(0.0, GUIDE_Y, CARRIAGE_HOME_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.20,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    rotary_stage = object_model.get_part("rotary_stage")
    carriage = object_model.get_part("carriage")
    turn = object_model.get_articulation("base_to_stage")
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

    turn_limits = turn.motion_limits
    lift_limits = lift.motion_limits

    ctx.check(
        "turntable joint is vertical revolute",
        turn.articulation_type == ArticulationType.REVOLUTE
        and tuple(turn.axis) == (0.0, 0.0, 1.0)
        and turn_limits is not None
        and turn_limits.lower == -pi
        and turn_limits.upper == pi,
        details=f"type={turn.articulation_type}, axis={turn.axis}, limits={turn_limits}",
    )
    ctx.check(
        "lift joint is vertical prismatic",
        lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(lift.axis) == (0.0, 0.0, 1.0)
        and lift_limits is not None
        and lift_limits.lower == 0.0
        and lift_limits.upper == CARRIAGE_TRAVEL,
        details=f"type={lift.articulation_type}, axis={lift.axis}, limits={lift_limits}",
    )

    ctx.expect_gap(
        rotary_stage,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="turntable_deck",
        negative_elem="base_body",
        name="turntable deck seats on base plinth",
    )
    ctx.expect_overlap(
        rotary_stage,
        base,
        axes="xy",
        min_overlap=0.10,
        elem_a="turntable_deck",
        elem_b="base_body",
        name="turntable deck overlaps base support footprint",
    )
    ctx.expect_gap(
        carriage,
        rotary_stage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="carriage_body",
        negative_elem="home_stop",
        name="carriage rests on lower stop at home",
    )

    home_position = ctx.part_world_position(carriage)
    if home_position is None or lift_limits is None or lift_limits.upper is None:
        ctx.fail("carriage pose references resolve", "missing carriage position or lift upper limit")
    else:
        with ctx.pose({lift: lift_limits.upper}):
            lifted_position = ctx.part_world_position(carriage)
            if lifted_position is None:
                ctx.fail("carriage lifted pose resolves", "missing carriage position in lifted pose")
            else:
                dz = lifted_position[2] - home_position[2]
                dx = abs(lifted_position[0] - home_position[0])
                dy = abs(lifted_position[1] - home_position[1])
                ctx.check(
                    "prismatic joint raises carriage vertically",
                    abs(dz - lift_limits.upper) <= 0.003 and dx <= 0.0015 and dy <= 0.0015,
                    details=f"dx={dx:.5f}, dy={dy:.5f}, dz={dz:.5f}, expected={lift_limits.upper:.5f}",
                )
                ctx.expect_gap(
                    carriage,
                    rotary_stage,
                    axis="z",
                    min_gap=lift_limits.upper - 0.004,
                    positive_elem="carriage_body",
                    negative_elem="home_stop",
                    name="carriage clears lower stop when raised",
                )

    if home_position is None:
        ctx.fail("carriage home pose resolves", "missing carriage home position")
    else:
        with ctx.pose({turn: pi / 2.0}):
            quarter_turn_position = ctx.part_world_position(carriage)
            if quarter_turn_position is None:
                ctx.fail("carriage quarter turn pose resolves", "missing carriage position after rotation")
            else:
                rotated_radius = (quarter_turn_position[0] ** 2 + quarter_turn_position[1] ** 2) ** 0.5
                home_radius = (home_position[0] ** 2 + home_position[1] ** 2) ** 0.5
                ctx.check(
                    "turntable carries offset guide around vertical axis",
                    abs(rotated_radius - abs(GUIDE_Y)) <= 0.010
                    and abs(home_radius - abs(GUIDE_Y)) <= 0.010
                    and quarter_turn_position[0] > 0.040
                    and abs(quarter_turn_position[1]) <= 0.015,
                    details=(
                        f"home={home_position}, quarter={quarter_turn_position}, "
                        f"home_radius={home_radius:.5f}, quarter_radius={rotated_radius:.5f}"
                    ),
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

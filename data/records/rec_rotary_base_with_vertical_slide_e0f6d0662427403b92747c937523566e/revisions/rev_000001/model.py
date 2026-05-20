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


FOOT_LENGTH = 0.48
FOOT_WIDTH = 0.34
FOOT_THICKNESS = 0.035
PEDESTAL_HEIGHT = 0.065
PEDESTAL_BASE_RADIUS = 0.090
PEDESTAL_TOP_RADIUS = 0.064
SEAT_RADIUS = 0.075
SEAT_HEIGHT = 0.018

PLATFORM_JOINT_Z = FOOT_THICKNESS + PEDESTAL_HEIGHT + SEAT_HEIGHT

TURNTABLE_HUB_RADIUS = 0.070
TURNTABLE_HUB_HEIGHT = 0.045
TURNTABLE_DISC_RADIUS = 0.170
TURNTABLE_DISC_THICKNESS = 0.018
TURNTABLE_DISC_Z0 = 0.008

GUIDE_SPINE_WIDTH = 0.110
GUIDE_SPINE_DEPTH = 0.030
GUIDE_SPINE_HEIGHT = 0.440
GUIDE_SPINE_Y = 0.070
GUIDE_SPINE_Z0 = 0.018

GUIDE_RAIL_WIDTH = 0.060
GUIDE_RAIL_DEPTH = 0.016
GUIDE_RAIL_HEIGHT = 0.340
GUIDE_RAIL_Y = 0.093
GUIDE_RAIL_Z0 = 0.090

GUIDE_RIB_WIDTH = 0.020
GUIDE_RIB_DEPTH = 0.055
GUIDE_RIB_HEIGHT = 0.085
GUIDE_RIB_X = 0.040
GUIDE_RIB_Y = 0.072
GUIDE_RIB_Z0 = 0.008

CARRIAGE_BLOCK_WIDTH = 0.074
CARRIAGE_BLOCK_DEPTH = 0.014
CARRIAGE_BLOCK_HEIGHT = 0.108
CARRIAGE_BLOCK_STANDOFF = 0.0
CARRIAGE_BLOCK_Y = GUIDE_RAIL_DEPTH / 2 + CARRIAGE_BLOCK_STANDOFF + CARRIAGE_BLOCK_DEPTH / 2

BRACKET_WEB_WIDTH = 0.050
BRACKET_WEB_DEPTH = 0.024
BRACKET_WEB_HEIGHT = 0.076
BRACKET_WEB_Y = 0.028

MOUNT_BLOCK_WIDTH = 0.120
MOUNT_BLOCK_DEPTH = 0.028
MOUNT_BLOCK_HEIGHT = 0.056
MOUNT_BLOCK_Y = 0.036

FRONT_PANEL_WIDTH = 0.180
FRONT_PANEL_DEPTH = 0.012
FRONT_PANEL_HEIGHT = 0.130
FRONT_PANEL_Y = 0.056

FRONT_PLATE_HOME_Z = 0.165
FRONT_PLATE_TRAVEL = 0.180


def make_foot_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(FOOT_LENGTH, FOOT_WIDTH, FOOT_THICKNESS)
        .translate((0.0, 0.0, FOOT_THICKNESS / 2))
        .edges("|Z")
        .fillet(0.028)
    )
    pedestal = (
        cq.Workplane("XY")
        .workplane(offset=FOOT_THICKNESS)
        .circle(PEDESTAL_BASE_RADIUS)
        .workplane(offset=PEDESTAL_HEIGHT)
        .circle(PEDESTAL_TOP_RADIUS)
        .loft(combine=True)
    )
    seat = (
        cq.Workplane("XY")
        .workplane(offset=FOOT_THICKNESS + PEDESTAL_HEIGHT)
        .circle(SEAT_RADIUS)
        .extrude(SEAT_HEIGHT)
    )
    return base.union(pedestal).union(seat)


def make_turntable_body_shape() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(TURNTABLE_HUB_RADIUS).extrude(TURNTABLE_HUB_HEIGHT)
    disc = (
        cq.Workplane("XY")
        .workplane(offset=TURNTABLE_DISC_Z0)
        .circle(TURNTABLE_DISC_RADIUS)
        .extrude(TURNTABLE_DISC_THICKNESS)
    )
    return hub.union(disc)


def make_upright_guide_shape() -> cq.Workplane:
    spine = (
        cq.Workplane("XY")
        .box(GUIDE_SPINE_WIDTH, GUIDE_SPINE_DEPTH, GUIDE_SPINE_HEIGHT)
        .translate(
            (
                0.0,
                GUIDE_SPINE_Y,
                GUIDE_SPINE_Z0 + GUIDE_SPINE_HEIGHT / 2,
            )
        )
    )
    rail = (
        cq.Workplane("XY")
        .box(GUIDE_RAIL_WIDTH, GUIDE_RAIL_DEPTH, GUIDE_RAIL_HEIGHT)
        .translate(
            (
                0.0,
                GUIDE_RAIL_Y,
                GUIDE_RAIL_Z0 + GUIDE_RAIL_HEIGHT / 2,
            )
        )
    )
    left_rib = (
        cq.Workplane("XY")
        .box(GUIDE_RIB_WIDTH, GUIDE_RIB_DEPTH, GUIDE_RIB_HEIGHT)
        .translate(
            (
                -GUIDE_RIB_X,
                GUIDE_RIB_Y,
                GUIDE_RIB_Z0 + GUIDE_RIB_HEIGHT / 2,
            )
        )
    )
    right_rib = (
        cq.Workplane("XY")
        .box(GUIDE_RIB_WIDTH, GUIDE_RIB_DEPTH, GUIDE_RIB_HEIGHT)
        .translate(
            (
                GUIDE_RIB_X,
                GUIDE_RIB_Y,
                GUIDE_RIB_Z0 + GUIDE_RIB_HEIGHT / 2,
            )
        )
    )
    return spine.union(rail).union(left_rib).union(right_rib)


def make_slider_carriage_shape() -> cq.Workplane:
    carriage_block = cq.Workplane("XY").box(
        CARRIAGE_BLOCK_WIDTH, CARRIAGE_BLOCK_DEPTH, CARRIAGE_BLOCK_HEIGHT
    ).translate((0.0, CARRIAGE_BLOCK_Y, 0.0))
    bracket_web = cq.Workplane("XY").box(
        BRACKET_WEB_WIDTH, BRACKET_WEB_DEPTH, BRACKET_WEB_HEIGHT
    ).translate((0.0, BRACKET_WEB_Y, 0.0))
    mount_block = cq.Workplane("XY").box(
        MOUNT_BLOCK_WIDTH, MOUNT_BLOCK_DEPTH, MOUNT_BLOCK_HEIGHT
    ).translate((0.0, MOUNT_BLOCK_Y, 0.0))
    return carriage_block.union(bracket_web).union(mount_block)


def make_front_panel_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(FRONT_PANEL_WIDTH, FRONT_PANEL_DEPTH, FRONT_PANEL_HEIGHT)
        .translate((0.0, FRONT_PANEL_Y, 0.0))
        .edges("|Z")
        .fillet(0.004)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_rotary_lift")

    foot_dark = model.material("foot_dark", rgba=(0.18, 0.20, 0.22, 1.0))
    platform_gray = model.material("platform_gray", rgba=(0.60, 0.63, 0.66, 1.0))
    rail_gray = model.material("rail_gray", rgba=(0.48, 0.50, 0.54, 1.0))
    plate_blue = model.material("plate_blue", rgba=(0.18, 0.39, 0.72, 1.0))

    foot = model.part("foot")
    foot.visual(
        mesh_from_cadquery(make_foot_shape(), "pedestal_foot"),
        material=foot_dark,
        name="pedestal_foot",
    )

    platform = model.part("platform")
    platform.visual(
        mesh_from_cadquery(make_turntable_body_shape(), "turntable_body"),
        material=platform_gray,
        name="turntable_body",
    )
    platform.visual(
        mesh_from_cadquery(make_upright_guide_shape(), "upright_guide"),
        material=rail_gray,
        name="upright_guide",
    )

    front_plate = model.part("front_plate")
    front_plate.visual(
        mesh_from_cadquery(make_slider_carriage_shape(), "slider_carriage"),
        material=rail_gray,
        name="slider_carriage",
    )
    front_plate.visual(
        mesh_from_cadquery(make_front_panel_shape(), "front_panel"),
        material=plate_blue,
        name="front_panel",
    )

    model.articulation(
        "foot_to_platform",
        ArticulationType.REVOLUTE,
        parent=foot,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, PLATFORM_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "platform_to_front_plate",
        ArticulationType.PRISMATIC,
        parent=platform,
        child=front_plate,
        origin=Origin(xyz=(0.0, GUIDE_RAIL_Y, FRONT_PLATE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.20,
            lower=0.0,
            upper=FRONT_PLATE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    foot = object_model.get_part("foot")
    platform = object_model.get_part("platform")
    front_plate = object_model.get_part("front_plate")
    turntable = object_model.get_articulation("foot_to_platform")
    lift = object_model.get_articulation("platform_to_front_plate")

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
        "required_parts_present",
        all(part is not None for part in (foot, platform, front_plate)),
        "foot, platform, and front_plate must all be authored",
    )
    ctx.check(
        "turntable_joint_configured",
        turntable.articulation_type == ArticulationType.REVOLUTE
        and tuple(turntable.axis) == (0.0, 0.0, 1.0)
        and turntable.motion_limits is not None
        and turntable.motion_limits.lower is not None
        and turntable.motion_limits.upper is not None
        and turntable.motion_limits.lower < 0.0 < turntable.motion_limits.upper,
        "platform should rotate about the vertical z axis with bidirectional travel",
    )
    ctx.check(
        "lift_joint_configured",
        lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(lift.axis) == (0.0, 0.0, 1.0)
        and lift.motion_limits is not None
        and lift.motion_limits.lower == 0.0
        and lift.motion_limits.upper == FRONT_PLATE_TRAVEL,
        "front plate should slide upward on a vertical prismatic joint",
    )

    with ctx.pose({turntable: 0.0, lift: 0.0}):
        ctx.expect_contact(
            platform,
            foot,
            elem_a="turntable_body",
            elem_b="pedestal_foot",
            contact_tol=0.001,
            name="turntable_body_seats_on_foot",
        )
        ctx.expect_overlap(
            platform,
            foot,
            axes="xy",
            elem_a="turntable_body",
            elem_b="pedestal_foot",
            min_overlap=0.12,
            name="turntable_centered_over_pedestal",
        )
        ctx.expect_gap(
            front_plate,
            platform,
            axis="z",
            min_gap=0.05,
            max_gap=0.08,
            positive_elem="front_panel",
            negative_elem="turntable_body",
            name="front_panel_clears_turntable_body",
        )

    with ctx.pose({turntable: 0.0, lift: FRONT_PLATE_TRAVEL * 0.5}):
        ctx.expect_contact(
            front_plate,
            platform,
            elem_a="slider_carriage",
            elem_b="upright_guide",
            contact_tol=0.001,
            name="slider_carriage_contacts_upright_guide",
        )
        ctx.expect_overlap(
            front_plate,
            platform,
            axes="xz",
            elem_a="slider_carriage",
            elem_b="upright_guide",
            min_overlap=0.04,
            name="slider_carriage_tracks_on_upright_guide",
        )

    with ctx.pose({turntable: 0.0, lift: 0.0}):
        home_pos = ctx.part_world_position(front_plate)
    with ctx.pose({turntable: 0.0, lift: FRONT_PLATE_TRAVEL}):
        raised_pos = ctx.part_world_position(front_plate)

    lift_ok = (
        home_pos is not None
        and raised_pos is not None
        and abs((raised_pos[2] - home_pos[2]) - FRONT_PLATE_TRAVEL) <= 1e-6
        and abs(raised_pos[0] - home_pos[0]) <= 1e-6
        and abs(raised_pos[1] - home_pos[1]) <= 1e-6
    )
    ctx.check(
        "front_plate_moves_straight_up",
        lift_ok,
        f"home={home_pos}, raised={raised_pos}, expected_dz={FRONT_PLATE_TRAVEL}",
    )

    mid_lift = FRONT_PLATE_TRAVEL * 0.5
    with ctx.pose({turntable: 0.0, lift: mid_lift}):
        start_pos = ctx.part_world_position(front_plate)
    with ctx.pose({turntable: 1.0, lift: mid_lift}):
        rotated_pos = ctx.part_world_position(front_plate)

    if start_pos is None or rotated_pos is None:
        rotate_ok = False
    else:
        start_radius = math.hypot(start_pos[0], start_pos[1])
        rotated_radius = math.hypot(rotated_pos[0], rotated_pos[1])
        rotate_ok = (
            abs(start_radius - rotated_radius) <= 1e-6
            and abs(rotated_pos[2] - start_pos[2]) <= 1e-6
            and abs(rotated_pos[0] - start_pos[0]) >= 0.05
        )
    ctx.check(
        "platform_rotation_swings_guide_and_plate",
        rotate_ok,
        f"start={start_pos}, rotated={rotated_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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


PLATE_T = 0.018
PLATE_W = 0.240
PLATE_H = 0.420
MOUNT_HOLE_D = 0.016
MOUNT_HOLE_Y = 0.075
MOUNT_HOLE_Z = 0.145

HOUSING_R = 0.063
HOUSING_LEN = 0.034
FLANGE_R = 0.072
FLANGE_LEN = 0.008
RIB_Y = 0.070
RIB_T = 0.018

ARM_HUB_R = 0.055
ARM_HUB_LEN = 0.038
ARM_BODY_X = ARM_HUB_LEN - 0.010
ARM_BODY_T = 0.032
ARM_FRONT_X = ARM_BODY_X + ARM_BODY_T
ARM_BODY_W = 0.080
ARM_TIP_W = 0.048
ARM_PROFILE_START_Y = 0.022
ARM_STRAIGHT_END_Y = 0.500
ARM_TIP_Y = 0.560

RAIL_H = 0.014
RAIL_W = 0.046
RAIL_START_Y = 0.120
RAIL_LEN = 0.400

CARRIAGE_DEPTH = 0.072
CARRIAGE_LEN = 0.100
CARRIAGE_W = 0.096
CARRIAGE_SLOT_CLEAR_Z = 0.006
TOOL_BOSS_R = 0.022
TOOL_BOSS_LEN = 0.018

RETRACTED_Y = 0.180
SLIDE_TRAVEL = 0.260
ROTARY_LOWER = -1.25
ROTARY_UPPER = 1.35


def _make_side_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(PLATE_T, PLATE_W, PLATE_H)
    plate = plate.edges("|X").fillet(0.016)
    plate = (
        plate.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-MOUNT_HOLE_Y, -MOUNT_HOLE_Z),
                (-MOUNT_HOLE_Y, MOUNT_HOLE_Z),
                (MOUNT_HOLE_Y, -MOUNT_HOLE_Z),
                (MOUNT_HOLE_Y, MOUNT_HOLE_Z),
            ]
        )
        .hole(MOUNT_HOLE_D)
    )

    housing = (
        cq.Workplane("YZ")
        .circle(HOUSING_R)
        .extrude(HOUSING_LEN)
        .translate((PLATE_T / 2.0, 0.0, 0.0))
    )
    flange = (
        cq.Workplane("YZ")
        .circle(FLANGE_R)
        .extrude(FLANGE_LEN)
        .translate((PLATE_T / 2.0 + HOUSING_LEN - FLANGE_LEN, 0.0, 0.0))
    )

    upper_rib = cq.Workplane("XY").box(
        HOUSING_LEN * 0.82,
        RIB_Y,
        RIB_T,
    ).translate(
        (
            PLATE_T / 2.0 + (HOUSING_LEN * 0.82) / 2.0,
            0.0,
            HOUSING_R - 0.008,
        )
    )
    lower_rib = cq.Workplane("XY").box(
        HOUSING_LEN * 0.82,
        RIB_Y,
        RIB_T,
    ).translate(
        (
            PLATE_T / 2.0 + (HOUSING_LEN * 0.82) / 2.0,
            0.0,
            -HOUSING_R + 0.008,
        )
    )

    bearing_recess = (
        cq.Workplane("YZ")
        .circle(0.032)
        .extrude(0.006)
        .translate((PLATE_T / 2.0 + HOUSING_LEN - 0.006, 0.0, 0.0))
    )

    return (
        plate.union(housing)
        .union(flange)
        .union(upper_rib)
        .union(lower_rib)
        .cut(bearing_recess)
    )


def _make_arm_shape() -> cq.Workplane:
    hub = cq.Workplane("YZ").circle(ARM_HUB_R).extrude(ARM_HUB_LEN)

    beam_profile = (
        cq.Workplane("YZ")
        .polyline(
            [
                (ARM_PROFILE_START_Y, -ARM_BODY_W / 2.0),
                (ARM_STRAIGHT_END_Y, -ARM_BODY_W / 2.0),
                (ARM_TIP_Y, -ARM_TIP_W / 2.0),
                (ARM_TIP_Y, ARM_TIP_W / 2.0),
                (ARM_STRAIGHT_END_Y, ARM_BODY_W / 2.0),
                (ARM_PROFILE_START_Y, ARM_BODY_W / 2.0),
            ]
        )
        .close()
        .extrude(ARM_BODY_T)
        .translate((ARM_BODY_X, 0.0, 0.0))
    )

    rail = cq.Workplane("XY").box(
        RAIL_H,
        RAIL_LEN,
        RAIL_W,
    ).translate(
        (
            ARM_FRONT_X + RAIL_H / 2.0,
            RAIL_START_Y + RAIL_LEN / 2.0,
            0.0,
        )
    )

    tip_pad = cq.Workplane("XY").box(
        0.012,
        0.036,
        ARM_TIP_W,
    ).translate((ARM_FRONT_X - 0.006, ARM_TIP_Y - 0.008, 0.0))

    return hub.union(beam_profile).union(rail).union(tip_pad)


def _make_carriage_shape() -> cq.Workplane:
    carriage = cq.Workplane("XY").box(
        CARRIAGE_DEPTH,
        CARRIAGE_LEN,
        CARRIAGE_W,
    ).translate((CARRIAGE_DEPTH / 2.0, 0.0, 0.0))

    rail_slot = cq.Workplane("XY").box(
        RAIL_H + 0.002,
        CARRIAGE_LEN + 0.004,
        RAIL_W + CARRIAGE_SLOT_CLEAR_Z,
    ).translate(((RAIL_H + 0.002) / 2.0, 0.0, 0.0))

    front_pad = cq.Workplane("XY").box(
        0.012,
        0.062,
        0.074,
    ).translate((CARRIAGE_DEPTH + 0.006, 0.0, 0.0))

    tool_boss = (
        cq.Workplane("YZ")
        .circle(TOOL_BOSS_R)
        .extrude(TOOL_BOSS_LEN)
        .translate((CARRIAGE_DEPTH, 0.0, 0.0))
    )
    tool_bore = (
        cq.Workplane("YZ")
        .circle(0.008)
        .extrude(TOOL_BOSS_LEN)
        .translate((CARRIAGE_DEPTH, 0.0, 0.0))
    )

    return carriage.cut(rail_slot).union(front_pad).union(tool_boss.cut(tool_bore))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_radial_arm_module")

    model.material("plate_gray", rgba=(0.46, 0.49, 0.52, 1.0))
    model.material("arm_graphite", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("carriage_orange", rgba=(0.78, 0.47, 0.16, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        mesh_from_cadquery(_make_side_plate_shape(), "side_plate"),
        material="plate_gray",
        name="plate_body",
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_make_arm_shape(), "radial_arm"),
        material="arm_graphite",
        name="arm_body",
    )

    tool_carriage = model.part("tool_carriage")
    tool_carriage.visual(
        mesh_from_cadquery(_make_carriage_shape(), "tool_carriage"),
        material="carriage_orange",
        name="carriage_body",
    )

    model.articulation(
        "side_plate_to_arm",
        ArticulationType.REVOLUTE,
        parent=side_plate,
        child=arm,
        origin=Origin(xyz=(PLATE_T / 2.0 + HOUSING_LEN, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.2,
            lower=ROTARY_LOWER,
            upper=ROTARY_UPPER,
        ),
    )
    model.articulation(
        "arm_to_carriage",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=tool_carriage,
        origin=Origin(xyz=(ARM_FRONT_X, RETRACTED_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.25,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    arm = object_model.get_part("arm")
    tool_carriage = object_model.get_part("tool_carriage")
    rotary = object_model.get_articulation("side_plate_to_arm")
    slide = object_model.get_articulation("arm_to_carriage")

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
        all(part is not None for part in (side_plate, arm, tool_carriage)),
        "side plate, arm, and tool carriage must all exist",
    )
    ctx.check(
        "required_joints_present",
        rotary is not None and slide is not None,
        "the rotary boss and the sliding carriage joints must both exist",
    )

    ctx.expect_contact(
        arm,
        side_plate,
        name="rotary_boss_contacts_side_plate_support",
    )
    ctx.expect_contact(
        tool_carriage,
        arm,
        name="tool_carriage_is_supported_on_arm_face",
    )
    ctx.expect_overlap(
        tool_carriage,
        arm,
        axes="yz",
        min_overlap=0.040,
        name="tool_carriage_stays_on_arm_guide_footprint",
    )

    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_contact(
            tool_carriage,
            arm,
            name="tool_carriage_remains_supported_fully_extended",
        )
        ctx.expect_origin_gap(
            tool_carriage,
            arm,
            axis="y",
            min_gap=RETRACTED_Y + SLIDE_TRAVEL - 0.002,
            max_gap=RETRACTED_Y + SLIDE_TRAVEL + 0.002,
            name="prismatic_joint_advances_carriage_along_arm",
        )

    with ctx.pose({rotary: 0.0, slide: 0.140}):
        neutral_pos = ctx.part_world_position(tool_carriage)
    with ctx.pose({rotary: 0.90, slide: 0.140}):
        raised_pos = ctx.part_world_position(tool_carriage)

    ctx.check(
        "positive_rotary_motion_lifts_the_arm",
        neutral_pos is not None
        and raised_pos is not None
        and raised_pos[2] > neutral_pos[2] + 0.12,
        f"expected positive rotary motion to raise carriage by > 0.12 m, got neutral={neutral_pos}, raised={raised_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

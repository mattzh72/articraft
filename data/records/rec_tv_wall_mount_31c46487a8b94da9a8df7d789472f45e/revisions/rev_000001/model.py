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


PLATE_T = 0.012
PLATE_W = 0.220
PLATE_H = 0.340
PLATE_SLOT_H = 0.050
PLATE_SLOT_W = 0.012

SHOULDER_HUB_T = 0.010
SHOULDER_BODY_L = 0.026
SHOULDER_ORIGIN_X = PLATE_T / 2.0 + SHOULDER_BODY_L + SHOULDER_HUB_T

ARM_BOSS_T = 0.014
ARM_BOSS_R = 0.038
ARM_W = 0.052
ARM_H = 0.082
ARM_WALL = 0.006
PRIMARY_ARM_L = 0.230
SECONDARY_ARM_L = 0.215

HEAD_HUB_T = 0.012
HEAD_HUB_R = 0.038
FRAME_RING_X = 0.012
TILT_AXIS_X = 0.030
FRAME_DEPTH = 0.012
FRAME_OUTER = 0.240
FRAME_INNER = 0.176
TILT_BRACKET_T = 0.016
TILT_BRACKET_H = 0.056

ROD_R = 0.010
ROD_HALF_SPAN = FRAME_INNER / 2.0 - TILT_BRACKET_T
ROD_LEN = ROD_HALF_SPAN * 2.0
CLAMP_X = 0.024
CLAMP_Y = 0.054
CLAMP_Z = 0.054
MOUNT_PLATE_T = 0.010
MOUNT_PLATE_W = 0.150
MOUNT_PLATE_H = 0.120
MOUNT_PLATE_X = 0.055


def _arm_link(length: float) -> cq.Workplane:
    shell_len = length - 2.0 * ARM_BOSS_T
    rail_center_x = length / 2.0
    side_rail_h = ARM_H - 2.0 * ARM_WALL
    top_bottom_w = ARM_W - 2.0 * ARM_WALL

    top_rail = cq.Workplane("XY").box(shell_len, top_bottom_w, ARM_WALL).translate(
        (rail_center_x, 0.0, ARM_H / 2.0 - ARM_WALL / 2.0)
    )
    bottom_rail = cq.Workplane("XY").box(shell_len, top_bottom_w, ARM_WALL).translate(
        (rail_center_x, 0.0, -ARM_H / 2.0 + ARM_WALL / 2.0)
    )
    left_rail = cq.Workplane("XY").box(shell_len, ARM_WALL, side_rail_h).translate(
        (rail_center_x, -ARM_W / 2.0 + ARM_WALL / 2.0, 0.0)
    )
    right_rail = cq.Workplane("XY").box(shell_len, ARM_WALL, side_rail_h).translate(
        (rail_center_x, ARM_W / 2.0 - ARM_WALL / 2.0, 0.0)
    )

    rear_boss = cq.Workplane("YZ").circle(ARM_BOSS_R).extrude(ARM_BOSS_T)
    front_boss = (
        cq.Workplane("YZ")
        .circle(ARM_BOSS_R)
        .extrude(ARM_BOSS_T)
        .translate((length - ARM_BOSS_T, 0.0, 0.0))
    )

    rear_reinforcement = cq.Workplane("XY").box(0.030, 0.044, 0.070).translate(
        (0.028, 0.0, 0.0)
    )
    front_reinforcement = cq.Workplane("XY").box(0.030, 0.044, 0.070).translate(
        (length - 0.028, 0.0, 0.0)
    )

    return (
        top_rail.union(bottom_rail)
        .union(left_rail)
        .union(right_rail)
        .union(rear_boss)
        .union(front_boss)
        .union(rear_reinforcement)
        .union(front_reinforcement)
        .clean()
    )


def _wall_plate() -> cq.Workplane:
    plate = cq.Workplane("XY").box(PLATE_T, PLATE_W, PLATE_H).edges("|X").fillet(0.010)

    slots = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-0.070, 0.110),
                (0.070, 0.110),
                (-0.070, -0.110),
                (0.070, -0.110),
            ]
        )
        .slot2D(PLATE_SLOT_H, PLATE_SLOT_W, 90)
        .extrude(PLATE_T * 3.0, both=True)
    )
    plate = plate.cut(slots)

    shoulder_body = cq.Workplane("XY").box(0.026, 0.082, 0.120).translate(
        (PLATE_T / 2.0 + 0.026 / 2.0, 0.0, 0.0)
    )
    shoulder_cap = (
        cq.Workplane("YZ")
        .circle(HEAD_HUB_R)
        .extrude(SHOULDER_HUB_T)
        .translate((SHOULDER_ORIGIN_X - SHOULDER_HUB_T, 0.0, 0.0))
    )

    upper_rib = (
        cq.Workplane("XZ")
        .polyline(
            [
                (PLATE_T / 2.0 - 0.001, 0.020),
                (PLATE_T / 2.0 - 0.001, 0.125),
                (PLATE_T / 2.0 + SHOULDER_BODY_L * 0.84, 0.052),
            ]
        )
        .close()
        .extrude(0.110, both=True)
    )
    lower_rib = (
        cq.Workplane("XZ")
        .polyline(
            [
                (PLATE_T / 2.0 - 0.001, -0.020),
                (PLATE_T / 2.0 + SHOULDER_BODY_L * 0.84, -0.052),
                (PLATE_T / 2.0 - 0.001, -0.125),
            ]
        )
        .close()
        .extrude(0.110, both=True)
    )

    return plate.union(shoulder_body).union(shoulder_cap).union(upper_rib).union(lower_rib).clean()


def _head_frame() -> cq.Workplane:
    beam_t = (FRAME_OUTER - FRAME_INNER) / 2.0
    side_beam_y = FRAME_INNER / 2.0 + beam_t / 2.0
    top_beam_z = FRAME_INNER / 2.0 + beam_t / 2.0
    pivot_r = 0.006
    rod_half = FRAME_INNER / 2.0
    yoke_outer_y = rod_half * 2.0 + 0.032
    yoke_inner_y = rod_half * 2.0
    yoke_outer_z = 0.110
    yoke_inner_z = 0.044
    yoke_x = 0.012

    left_beam = cq.Workplane("XY").box(FRAME_DEPTH, beam_t, FRAME_OUTER).translate(
        (FRAME_RING_X, -side_beam_y, 0.0)
    )
    right_beam = cq.Workplane("XY").box(FRAME_DEPTH, beam_t, FRAME_OUTER).translate(
        (FRAME_RING_X, side_beam_y, 0.0)
    )
    top_beam = cq.Workplane("XY").box(FRAME_DEPTH, FRAME_INNER, beam_t).translate(
        (FRAME_RING_X, 0.0, top_beam_z)
    )
    bottom_beam = cq.Workplane("XY").box(FRAME_DEPTH, FRAME_INNER, beam_t).translate(
        (FRAME_RING_X, 0.0, -top_beam_z)
    )

    swivel_hub = cq.Workplane("YZ").circle(HEAD_HUB_R).extrude(HEAD_HUB_T)
    center_neck = cq.Workplane("XY").box(0.034, 0.050, 0.050).translate((0.017, 0.0, 0.0))
    left_bridge = cq.Workplane("XY").box(0.028, 0.032, 0.030).translate((0.020, -0.084, 0.0))
    right_bridge = cq.Workplane("XY").box(0.028, 0.032, 0.030).translate((0.020, 0.084, 0.0))

    yoke_outer = cq.Workplane("XY").box(yoke_x, yoke_outer_y, yoke_outer_z).translate(
        (TILT_AXIS_X, 0.0, 0.0)
    )
    yoke_inner = cq.Workplane("XY").box(yoke_x + 0.004, yoke_inner_y, yoke_inner_z).translate(
        (TILT_AXIS_X, 0.0, 0.0)
    )
    yoke = yoke_outer.cut(yoke_inner)
    pivot_relief = (
        cq.Workplane("XZ")
        .circle(pivot_r + 0.0004)
        .extrude(yoke_inner_y / 2.0 + 0.002, both=True)
        .translate((TILT_AXIS_X, 0.0, 0.0))
    )
    yoke = yoke.cut(pivot_relief)

    return (
        left_beam.union(right_beam)
        .union(top_beam)
        .union(bottom_beam)
        .union(swivel_hub)
        .union(center_neck)
        .union(left_bridge)
        .union(right_bridge)
        .union(yoke)
        .clean()
    )


def _tilt_crossbar() -> cq.Workplane:
    rod_r = 0.006
    rod_half = FRAME_INNER / 2.0

    rod = cq.Workplane("XZ").circle(rod_r).extrude(rod_half * 2.0, both=True)
    center_boss = cq.Workplane("XZ").circle(0.0095).extrude(0.028, both=True)
    clamp = cq.Workplane("XY").box(0.016, 0.032, 0.040).translate((0.008, 0.0, 0.0))
    spine = cq.Workplane("XY").box(0.040, 0.022, 0.086).translate((0.032, 0.0, 0.0))
    plate = cq.Workplane("XY").box(0.012, 0.120, 0.115).translate((0.058, 0.0, 0.0))
    plate = (
        plate.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.035, -0.035),
                (0.035, -0.035),
                (-0.035, 0.035),
                (0.035, 0.035),
            ]
        )
        .hole(0.008)
    )

    return rod.union(center_boss).union(clamp).union(spine).union(plate).clean()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_wall_display_mount")

    powder_black = model.material("powder_black", color=(0.15, 0.15, 0.17, 1.0))
    graphite = model.material("graphite", color=(0.22, 0.23, 0.25, 1.0))
    black_oxide = model.material("black_oxide", color=(0.10, 0.10, 0.11, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        mesh_from_cadquery(_wall_plate(), "wall_plate"),
        material=powder_black,
        name="wall_plate_visual",
    )

    primary_arm = model.part("primary_arm")
    primary_arm.visual(
        mesh_from_cadquery(_arm_link(PRIMARY_ARM_L), "primary_arm"),
        material=graphite,
        name="primary_arm_visual",
    )

    secondary_arm = model.part("secondary_arm")
    secondary_arm.visual(
        mesh_from_cadquery(_arm_link(SECONDARY_ARM_L), "secondary_arm"),
        material=graphite,
        name="secondary_arm_visual",
    )

    head_frame = model.part("head_frame")
    head_frame.visual(
        mesh_from_cadquery(_head_frame(), "head_frame"),
        material=powder_black,
        name="head_frame_visual",
    )

    tilt_crossbar = model.part("tilt_crossbar")
    tilt_crossbar.visual(
        mesh_from_cadquery(_tilt_crossbar(), "tilt_crossbar"),
        material=black_oxide,
        name="tilt_crossbar_visual",
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=primary_arm,
        origin=Origin(xyz=(SHOULDER_ORIGIN_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.2,
            lower=-1.55,
            upper=1.55,
        ),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=primary_arm,
        child=secondary_arm,
        origin=Origin(xyz=(PRIMARY_ARM_L, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.4,
            lower=-2.45,
            upper=2.45,
        ),
    )
    model.articulation(
        "swivel_joint",
        ArticulationType.REVOLUTE,
        parent=secondary_arm,
        child=head_frame,
        origin=Origin(xyz=(SECONDARY_ARM_L, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.5,
            lower=-1.70,
            upper=1.70,
        ),
    )
    model.articulation(
        "tilt_joint",
        ArticulationType.REVOLUTE,
        parent=head_frame,
        child=tilt_crossbar,
        origin=Origin(xyz=(TILT_AXIS_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=-0.65,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    primary_arm = object_model.get_part("primary_arm")
    secondary_arm = object_model.get_part("secondary_arm")
    head_frame = object_model.get_part("head_frame")
    tilt_crossbar = object_model.get_part("tilt_crossbar")

    shoulder = object_model.get_articulation("shoulder_joint")
    elbow = object_model.get_articulation("elbow_joint")
    swivel = object_model.get_articulation("swivel_joint")
    tilt = object_model.get_articulation("tilt_joint")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        head_frame,
        tilt_crossbar,
        reason="tilt stage uses a captured trunnion axle passing through the head-frame yoke; this bare mount shows the pin fit explicitly",
    )

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

    for part_name in (
        "wall_plate",
        "primary_arm",
        "secondary_arm",
        "head_frame",
        "tilt_crossbar",
    ):
        ctx.check(f"{part_name}_present", object_model.get_part(part_name) is not None)

    ctx.expect_contact(primary_arm, wall_plate, contact_tol=0.002, name="shoulder_mount_contact")
    ctx.expect_contact(primary_arm, secondary_arm, contact_tol=0.002, name="elbow_mount_contact")
    ctx.expect_contact(secondary_arm, head_frame, contact_tol=0.002, name="swivel_mount_contact")
    ctx.expect_contact(head_frame, tilt_crossbar, contact_tol=0.002, name="tilt_mount_contact")

    ctx.check(
        "swing_axes_vertical",
        shoulder.axis == (0.0, 0.0, 1.0)
        and elbow.axis == (0.0, 0.0, 1.0)
        and swivel.axis == (0.0, 0.0, 1.0),
        details=(
            f"shoulder={shoulder.axis}, elbow={elbow.axis}, swivel={swivel.axis}; "
            "all three support stages should yaw about vertical axes"
        ),
    )
    ctx.check(
        "tilt_axis_horizontal",
        tilt.axis == (0.0, 1.0, 0.0),
        details=f"tilt axis should run left-right across the head frame, got {tilt.axis}",
    )

    elbow_rest = ctx.part_world_position(secondary_arm)
    with ctx.pose(shoulder_joint=0.65):
        elbow_swung = ctx.part_world_position(secondary_arm)
    ctx.check(
        "shoulder_positive_motion_swings_arm_sideways",
        elbow_rest is not None
        and elbow_swung is not None
        and elbow_swung[1] > elbow_rest[1] + 0.08,
        details=f"secondary arm origin should move toward +Y when shoulder opens: rest={elbow_rest}, swung={elbow_swung}",
    )

    head_rest = ctx.part_world_position(head_frame)
    with ctx.pose(elbow_joint=0.80):
        head_folded = ctx.part_world_position(head_frame)
    ctx.check(
        "elbow_positive_motion_folds_forearm",
        head_rest is not None
        and head_folded is not None
        and head_folded[1] > head_rest[1] + 0.08,
        details=f"head frame origin should move toward +Y when elbow folds: rest={head_rest}, folded={head_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

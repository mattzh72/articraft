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


PLATE_L = 0.220
PLATE_W = 0.120
PLATE_T = 0.006
PLATE_CORNER = 0.010

SLOT_L = 0.034
SLOT_W = 0.007

SHOULDER_X = -0.044
SHOULDER_Z = -0.036
BASE_EAR_T = 0.006
BASE_INNER_GAP = 0.030
BASE_EAR_OUTER = BASE_INNER_GAP / 2.0 + BASE_EAR_T / 2.0

LOWER_LEN = 0.126
LOWER_DROP = -0.020
LOWER_BARREL_R = 0.010
LOWER_BARREL_L = BASE_INNER_GAP
LOWER_RAIL_T = 0.006
LOWER_RAIL_H = 0.012
LOWER_RAIL_SPAN = 0.028
ELBOW_EAR_T = 0.006
ELBOW_INNER_GAP = 0.022
ELBOW_EAR_OUTER = ELBOW_INNER_GAP / 2.0 + ELBOW_EAR_T / 2.0

UPPER_LEN = 0.094
UPPER_DROP = -0.010
UPPER_BARREL_R = 0.0085
UPPER_BARREL_L = ELBOW_INNER_GAP
UPPER_RAIL_T = 0.0055
UPPER_RAIL_H = 0.010
UPPER_RAIL_SPAN = 0.023

PAN_PAD_R = 0.017
PAN_PAD_T = 0.008
PAN_FLANGE_R = 0.0155
PAN_FLANGE_T = 0.005

PITCH_X = 0.010
PITCH_Z = -0.028
PITCH_EAR_T = 0.005
PITCH_INNER_GAP = 0.038
PITCH_EAR_OUTER = PITCH_INNER_GAP / 2.0 + PITCH_EAR_T / 2.0
FRAME_TRUNNION_R = 0.0065

FRAME_W = 0.112
FRAME_H = 0.074
FRAME_INNER_W = 0.082
FRAME_INNER_H = 0.048
FRAME_T = 0.008
FRAME_CENTER_Z = -0.010


def _beam_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    width_y: float,
    height_z: float,
) -> cq.Workplane:
    dx = end[0] - start[0]
    dz = end[2] - start[2]
    length = math.hypot(dx, dz)
    angle_deg = math.degrees(math.atan2(-dz, dx))
    midpoint = (
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    )
    return (
        cq.Workplane("XY")
        .box(length, width_y, height_z)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle_deg)
        .translate(midpoint)
    )


def _plate_slab() -> cq.Workplane:
    slot_points = [
        (-0.062, -0.032),
        (-0.062, 0.032),
        (0.062, -0.032),
        (0.062, 0.032),
    ]
    return (
        cq.Workplane("XY")
        .box(PLATE_L, PLATE_W, PLATE_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(PLATE_CORNER)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(slot_points)
        .slot2D(SLOT_L, SLOT_W, angle=0)
        .cutThruAll()
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .center(-0.080, 0.0)
        .slot2D(0.026, 0.012, angle=90)
        .cutThruAll()
    )


def _mount_bracket() -> cq.Workplane:
    back_pad = (
        cq.Workplane("XY")
        .box(0.050, 0.050, 0.008, centered=(True, True, False))
        .translate((SHOULDER_X - 0.034, 0.0, -0.008))
    )
    drop_post = (
        cq.Workplane("XY")
        .box(0.018, 0.024, 0.024, centered=(False, True, False))
        .translate((SHOULDER_X - 0.022, 0.0, SHOULDER_Z - 0.012))
    )
    bracket = back_pad.union(drop_post)
    for y in (-BASE_EAR_OUTER, BASE_EAR_OUTER):
        ear = (
            cq.Workplane("XY")
            .box(0.014, BASE_EAR_T, 0.022, centered=(False, True, False))
            .translate((SHOULDER_X - 0.014, y, SHOULDER_Z - 0.011))
        )
        bracket = bracket.union(ear)
    return bracket


def _lower_arm() -> cq.Workplane:
    arm = (
        cq.Workplane("XY")
        .box(0.016, BASE_INNER_GAP, 0.020, centered=(False, True, False))
        .translate((0.0, 0.0, -0.010))
    )
    for y in (-0.010, 0.010):
        rail = _beam_between(
            (0.018, y, -0.005),
            (LOWER_LEN - 0.020, y, LOWER_DROP + 0.003),
            width_y=0.005,
            height_z=0.010,
        )
        arm = arm.union(rail)

    arm = arm.union(
        _beam_between(
            (0.024, 0.0, -0.010),
            (LOWER_LEN - 0.028, 0.0, LOWER_DROP - 0.004),
            width_y=0.012,
            height_z=0.006,
        )
    )
    arm = arm.union(
        cq.Workplane("XY")
        .box(0.016, 0.020, 0.012, centered=(False, True, False))
        .translate((LOWER_LEN - 0.022, 0.0, LOWER_DROP - 0.012))
    )

    for y in (-ELBOW_EAR_OUTER, ELBOW_EAR_OUTER):
        ear = (
            cq.Workplane("XY")
            .box(0.014, ELBOW_EAR_T, 0.022, centered=(False, True, False))
            .translate((LOWER_LEN - 0.014, y, LOWER_DROP - 0.011))
        )
        arm = arm.union(ear)
    return arm


def _upper_arm() -> cq.Workplane:
    arm = (
        cq.Workplane("XY")
        .box(0.014, ELBOW_INNER_GAP, 0.018, centered=(False, True, False))
        .translate((0.0, 0.0, -0.009))
    )
    for y in (-0.008, 0.008):
        rail = _beam_between(
            (0.018, y, -0.004),
            (UPPER_LEN - 0.028, y, UPPER_DROP - 0.002),
            width_y=0.0045,
            height_z=0.008,
        )
        arm = arm.union(rail)

    arm = arm.union(
        _beam_between(
            (0.022, 0.0, -0.008),
            (UPPER_LEN - 0.030, 0.0, UPPER_DROP - 0.004),
            width_y=0.010,
            height_z=0.006,
        )
    )
    arm = arm.union(
        cq.Workplane("XY")
        .box(0.012, 0.016, 0.012, centered=(False, True, False))
        .translate((UPPER_LEN - 0.022, 0.0, UPPER_DROP - 0.006))
    )
    arm = arm.union(
        cq.Workplane("XY")
        .box(0.018, 0.022, 0.006, centered=(False, True, False))
        .translate((UPPER_LEN - 0.018, 0.0, 0.0))
    )
    return arm


def _head_yoke() -> cq.Workplane:
    yoke = (
        cq.Workplane("XY")
        .box(0.018, 0.022, 0.008, centered=(False, True, False))
        .translate((0.0, 0.0, 0.0))
    )
    for y in (-0.014, 0.014):
        yoke = yoke.union(
            _beam_between(
                (0.010, y, 0.004),
                (PITCH_X - 0.006, y, PITCH_Z + 0.006),
                width_y=0.004,
                height_z=0.007,
            )
        )
    yoke = yoke.union(
        cq.Workplane("XY")
        .box(0.006, 0.010, 0.016, centered=(False, True, False))
        .translate((PITCH_X - 0.006, 0.0, PITCH_Z - 0.008))
    )
    for y in (-PITCH_EAR_OUTER, PITCH_EAR_OUTER):
        ear_plate = (
            cq.Workplane("XY")
            .box(0.010, PITCH_EAR_T, 0.018, centered=(False, True, False))
            .translate((PITCH_X - 0.010, y, PITCH_Z - 0.009))
        )
        yoke = yoke.union(ear_plate)
    return yoke


def _head_frame() -> cq.Workplane:
    ring = (
        cq.Workplane("XY")
        .box(FRAME_T, FRAME_W, FRAME_H)
        .translate((0.050, 0.0, FRAME_CENTER_Z))
        .cut(
            cq.Workplane("XY")
            .box(FRAME_T * 1.8, FRAME_INNER_W, FRAME_INNER_H)
            .translate((0.050, 0.0, FRAME_CENTER_Z))
        )
    )
    tongue = (
        cq.Workplane("XY")
        .box(0.006, 0.010, 0.016, centered=(False, True, False))
        .translate((0.0, 0.0, -0.008))
    )
    neck = _beam_between(
        (0.006, 0.0, -0.004),
        (0.034, 0.0, -0.004),
        width_y=0.012,
        height_z=0.008,
    )
    lower_tab = (
        cq.Workplane("XY")
        .box(0.010, 0.018, 0.010, centered=(False, True, False))
        .translate((0.028, 0.0, -0.024))
    )
    return ring.union(tongue).union(neck).union(lower_tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_shelf_monitor_mount")

    model.material("plate_gray", color=(0.22, 0.22, 0.24))
    model.material("arm_black", color=(0.10, 0.10, 0.11))
    model.material("pivot_black", color=(0.12, 0.12, 0.13))
    model.material("frame_dark", color=(0.14, 0.14, 0.15))

    top_plate = model.part("top_plate")
    top_plate.visual(
        mesh_from_cadquery(_plate_slab(), "top_plate_slab"),
        origin=Origin(),
        material="plate_gray",
        name="plate_slab",
    )
    top_plate.visual(
        mesh_from_cadquery(_mount_bracket(), "top_plate_bracket"),
        origin=Origin(),
        material="plate_gray",
        name="mount_bracket",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        mesh_from_cadquery(_lower_arm(), "lower_arm_shell"),
        origin=Origin(),
        material="arm_black",
        name="lower_arm_shell",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_upper_arm(), "upper_arm_shell"),
        origin=Origin(),
        material="arm_black",
        name="upper_arm_shell",
    )

    head_yoke = model.part("head_yoke")
    head_yoke.visual(
        mesh_from_cadquery(_head_yoke(), "head_yoke_shell"),
        origin=Origin(),
        material="pivot_black",
        name="head_yoke_shell",
    )

    head_frame = model.part("head_frame")
    head_frame.visual(
        mesh_from_cadquery(_head_frame(), "head_frame_shell"),
        origin=Origin(),
        material="frame_dark",
        name="frame_shell",
    )

    model.articulation(
        "plate_to_lower",
        ArticulationType.REVOLUTE,
        parent=top_plate,
        child=lower_arm,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.4,
            lower=-0.35,
            upper=2.15,
        ),
    )
    model.articulation(
        "lower_to_upper",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_LEN, 0.0, LOWER_DROP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.6,
            lower=-2.35,
            upper=0.75,
        ),
    )
    model.articulation(
        "upper_to_pan",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=head_yoke,
        origin=Origin(xyz=(UPPER_LEN, 0.0, UPPER_DROP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.2,
            lower=-2.7,
            upper=2.7,
        ),
    )
    model.articulation(
        "pan_to_head",
        ArticulationType.REVOLUTE,
        parent=head_yoke,
        child=head_frame,
        origin=Origin(xyz=(PITCH_X, 0.0, PITCH_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.5,
            velocity=2.0,
            lower=-1.25,
            upper=1.60,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_plate = object_model.get_part("top_plate")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    head_yoke = object_model.get_part("head_yoke")
    head_frame = object_model.get_part("head_frame")

    shoulder = object_model.get_articulation("plate_to_lower")
    elbow = object_model.get_articulation("lower_to_upper")
    pan = object_model.get_articulation("upper_to_pan")
    pitch = object_model.get_articulation("pan_to_head")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0025)
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
        all(
            part is not None
            for part in (top_plate, lower_arm, upper_arm, head_yoke, head_frame)
        ),
        "Expected all mount parts to be present.",
    )
    ctx.check(
        "joint_axes_match_monitor_mount",
        shoulder.axis == (0.0, 1.0, 0.0)
        and elbow.axis == (0.0, 1.0, 0.0)
        and pan.axis == (0.0, 0.0, 1.0)
        and pitch.axis == (0.0, 1.0, 0.0),
        "Expected two fold joints on Y, pan on Z, and pitch on Y.",
    )

    ctx.expect_contact(top_plate, lower_arm, name="shoulder_pivot_is_supported")
    ctx.expect_contact(lower_arm, upper_arm, name="elbow_pivot_is_supported")
    ctx.expect_contact(
        upper_arm,
        head_yoke,
        contact_tol=0.0025,
        name="pan_stack_stays_supported",
    )
    ctx.expect_contact(head_yoke, head_frame, name="pitch_hinge_stays_supported")

    ctx.expect_gap(
        top_plate,
        head_frame,
        axis="z",
        positive_elem="plate_slab",
        negative_elem="frame_shell",
        min_gap=0.020,
        name="head_hangs_below_plate_in_rest_pose",
    )

    with ctx.pose(
        {
            shoulder: 0.8,
            elbow: -2.3,
            pan: 2.5,
            pitch: 1.2,
        }
    ):
        ctx.expect_gap(
            top_plate,
            head_frame,
            axis="z",
            positive_elem="plate_slab",
            negative_elem="frame_shell",
            max_gap=0.012,
            max_penetration=0.0,
            name="folded_head_frame_stays_in_a_shallow_stack",
        )
        ctx.expect_overlap(
            top_plate,
            head_frame,
            axes="xy",
            elem_a="plate_slab",
            elem_b="frame_shell",
            min_overlap=0.035,
            name="folded_head_frame_tucks_under_plate_footprint",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

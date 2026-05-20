from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FRAME_OUTER_WIDTH = 0.90
FRAME_OUTER_HEIGHT = 0.70
FRAME_DEPTH = 0.06
FRAME_MEMBER = 0.03
STOP_THICKNESS = 0.012
STOP_DEPTH = 0.010

SASH_WIDTH = 0.82
SASH_HEIGHT = 0.62
SASH_DEPTH = 0.032
SASH_RAIL = 0.028
GLASS_THICKNESS = 0.004

HINGE_AXIS_Y = 0.014
HINGE_AXIS_Z = 0.314
HINGE_RADIUS = 0.007

STAY_ROOT_X = 0.406
FRAME_PIN_X = 0.417
SASH_PIN_X = 0.373
STAY_PIN_RADIUS = 0.0055
FRAME_STAY_PIVOT_Y = -0.014
FRAME_STAY_PIVOT_Z = 0.118
SASH_PIN_LOCAL_Y = -0.016
SASH_PIN_LOCAL_Z = -0.100
LOWER_ARM_LENGTH = 0.062
UPPER_ARM_LENGTH = 0.070

VENT_OPEN_ANGLE = 0.60


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _rotate_yz_about_x(y: float, z: float, angle: float) -> tuple[float, float]:
    return (y * cos(angle) - z * sin(angle), y * sin(angle) + z * cos(angle))


def _sash_pin_yz(sash_angle: float) -> tuple[float, float]:
    dy, dz = _rotate_yz_about_x(SASH_PIN_LOCAL_Y, SASH_PIN_LOCAL_Z, sash_angle)
    return (HINGE_AXIS_Y + dy, HINGE_AXIS_Z + dz)


def _solve_stay_angles(sash_angle: float) -> tuple[float, float]:
    ay = FRAME_STAY_PIVOT_Y
    az = FRAME_STAY_PIVOT_Z
    cy, cz = _sash_pin_yz(sash_angle)

    dy = cy - ay
    dz = cz - az
    distance = sqrt(dy * dy + dz * dz)
    if distance <= 1e-9:
        return (0.0, 0.0)

    a = (
        LOWER_ARM_LENGTH * LOWER_ARM_LENGTH
        - UPPER_ARM_LENGTH * UPPER_ARM_LENGTH
        + distance * distance
    ) / (2.0 * distance)
    h_sq = LOWER_ARM_LENGTH * LOWER_ARM_LENGTH - a * a
    h = sqrt(max(h_sq, 0.0))

    uy = dy / distance
    uz = dz / distance
    mid_y = ay + a * uy
    mid_z = az + a * uz
    perp_y = -uz
    perp_z = uy

    candidate_a = (mid_y + h * perp_y, mid_z + h * perp_z)
    candidate_b = (mid_y - h * perp_y, mid_z - h * perp_z)
    elbow_y, elbow_z = candidate_a if candidate_a[0] < candidate_b[0] else candidate_b

    lower_abs = atan2(-(elbow_y - ay), elbow_z - az)
    upper_abs = atan2(-(cy - elbow_y), cz - elbow_z)
    return (lower_abs, upper_abs - lower_abs)


CLOSED_LOWER_ANGLE, CLOSED_UPPER_REL = _solve_stay_angles(0.0)
OPEN_LOWER_ANGLE, OPEN_UPPER_REL = _solve_stay_angles(VENT_OPEN_ANGLE)
OPEN_LOWER_DELTA = OPEN_LOWER_ANGLE - CLOSED_LOWER_ANGLE
OPEN_UPPER_DELTA = OPEN_UPPER_REL - CLOSED_UPPER_REL


def _add_hinge_knuckles(part, *, on_sash: bool) -> None:
    y = 0.009 if on_sash else 0.037
    z = 0.0 if on_sash else HINGE_AXIS_Z
    segments = (
        [(-0.170, 0.130), (0.170, 0.130)]
        if on_sash
        else [(-0.285, 0.100), (0.0, 0.130), (0.285, 0.100)]
    )
    for index, (x, length) in enumerate(segments, start=1):
        part.visual(
            Cylinder(radius=HINGE_RADIUS, length=length),
            origin=Origin(xyz=(x, y, z), rpy=(0.0, 1.57079632679, 0.0)),
            material="hardware_steel",
            name=f"hinge_knuckle_{index}",
        )


def _add_stay_brackets(frame, sash, side: float) -> None:
    frame_pin_x = side * FRAME_PIN_X
    sash_pin_x = side * SASH_PIN_X
    frame_name = "left" if side < 0.0 else "right"

    frame.visual(
        Box((0.016, 0.004, 0.050)),
        origin=Origin(xyz=(side * 0.414, -0.018, FRAME_STAY_PIVOT_Z)),
        material="hardware_steel",
        name=f"{frame_name}_frame_bracket",
    )
    frame.visual(
        Box((0.003, 0.010, 0.024)),
        origin=Origin(
            xyz=(frame_pin_x - side * 0.0015, -0.014, FRAME_STAY_PIVOT_Z),
        ),
        material="hardware_steel",
        name=f"{frame_name}_frame_clip_outer",
    )
    frame.visual(
        Box((0.003, 0.010, 0.024)),
        origin=Origin(
            xyz=(frame_pin_x - side * 0.0125, -0.014, FRAME_STAY_PIVOT_Z),
        ),
        material="hardware_steel",
        name=f"{frame_name}_frame_clip_inner",
    )
    frame.visual(
        Cylinder(radius=STAY_PIN_RADIUS, length=0.006),
        origin=Origin(
            xyz=(frame_pin_x, FRAME_STAY_PIVOT_Y, FRAME_STAY_PIVOT_Z),
            rpy=(0.0, 1.57079632679, 0.0),
        ),
        material="hardware_steel",
        name=f"{frame_name}_frame_pin",
    )

    sash.visual(
        Box((0.018, 0.004, 0.044)),
        origin=Origin(xyz=(side * 0.379, -0.018, -0.100)),
        material="hardware_steel",
        name=f"{frame_name}_sash_bracket",
    )
    sash.visual(
        Box((0.003, 0.010, 0.024)),
        origin=Origin(
            xyz=(sash_pin_x + side * 0.0135, -0.014, -0.100),
        ),
        material="hardware_steel",
        name=f"{frame_name}_sash_clip_outer",
    )
    sash.visual(
        Box((0.003, 0.010, 0.024)),
        origin=Origin(
            xyz=(sash_pin_x + side * 0.0025, -0.014, -0.100),
        ),
        material="hardware_steel",
        name=f"{frame_name}_sash_clip_inner",
    )
    sash.visual(
        Cylinder(radius=STAY_PIN_RADIUS, length=0.008),
        origin=Origin(
            xyz=(sash_pin_x, SASH_PIN_LOCAL_Y, SASH_PIN_LOCAL_Z),
            rpy=(0.0, 1.57079632679, 0.0),
        ),
        material="hardware_steel",
        name=f"{frame_name}_sash_pin",
    )


def _build_lower_stay(model: ArticulatedObject, name: str, side: float):
    stay = model.part(name)
    stay.inertial = Inertial.from_geometry(
        Box((0.020, 0.012, LOWER_ARM_LENGTH)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, LOWER_ARM_LENGTH * 0.5)),
    )
    stay.visual(
        Box((0.008, 0.004, LOWER_ARM_LENGTH - 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * LOWER_ARM_LENGTH)),
        material="stay_steel",
        name="arm_body",
    )
    stay.visual(
        Box((0.008, 0.004, 0.012)),
        origin=Origin(xyz=(side * 0.002, 0.0, 0.006)),
        material="stay_steel",
        name="root_tab",
    )
    stay.visual(
        Box((0.008, 0.004, 0.012)),
        origin=Origin(xyz=(-side * 0.002, 0.0, LOWER_ARM_LENGTH - 0.006)),
        material="stay_steel",
        name="elbow_tab",
    )
    stay.visual(
        Cylinder(radius=STAY_PIN_RADIUS, length=0.008),
        origin=Origin(
            xyz=(side * 0.004, 0.0, 0.0),
            rpy=(0.0, 1.57079632679, 0.0),
        ),
        material="stay_steel",
        name="root_pin",
    )
    return stay


def _build_upper_stay(model: ArticulatedObject, name: str, side: float):
    stay = model.part(name)
    stay.inertial = Inertial.from_geometry(
        Box((0.028, 0.012, UPPER_ARM_LENGTH)),
        mass=0.16,
        origin=Origin(xyz=(-side * 0.012, 0.0, UPPER_ARM_LENGTH * 0.5)),
    )
    stay.visual(
        Box((0.008, 0.004, UPPER_ARM_LENGTH - 0.024)),
        origin=Origin(xyz=(-side * 0.012, 0.0, 0.5 * UPPER_ARM_LENGTH)),
        material="stay_steel",
        name="arm_body",
    )
    stay.visual(
        Box((0.020, 0.004, 0.012)),
        origin=Origin(xyz=(-side * 0.004, 0.0, 0.006)),
        material="stay_steel",
        name="root_tab",
    )
    stay.visual(
        Box((0.020, 0.004, 0.012)),
        origin=Origin(xyz=(-side * 0.018, 0.0, UPPER_ARM_LENGTH - 0.006)),
        material="stay_steel",
        name="tip_tab",
    )
    stay.visual(
        Cylinder(radius=STAY_PIN_RADIUS, length=0.008),
        origin=Origin(
            xyz=(side * 0.004, 0.0, 0.0),
            rpy=(0.0, 1.57079632679, 0.0),
        ),
        material="stay_steel",
        name="root_pin",
    )
    stay.visual(
        Cylinder(radius=STAY_PIN_RADIUS, length=0.008),
        origin=Origin(
            xyz=(-side * 0.025, 0.0, UPPER_ARM_LENGTH),
            rpy=(0.0, 1.57079632679, 0.0),
        ),
        material="stay_steel",
        name="tip_pin",
    )
    return stay


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="greenhouse_vent_window")

    model.material("frame_paint", rgba=(0.86, 0.88, 0.90, 1.0))
    model.material("sash_paint", rgba=(0.79, 0.82, 0.85, 1.0))
    model.material("hardware_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("stay_steel", rgba=(0.50, 0.52, 0.55, 1.0))
    model.material("glass_tint", rgba=(0.72, 0.88, 0.93, 0.35))
    model.material("gasket_dark", rgba=(0.16, 0.17, 0.18, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_OUTER_WIDTH, FRAME_DEPTH, FRAME_OUTER_HEIGHT)),
        mass=8.5,
    )
    frame.visual(
        Box((FRAME_MEMBER, FRAME_DEPTH, FRAME_OUTER_HEIGHT)),
        origin=Origin(xyz=(-0.5 * FRAME_OUTER_WIDTH + 0.5 * FRAME_MEMBER, 0.0, 0.0)),
        material="frame_paint",
        name="left_jamb",
    )
    frame.visual(
        Box((FRAME_MEMBER, FRAME_DEPTH, FRAME_OUTER_HEIGHT)),
        origin=Origin(xyz=(0.5 * FRAME_OUTER_WIDTH - 0.5 * FRAME_MEMBER, 0.0, 0.0)),
        material="frame_paint",
        name="right_jamb",
    )
    frame.visual(
        Box((FRAME_OUTER_WIDTH - 2.0 * FRAME_MEMBER, FRAME_DEPTH, FRAME_MEMBER)),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * FRAME_OUTER_HEIGHT - 0.5 * FRAME_MEMBER)),
        material="frame_paint",
        name="header",
    )
    frame.visual(
        Box((FRAME_OUTER_WIDTH - 2.0 * FRAME_MEMBER, FRAME_DEPTH, FRAME_MEMBER)),
        origin=Origin(xyz=(0.0, 0.0, -0.5 * FRAME_OUTER_HEIGHT + 0.5 * FRAME_MEMBER)),
        material="frame_paint",
        name="sill",
    )
    side_stop_x = 0.5 * SASH_WIDTH + 0.5 * (FRAME_MEMBER - SASH_RAIL) - 0.004
    lower_stop_top = 0.075
    upper_stop_bottom = 0.165
    lower_stop_height = lower_stop_top - (-0.5 * FRAME_OUTER_HEIGHT + FRAME_MEMBER)
    upper_stop_height = (0.5 * FRAME_OUTER_HEIGHT - FRAME_MEMBER) - upper_stop_bottom
    stop_y = -0.5 * FRAME_DEPTH + 0.5 * STOP_DEPTH + 0.006
    frame.visual(
        Box((STOP_THICKNESS, STOP_DEPTH, lower_stop_height)),
        origin=Origin(
            xyz=(
                -side_stop_x,
                stop_y,
                -0.5 * FRAME_OUTER_HEIGHT + FRAME_MEMBER + 0.5 * lower_stop_height,
            )
        ),
        material="gasket_dark",
        name="left_stop_lower",
    )
    frame.visual(
        Box((STOP_THICKNESS, STOP_DEPTH, upper_stop_height)),
        origin=Origin(
            xyz=(
                -side_stop_x,
                stop_y,
                upper_stop_bottom + 0.5 * upper_stop_height,
            )
        ),
        material="gasket_dark",
        name="left_stop_upper",
    )
    frame.visual(
        Box((STOP_THICKNESS, STOP_DEPTH, lower_stop_height)),
        origin=Origin(
            xyz=(
                side_stop_x,
                stop_y,
                -0.5 * FRAME_OUTER_HEIGHT + FRAME_MEMBER + 0.5 * lower_stop_height,
            )
        ),
        material="gasket_dark",
        name="right_stop_lower",
    )
    frame.visual(
        Box((STOP_THICKNESS, STOP_DEPTH, upper_stop_height)),
        origin=Origin(
            xyz=(
                side_stop_x,
                stop_y,
                upper_stop_bottom + 0.5 * upper_stop_height,
            )
        ),
        material="gasket_dark",
        name="right_stop_upper",
    )
    frame.visual(
        Box((SASH_WIDTH, STOP_DEPTH, STOP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -0.5 * FRAME_DEPTH + 0.5 * STOP_DEPTH + 0.006,
                0.5 * SASH_HEIGHT + 0.5 * STOP_THICKNESS - 0.001,
            )
        ),
        material="gasket_dark",
        name="head_stop",
    )
    frame.visual(
        Box((SASH_WIDTH, STOP_DEPTH, STOP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -0.5 * FRAME_DEPTH + 0.5 * STOP_DEPTH + 0.006,
                -0.5 * SASH_HEIGHT - 0.5 * STOP_THICKNESS + 0.001,
            )
        ),
        material="gasket_dark",
        name="sill_stop",
    )
    frame.visual(
        Box((0.100, 0.014, 0.020)),
        origin=Origin(xyz=(-0.285, 0.036, HINGE_AXIS_Z + 0.006)),
        material="hardware_steel",
        name="left_hinge_mount",
    )
    frame.visual(
        Box((0.130, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, 0.036, HINGE_AXIS_Z + 0.006)),
        material="hardware_steel",
        name="center_hinge_mount",
    )
    frame.visual(
        Box((0.100, 0.014, 0.020)),
        origin=Origin(xyz=(0.285, 0.036, HINGE_AXIS_Z + 0.006)),
        material="hardware_steel",
        name="right_hinge_mount",
    )
    _add_hinge_knuckles(frame, on_sash=False)

    sash = model.part("sash")
    sash.inertial = Inertial.from_geometry(
        Box((SASH_WIDTH, SASH_DEPTH, SASH_HEIGHT)),
        mass=5.2,
        origin=Origin(xyz=(0.0, 0.0, -0.5 * SASH_HEIGHT)),
    )
    sash.visual(
        Box((SASH_RAIL, SASH_DEPTH, SASH_HEIGHT)),
        origin=Origin(xyz=(-0.5 * SASH_WIDTH + 0.5 * SASH_RAIL, 0.0, -0.5 * SASH_HEIGHT)),
        material="sash_paint",
        name="left_stile",
    )
    sash.visual(
        Box((SASH_RAIL, SASH_DEPTH, SASH_HEIGHT)),
        origin=Origin(xyz=(0.5 * SASH_WIDTH - 0.5 * SASH_RAIL, 0.0, -0.5 * SASH_HEIGHT)),
        material="sash_paint",
        name="right_stile",
    )
    sash.visual(
        Box((SASH_WIDTH, SASH_DEPTH, SASH_RAIL)),
        origin=Origin(xyz=(0.0, 0.0, -0.5 * SASH_RAIL)),
        material="sash_paint",
        name="top_rail",
    )
    sash.visual(
        Box((SASH_WIDTH, SASH_DEPTH, SASH_RAIL)),
        origin=Origin(xyz=(0.0, 0.0, -SASH_HEIGHT + 0.5 * SASH_RAIL)),
        material="sash_paint",
        name="bottom_rail",
    )
    sash.visual(
        Box(
            (
                SASH_WIDTH - 2.0 * SASH_RAIL,
                GLASS_THICKNESS,
                SASH_HEIGHT - 2.0 * SASH_RAIL,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                -0.007,
                -0.5 * SASH_HEIGHT,
            )
        ),
        material="glass_tint",
        name="glass",
    )
    bead_width = 0.012
    sash.visual(
        Box((bead_width, 0.008, SASH_HEIGHT - 2.0 * SASH_RAIL + 0.010)),
        origin=Origin(
            xyz=(-0.5 * SASH_WIDTH + SASH_RAIL - 0.5 * bead_width, -0.011, -0.5 * SASH_HEIGHT),
        ),
        material="gasket_dark",
        name="left_glazing_bead",
    )
    sash.visual(
        Box((bead_width, 0.008, SASH_HEIGHT - 2.0 * SASH_RAIL + 0.010)),
        origin=Origin(
            xyz=(0.5 * SASH_WIDTH - SASH_RAIL + 0.5 * bead_width, -0.011, -0.5 * SASH_HEIGHT),
        ),
        material="gasket_dark",
        name="right_glazing_bead",
    )
    sash.visual(
        Box((SASH_WIDTH - 2.0 * SASH_RAIL + 0.010, 0.008, bead_width)),
        origin=Origin(xyz=(0.0, -0.011, -SASH_RAIL + 0.5 * bead_width)),
        material="gasket_dark",
        name="top_glazing_bead",
    )
    sash.visual(
        Box((SASH_WIDTH - 2.0 * SASH_RAIL + 0.010, 0.008, bead_width)),
        origin=Origin(xyz=(0.0, -0.011, -SASH_HEIGHT + SASH_RAIL - 0.5 * bead_width)),
        material="gasket_dark",
        name="bottom_glazing_bead",
    )
    _add_hinge_knuckles(sash, on_sash=True)
    _add_stay_brackets(frame, sash, -1.0)
    _add_stay_brackets(frame, sash, 1.0)

    left_lower = _build_lower_stay(model, "left_lower_stay", -1.0)
    left_upper = _build_upper_stay(model, "left_upper_stay", -1.0)
    right_lower = _build_lower_stay(model, "right_lower_stay", 1.0)
    right_upper = _build_upper_stay(model, "right_upper_stay", 1.0)

    model.articulation(
        "vent_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.0, lower=0.0, upper=1.05),
    )
    model.articulation(
        "left_lower_stay_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_lower,
        origin=Origin(
            xyz=(-STAY_ROOT_X, FRAME_STAY_PIVOT_Y, FRAME_STAY_PIVOT_Z),
            rpy=(CLOSED_LOWER_ANGLE, 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=-0.80, upper=0.80),
    )
    model.articulation(
        "left_upper_stay_pivot",
        ArticulationType.REVOLUTE,
        parent=left_lower,
        child=left_upper,
        origin=Origin(
            xyz=(0.0, 0.0, LOWER_ARM_LENGTH),
            rpy=(CLOSED_UPPER_REL, 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=-1.10, upper=1.10),
    )
    model.articulation(
        "right_lower_stay_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_lower,
        origin=Origin(
            xyz=(STAY_ROOT_X, FRAME_STAY_PIVOT_Y, FRAME_STAY_PIVOT_Z),
            rpy=(CLOSED_LOWER_ANGLE, 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=-0.80, upper=0.80),
    )
    model.articulation(
        "right_upper_stay_pivot",
        ArticulationType.REVOLUTE,
        parent=right_lower,
        child=right_upper,
        origin=Origin(
            xyz=(0.0, 0.0, LOWER_ARM_LENGTH),
            rpy=(CLOSED_UPPER_REL, 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=-1.10, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    left_lower = object_model.get_part("left_lower_stay")
    left_upper = object_model.get_part("left_upper_stay")
    right_lower = object_model.get_part("right_lower_stay")
    right_upper = object_model.get_part("right_upper_stay")
    vent = object_model.get_articulation("vent_hinge")
    left_lower_joint = object_model.get_articulation("left_lower_stay_pivot")
    left_upper_joint = object_model.get_articulation("left_upper_stay_pivot")
    right_lower_joint = object_model.get_articulation("right_lower_stay_pivot")
    right_upper_joint = object_model.get_articulation("right_upper_stay_pivot")

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
        "hinge_and_stay_axes_are_horizontal",
        vent.axis == (1.0, 0.0, 0.0)
        and left_lower_joint.axis == (1.0, 0.0, 0.0)
        and left_upper_joint.axis == (1.0, 0.0, 0.0)
        and right_lower_joint.axis == (1.0, 0.0, 0.0)
        and right_upper_joint.axis == (1.0, 0.0, 0.0),
        "The vent sash and both side stays should all rotate about horizontal X axes.",
    )
    ctx.check(
        "vent_opens_outward_only",
        vent.motion_limits is not None
        and vent.motion_limits.lower == 0.0
        and vent.motion_limits.upper is not None
        and vent.motion_limits.upper >= 0.9,
        "The primary vent hinge should start closed and open outward through a realistic range.",
    )

    with ctx.pose(
        vent_hinge=0.0,
        left_lower_stay_pivot=0.0,
        left_upper_stay_pivot=0.0,
        right_lower_stay_pivot=0.0,
        right_upper_stay_pivot=0.0,
    ):
        ctx.expect_overlap(
            sash,
            frame,
            axes="xz",
            min_overlap=0.60,
            name="closed_sash_fills_frame_projection",
        )
        ctx.expect_origin_gap(
            sash,
            frame,
            axis="z",
            min_gap=0.30,
            max_gap=0.33,
            name="hinge_axis_sits_near_the_header",
        )
        ctx.expect_contact(
            left_lower,
            frame,
            elem_a="root_pin",
            elem_b="left_frame_pin",
            name="left_lower_stay_is_pinned_to_frame",
        )
        ctx.expect_contact(
            right_lower,
            frame,
            elem_a="root_pin",
            elem_b="right_frame_pin",
            name="right_lower_stay_is_pinned_to_frame",
        )
        ctx.expect_contact(
            left_upper,
            sash,
            elem_a="tip_pin",
            elem_b="left_sash_pin",
            name="left_upper_stay_is_clipped_to_sash_closed",
        )
        ctx.expect_contact(
            right_upper,
            sash,
            elem_a="tip_pin",
            elem_b="right_sash_pin",
            name="right_upper_stay_is_clipped_to_sash_closed",
        )

    with ctx.pose(
        vent_hinge=VENT_OPEN_ANGLE,
        left_lower_stay_pivot=OPEN_LOWER_DELTA,
        left_upper_stay_pivot=OPEN_UPPER_DELTA,
        right_lower_stay_pivot=OPEN_LOWER_DELTA,
        right_upper_stay_pivot=OPEN_UPPER_DELTA,
    ):
        ctx.expect_gap(
            sash,
            frame,
            axis="y",
            positive_elem="bottom_rail",
            negative_elem="sill",
            min_gap=0.22,
            name="bottom_rail_swings_clear_of_frame_when_open",
        )
        ctx.expect_contact(
            left_upper,
            sash,
            elem_a="tip_pin",
            elem_b="left_sash_pin",
            name="left_upper_stay_remains_clipped_open",
        )
        ctx.expect_contact(
            right_upper,
            sash,
            elem_a="tip_pin",
            elem_b="right_sash_pin",
            name="right_upper_stay_remains_clipped_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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


OUTER_WIDTH = 0.34
OUTER_HEIGHT = 0.24
FRAME_DEPTH = 0.045
FRONT_LIP_DEPTH = 0.012
OUTER_STEP = 0.016
FRAME_BORDER = 0.035
INNER_WIDTH = OUTER_WIDTH - 2.0 * FRAME_BORDER
INNER_HEIGHT = OUTER_HEIGHT - 2.0 * FRAME_BORDER
FRONT_REVEAL_EXTRA = 0.020

BLADE_COUNT = 4
BLADE_DEPTH = 0.026
BLADE_THICKNESS = 0.008
PIVOT_PIN_RADIUS = 0.0055
PIVOT_HOLE_RADIUS = 0.0062
PIVOT_PIN_LENGTH = 0.014
PIVOT_COLLAR_RADIUS = 0.008
PIVOT_COLLAR_THICKNESS = 0.004
BLADE_END_CLEAR = 0.018
TRUNNION_ARM_RADIUS = 0.0044
TRUNNION_ARM_LENGTH = BLADE_END_CLEAR - PIVOT_COLLAR_THICKNESS
BLADE_BODY_LENGTH = INNER_WIDTH - 2.0 * BLADE_END_CLEAR
BLADE_REST_ANGLE = 0.48
BLADE_SWING = 0.32


def blade_names() -> list[str]:
    return [f"blade_{index + 1}" for index in range(BLADE_COUNT)]


def blade_joint_names() -> list[str]:
    return [f"frame_to_blade_{index + 1}" for index in range(BLADE_COUNT)]


def blade_axis_z_positions() -> list[float]:
    pitch = INNER_HEIGHT / BLADE_COUNT
    bottom = -INNER_HEIGHT / 2.0 + pitch / 2.0
    return [bottom + index * pitch for index in range(BLADE_COUNT)]


def make_frame_shape() -> cq.Workplane:
    front_trim = cq.Workplane("XY").box(
        OUTER_WIDTH,
        FRONT_LIP_DEPTH,
        OUTER_HEIGHT,
    ).translate((0.0, -FRAME_DEPTH / 2.0 + FRONT_LIP_DEPTH / 2.0, 0.0))

    rear_body = cq.Workplane("XY").box(
        OUTER_WIDTH - OUTER_STEP,
        FRAME_DEPTH - FRONT_LIP_DEPTH,
        OUTER_HEIGHT - OUTER_STEP,
    ).translate((0.0, FRONT_LIP_DEPTH / 2.0, 0.0))

    frame = front_trim.union(rear_body)

    through_opening = cq.Workplane("XY").box(
        INNER_WIDTH,
        FRAME_DEPTH + 0.004,
        INNER_HEIGHT,
    )
    front_reveal = cq.Workplane("XY").box(
        INNER_WIDTH + FRONT_REVEAL_EXTRA,
        FRONT_LIP_DEPTH + 0.003,
        INNER_HEIGHT + FRONT_REVEAL_EXTRA,
    ).translate((0.0, -FRAME_DEPTH / 2.0 + FRONT_LIP_DEPTH / 2.0, 0.0))

    frame = frame.cut(through_opening).cut(front_reveal)

    left_hole_start = -OUTER_WIDTH / 2.0 - 0.001
    right_hole_start = INNER_WIDTH / 2.0 - 0.001
    hole_length = FRAME_BORDER + 0.002

    for z_pos in blade_axis_z_positions():
        left_hole = (
            cq.Workplane("YZ")
            .center(0.0, z_pos)
            .circle(PIVOT_HOLE_RADIUS)
            .extrude(hole_length)
            .translate((left_hole_start, 0.0, 0.0))
        )
        right_hole = (
            cq.Workplane("YZ")
            .center(0.0, z_pos)
            .circle(PIVOT_HOLE_RADIUS)
            .extrude(hole_length)
            .translate((right_hole_start, 0.0, 0.0))
        )
        frame = frame.cut(left_hole).cut(right_hole)

    return frame


def make_blade_shape() -> cq.Workplane:
    body = (
        cq.Workplane("YZ")
        .ellipse(BLADE_DEPTH / 2.0, BLADE_THICKNESS / 2.0)
        .extrude(BLADE_BODY_LENGTH)
        .translate((BLADE_END_CLEAR, 0.0, 0.0))
    )

    left_collar = (
        cq.Workplane("YZ")
        .circle(PIVOT_COLLAR_RADIUS)
        .extrude(PIVOT_COLLAR_THICKNESS)
    )
    right_collar = (
        cq.Workplane("YZ")
        .circle(PIVOT_COLLAR_RADIUS)
        .extrude(PIVOT_COLLAR_THICKNESS)
        .translate((INNER_WIDTH - PIVOT_COLLAR_THICKNESS, 0.0, 0.0))
    )

    left_arm = (
        cq.Workplane("YZ")
        .circle(TRUNNION_ARM_RADIUS)
        .extrude(TRUNNION_ARM_LENGTH)
        .translate((PIVOT_COLLAR_THICKNESS, 0.0, 0.0))
    )
    right_arm = (
        cq.Workplane("YZ")
        .circle(TRUNNION_ARM_RADIUS)
        .extrude(TRUNNION_ARM_LENGTH)
        .translate((INNER_WIDTH - BLADE_END_CLEAR, 0.0, 0.0))
    )

    left_pin = (
        cq.Workplane("YZ")
        .circle(PIVOT_PIN_RADIUS)
        .extrude(PIVOT_PIN_LENGTH)
        .translate((-PIVOT_PIN_LENGTH, 0.0, 0.0))
    )
    right_pin = (
        cq.Workplane("YZ")
        .circle(PIVOT_PIN_RADIUS)
        .extrude(PIVOT_PIN_LENGTH)
        .translate((INNER_WIDTH, 0.0, 0.0))
    )

    return (
        body.union(left_collar)
        .union(right_collar)
        .union(left_arm)
        .union(right_arm)
        .union(left_pin)
        .union(right_pin)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_vent")

    frame_material = model.material("frame_finish", rgba=(0.86, 0.87, 0.88, 1.0))
    blade_material = model.material("blade_finish", rgba=(0.59, 0.62, 0.66, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(make_frame_shape(), "frame_shell"),
        material=frame_material,
        name="frame_shell",
    )

    for index, z_pos in enumerate(blade_axis_z_positions(), start=1):
        blade = model.part(f"blade_{index}")
        blade.visual(
            mesh_from_cadquery(make_blade_shape(), f"blade_{index}_shell"),
            origin=Origin(rpy=(BLADE_REST_ANGLE, 0.0, 0.0)),
            material=blade_material,
            name="blade_shell",
        )

        model.articulation(
            f"frame_to_blade_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=blade,
            origin=Origin(xyz=(-INNER_WIDTH / 2.0, 0.0, z_pos)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=2.0,
                lower=-BLADE_SWING,
                upper=BLADE_SWING,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    blades = [object_model.get_part(name) for name in blade_names()]
    blade_joints = [object_model.get_articulation(name) for name in blade_joint_names()]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    for blade in blades:
        ctx.allow_overlap(
            blade,
            frame,
            reason=(
                "Each louver is carried by integral end trunnions seated in the "
                "frame journals; the parent-child pivot interface intentionally "
                "shares the bearing volume at those supports."
            ),
        )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    for blade, joint in zip(blades, blade_joints):
        ctx.expect_contact(blade, frame, name=f"{blade.name}_mounted_to_frame")
        ctx.expect_within(
            blade,
            frame,
            axes="xz",
            margin=0.0,
            name=f"{blade.name}_stays_within_frame_envelope",
        )
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_axis_is_longitudinal",
            tuple(round(value, 6) for value in joint.axis) == (1.0, 0.0, 0.0),
            f"expected axis (1, 0, 0), got {joint.axis}",
        )
        ctx.check(
            f"{joint.name}_has_bidirectional_range",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper,
            f"expected lower < 0 < upper, got {limits}",
        )

    for lower_blade, upper_blade in zip(blades[:-1], blades[1:]):
        ctx.expect_gap(
            upper_blade,
            lower_blade,
            axis="z",
            min_gap=0.015,
            name=f"{lower_blade.name}_to_{upper_blade.name}_rest_spacing",
        )

    middle_blade = blades[1]
    middle_joint = blade_joints[1]
    middle_limits = middle_joint.motion_limits

    with ctx.pose({middle_joint: middle_limits.upper}):
        ctx.expect_contact(
            middle_blade,
            frame,
            name="middle_blade_stays_supported_at_upper_limit",
        )
        ctx.expect_gap(
            blades[1],
            blades[0],
            axis="z",
            min_gap=0.008,
            name="middle_blade_keeps_lower_clearance_opened",
        )
        ctx.expect_gap(
            blades[2],
            blades[1],
            axis="z",
            min_gap=0.008,
            name="middle_blade_keeps_upper_clearance_opened",
        )

    with ctx.pose({middle_joint: middle_limits.lower}):
        ctx.expect_contact(
            middle_blade,
            frame,
            name="middle_blade_stays_supported_at_lower_limit",
        )
        ctx.expect_gap(
            blades[1],
            blades[0],
            axis="z",
            min_gap=0.008,
            name="middle_blade_keeps_lower_clearance_closed",
        )
        ctx.expect_gap(
            blades[2],
            blades[1],
            axis="z",
            min_gap=0.008,
            name="middle_blade_keeps_upper_clearance_closed",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

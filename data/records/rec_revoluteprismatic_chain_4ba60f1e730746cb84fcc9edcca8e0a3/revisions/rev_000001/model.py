from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import radians, sqrt

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


SWING_UPPER = radians(100.0)
SLIDE_TRAVEL = 0.070
CARRIAGE_X_OFFSET = 0.055
CARRIAGE_Z_OFFSET = 0.018


def _base_bracket_body_shape() -> cq.Workplane:
    wall_plate = cq.Workplane("XY").box(0.014, 0.090, 0.130).translate((-0.078, -0.060, 0.0))
    upper_strut = cq.Workplane("XY").box(0.078, 0.012, 0.012).translate((-0.039, -0.028, 0.026))
    lower_strut = cq.Workplane("XY").box(0.078, 0.012, 0.012).translate((-0.039, -0.028, -0.026))
    upper_anchor = cq.Workplane("XY").circle(0.024).extrude(0.009).translate((0.0, 0.0, 0.016))
    lower_anchor = cq.Workplane("XY").circle(0.024).extrude(0.009).translate((0.0, 0.0, -0.025))
    base_foot = cq.Workplane("XY").box(0.040, 0.040, 0.016).translate((-0.065, -0.070, -0.057))

    return (
        wall_plate.union(upper_strut)
        .union(lower_strut)
        .union(upper_anchor)
        .union(lower_anchor)
        .union(base_foot)
        .edges("|X")
        .fillet(0.003)
    )


def _base_pivot_pin_shape() -> cq.Workplane:
    return cq.Workplane("XY").circle(0.018).extrude(0.050).translate((0.0, 0.0, -0.025))


def _primary_arm_shape() -> cq.Workplane:
    hub = (
        cq.Workplane("XY")
        .circle(0.032)
        .circle(0.018)
        .extrude(0.030)
        .translate((0.0, 0.0, -0.015))
    )
    arm_bar = cq.Workplane("XY").box(0.188, 0.045, 0.026).translate((0.126, 0.0, 0.0))
    arm_bar = arm_bar.edges("|Z").fillet(0.004)
    nose = cq.Workplane("XY").box(0.038, 0.052, 0.026).translate((0.239, 0.0, 0.0))
    return hub.union(arm_bar).union(nose)


def _slide_rail_shape() -> cq.Workplane:
    rail = cq.Workplane("XY").box(0.160, 0.032, 0.018).translate((0.080, 0.0, 0.009))
    rail = rail.edges("|X").fillet(0.002)
    return rail


def _carriage_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.070, 0.060, 0.028)
        .translate((0.0, 0.0, 0.014))
        .edges("|Z")
        .fillet(0.003)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_out_slide_mechanism")

    bracket_gray = model.material("bracket_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    arm_steel = model.material("arm_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    rail_black = model.material("rail_black", rgba=(0.10, 0.10, 0.12, 1.0))
    carriage_orange = model.material("carriage_orange", rgba=(0.86, 0.47, 0.14, 1.0))

    base_bracket = model.part("base_bracket")
    base_bracket.visual(
        mesh_from_cadquery(_base_bracket_body_shape(), "base_bracket_body"),
        material=bracket_gray,
        name="base_bracket_body",
    )
    base_bracket.visual(
        mesh_from_cadquery(_base_pivot_pin_shape(), "base_pivot_pin"),
        material=arm_steel,
        name="base_pivot_pin",
    )

    primary_arm = model.part("primary_arm")
    primary_arm.visual(
        mesh_from_cadquery(_primary_arm_shape(), "primary_arm"),
        material=arm_steel,
        name="primary_arm_body",
    )

    slide_rail = model.part("slide_rail")
    slide_rail.visual(
        mesh_from_cadquery(_slide_rail_shape(), "slide_rail"),
        material=rail_black,
        name="slide_rail_body",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage"),
        material=carriage_orange,
        name="carriage_body",
    )

    model.articulation(
        "base_to_primary_arm",
        ArticulationType.REVOLUTE,
        parent=base_bracket,
        child=primary_arm,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.5,
            lower=0.0,
            upper=SWING_UPPER,
        ),
    )

    model.articulation(
        "arm_to_slide_rail",
        ArticulationType.FIXED,
        parent=primary_arm,
        child=slide_rail,
        origin=Origin(xyz=(0.095, 0.0, 0.013)),
    )

    model.articulation(
        "slide_rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=slide_rail,
        child=carriage,
        origin=Origin(xyz=(0.055, 0.0, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.25,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_bracket = object_model.get_part("base_bracket")
    primary_arm = object_model.get_part("primary_arm")
    slide_rail = object_model.get_part("slide_rail")
    carriage = object_model.get_part("carriage")
    swing_joint = object_model.get_articulation("base_to_primary_arm")
    slide_joint = object_model.get_articulation("slide_rail_to_carriage")

    def origin_distance(part_a, part_b) -> float | None:
        pos_a = ctx.part_world_position(part_a)
        pos_b = ctx.part_world_position(part_b)
        if pos_a is None or pos_b is None:
            return None
        return sqrt(
            (pos_a[0] - pos_b[0]) ** 2
            + (pos_a[1] - pos_b[1]) ** 2
            + (pos_a[2] - pos_b[2]) ** 2
        )

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        base_bracket,
        primary_arm,
        elem_a="base_pivot_pin",
        elem_b="primary_arm_body",
        reason="The vertical hinge pin intentionally passes through the primary arm hub as the revolute bearing interface.",
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
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=24)

    ctx.check(
        "parts_present",
        all(
            part is not None
            for part in (base_bracket, primary_arm, slide_rail, carriage)
        ),
        "Expected base bracket, primary arm, slide rail, and carriage parts.",
    )
    ctx.check(
        "swing_joint_axis_vertical",
        tuple(swing_joint.axis) == (0.0, 0.0, 1.0),
        f"Expected swing joint axis (0, 0, 1), got {swing_joint.axis}.",
    )
    ctx.check(
        "slide_joint_axis_longitudinal",
        tuple(slide_joint.axis) == (1.0, 0.0, 0.0),
        f"Expected slide joint axis (1, 0, 0), got {slide_joint.axis}.",
    )
    ctx.check(
        "swing_limit_realistic",
        swing_joint.motion_limits is not None
        and swing_joint.motion_limits.lower == 0.0
        and swing_joint.motion_limits.upper is not None
        and 1.4 <= swing_joint.motion_limits.upper <= 1.9,
        f"Unexpected swing limits: {swing_joint.motion_limits}.",
    )
    ctx.check(
        "slide_travel_realistic",
        slide_joint.motion_limits is not None
        and slide_joint.motion_limits.lower == 0.0
        and slide_joint.motion_limits.upper is not None
        and 0.06 <= slide_joint.motion_limits.upper <= 0.08,
        f"Unexpected slide limits: {slide_joint.motion_limits}.",
    )

    with ctx.pose({swing_joint: 0.0, slide_joint: 0.0}):
        ctx.expect_contact(base_bracket, primary_arm, name="rest_arm_supported_by_base")
        ctx.expect_contact(primary_arm, slide_rail, name="rail_fixed_to_arm_contact")
        ctx.expect_contact(carriage, slide_rail, name="rest_carriage_supported_on_rail")
        ctx.expect_overlap(
            slide_rail,
            primary_arm,
            axes="xy",
            min_overlap=0.03,
            name="rail_has_mounting_footprint_on_arm",
        )
        ctx.expect_overlap(
            carriage,
            slide_rail,
            axes="xy",
            min_overlap=0.03,
            name="rest_carriage_overlaps_rail_footprint",
        )
        rest_distance = origin_distance(carriage, slide_rail)
        ctx.check(
            "rest_carriage_position_on_rail",
            rest_distance is not None
            and abs(rest_distance - sqrt(CARRIAGE_X_OFFSET**2 + CARRIAGE_Z_OFFSET**2)) <= 0.001,
            f"Expected carriage/rail origin distance about {sqrt(CARRIAGE_X_OFFSET**2 + CARRIAGE_Z_OFFSET**2)}, got {rest_distance}.",
        )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    if swing_joint.motion_limits is not None:
        for pose_name, angle in (
            ("lower", swing_joint.motion_limits.lower),
            ("upper", swing_joint.motion_limits.upper),
        ):
            if angle is None:
                continue
            with ctx.pose({swing_joint: angle, slide_joint: 0.0}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"swing_{pose_name}_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"swing_{pose_name}_no_floating")
                ctx.expect_contact(
                    base_bracket,
                    primary_arm,
                    name=f"swing_{pose_name}_arm_stays_on_pin",
                )
                ctx.expect_contact(
                    carriage,
                    slide_rail,
                    name=f"swing_{pose_name}_carriage_stays_supported",
                )
                swing_distance = origin_distance(carriage, slide_rail)
                ctx.check(
                    f"swing_{pose_name}_carriage_origin_distance",
                    swing_distance is not None
                    and abs(swing_distance - sqrt(CARRIAGE_X_OFFSET**2 + CARRIAGE_Z_OFFSET**2)) <= 0.001,
                    f"Expected carriage/rail origin distance about {sqrt(CARRIAGE_X_OFFSET**2 + CARRIAGE_Z_OFFSET**2)}, got {swing_distance}.",
                )

    if slide_joint.motion_limits is not None:
        for pose_name, travel in (
            ("lower", slide_joint.motion_limits.lower),
            ("upper", slide_joint.motion_limits.upper),
        ):
            if travel is None:
                continue
            with ctx.pose({swing_joint: SWING_UPPER, slide_joint: travel}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"slide_{pose_name}_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"slide_{pose_name}_no_floating")
                ctx.expect_contact(
                    carriage,
                    slide_rail,
                    name=f"slide_{pose_name}_carriage_stays_on_rail",
                )
                ctx.expect_overlap(
                    carriage,
                    slide_rail,
                    axes="xy",
                    min_overlap=0.03,
                    name=f"slide_{pose_name}_carriage_overlaps_rail_footprint",
                )
                slide_distance = origin_distance(carriage, slide_rail)
                if pose_name == "lower":
                    ctx.check(
                        "slide_lower_carriage_origin_position",
                        slide_distance is not None
                        and abs(slide_distance - sqrt(CARRIAGE_X_OFFSET**2 + CARRIAGE_Z_OFFSET**2)) <= 0.001,
                        f"Expected carriage/rail origin distance about {sqrt(CARRIAGE_X_OFFSET**2 + CARRIAGE_Z_OFFSET**2)}, got {slide_distance}.",
                    )
                else:
                    ctx.check(
                        "slide_upper_carriage_origin_position",
                        slide_distance is not None
                        and abs(
                            slide_distance
                            - sqrt((CARRIAGE_X_OFFSET + SLIDE_TRAVEL) ** 2 + CARRIAGE_Z_OFFSET**2)
                        )
                        <= 0.001,
                        f"Expected carriage/rail origin distance about {sqrt((CARRIAGE_X_OFFSET + SLIDE_TRAVEL) ** 2 + CARRIAGE_Z_OFFSET**2)}, got {slide_distance}.",
                    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

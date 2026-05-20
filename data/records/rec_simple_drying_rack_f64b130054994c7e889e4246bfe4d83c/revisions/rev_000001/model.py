from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


Vec3 = tuple[float, float, float]


def _tube_origin(start: Vec3, end: Vec3) -> tuple[float, Origin]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("tube endpoints must differ")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    center = ((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5)
    return length, Origin(xyz=center, rpy=(0.0, pitch, yaw))


def _add_tube(part, start: Vec3, end: Vec3, radius: float, material: str, name: str) -> None:
    length, origin = _tube_origin(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _add_box(
    part,
    size: tuple[float, float, float],
    center: Vec3,
    material: str,
    name: str,
    *,
    rpy: Vec3 = (0.0, 0.0, 0.0),
) -> None:
    part.visual(Box(size), origin=Origin(xyz=center, rpy=rpy), material=material, name=name)


def _overall_span(ctx: TestContext, parts) -> tuple[float, float, float] | None:
    mins = [float("inf"), float("inf"), float("inf")]
    maxs = [float("-inf"), float("-inf"), float("-inf")]
    for part in parts:
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            return None
        low, high = aabb
        for i in range(3):
            mins[i] = min(mins[i], low[i])
            maxs[i] = max(maxs[i], high[i])
    return (maxs[0] - mins[0], maxs[1] - mins[1], maxs[2] - mins[2])


def _build_center_frame(model: ArticulatedObject, tube_color: str, hinge_color: str, foot_color: str):
    center = model.part("center_frame")

    tube_r = 0.004
    half_width = 0.130
    half_depth = 0.085
    top_z = 0.152
    hinge_z = 0.120
    foot_z = 0.012
    foot_y = 0.102
    foot_half_width = 0.115
    joint_x = 0.136

    _add_tube(center, (-half_width, -half_depth, top_z), (-half_width, half_depth, top_z), tube_r, tube_color, "left_top_side")
    _add_tube(center, (half_width, -half_depth, top_z), (half_width, half_depth, top_z), tube_r, tube_color, "right_top_side")
    for y, name in ((-0.055, "rear_drying_rail"), (0.0, "center_drying_rail"), (0.055, "front_drying_rail")):
        _add_tube(center, (-half_width, y, top_z), (half_width, y, top_z), tube_r, tube_color, name)

    _add_tube(center, (-0.131, -0.067, hinge_z), (-0.131, 0.067, hinge_z), tube_r * 0.95, hinge_color, "left_hinge_rail")
    _add_tube(center, (0.131, -0.067, hinge_z), (0.131, 0.067, hinge_z), tube_r * 0.95, hinge_color, "right_hinge_rail")
    for x in (-0.131, 0.131):
        for y, name in ((-0.055, "rear"), (0.0, "mid"), (0.055, "front")):
            _add_tube(
                center,
                (x, y, hinge_z),
                (math.copysign(half_width, x), y, top_z),
                tube_r * 0.9,
                hinge_color,
                f"{'left' if x < 0.0 else 'right'}_{name}_hinge_post",
            )

    _add_tube(center, (-foot_half_width, foot_y, foot_z), (foot_half_width, foot_y, foot_z), tube_r * 1.05, tube_color, "front_foot")
    _add_tube(center, (-foot_half_width, -foot_y, foot_z), (foot_half_width, -foot_y, foot_z), tube_r * 1.05, tube_color, "rear_foot")
    leg_targets = (
        (-half_width, -half_depth, top_z, -foot_half_width, -foot_y, foot_z, "left_rear_leg"),
        (-half_width, half_depth, top_z, -foot_half_width, foot_y, foot_z, "left_front_leg"),
        (half_width, -half_depth, top_z, foot_half_width, -foot_y, foot_z, "right_rear_leg"),
        (half_width, half_depth, top_z, foot_half_width, foot_y, foot_z, "right_front_leg"),
    )
    for sx, sy, sz, ex, ey, ez, name in leg_targets:
        _add_tube(center, (sx, sy, sz), (ex, ey, ez), tube_r, tube_color, name)

    for sign, side_name in ((-1.0, "left"), (1.0, "right")):
        for y, pos_name in ((-0.055, "rear"), (0.055, "front")):
            _add_box(
                center,
                (0.010, 0.022, 0.014),
                (sign * (joint_x - 0.005), y, hinge_z),
                hinge_color,
                f"{side_name}_{pos_name}_lower_clevis",
            )

    for x, y, name in (
        (-foot_half_width, foot_y, "left_front_foot_pad"),
        (foot_half_width, foot_y, "right_front_foot_pad"),
        (-foot_half_width, -foot_y, "left_rear_foot_pad"),
        (foot_half_width, -foot_y, "right_rear_foot_pad"),
    ):
        _add_box(center, (0.018, 0.015, 0.010), (x, y, 0.005), foot_color, name)

    return center


def _build_linkage(model: ArticulatedObject, name: str, sign: float, color: str):
    linkage = model.part(name)
    arm_y = 0.055
    arm_out = sign * 0.028
    arm_up = 0.055

    for y, pos_name in ((-arm_y, "rear"), (arm_y, "front")):
        _add_box(
            linkage,
            (0.010, 0.022, 0.014),
            (sign * 0.005, y, 0.0),
            color,
            f"{pos_name}_lower_ear",
        )
        _add_tube(
            linkage,
            (sign * 0.010, y, 0.0),
            (arm_out, y, arm_up),
            0.0036,
            color,
            f"{pos_name}_arm",
        )
        _add_box(
            linkage,
            (0.008, 0.022, 0.014),
            (arm_out, y, arm_up),
            color,
            f"{pos_name}_upper_ear",
        )

    _add_tube(linkage, (sign * 0.010, -arm_y, 0.0), (sign * 0.010, arm_y, 0.0), 0.0036, color, "lower_spreader")
    return linkage


def _build_wing(model: ArticulatedObject, name: str, sign: float, tube_color: str, hinge_color: str):
    wing = model.part(name)
    tube_r = 0.004
    root_x = sign * 0.038
    outer_x = sign * 0.120
    half_depth = 0.085

    _add_tube(wing, (root_x, -half_depth, 0.0), (root_x, half_depth, 0.0), tube_r, tube_color, "inner_hinge_rail")
    _add_tube(wing, (outer_x, -half_depth, 0.0), (outer_x, half_depth, 0.0), tube_r, tube_color, "outer_side_rail")
    for y, rail_name in (
        (-half_depth, "rear_perimeter_rail"),
        (-0.042, "rear_inner_rail"),
        (0.0, "center_inner_rail"),
        (0.042, "front_inner_rail"),
        (half_depth, "front_perimeter_rail"),
    ):
        _add_tube(wing, (root_x, y, 0.0), (outer_x, y, 0.0), tube_r, tube_color, rail_name)

    for y, pos_name in ((-0.055, "rear"), (0.055, "front")):
        _add_box(
            wing,
            (0.022, 0.022, 0.014),
            (sign * 0.011, y, 0.0),
            hinge_color,
            f"{pos_name}_root_plate",
        )
        _add_tube(
            wing,
            (sign * 0.022, y, 0.0),
            (root_x, y, 0.0),
            0.0032,
            hinge_color,
            f"{pos_name}_root_connector",
        )

    return wing


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_foldable_drying_rack")

    tube_white = model.material("tube_white", rgba=(0.93, 0.93, 0.95, 1.0))
    hinge_gray = model.material("hinge_gray", rgba=(0.33, 0.35, 0.38, 1.0))
    foot_black = model.material("foot_black", rgba=(0.14, 0.14, 0.15, 1.0))

    center = _build_center_frame(model, tube_white.name, hinge_gray.name, foot_black.name)
    left_linkage = _build_linkage(model, "left_linkage", -1.0, hinge_gray.name)
    right_linkage = _build_linkage(model, "right_linkage", 1.0, hinge_gray.name)
    left_wing = _build_wing(model, "left_wing", -1.0, tube_white.name, hinge_gray.name)
    right_wing = _build_wing(model, "right_wing", 1.0, tube_white.name, hinge_gray.name)

    model.articulation(
        "center_to_left_linkage",
        ArticulationType.REVOLUTE,
        parent=center,
        child=left_linkage,
        origin=Origin(xyz=(-0.136, 0.0, 0.120)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=0.50),
    )
    model.articulation(
        "left_linkage_to_left_wing",
        ArticulationType.REVOLUTE,
        parent=left_linkage,
        child=left_wing,
        origin=Origin(xyz=(-0.032, 0.0, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "center_to_right_linkage",
        ArticulationType.REVOLUTE,
        parent=center,
        child=right_linkage,
        origin=Origin(xyz=(0.136, 0.0, 0.120)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=0.50),
    )
    model.articulation(
        "right_linkage_to_right_wing",
        ArticulationType.REVOLUTE,
        parent=right_linkage,
        child=right_wing,
        origin=Origin(xyz=(0.032, 0.0, 0.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    center = object_model.get_part("center_frame")
    left_linkage = object_model.get_part("left_linkage")
    right_linkage = object_model.get_part("right_linkage")
    left_wing = object_model.get_part("left_wing")
    right_wing = object_model.get_part("right_wing")

    left_link_joint = object_model.get_articulation("center_to_left_linkage")
    left_wing_joint = object_model.get_articulation("left_linkage_to_left_wing")
    right_link_joint = object_model.get_articulation("center_to_right_linkage")
    right_wing_joint = object_model.get_articulation("right_linkage_to_right_wing")

    all_parts = (center, left_linkage, right_linkage, left_wing, right_wing)

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

    ctx.expect_contact(center, left_linkage, contact_tol=1e-5, name="left_linkage_is_mounted_to_center")
    ctx.expect_contact(center, right_linkage, contact_tol=1e-5, name="right_linkage_is_mounted_to_center")
    ctx.expect_contact(left_linkage, left_wing, contact_tol=1e-5, name="left_wing_is_carried_by_linkage")
    ctx.expect_contact(right_linkage, right_wing, contact_tol=1e-5, name="right_wing_is_carried_by_linkage")

    with ctx.pose(
        {
            left_link_joint: 0.0,
            left_wing_joint: 0.0,
            right_link_joint: 0.0,
            right_wing_joint: 0.0,
        }
    ):
        ctx.expect_gap(left_wing, center, axis="z", min_gap=0.010, name="left_wing_clears_center_in_open_pose")
        ctx.expect_gap(right_wing, center, axis="z", min_gap=0.010, name="right_wing_clears_center_in_open_pose")
        open_span = _overall_span(ctx, all_parts)
        ctx.check(
            "open_pose_reads_as_full_rack",
            open_span is not None and 0.58 <= open_span[0] <= 0.68 and 0.18 <= open_span[2] <= 0.26,
            f"expected open span roughly 0.58-0.68 m wide and 0.18-0.26 m tall, got {open_span}",
        )

    with ctx.pose(
        {
            left_link_joint: 0.50,
            left_wing_joint: 1.45,
            right_link_joint: 0.50,
            right_wing_joint: 1.45,
        }
    ):
        ctx.expect_origin_gap(center, left_wing, axis="x", min_gap=0.12, name="left_wing_stays_outboard_when_stowed")
        ctx.expect_origin_gap(right_wing, center, axis="x", min_gap=0.12, name="right_wing_stays_outboard_when_stowed")
        ctx.expect_gap(left_wing, center, axis="z", min_gap=0.020, name="left_wing_stacks_above_center_when_stowed")
        ctx.expect_gap(right_wing, center, axis="z", min_gap=0.020, name="right_wing_stacks_above_center_when_stowed")
        stowed_span = _overall_span(ctx, all_parts)
        ctx.check(
            "stowed_pose_keeps_desktop_footprint_compact",
            stowed_span is not None and stowed_span[0] <= 0.34 and stowed_span[2] <= 0.31,
            f"expected upper-stop stowed envelope within 0.34 m width and 0.31 m height, got {stowed_span}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

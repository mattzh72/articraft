from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    place_on_surface,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _reflector_shell():
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.050, -0.040),
            (0.170, 0.000),
            (0.430, 0.080),
            (0.670, 0.150),
            (0.800, 0.200),
        ],
        [
            (0.034, -0.028),
            (0.156, 0.008),
            (0.414, 0.086),
            (0.652, 0.154),
            (0.782, 0.184),
        ],
        segments=80,
        lip_samples=8,
    )
    shell.rotate_y(math.pi / 2.0)
    return shell


def _feed_support(side_sign: float):
    return tube_from_spline_points(
        [
            (0.310, 0.245 * side_sign, 0.000),
            (0.405, 0.180 * side_sign, -0.015),
            (0.520, 0.085 * side_sign, -0.010),
            (0.600, 0.000, 0.000),
        ],
        radius=0.014,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )


def _feed_nose():
    nose = ConeGeometry(radius=0.036, height=0.060, radial_segments=28, closed=True)
    nose.rotate_y(-math.pi / 2.0)
    nose.translate(0.530, 0.000, 0.000)
    return nose


def _ring_side_brace(side_sign: float):
    return tube_from_spline_points(
        [
            (0.010, 0.225 * side_sign, 0.000),
            (0.130, 0.235 * side_sign, 0.000),
            (0.220, 0.242 * side_sign, 0.000),
            (0.310, 0.245 * side_sign, 0.000),
        ],
        radius=0.015,
        samples_per_segment=14,
        radial_segments=16,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_tracking_dish")

    base_gray = model.material("base_gray", rgba=(0.40, 0.42, 0.45, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.19, 1.0))
    reflector_white = model.material("reflector_white", rgba=(0.88, 0.90, 0.92, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.75, 0.79, 1.0))
    knob_black = model.material("knob_black", rgba=(0.09, 0.10, 0.11, 1.0))
    signal_gold = model.material("signal_gold", rgba=(0.63, 0.56, 0.40, 1.0))

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Cylinder(radius=0.44, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=base_gray,
        name="foundation_plinth",
    )
    pedestal_base.visual(
        Cylinder(radius=0.25, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=base_gray,
        name="pedestal_skirt",
    )
    pedestal_base.visual(
        Cylinder(radius=0.19, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=dark_steel,
        name="bearing_cap",
    )

    rotating_column = model.part("rotating_column")
    rotating_column.visual(
        Cylinder(radius=0.12, length=0.96),
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
        material=base_gray,
        name="column_shaft",
    )
    rotating_column.visual(
        Box((0.20, 0.18, 0.12)),
        origin=Origin(xyz=(-0.08, 0.0, 0.72)),
        material=base_gray,
        name="drive_neck",
    )
    rotating_column.visual(
        Box((0.08, 0.20, 0.52)),
        origin=Origin(xyz=(-0.36, 0.0, 0.98)),
        material=dark_steel,
        name="fork_carriage",
    )
    rotating_column.visual(
        Box((0.22, 0.18, 0.10)),
        origin=Origin(xyz=(-0.23, 0.0, 0.82)),
        material=dark_steel,
        name="fork_stem",
    )
    rotating_column.visual(
        Box((0.08, 0.06, 0.62)),
        origin=Origin(xyz=(-0.34, -0.31, 1.04)),
        material=base_gray,
        name="left_side_plate",
    )
    right_side_plate = rotating_column.visual(
        Box((0.08, 0.06, 0.62)),
        origin=Origin(xyz=(-0.34, 0.31, 1.04)),
        material=base_gray,
        name="right_side_plate",
    )
    rotating_column.visual(
        Box((0.08, 0.68, 0.08)),
        origin=Origin(xyz=(-0.34, 0.0, 1.24)),
        material=base_gray,
        name="fork_bridge",
    )
    rotating_column.visual(
        Cylinder(radius=0.115, length=0.03),
        origin=Origin(xyz=(-0.20, -0.265, 1.03), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_fork_boss",
    )
    rotating_column.visual(
        Cylinder(radius=0.115, length=0.03),
        origin=Origin(xyz=(-0.20, 0.265, 1.03), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_fork_boss",
    )

    reflector_shell = _mesh("tracking_dish_reflector_shell", _reflector_shell())
    rear_ring = TorusGeometry(radius=0.22, tube=0.028, radial_segments=18, tubular_segments=72)
    rear_ring.rotate_y(math.pi / 2.0)
    feed_nose = _mesh("tracking_dish_feed_nose", _feed_nose())
    left_support = _mesh("tracking_dish_feed_support_left", _feed_support(-1.0))
    right_support = _mesh("tracking_dish_feed_support_right", _feed_support(1.0))
    rear_ring_mesh = _mesh("tracking_dish_rear_ring", rear_ring)
    left_ring_brace = _mesh("tracking_dish_left_ring_brace", _ring_side_brace(-1.0))
    right_ring_brace = _mesh("tracking_dish_right_ring_brace", _ring_side_brace(1.0))

    dish_assembly = model.part("dish_assembly")
    dish_assembly.visual(
        reflector_shell,
        origin=Origin(xyz=(0.44, 0.0, 0.0)),
        material=reflector_white,
        name="main_reflector",
    )
    dish_assembly.visual(
        rear_ring_mesh,
        origin=Origin(xyz=(0.38, 0.0, 0.0)),
        material=dark_steel,
        name="rear_ring",
    )
    dish_assembly.visual(
        Box((0.03, 0.50, 0.03)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=dark_steel,
        name="back_frame_cross",
    )
    dish_assembly.visual(
        Box((0.44, 0.06, 0.04)),
        origin=Origin(xyz=(0.22, 0.0, 0.0)),
        material=dark_steel,
        name="back_frame_spine",
    )
    dish_assembly.visual(
        Box((0.10, 0.04, 0.46)),
        origin=Origin(xyz=(0.42, 0.0, 0.0)),
        material=dark_steel,
        name="back_frame_vertical",
    )
    dish_assembly.visual(
        Cylinder(radius=0.095, length=0.05),
        origin=Origin(xyz=(0.0, -0.225, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_trunnion_hub",
    )
    dish_assembly.visual(
        Cylinder(radius=0.095, length=0.05),
        origin=Origin(xyz=(0.0, 0.225, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_trunnion_hub",
    )
    dish_assembly.visual(
        left_support,
        material=aluminum,
        name="left_feed_support",
    )
    dish_assembly.visual(
        right_support,
        material=aluminum,
        name="right_feed_support",
    )
    dish_assembly.visual(
        left_ring_brace,
        material=dark_steel,
        name="left_ring_brace",
    )
    dish_assembly.visual(
        right_ring_brace,
        material=dark_steel,
        name="right_ring_brace",
    )
    dish_assembly.visual(
        Box((0.08, 0.08, 0.08)),
        origin=Origin(xyz=(0.60, 0.0, 0.0)),
        material=dark_steel,
        name="feed_receiver_block",
    )
    dish_assembly.visual(
        Cylinder(radius=0.028, length=0.16),
        origin=Origin(xyz=(0.72, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=signal_gold,
        name="feed_horn",
    )
    dish_assembly.visual(
        feed_nose,
        origin=Origin(xyz=(0.14, 0.0, 0.0)),
        material=signal_gold,
        name="feed_horn_nose",
    )

    lock_knob = model.part("lock_knob")
    lock_knob.visual(
        Cylinder(radius=0.012, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="knob_shaft",
    )
    lock_knob.visual(
        Cylinder(radius=0.040, length=0.034),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="knob_body",
    )
    lock_knob.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.0, 0.044, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="knob_cap",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.CONTINUOUS,
        parent=pedestal_base,
        child=rotating_column,
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=1.0),
    )
    model.articulation(
        "column_to_dish",
        ArticulationType.REVOLUTE,
        parent=rotating_column,
        child=dish_assembly,
        origin=Origin(xyz=(-0.20, 0.0, 1.03)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=0.9,
            lower=-0.45,
            upper=1.10,
        ),
    )
    model.articulation(
        "column_to_lock_knob",
        ArticulationType.CONTINUOUS,
        parent=rotating_column,
        child=lock_knob,
        origin=place_on_surface(
            lock_knob,
            right_side_plate,
            point_hint=(-0.34, 0.34, 1.03),
            child_axis="+y",
            clearance=0.0,
            prefer_collisions=False,
            child_prefer_collisions=False,
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    required_parts = {"pedestal_base", "rotating_column", "dish_assembly", "lock_knob"}
    required_joints = {"base_to_column", "column_to_dish", "column_to_lock_knob"}
    part_names = {part.name for part in object_model.parts}
    joint_names = {joint.name for joint in object_model.articulations}
    parts_ok = ctx.check(
        "required_parts_present",
        required_parts.issubset(part_names),
        f"missing parts: {sorted(required_parts - part_names)}",
    )
    joints_ok = ctx.check(
        "required_articulations_present",
        required_joints.issubset(joint_names),
        f"missing articulations: {sorted(required_joints - joint_names)}",
    )
    if not (parts_ok and joints_ok):
        return ctx.report()

    pedestal_base = object_model.get_part("pedestal_base")
    rotating_column = object_model.get_part("rotating_column")
    dish_assembly = object_model.get_part("dish_assembly")
    lock_knob = object_model.get_part("lock_knob")
    azimuth = object_model.get_articulation("base_to_column")
    elevation = object_model.get_articulation("column_to_dish")
    knob_joint = object_model.get_articulation("column_to_lock_knob")

    ctx.check(
        "azimuth_axis_is_vertical",
        azimuth.axis == (0.0, 0.0, 1.0),
        f"expected vertical azimuth axis, got {azimuth.axis}",
    )
    ctx.check(
        "elevation_axis_is_horizontal",
        elevation.axis == (0.0, 1.0, 0.0),
        f"expected horizontal elevation axis, got {elevation.axis}",
    )
    ctx.check(
        "lock_knob_axis_matches_side_plate_normal",
        knob_joint.axis == (0.0, 1.0, 0.0),
        f"expected knob axis along local y, got {knob_joint.axis}",
    )
    elevation_limits = elevation.motion_limits
    ctx.check(
        "elevation_motion_limits_are_realistic",
        elevation_limits is not None
        and elevation_limits.lower is not None
        and elevation_limits.upper is not None
        and elevation_limits.lower <= -0.40
        and elevation_limits.upper >= 1.00,
        f"unexpected elevation limits: {elevation_limits}",
    )

    ctx.expect_contact(
        rotating_column,
        pedestal_base,
        elem_a="column_shaft",
        elem_b="bearing_cap",
        name="column_bears_on_pedestal",
    )
    ctx.expect_overlap(
        rotating_column,
        pedestal_base,
        axes="xy",
        min_overlap=0.20,
        elem_a="column_shaft",
        elem_b="foundation_plinth",
        name="column_centered_over_pedestal",
    )
    ctx.expect_contact(
        dish_assembly,
        rotating_column,
        elem_a="left_trunnion_hub",
        elem_b="left_fork_boss",
        name="left_trunnion_supported",
    )
    ctx.expect_contact(
        dish_assembly,
        rotating_column,
        elem_a="right_trunnion_hub",
        elem_b="right_fork_boss",
        name="right_trunnion_supported",
    )
    ctx.expect_gap(
        rotating_column,
        dish_assembly,
        axis="y",
        positive_elem="right_side_plate",
        negative_elem="right_trunnion_hub",
        min_gap=0.02,
        max_gap=0.12,
        name="right_trunnion_stays_inside_fork",
    )
    ctx.expect_gap(
        dish_assembly,
        rotating_column,
        axis="y",
        positive_elem="left_trunnion_hub",
        negative_elem="left_side_plate",
        min_gap=0.02,
        max_gap=0.12,
        name="left_trunnion_stays_inside_fork",
    )
    ctx.expect_contact(
        lock_knob,
        rotating_column,
        elem_a="knob_shaft",
        elem_b="right_side_plate",
        name="lock_knob_mounted_to_side_plate",
    )
    with ctx.pose({elevation: elevation.motion_limits.lower}):
        ctx.expect_gap(
            rotating_column,
            dish_assembly,
            axis="y",
            positive_elem="right_side_plate",
            negative_elem="right_trunnion_hub",
            min_gap=0.02,
            max_gap=0.12,
            name="right_trunnion_captured_at_low_elevation",
        )
        ctx.expect_gap(
            dish_assembly,
            rotating_column,
            axis="y",
            positive_elem="left_trunnion_hub",
            negative_elem="left_side_plate",
            min_gap=0.02,
            max_gap=0.12,
            name="left_trunnion_captured_at_low_elevation",
        )
    with ctx.pose({elevation: elevation.motion_limits.upper}):
        ctx.expect_gap(
            rotating_column,
            dish_assembly,
            axis="y",
            positive_elem="right_side_plate",
            negative_elem="right_trunnion_hub",
            min_gap=0.02,
            max_gap=0.12,
            name="right_trunnion_captured_at_high_elevation",
        )
        ctx.expect_gap(
            dish_assembly,
            rotating_column,
            axis="y",
            positive_elem="left_trunnion_hub",
            negative_elem="left_side_plate",
            min_gap=0.02,
            max_gap=0.12,
            name="left_trunnion_captured_at_high_elevation",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

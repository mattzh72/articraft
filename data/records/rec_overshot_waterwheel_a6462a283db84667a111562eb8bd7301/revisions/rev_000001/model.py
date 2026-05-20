from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin, tau

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _add_box(
    part,
    name: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    material=None,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(
        Box(size),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder_y(
    part,
    name: str,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    *,
    material=None,
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_rim_hoop(
    part,
    name: str,
    radius: float,
    y: float,
    *,
    radial_depth: float,
    thickness: float,
    material=None,
    samples: int = 32,
):
    profile = rounded_rect_profile(
        radial_depth,
        thickness,
        radius=min(0.012, radial_depth * 0.15, thickness * 0.45),
        corner_segments=4,
    )
    path = [
        (radius * cos(tau * i / samples), y, radius * sin(tau * i / samples))
        for i in range(samples)
    ]
    hoop = sweep_profile_along_spline(
        path,
        profile=profile,
        samples_per_segment=4,
        closed_spline=True,
        cap_profile=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    part.visual(
        mesh_from_geometry(hoop, name),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="timber_mill_waterwheel")

    timber = model.material("timber", rgba=(0.47, 0.33, 0.20, 1.0))
    dark_timber = model.material("dark_timber", rgba=(0.34, 0.23, 0.14, 1.0))
    stone = model.material("stone", rgba=(0.50, 0.52, 0.54, 1.0))
    iron = model.material("iron", rgba=(0.24, 0.25, 0.28, 1.0))

    support = model.part("support_frame")
    wheel = model.part("wheel")

    wheel_center_x = 0.40
    wheel_center_z = 1.55

    # Connected trough and mill framing.
    _add_box(
        support,
        "base_slab",
        (3.60, 2.40, 0.14),
        (0.0, 0.0, 0.07),
        material=stone,
    )
    _add_box(
        support,
        "trough_floor",
        (3.10, 1.30, 0.06),
        (0.0, 0.0, 0.17),
        material=dark_timber,
    )
    _add_box(
        support,
        "trough_wall_pos",
        (3.10, 0.08, 0.22),
        (0.0, 0.69, 0.28),
        material=timber,
    )
    _add_box(
        support,
        "trough_wall_neg",
        (3.10, 0.08, 0.22),
        (0.0, -0.69, 0.28),
        material=timber,
    )
    _add_box(
        support,
        "trough_end_wall",
        (0.10, 1.46, 0.22),
        (-1.50, 0.0, 0.28),
        material=timber,
    )

    _add_box(
        support,
        "mill_post_front",
        (0.14, 0.14, 1.56),
        (-1.10, 0.84, 0.92),
        material=timber,
    )
    _add_box(
        support,
        "mill_post_back",
        (0.14, 0.14, 1.56),
        (-1.10, -0.84, 0.92),
        material=timber,
    )
    _add_box(
        support,
        "mill_top_beam",
        (0.18, 1.82, 0.14),
        (-1.10, 0.0, 1.63),
        material=dark_timber,
    )

    brace_length = 1.78
    brace_angle = 0.92
    _add_box(
        support,
        "mill_brace_front",
        (brace_length, 0.10, 0.10),
        (-0.83, 0.84, 0.93),
        material=dark_timber,
        rpy=(0.0, brace_angle, 0.0),
    )
    _add_box(
        support,
        "mill_brace_back",
        (brace_length, 0.10, 0.10),
        (-0.83, -0.84, 0.93),
        material=dark_timber,
        rpy=(0.0, brace_angle, 0.0),
    )

    post_xs = (0.20, 0.60)
    support_y_positions = (0.58, -0.58)
    for side_name, side_y in (("pos", support_y_positions[0]), ("neg", support_y_positions[1])):
        for idx, post_x in enumerate(post_xs):
            _add_box(
                support,
                f"axle_post_{side_name}_{idx}",
                (0.12, 0.12, 1.39),
                (post_x, side_y, 0.835),
                material=timber,
            )
        _add_box(
            support,
            f"axle_bed_{side_name}",
            (0.46, 0.12, 0.10),
            (wheel_center_x, side_y, 1.44),
            material=dark_timber,
        )
        _add_box(
            support,
            f"axle_block_{side_name}_left_cheek",
            (0.04, 0.12, 0.16),
            (wheel_center_x - 0.081, side_y, 1.57),
            material=timber,
        )
        _add_box(
            support,
            f"axle_block_{side_name}_right_cheek",
            (0.04, 0.12, 0.16),
            (wheel_center_x + 0.081, side_y, 1.57),
            material=timber,
        )

    # Waterwheel assembly.
    _add_cylinder_y(
        wheel,
        "axle",
        radius=0.06,
        length=1.04,
        center=(0.0, 0.0, 0.0),
        material=iron,
    )
    _add_cylinder_y(
        wheel,
        "hub",
        radius=0.18,
        length=0.34,
        center=(0.0, 0.0, 0.0),
        material=dark_timber,
    )

    inner_clear_width = 0.544
    bucket_count = 16
    side_ring_y = 0.30
    side_ring_thickness = 0.06
    inner_ring_radius = 0.88
    bucket_floor_radius = 1.05
    bucket_wall_radius = 1.01

    _add_rim_hoop(
        wheel,
        "rim_hoop_pos",
        radius=1.02,
        y=side_ring_y,
        radial_depth=0.36,
        thickness=side_ring_thickness,
        material=timber,
    )
    _add_rim_hoop(
        wheel,
        "rim_hoop_neg",
        radius=1.02,
        y=-side_ring_y,
        radial_depth=0.36,
        thickness=side_ring_thickness,
        material=timber,
    )

    for i in range(bucket_count):
        theta = tau * i / bucket_count
        ring_rpy = (0.0, -theta, 0.0)

        inner_x = inner_ring_radius * cos(theta)
        inner_z = inner_ring_radius * sin(theta)
        _add_box(
            wheel,
            f"inner_ring_{i}",
            (0.08, inner_clear_width, 0.35),
            (inner_x, 0.0, inner_z),
            material=dark_timber,
            rpy=ring_rpy,
        )

        floor_x = bucket_floor_radius * cos(theta)
        floor_z = bucket_floor_radius * sin(theta)
        _add_box(
            wheel,
            f"bucket_floor_{i}",
            (0.10, inner_clear_width, 0.40),
            (floor_x, 0.0, floor_z),
            material=timber,
            rpy=ring_rpy,
        )

        wall_theta = theta + (tau / (2.0 * bucket_count))
        wall_x = bucket_wall_radius * cos(wall_theta)
        wall_z = bucket_wall_radius * sin(wall_theta)
        _add_box(
            wheel,
            f"bucket_wall_{i}",
            (0.20, inner_clear_width, 0.07),
            (wall_x, 0.0, wall_z),
            material=timber,
            rpy=(0.0, -wall_theta, 0.0),
        )

    for i in range(8):
        theta = tau * i / 8.0
        spoke_radius = 0.53
        _add_box(
            wheel,
            f"spoke_{i}",
            (0.70, inner_clear_width, 0.08),
            (spoke_radius * cos(theta), 0.0, spoke_radius * sin(theta)),
            material=dark_timber,
            rpy=(0.0, -theta, 0.0),
        )

    model.articulation(
        "wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(wheel_center_x, 0.0, wheel_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_frame")
    wheel = object_model.get_part("wheel")
    axle_joint = object_model.get_articulation("wheel_axle")
    wheel_visual_names = {visual.name for visual in wheel.visuals}
    bucket_floor_count = sum(name.startswith("bucket_floor_") for name in wheel_visual_names)
    bucket_wall_count = sum(name.startswith("bucket_wall_") for name in wheel_visual_names)
    spoke_count = sum(name.startswith("spoke_") for name in wheel_visual_names)

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

    wheel_aabb = ctx.part_world_aabb(wheel)
    support_aabb = ctx.part_world_aabb(support)
    mill_post_aabb = ctx.part_element_world_aabb(support, elem="mill_post_front")
    trough_aabb = ctx.part_element_world_aabb(support, elem="trough_floor")

    wheel_height = None if wheel_aabb is None else wheel_aabb[1][2] - wheel_aabb[0][2]
    wheel_width = None if wheel_aabb is None else wheel_aabb[1][1] - wheel_aabb[0][1]
    support_length = None if support_aabb is None else support_aabb[1][0] - support_aabb[0][0]
    frame_gap_x = (
        None
        if wheel_aabb is None or mill_post_aabb is None
        else wheel_aabb[0][0] - mill_post_aabb[1][0]
    )

    ctx.check(
        "wheel_dimensions_realistic",
        wheel_height is not None
        and wheel_width is not None
        and 2.30 <= wheel_height <= 2.45
        and 1.00 <= wheel_width <= 1.06,
        f"wheel dims expected about 2.4 m tall and 1.04 m wide, got height={wheel_height}, width={wheel_width}",
    )
    ctx.check(
        "support_length_realistic",
        support_length is not None and 3.40 <= support_length <= 3.80,
        f"support length expected about 3.6 m, got {support_length}",
    )
    ctx.check(
        "wheel_beside_mill_frame",
        frame_gap_x is not None and frame_gap_x >= 0.20,
        f"wheel should sit beside the mill frame with visible side gap, got {frame_gap_x}",
    )
    ctx.check(
        "wheel_axle_joint_type",
        axle_joint.joint_type == ArticulationType.CONTINUOUS,
        f"expected continuous axle, got {axle_joint.joint_type}",
    )
    ctx.check(
        "wheel_axle_axis_horizontal",
        tuple(axle_joint.axis) == (0.0, 1.0, 0.0),
        f"expected horizontal axle axis (0, 1, 0), got {axle_joint.axis}",
    )
    ctx.check(
        "wheel_axle_is_unbounded",
        axle_joint.motion_limits is not None
        and axle_joint.motion_limits.lower is None
        and axle_joint.motion_limits.upper is None,
        "continuous axle should not define lower/upper bounds",
    )
    ctx.check(
        "trough_present_beneath_wheel",
        trough_aabb is not None and trough_aabb[1][2] > 0.0,
        "expected a named trough floor visual under the wheel",
    )
    ctx.check(
        "deep_rim_hoops_present",
        "rim_hoop_pos" in wheel_visual_names and "rim_hoop_neg" in wheel_visual_names,
        "expected continuous side rim hoops to define the deep wheel rim",
    )
    ctx.check(
        "bucket_set_complete",
        bucket_floor_count == 16 and bucket_wall_count == 16,
        f"expected 16 bucket floors and 16 bucket dividers, got floors={bucket_floor_count}, walls={bucket_wall_count}",
    )
    ctx.check(
        "spoke_set_complete",
        spoke_count == 8,
        f"expected 8 timber spokes, got {spoke_count}",
    )

    ctx.expect_contact(
        wheel,
        support,
        elem_a="axle",
        elem_b="axle_bed_pos",
        name="axle_contacts_positive_bed",
    )
    ctx.expect_contact(
        wheel,
        support,
        elem_a="axle",
        elem_b="axle_bed_neg",
        name="axle_contacts_negative_bed",
    )
    ctx.expect_gap(
        wheel,
        support,
        axis="z",
        min_gap=0.12,
        max_gap=0.18,
        negative_elem="trough_floor",
        name="wheel_above_trough_floor",
    )
    ctx.expect_overlap(
        wheel,
        support,
        axes="x",
        min_overlap=2.20,
        elem_b="trough_floor",
        name="wheel_over_trough_span",
    )

    for label, angle in (("rest", 0.0), ("quarter_turn", pi / 4.0), ("half_turn", pi / 2.0)):
        with ctx.pose({axle_joint: angle}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{label}_no_floating")
            ctx.expect_contact(
                wheel,
                support,
                elem_a="axle",
                elem_b="axle_bed_pos",
                name=f"{label}_axle_contacts_positive_bed",
            )
            ctx.expect_contact(
                wheel,
                support,
                elem_a="axle",
                elem_b="axle_bed_neg",
                name=f"{label}_axle_contacts_negative_bed",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

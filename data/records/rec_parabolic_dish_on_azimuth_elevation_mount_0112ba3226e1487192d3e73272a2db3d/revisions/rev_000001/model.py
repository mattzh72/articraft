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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, name: str, a, b, *, radius: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _build_reflector_shell(
    *,
    diameter: float,
    depth: float,
    thickness: float,
    samples: int = 12,
):
    radius = diameter * 0.5
    hub_radius = max(0.045, diameter * 0.035)
    front_profile: list[tuple[float, float]] = []
    back_profile: list[tuple[float, float]] = []
    for index in range(samples + 1):
        t = index / samples
        radial = hub_radius + (radius - hub_radius) * t
        radial_fraction = radial / radius
        parabola_x = depth * radial_fraction * radial_fraction
        front_profile.append((radial, parabola_x))
        back_profile.append(
            (
                max(0.0, radial - thickness * (0.18 + 0.82 * t)),
                parabola_x - thickness * (0.55 + 0.45 * t),
            )
        )
    return LatheGeometry.from_shell_profiles(
        front_profile,
        back_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="satellite_communication_dish")

    foundation_gray = model.material("foundation_gray", rgba=(0.63, 0.64, 0.66, 1.0))
    structure_white = model.material("structure_white", rgba=(0.89, 0.91, 0.92, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.39, 0.41, 0.44, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.19, 1.0))
    antenna_white = model.material("antenna_white", rgba=(0.95, 0.96, 0.97, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))

    reflector_shell_mesh = _save_mesh(
        "reflector_shell",
        _build_reflector_shell(
            diameter=1.80,
            depth=0.25,
            thickness=0.020,
        ).rotate_y(math.pi / 2.0),
    )
    rim_ring_mesh = _save_mesh(
        "reflector_rim",
        TorusGeometry(
            radius=0.90,
            tube=0.026,
            radial_segments=18,
            tubular_segments=80,
        ).rotate_y(math.pi / 2.0),
    )
    rear_ring_mesh = _save_mesh(
        "rear_frame_ring",
        TorusGeometry(
            radius=0.42,
            tube=0.030,
            radial_segments=16,
            tubular_segments=64,
        ).rotate_y(math.pi / 2.0),
    )
    hub_ring_mesh = _save_mesh(
        "hub_ring",
        TorusGeometry(
            radius=0.18,
            tube=0.026,
            radial_segments=16,
            tubular_segments=48,
        ).rotate_y(math.pi / 2.0),
    )
    rear_rib_mesh = _save_mesh(
        "rear_rib",
        tube_from_spline_points(
            [
                (0.03, 0.0, 0.04),
                (0.08, 0.0, 0.14),
                (0.15, 0.0, 0.28),
                (0.22, 0.0, 0.40),
            ],
            radius=0.022,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    feed_boom_mesh = _save_mesh(
        "feed_boom",
        tube_from_spline_points(
            [
                (0.06, 0.0, -0.10),
                (0.22, 0.0, -0.07),
                (0.54, 0.0, -0.03),
                (0.84, 0.0, -0.01),
            ],
            radius=0.022,
            samples_per_segment=16,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    left_feed_brace_mesh = _save_mesh(
        "left_feed_brace",
        tube_from_spline_points(
            [
                (0.08, 0.12, -0.08),
                (0.26, 0.10, -0.05),
                (0.54, 0.06, -0.03),
                (0.84, 0.0, -0.01),
            ],
            radius=0.016,
            samples_per_segment=16,
            radial_segments=14,
            cap_ends=True,
        ),
    )
    right_feed_brace_mesh = _save_mesh(
        "right_feed_brace",
        tube_from_spline_points(
            [
                (0.08, -0.12, -0.08),
                (0.26, -0.10, -0.05),
                (0.54, -0.06, -0.03),
                (0.84, 0.0, -0.01),
            ],
            radius=0.016,
            samples_per_segment=16,
            radial_segments=14,
            cap_ends=True,
        ),
    )

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Box((1.50, 1.50, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=foundation_gray,
        name="foundation",
    )
    pedestal_base.visual(
        Box((0.60, 0.46, 0.34)),
        origin=Origin(xyz=(0.38, 0.0, 0.35)),
        material=machinery_gray,
        name="equipment_cabinet",
    )
    pedestal_base.visual(
        Cylinder(radius=0.30, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
        material=machinery_gray,
        name="pedestal_plinth",
    )
    pedestal_base.visual(
        Cylinder(radius=0.23, length=1.15),
        origin=Origin(xyz=(0.0, 0.0, 0.755)),
        material=structure_white,
        name="pedestal_column",
    )
    pedestal_base.visual(
        Cylinder(radius=0.34, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 1.39)),
        material=dark_steel,
        name="azimuth_bearing",
    )
    pedestal_base.inertial = Inertial.from_geometry(
        Box((1.50, 1.50, 1.45)),
        mass=950.0,
        origin=Origin(xyz=(0.0, 0.0, 0.725)),
    )

    yoke_mount = model.part("yoke_mount")
    yoke_mount.visual(
        Cylinder(radius=0.34, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=dark_steel,
        name="turntable_drum",
    )
    yoke_mount.visual(
        Cylinder(radius=0.46, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=machinery_gray,
        name="rotating_deck",
    )
    yoke_mount.visual(
        Cylinder(radius=0.18, length=0.54),
        origin=Origin(xyz=(-0.03, 0.0, 0.43)),
        material=structure_white,
        name="center_post",
    )
    _add_member(
        yoke_mount,
        "rear_spine",
        (-0.08, 0.0, 0.16),
        (-0.10, 0.0, 1.12),
        radius=0.050,
        material=machinery_gray,
    )
    _add_member(
        yoke_mount,
        "left_leg",
        (-0.06, 0.38, 0.16),
        (-0.08, 0.56, 0.78),
        radius=0.048,
        material=structure_white,
    )
    _add_member(
        yoke_mount,
        "right_leg",
        (-0.06, -0.38, 0.16),
        (-0.08, -0.56, 0.78),
        radius=0.048,
        material=structure_white,
    )
    _add_member(
        yoke_mount,
        "left_arm",
        (-0.08, 0.56, 0.78),
        (-0.10, 0.68, 1.20),
        radius=0.042,
        material=structure_white,
    )
    _add_member(
        yoke_mount,
        "right_arm",
        (-0.08, -0.56, 0.78),
        (-0.10, -0.68, 1.20),
        radius=0.042,
        material=structure_white,
    )
    _add_member(
        yoke_mount,
        "left_crown",
        (-0.10, 0.0, 1.12),
        (-0.10, 0.68, 1.20),
        radius=0.032,
        material=machinery_gray,
    )
    _add_member(
        yoke_mount,
        "right_crown",
        (-0.10, 0.0, 1.12),
        (-0.10, -0.68, 1.20),
        radius=0.032,
        material=machinery_gray,
    )
    _add_member(
        yoke_mount,
        "upper_tie",
        (-0.10, -0.68, 1.20),
        (-0.10, 0.68, 1.20),
        radius=0.030,
        material=machinery_gray,
    )
    _add_member(
        yoke_mount,
        "left_tip_link",
        (-0.10, 0.68, 1.20),
        (0.08, 0.68, 1.20),
        radius=0.028,
        material=machinery_gray,
    )
    _add_member(
        yoke_mount,
        "right_tip_link",
        (-0.10, -0.68, 1.20),
        (0.08, -0.68, 1.20),
        radius=0.028,
        material=machinery_gray,
    )
    yoke_mount.visual(
        Cylinder(radius=0.10, length=0.22),
        origin=Origin(xyz=(0.08, 0.67, 1.20), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_bearing",
    )
    yoke_mount.visual(
        Cylinder(radius=0.10, length=0.22),
        origin=Origin(xyz=(0.08, -0.67, 1.20), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_bearing",
    )
    yoke_mount.visual(
        Box((0.20, 0.24, 0.28)),
        origin=Origin(xyz=(-0.04, 0.42, 0.36)),
        material=machinery_gray,
        name="left_drive_box",
    )
    yoke_mount.visual(
        Box((0.20, 0.24, 0.28)),
        origin=Origin(xyz=(-0.04, -0.42, 0.36)),
        material=machinery_gray,
        name="right_drive_box",
    )
    yoke_mount.inertial = Inertial.from_geometry(
        Box((0.90, 1.50, 1.16)),
        mass=280.0,
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
    )

    reflector_assembly = model.part("reflector_assembly")
    reflector_assembly.visual(
        Cylinder(radius=0.095, length=1.12),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_shaft",
    )
    reflector_assembly.visual(
        Cylinder(radius=0.14, length=0.28),
        origin=Origin(xyz=(0.12, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery_gray,
        name="hub_barrel",
    )
    reflector_assembly.visual(
        Box((0.18, 0.48, 0.30)),
        origin=Origin(xyz=(0.05, 0.0, 0.0)),
        material=machinery_gray,
        name="hub_block",
    )
    reflector_assembly.visual(
        reflector_shell_mesh,
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
        material=antenna_white,
        name="reflector_shell",
    )
    reflector_assembly.visual(
        rim_ring_mesh,
        origin=Origin(xyz=(0.43, 0.0, 0.0)),
        material=aluminum,
        name="rim_stiffener",
    )
    reflector_assembly.visual(
        rear_ring_mesh,
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
        material=dark_steel,
        name="rear_frame_ring",
    )
    reflector_assembly.visual(
        hub_ring_mesh,
        origin=Origin(xyz=(0.12, 0.0, 0.0)),
        material=dark_steel,
        name="hub_ring",
    )
    for index in range(8):
        angle = index * math.tau / 8.0
        reflector_assembly.visual(
            rear_rib_mesh,
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=dark_steel,
            name=f"rear_rib_{index:02d}",
        )
    reflector_assembly.visual(feed_boom_mesh, material=structure_white, name="feed_boom")
    reflector_assembly.visual(left_feed_brace_mesh, material=aluminum, name="left_feed_brace")
    reflector_assembly.visual(right_feed_brace_mesh, material=aluminum, name="right_feed_brace")
    reflector_assembly.visual(
        Cylinder(radius=0.055, length=0.10),
        origin=Origin(xyz=(0.85, 0.0, -0.01), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="feed_support_collar",
    )
    reflector_assembly.visual(
        Cylinder(radius=0.048, length=0.18),
        origin=Origin(xyz=(0.97, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="feed_horn",
    )
    reflector_assembly.inertial = Inertial.from_geometry(
        Box((1.15, 1.90, 1.90)),
        mass=220.0,
        origin=Origin(xyz=(0.34, 0.0, 0.0)),
    )

    model.articulation(
        "azimuth_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal_base,
        child=yoke_mount,
        origin=Origin(xyz=(0.0, 0.0, 1.45)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4500.0,
            velocity=0.8,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "dish_elevation",
        ArticulationType.REVOLUTE,
        parent=yoke_mount,
        child=reflector_assembly,
        origin=Origin(xyz=(0.08, 0.0, 1.20)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3200.0,
            velocity=0.9,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal_base = object_model.get_part("pedestal_base")
    yoke_mount = object_model.get_part("yoke_mount")
    reflector_assembly = object_model.get_part("reflector_assembly")

    azimuth_yaw = object_model.get_articulation("azimuth_yaw")
    dish_elevation = object_model.get_articulation("dish_elevation")

    pedestal_base.get_visual("foundation")
    pedestal_base.get_visual("pedestal_column")
    pedestal_base.get_visual("azimuth_bearing")
    yoke_mount.get_visual("turntable_drum")
    yoke_mount.get_visual("left_bearing")
    yoke_mount.get_visual("right_bearing")
    yoke_mount.get_visual("left_arm")
    reflector_assembly.get_visual("reflector_shell")
    reflector_assembly.get_visual("rear_frame_ring")
    reflector_assembly.get_visual("feed_boom")
    reflector_assembly.get_visual("feed_horn")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(max_pose_samples=12)
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=20)

    ctx.check(
        "azimuth_axis_is_vertical",
        azimuth_yaw.axis == (0.0, 0.0, 1.0),
        details=f"axis={azimuth_yaw.axis}",
    )
    ctx.check(
        "elevation_axis_is_horizontal",
        dish_elevation.axis == (0.0, -1.0, 0.0),
        details=f"axis={dish_elevation.axis}",
    )
    ctx.check(
        "azimuth_span_is_full_rotation_class",
        azimuth_yaw.motion_limits is not None
        and azimuth_yaw.motion_limits.lower is not None
        and azimuth_yaw.motion_limits.upper is not None
        and azimuth_yaw.motion_limits.lower <= -3.0
        and azimuth_yaw.motion_limits.upper >= 3.0,
        details=str(azimuth_yaw.motion_limits),
    )
    ctx.check(
        "elevation_limits_are_realistic",
        dish_elevation.motion_limits is not None
        and dish_elevation.motion_limits.lower == 0.0
        and dish_elevation.motion_limits.upper is not None
        and 1.30 <= dish_elevation.motion_limits.upper <= 1.55,
        details=str(dish_elevation.motion_limits),
    )

    ctx.expect_contact(
        yoke_mount,
        pedestal_base,
        elem_a="turntable_drum",
        elem_b="azimuth_bearing",
        name="turntable_contacts_bearing",
    )
    ctx.expect_contact(
        reflector_assembly,
        yoke_mount,
        elem_a="trunnion_shaft",
        elem_b="left_bearing",
        name="left_trunnion_contacts_left_bearing",
    )
    ctx.expect_contact(
        reflector_assembly,
        yoke_mount,
        elem_a="trunnion_shaft",
        elem_b="right_bearing",
        name="right_trunnion_contacts_right_bearing",
    )

    def _aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    feed_rest_aabb = ctx.part_element_world_aabb(reflector_assembly, elem="feed_horn")
    ctx.check("feed_horn_rest_aabb_exists", feed_rest_aabb is not None, details="missing feed horn aabb")
    if feed_rest_aabb is not None:
        feed_rest_center = _aabb_center(feed_rest_aabb)
        ctx.check(
            "feed_horn_forward_of_axis_at_rest",
            feed_rest_center[0] > 0.85,
            details=f"feed_center={feed_rest_center}",
        )

    elevation_limits = dish_elevation.motion_limits
    if elevation_limits is not None and elevation_limits.upper is not None:
        with ctx.pose({dish_elevation: elevation_limits.lower or 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="dish_elevation_lower_no_overlap")
            ctx.fail_if_isolated_parts(
                max_pose_samples=1,
                name="dish_elevation_lower_no_floating",
            )
            ctx.expect_contact(
                reflector_assembly,
                yoke_mount,
                elem_a="trunnion_shaft",
                elem_b="left_bearing",
                name="left_trunnion_contact_lower_pose",
            )
            ctx.expect_contact(
                reflector_assembly,
                yoke_mount,
                elem_a="trunnion_shaft",
                elem_b="right_bearing",
                name="right_trunnion_contact_lower_pose",
            )
            ctx.expect_gap(
                reflector_assembly,
                pedestal_base,
                axis="z",
                min_gap=0.08,
                name="dish_clears_pedestal_low_elevation",
            )
            feed_low_aabb = ctx.part_element_world_aabb(reflector_assembly, elem="feed_horn")

        with ctx.pose({dish_elevation: elevation_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="dish_elevation_upper_no_overlap")
            ctx.fail_if_isolated_parts(
                max_pose_samples=1,
                name="dish_elevation_upper_no_floating",
            )
            ctx.expect_contact(
                reflector_assembly,
                yoke_mount,
                elem_a="trunnion_shaft",
                elem_b="left_bearing",
                name="left_trunnion_contact_upper_pose",
            )
            ctx.expect_contact(
                reflector_assembly,
                yoke_mount,
                elem_a="trunnion_shaft",
                elem_b="right_bearing",
                name="right_trunnion_contact_upper_pose",
            )
            feed_high_aabb = ctx.part_element_world_aabb(reflector_assembly, elem="feed_horn")

        if feed_low_aabb is not None and feed_high_aabb is not None:
            feed_low_center = _aabb_center(feed_low_aabb)
            feed_high_center = _aabb_center(feed_high_aabb)
            ctx.check(
                "feed_horn_rises_with_elevation",
                feed_high_center[2] > feed_low_center[2] + 0.60,
                details=f"low={feed_low_center}, high={feed_high_center}",
            )

    azimuth_limits = azimuth_yaw.motion_limits
    if azimuth_limits is not None and azimuth_limits.lower is not None and azimuth_limits.upper is not None:
        with ctx.pose({azimuth_yaw: azimuth_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="azimuth_yaw_lower_no_overlap")
            ctx.fail_if_isolated_parts(
                max_pose_samples=1,
                name="azimuth_yaw_lower_no_floating",
            )
        with ctx.pose({azimuth_yaw: math.pi / 2.0}):
            feed_quarter_turn_aabb = ctx.part_element_world_aabb(reflector_assembly, elem="feed_horn")
        if feed_rest_aabb is not None and feed_quarter_turn_aabb is not None:
            feed_rest_center = _aabb_center(feed_rest_aabb)
            feed_quarter_turn_center = _aabb_center(feed_quarter_turn_aabb)
            ctx.check(
                "azimuth_rotates_feed_horn_around_pedestal",
                abs(feed_quarter_turn_center[1]) > 0.70
                and abs(feed_quarter_turn_center[0]) < feed_rest_center[0] - 0.40,
                details=f"rest={feed_rest_center}, quarter={feed_quarter_turn_center}",
            )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

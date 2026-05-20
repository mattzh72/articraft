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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    superellipse_profile,
)


def _circular_profile(radius: float, segments: int = 56) -> list[tuple[float, float]]:
    return superellipse_profile(
        radius * 2.0,
        radius * 2.0,
        exponent=2.0,
        segments=segments,
    )


def _ring_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    segments: int = 56,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circular_profile(outer_radius, segments=segments),
            [_circular_profile(inner_radius, segments=max(24, segments // 2))],
            height,
            center=True,
        ),
        name,
    )


def _reflector_shell_mesh(name: str):
    outer_profile = [
        (0.02, 0.00),
        (0.15, 0.02),
        (0.25, 0.08),
        (0.31, 0.16),
        (0.34, 0.26),
        (0.36, 0.36),
        (0.37, 0.42),
    ]
    inner_profile = [
        (0.0, 0.02),
        (0.11, 0.04),
        (0.20, 0.09),
        (0.27, 0.16),
        (0.30, 0.26),
        (0.32, 0.36),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=52,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        name,
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lighthouse_lantern_assembly")

    tower_white = model.material("tower_white", rgba=(0.92, 0.93, 0.90, 1.0))
    iron_black = model.material("iron_black", rgba=(0.16, 0.17, 0.18, 1.0))
    deck_gray = model.material("deck_gray", rgba=(0.42, 0.44, 0.46, 1.0))
    lantern_glass = model.material("lantern_glass", rgba=(0.72, 0.87, 0.94, 0.28))
    beacon_brass = model.material("beacon_brass", rgba=(0.63, 0.52, 0.25, 1.0))
    reflector_silver = model.material("reflector_silver", rgba=(0.86, 0.88, 0.90, 1.0))
    lamp_glow = model.material("lamp_glow", rgba=(1.0, 0.89, 0.54, 0.82))

    tower_top = model.part("tower_top")

    tower_frustum = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 0.0),
                (2.35, 0.0),
                (2.28, 0.20),
                (2.05, 1.10),
                (1.82, 1.95),
                (1.62, 2.55),
                (0.0, 2.55),
            ],
            segments=72,
        ),
        "tower_top_frustum",
    )
    tower_top.visual(
        tower_frustum,
        material=tower_white,
        name="tower_frustum",
    )
    tower_top.visual(
        Cylinder(radius=2.05, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 2.63)),
        material=deck_gray,
        name="gallery_deck",
    )
    tower_top.visual(
        _ring_mesh(
            "lantern_curb_ring",
            outer_radius=1.42,
            inner_radius=1.08,
            height=0.22,
            segments=72,
        ),
        origin=Origin(xyz=(0.0, 0.0, 2.82)),
        material=tower_white,
        name="lantern_curb",
    )
    tower_top.visual(
        Cylinder(radius=1.50, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 4.79)),
        material=iron_black,
        name="roof_base_ring",
    )

    railing_radius = 1.83
    post_count = 16
    for index in range(post_count):
        angle = (2.0 * math.pi * index) / post_count
        tower_top.visual(
            Cylinder(radius=0.028, length=0.88),
            origin=Origin(
                xyz=(railing_radius * math.cos(angle), railing_radius * math.sin(angle), 3.13)
            ),
            material=iron_black,
            name=f"rail_post_{index:02d}",
        )

    tower_top.visual(
        mesh_from_geometry(
            TorusGeometry(radius=1.83, tube=0.03, radial_segments=18, tubular_segments=80),
            "gallery_top_rail",
        ),
        origin=Origin(xyz=(0.0, 0.0, 3.55)),
        material=iron_black,
        name="gallery_top_rail",
    )
    tower_top.visual(
        mesh_from_geometry(
            TorusGeometry(radius=1.81, tube=0.022, radial_segments=16, tubular_segments=72),
            "gallery_mid_rail",
        ),
        origin=Origin(xyz=(0.0, 0.0, 3.13)),
        material=iron_black,
        name="gallery_mid_rail",
    )

    pane_count = 12
    pane_radius = 1.24
    pane_width = 0.62
    pane_height = 1.84
    pane_center_z = 3.83
    mullion_radius = 1.33
    for index in range(pane_count):
        angle = (2.0 * math.pi * index) / pane_count
        tower_top.visual(
            Box((0.032, pane_width, pane_height)),
            origin=Origin(
                xyz=(pane_radius * math.cos(angle), pane_radius * math.sin(angle), pane_center_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=lantern_glass,
            name=f"glass_panel_{index:02d}",
        )
        mullion_angle = angle + (math.pi / pane_count)
        tower_top.visual(
            Box((0.10, 0.08, 1.92)),
            origin=Origin(
                xyz=(
                    mullion_radius * math.cos(mullion_angle),
                    mullion_radius * math.sin(mullion_angle),
                    3.83,
                ),
                rpy=(0.0, 0.0, mullion_angle),
            ),
            material=iron_black,
            name=f"mullion_{index:02d}",
        )

    roof_shell = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 1.08),
                (0.12, 1.06),
                (0.42, 0.96),
                (0.82, 0.72),
                (1.12, 0.42),
                (1.32, 0.17),
                (1.48, 0.0),
                (0.0, 0.0),
            ],
            segments=72,
        ),
        "lantern_roof_shell",
    )
    tower_top.visual(
        roof_shell,
        origin=Origin(xyz=(0.0, 0.0, 4.75)),
        material=iron_black,
        name="roof_shell",
    )
    tower_top.visual(
        Cylinder(radius=0.25, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 6.00)),
        material=iron_black,
        name="ventilator_body",
    )
    tower_top.visual(
        Sphere(radius=0.16),
        origin=Origin(xyz=(0.0, 0.0, 6.25)),
        material=iron_black,
        name="ventilator_cap",
    )
    tower_top.inertial = Inertial.from_geometry(
        Box((4.7, 4.7, 6.4)),
        mass=9000.0,
        origin=Origin(xyz=(0.0, 0.0, 3.2)),
    )

    central_shaft = model.part("central_shaft")
    central_shaft.visual(
        Cylinder(radius=0.28, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=iron_black,
        name="pedestal_base",
    )
    central_shaft.visual(
        Cylinder(radius=0.09, length=1.12),
        origin=Origin(xyz=(0.0, 0.0, 0.74)),
        material=iron_black,
        name="main_shaft",
    )
    central_shaft.visual(
        Cylinder(radius=0.12, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 1.36)),
        material=beacon_brass,
        name="upper_bearing",
    )
    central_shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.28, length=1.48),
        mass=280.0,
        origin=Origin(xyz=(0.0, 0.0, 0.74)),
    )

    model.articulation(
        "tower_to_shaft",
        ArticulationType.FIXED,
        parent=tower_top,
        child=central_shaft,
        origin=Origin(xyz=(0.0, 0.0, 2.71)),
    )

    beacon_carriage = model.part("beacon_carriage")
    beacon_carriage.visual(
        _ring_mesh(
            "beacon_turntable_ring",
            outer_radius=0.55,
            inner_radius=0.12,
            height=0.12,
            segments=64,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=beacon_brass,
        name="turntable_ring",
    )
    beacon_carriage.visual(
        _ring_mesh(
            "beacon_upper_guard_ring",
            outer_radius=0.46,
            inner_radius=0.18,
            height=0.08,
            segments=56,
        ),
        origin=Origin(xyz=(0.0, 0.0, 1.31)),
        material=beacon_brass,
        name="top_ring",
    )
    beacon_carriage.visual(
        Box((0.22, 0.10, 0.08)),
        origin=Origin(xyz=(-0.22, 0.0, 0.76)),
        material=beacon_brass,
        name="lamp_crosshead_west",
    )
    beacon_carriage.visual(
        Box((0.22, 0.10, 0.08)),
        origin=Origin(xyz=(0.22, 0.0, 0.76)),
        material=beacon_brass,
        name="lamp_crosshead_east",
    )

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            beacon_carriage.visual(
                Box((0.08, 0.08, 1.22)),
                origin=Origin(xyz=(0.34 * x_sign, 0.16 * y_sign, 0.67)),
                material=beacon_brass,
                name=f"frame_post_{'p' if x_sign > 0 else 'm'}{'p' if y_sign > 0 else 'm'}",
            )

    for x_sign in (-1.0, 1.0):
        beacon_carriage.visual(
            _reflector_shell_mesh(f"reflector_shell_{'east' if x_sign > 0 else 'west'}"),
            origin=Origin(
                xyz=(0.27 * x_sign, 0.0, 0.78),
                rpy=(0.0, math.pi / 2.0 if x_sign > 0 else -math.pi / 2.0, 0.0),
            ),
            material=reflector_silver,
            name=f"reflector_{'east' if x_sign > 0 else 'west'}",
        )
        beacon_carriage.visual(
            Cylinder(radius=0.06, length=0.12),
            origin=Origin(
                xyz=(0.18 * x_sign, 0.0, 0.76),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=beacon_brass,
            name=f"lamp_mount_{'east' if x_sign > 0 else 'west'}",
        )
        beacon_carriage.visual(
            Cylinder(radius=0.04, length=0.12),
            origin=Origin(
                xyz=(0.26 * x_sign, 0.0, 0.78),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=iron_black,
            name=f"burner_{'east' if x_sign > 0 else 'west'}",
        )
        beacon_carriage.visual(
            Sphere(radius=0.08),
            origin=Origin(xyz=(0.22 * x_sign, 0.0, 0.78)),
            material=lamp_glow,
            name=f"lamp_globe_{'east' if x_sign > 0 else 'west'}",
        )

    beacon_carriage.visual(
        Cylinder(radius=0.10, length=0.18),
        origin=Origin(xyz=(0.43, 0.0, 0.19), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_black,
        name="motor_box",
    )
    beacon_carriage.inertial = Inertial.from_geometry(
        Box((1.10, 0.72, 1.42)),
        mass=850.0,
        origin=Origin(xyz=(0.0, 0.0, 0.71)),
    )

    model.articulation(
        "shaft_to_beacon",
        ArticulationType.CONTINUOUS,
        parent=central_shaft,
        child=beacon_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2400.0, velocity=0.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower_top = object_model.get_part("tower_top")
    central_shaft = object_model.get_part("central_shaft")
    beacon_carriage = object_model.get_part("beacon_carriage")
    shaft_to_beacon = object_model.get_articulation("shaft_to_beacon")

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

    ctx.fail_if_isolated_parts(max_pose_samples=12, name="beacon_spin_no_floating")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=12,
        ignore_adjacent=False,
        ignore_fixed=False,
    )

    ctx.expect_contact(
        central_shaft,
        tower_top,
        elem_a="pedestal_base",
        elem_b="gallery_deck",
        contact_tol=1e-4,
        name="shaft_seated_on_gallery_deck",
    )
    ctx.expect_contact(
        beacon_carriage,
        central_shaft,
        elem_a="turntable_ring",
        elem_b="pedestal_base",
        contact_tol=1e-4,
        name="turntable_seated_on_pedestal",
    )
    ctx.expect_origin_distance(
        beacon_carriage,
        central_shaft,
        axes="xy",
        max_dist=1e-6,
        name="beacon_coaxial_with_shaft",
    )
    ctx.expect_gap(
        tower_top,
        beacon_carriage,
        axis="z",
        positive_elem="roof_shell",
        negative_elem="top_ring",
        min_gap=0.35,
        name="beacon_clear_of_roof",
    )

    limits = shaft_to_beacon.motion_limits
    ctx.check(
        "shaft_to_beacon_is_vertical_continuous",
        shaft_to_beacon.joint_type == ArticulationType.CONTINUOUS
        and tuple(shaft_to_beacon.axis) == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=(
            f"type={shaft_to_beacon.joint_type}, axis={shaft_to_beacon.axis}, "
            f"limits=({None if limits is None else limits.lower}, {None if limits is None else limits.upper})"
        ),
    )

    with ctx.pose({shaft_to_beacon: 0.0}):
        ctx.expect_gap(
            tower_top,
            beacon_carriage,
            axis="x",
            positive_elem="glass_panel_00",
            negative_elem="motor_box",
            min_gap=0.45,
            name="motor_clear_of_east_glass",
        )
        ctx.expect_contact(
            beacon_carriage,
            central_shaft,
            elem_a="turntable_ring",
            elem_b="pedestal_base",
            contact_tol=1e-4,
            name="turntable_contact_at_zero_pose",
        )

    with ctx.pose({shaft_to_beacon: math.pi / 2.0}):
        ctx.expect_gap(
            tower_top,
            beacon_carriage,
            axis="y",
            positive_elem="glass_panel_03",
            negative_elem="motor_box",
            min_gap=0.45,
            name="motor_clear_of_north_glass",
        )
        ctx.expect_contact(
            beacon_carriage,
            central_shaft,
            elem_a="turntable_ring",
            elem_b="pedestal_base",
            contact_tol=1e-4,
            name="turntable_contact_at_quarter_pose",
        )

    motor_rest_aabb = ctx.part_element_world_aabb(beacon_carriage, elem="motor_box")
    with ctx.pose({shaft_to_beacon: math.pi / 2.0}):
        motor_quarter_aabb = ctx.part_element_world_aabb(beacon_carriage, elem="motor_box")

    ok_rotation = False
    details = "motor box AABB unavailable"
    if motor_rest_aabb is not None and motor_quarter_aabb is not None:
        motor_rest = _aabb_center(motor_rest_aabb)
        motor_quarter = _aabb_center(motor_quarter_aabb)
        ok_rotation = (
            motor_rest[0] > 0.30
            and abs(motor_rest[1]) < 0.05
            and motor_quarter[1] > 0.30
            and abs(motor_quarter[0]) < 0.05
        )
        details = (
            f"motor center at zero pose={motor_rest}, "
            f"quarter pose={motor_quarter}"
        )
    ctx.check("motor_box_rotates_around_vertical_axis", ok_rotation, details=details)

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

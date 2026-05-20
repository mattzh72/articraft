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
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _ring_mesh(name: str, *, radius: float, tube: float):
    return mesh_from_geometry(
        TorusGeometry(
            radius=radius,
            tube=tube,
            radial_segments=16,
            tubular_segments=56,
        ).rotate_y(math.pi / 2.0),
        name,
    )


def _spinner_mesh():
    return mesh_from_geometry(
        ConeGeometry(radius=0.017, height=0.024, radial_segments=28, closed=True)
        .rotate_y(-math.pi / 2.0)
        .translate(0.030, 0.0, 0.0),
        "fan_spinner",
    )


def _rotor_blade_mesh():
    return mesh_from_geometry(
        section_loft(
            [
                [
                    (-0.0045, -0.015, -0.0010),
                    (-0.0015, -0.015, 0.0010),
                    (0.0040, -0.015, 0.0005),
                    (0.0025, -0.015, -0.0012),
                ],
                [
                    (-0.0038, 0.000, -0.0009),
                    (-0.0010, 0.000, 0.0009),
                    (0.0036, 0.000, 0.0005),
                    (0.0022, 0.000, -0.0010),
                ],
                [
                    (-0.0028, 0.015, -0.0006),
                    (-0.0006, 0.015, 0.0006),
                    (0.0026, 0.015, 0.0003),
                    (0.0015, 0.015, -0.0007),
                ],
            ]
        ),
        "fan_rotor_blade",
    )


def _spoke_origin(x_pos: float, angle: float, *, radius_start: float, radius_end: float) -> Origin:
    length = radius_end - radius_start
    radius_mid = 0.5 * (radius_start + radius_end)
    return Origin(
        xyz=(x_pos, radius_mid * math.cos(angle), radius_mid * math.sin(angle)),
        rpy=(angle - math.pi / 2.0, 0.0, 0.0),
    )


def _axial_rod_origin(radius: float, angle: float, *, x_center: float) -> Origin:
    return Origin(
        xyz=(x_center, radius * math.cos(angle), radius * math.sin(angle)),
        rpy=(0.0, math.pi / 2.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_tilting_fan")

    shell_white = model.material("shell_white", rgba=(0.92, 0.92, 0.90, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    dark_charcoal = model.material("dark_charcoal", rgba=(0.17, 0.18, 0.20, 1.0))
    blade_gray = model.material("blade_gray", rgba=(0.78, 0.80, 0.82, 0.96))

    base_shell_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.190, 0.140, 0.028), 0.022),
        "fan_base_shell",
    )
    ring_outer_mesh = _ring_mesh("guard_outer_ring", radius=0.082, tube=0.0035)
    spinner_mesh = _spinner_mesh()
    rotor_blade_mesh = _rotor_blade_mesh()

    base = model.part("base")
    base.visual(
        base_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=shell_white,
        name="base_shell",
    )
    base.visual(
        Box((0.078, 0.088, 0.050)),
        origin=Origin(xyz=(-0.040, 0.0, 0.047)),
        material=shell_white,
        name="control_pod",
    )
    base.visual(
        Box((0.040, 0.062, 0.050)),
        origin=Origin(xyz=(-0.020, 0.0, 0.097)),
        material=shell_white,
        name="pivot_bridge",
    )
    base.visual(
        Box((0.050, 0.110, 0.024)),
        origin=Origin(xyz=(-0.020, 0.062, 0.106)),
        material=shell_white,
        name="left_arm",
    )
    base.visual(
        Box((0.050, 0.110, 0.024)),
        origin=Origin(xyz=(-0.020, -0.062, 0.106)),
        material=shell_white,
        name="right_arm",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.0, 0.114, 0.130), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_gray,
        name="left_collar",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.0, -0.114, 0.130), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_gray,
        name="right_collar",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.190, 0.140, 0.150)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.042, length=0.056),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_charcoal,
        name="motor_can",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.014),
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="spindle_housing",
    )
    head.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(0.084, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_charcoal,
        name="spindle",
    )
    head.visual(
        Box((0.030, 0.162, 0.028)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=warm_gray,
        name="axle_bridge",
    )
    head.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(0.0, 0.093, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_gray,
        name="left_trunnion",
    )
    head.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(0.0, -0.093, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_gray,
        name="right_trunnion",
    )

    for visual_name, x_pos, mesh in (
        ("rear_outer_ring", 0.072, ring_outer_mesh),
        ("front_outer_ring", 0.128, ring_outer_mesh),
    ):
        head.visual(
            mesh,
            origin=Origin(xyz=(x_pos, 0.0, 0.0)),
            material=warm_gray,
            name=visual_name,
        )

    front_angles = [index * math.tau / 6.0 for index in range(6)]
    for index, angle in enumerate(front_angles):
        head.visual(
            Cylinder(radius=0.0026, length=0.064),
            origin=_spoke_origin(0.128, angle, radius_start=0.018, radius_end=0.082),
            material=warm_gray,
            name=f"front_spoke_{index}",
        )

    rear_angles = [math.pi / 4.0 + index * math.tau / 4.0 for index in range(4)]
    for index, angle in enumerate(rear_angles):
        head.visual(
            Cylinder(radius=0.0028, length=0.044),
            origin=_spoke_origin(0.072, angle, radius_start=0.038, radius_end=0.082),
            material=warm_gray,
            name=f"rear_spoke_{index}",
        )

    rod_angles = [math.pi / 6.0 + index * math.tau / 6.0 for index in range(6)]
    for index, angle in enumerate(rod_angles):
        head.visual(
            Cylinder(radius=0.0028, length=0.056),
            origin=_axial_rod_origin(0.082, angle, x_center=0.098),
            material=warm_gray,
            name=f"cage_rod_{index}",
        )

    head.inertial = Inertial.from_geometry(
        Box((0.240, 0.210, 0.180)),
        mass=0.95,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_charcoal,
        name="hub_shell",
    )
    rotor.visual(spinner_mesh, material=warm_gray, name="spinner")
    blade_angles = [index * math.tau / 5.0 for index in range(5)]
    for index, angle in enumerate(blade_angles):
        rotor.visual(
            rotor_blade_mesh,
            origin=Origin(
                xyz=(0.010, 0.024 * math.cos(angle), 0.024 * math.sin(angle)),
                rpy=(angle, 0.16, 0.0),
            ),
            material=blade_gray,
            name=f"blade_{index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.042, length=0.024),
        mass=0.18,
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.5,
            lower=-math.radians(8.0),
            upper=math.radians(25.0),
        ),
    )

    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.094, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=24.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("base_to_head")
    spin = object_model.get_articulation("head_to_rotor")

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
    ctx.expect_contact(
        head,
        base,
        elem_a="left_trunnion",
        elem_b="left_collar",
        contact_tol=0.0005,
        name="left_side_pivot_is_seated",
    )
    ctx.expect_contact(
        head,
        base,
        elem_a="right_trunnion",
        elem_b="right_collar",
        contact_tol=0.0005,
        name="right_side_pivot_is_seated",
    )
    ctx.expect_contact(
        rotor,
        head,
        elem_a="hub_shell",
        elem_b="spindle",
        contact_tol=0.0005,
        name="rotor_hub_contacts_spindle",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="yz",
        inner_elem="blade_0",
        outer_elem="front_outer_ring",
        margin=0.0,
        name="blade_tip_clearance_inside_guard",
    )
    ctx.expect_gap(
        rotor,
        head,
        axis="x",
        positive_elem="blade_0",
        negative_elem="motor_can",
        min_gap=0.010,
        name="blade_plane_clears_motor_can",
    )
    ctx.expect_gap(
        head,
        rotor,
        axis="x",
        positive_elem="front_outer_ring",
        negative_elem="blade_0",
        min_gap=0.010,
        name="front_guard_sits_ahead_of_blades",
    )
    ctx.expect_gap(
        rotor,
        head,
        axis="x",
        positive_elem="blade_0",
        negative_elem="rear_outer_ring",
        min_gap=0.008,
        name="rear_guard_sits_behind_blades",
    )

    stowed_ring = None
    neutral_ring = None
    with ctx.pose({tilt: tilt.motion_limits.lower}):
        stowed_ring = ctx.part_element_world_aabb(head, elem="front_outer_ring")
    with ctx.pose({tilt: 0.0}):
        neutral_ring = ctx.part_element_world_aabb(head, elem="front_outer_ring")
    if stowed_ring is None or neutral_ring is None:
        ctx.fail("stow_pose_compacts_height", "Could not resolve front ring AABBs for stow check.")
    else:
        ctx.check(
            "stow_pose_compacts_height",
            stowed_ring[1][2] < neutral_ring[1][2] - 0.008,
            details=(
                f"stowed_max_z={stowed_ring[1][2]:.4f}, "
                f"neutral_max_z={neutral_ring[1][2]:.4f}"
            ),
        )

    rest_aabb = None
    raised_aabb = None
    with ctx.pose({tilt: 0.0}):
        rest_aabb = ctx.part_element_world_aabb(head, elem="front_outer_ring")
    with ctx.pose({tilt: math.radians(20.0)}):
        raised_aabb = ctx.part_element_world_aabb(head, elem="front_outer_ring")
    if rest_aabb is None or raised_aabb is None:
        ctx.fail("tilt_positive_lifts_head", "Could not resolve front guard AABBs for tilt check.")
    else:
        ctx.check(
            "tilt_positive_lifts_head",
            raised_aabb[1][2] > rest_aabb[1][2] + 0.015,
            details=(
                f"rest_max_z={rest_aabb[1][2]:.4f}, "
                f"raised_max_z={raised_aabb[1][2]:.4f}"
            ),
        )

    with ctx.pose({spin: math.pi / 5.0}):
        ctx.expect_within(
            rotor,
            head,
            axes="yz",
            inner_elem="blade_2",
            outer_elem="front_outer_ring",
            margin=0.0,
            name="rotor_stays_guarded_in_spin_pose",
        )
        ctx.expect_gap(
            head,
            rotor,
            axis="x",
            positive_elem="front_outer_ring",
            negative_elem="blade_2",
            min_gap=0.010,
            name="spin_pose_keeps_front_clearance",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

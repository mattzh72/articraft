from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rect_loop_yz(x_pos: float, width: float, height: float, z_center: float) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (x_pos, -half_w, z_center - half_h),
        (x_pos, half_w, z_center - half_h),
        (x_pos, half_w, z_center + half_h),
        (x_pos, -half_w, z_center + half_h),
    ]


def _loop_on_y(profile_xz: list[tuple[float, float]], y_pos: float) -> list[tuple[float, float, float]]:
    return [(x_pos, y_pos, z_pos) for x_pos, z_pos in profile_xz]


def _build_cheek_mesh(*, thickness: float) -> object:
    side_profile = [
        (-0.30, 0.64),
        (-0.24, 0.79),
        (-0.18, 0.92),
        (0.02, 0.97),
        (0.28, 0.95),
        (0.52, 0.89),
        (0.60, 0.76),
        (0.46, 0.68),
        (0.10, 0.66),
        (-0.14, 0.62),
    ]
    cheek_geom = section_loft(
        [
            _loop_on_y(side_profile, -thickness * 0.5),
            _loop_on_y(side_profile, thickness * 0.5),
        ]
    )
    return _save_mesh("carriage_cheek", cheek_geom)


def _build_trail_mesh() -> object:
    trail_geom = section_loft(
        [
            _rect_loop_yz(-0.26, 0.18, 0.18, 0.63),
            _rect_loop_yz(-0.72, 0.12, 0.14, 0.48),
            _rect_loop_yz(-1.18, 0.09, 0.10, 0.35),
        ]
    )
    return _save_mesh("trail_beam", trail_geom)


def _build_wheel_wood_mesh(*, wheel_radius: float, width: float, spoke_count: int) -> object:
    wood_geom = TorusGeometry(
        radius=wheel_radius - 0.08,
        tube=0.050,
        radial_segments=16,
        tubular_segments=64,
    ).rotate_x(pi / 2.0)
    wood_geom.merge(
        TorusGeometry(
            radius=0.110,
            tube=0.026,
            radial_segments=12,
            tubular_segments=36,
        ).rotate_x(pi / 2.0)
    )

    spoke_length = 0.39
    for spoke_index in range(spoke_count):
        angle = (2.0 * pi * spoke_index) / spoke_count
        spoke = BoxGeometry((0.030, width * 0.62, spoke_length))
        spoke.translate(0.0, 0.0, 0.31)
        spoke.rotate_y(angle)
        wood_geom.merge(spoke)

    return _save_mesh("wheel_wood", wood_geom)


def _build_barrel_mesh() -> object:
    outer_profile = [
        (0.095, -0.38),
        (0.108, -0.26),
        (0.106, -0.08),
        (0.100, 0.12),
        (0.089, 0.42),
        (0.079, 0.74),
        (0.073, 0.92),
        (0.081, 1.02),
        (0.077, 1.06),
    ]
    inner_profile = [
        (0.0, -0.38),
        (0.018, -0.22),
        (0.033, -0.08),
        (0.038, 0.26),
        (0.039, 1.06),
    ]
    barrel_geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    ).rotate_y(pi / 2.0)
    return _save_mesh("barrel_shell", barrel_geom)


def _build_torus_mesh(name: str, *, radius: float, tube: float, rotate_x_angle: float = 0.0, rotate_y_angle: float = 0.0):
    geom = TorusGeometry(radius=radius, tube=tube, radial_segments=12, tubular_segments=64)
    if rotate_x_angle:
        geom.rotate_x(rotate_x_angle)
    if rotate_y_angle:
        geom.rotate_y(rotate_y_angle)
    return _save_mesh(name, geom)


def _build_hub_cap_mesh(name: str) -> object:
    return _save_mesh(
        name,
        ConeGeometry(radius=0.060, height=0.05, radial_segments=20, closed=True).rotate_x(pi / 2.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swedish_regimental_three_pounder")

    carriage_wood = model.material("carriage_wood", rgba=(0.62, 0.46, 0.25, 1.0))
    wheel_wood = model.material("wheel_wood", rgba=(0.56, 0.40, 0.20, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.18, 0.18, 0.19, 1.0))
    iron_tire = model.material("iron_tire", rgba=(0.12, 0.12, 0.13, 1.0))

    cheek_mesh = _build_cheek_mesh(thickness=0.05)
    trail_mesh = _build_trail_mesh()
    wheel_wood_mesh = _build_wheel_wood_mesh(wheel_radius=0.62, width=0.12, spoke_count=12)
    barrel_mesh = _build_barrel_mesh()
    reinforce_ring_mesh = _build_torus_mesh("reinforce_ring", radius=0.102, tube=0.007, rotate_y_angle=pi / 2.0)
    muzzle_ring_mesh = _build_torus_mesh("muzzle_ring", radius=0.078, tube=0.005, rotate_y_angle=pi / 2.0)
    wheel_tire_mesh = _build_torus_mesh("wheel_tire", radius=0.604, tube=0.015, rotate_x_angle=pi / 2.0)
    hub_cap_mesh = _build_hub_cap_mesh("hub_cap")

    carriage = model.part("carriage")
    carriage.visual(cheek_mesh, origin=Origin(xyz=(0.0, 0.265, 0.0)), material=carriage_wood, name="left_cheek")
    carriage.visual(cheek_mesh, origin=Origin(xyz=(0.0, -0.265, 0.0)), material=carriage_wood, name="right_cheek")
    carriage.visual(trail_mesh, material=carriage_wood, name="trail_beam")
    carriage.visual(
        Box((0.22, 0.50, 0.18)),
        origin=Origin(xyz=(-0.18, 0.0, 0.71)),
        material=carriage_wood,
        name="rear_transom",
    )
    carriage.visual(
        Box((0.16, 0.50, 0.14)),
        origin=Origin(xyz=(0.18, 0.0, 0.74)),
        material=carriage_wood,
        name="front_transom",
    )
    carriage.visual(
        Box((0.16, 0.22, 0.05)),
        origin=Origin(xyz=(-0.10, 0.0, 0.785)),
        material=carriage_wood,
        name="quoin_block",
    )
    carriage.visual(
        Cylinder(radius=0.050, length=1.15),
        origin=Origin(xyz=(-0.02, 0.0, 0.62), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="axle",
    )
    carriage.visual(
        Cylinder(radius=0.028, length=0.16),
        origin=Origin(xyz=(-1.18, 0.0, 0.34), rpy=(pi / 2.0, 0.0, 0.0)),
        material=carriage_wood,
        name="trail_handle",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((2.00, 1.20, 0.70)),
        mass=260.0,
        origin=Origin(xyz=(-0.30, 0.0, 0.58)),
    )

    barrel = model.part("barrel")
    barrel.visual(barrel_mesh, material=dark_iron, name="barrel_shell")
    barrel.visual(
        Cylinder(radius=0.032, length=0.48),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="trunnion_axle",
    )
    barrel.visual(
        Sphere(radius=0.036),
        origin=Origin(xyz=(-0.468, 0.0, 0.0)),
        material=dark_iron,
        name="cascabel_knob",
    )
    barrel.visual(
        Cylinder(radius=0.024, length=0.11),
        origin=Origin(xyz=(-0.385, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_iron,
        name="cascabel_neck",
    )
    barrel.visual(
        reinforce_ring_mesh,
        origin=Origin(xyz=(-0.20, 0.0, 0.0)),
        material=dark_iron,
        name="rear_reinforce_ring",
    )
    barrel.visual(
        muzzle_ring_mesh,
        origin=Origin(xyz=(0.92, 0.0, 0.0)),
        material=dark_iron,
        name="muzzle_astragal",
    )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.095, length=1.52),
        mass=155.0,
        origin=Origin(xyz=(0.26, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )

    left_wheel = model.part("left_wheel")
    left_wheel.visual(wheel_wood_mesh, material=wheel_wood, name="left_wheel_wood")
    left_wheel.visual(
        wheel_tire_mesh,
        material=iron_tire,
        name="left_wheel_tire",
    )
    left_wheel.visual(
        Cylinder(radius=0.108, length=0.14),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="left_hub_band",
    )
    left_wheel.visual(
        hub_cap_mesh,
        origin=Origin(xyz=(0.0, 0.085, 0.0)),
        material=dark_iron,
        name="left_outer_hub_cap",
    )
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.62, length=0.12),
        mass=48.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(wheel_wood_mesh, material=wheel_wood, name="right_wheel_wood")
    right_wheel.visual(
        wheel_tire_mesh,
        material=iron_tire,
        name="right_wheel_tire",
    )
    right_wheel.visual(
        Cylinder(radius=0.108, length=0.14),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="right_hub_band",
    )
    right_wheel.visual(
        hub_cap_mesh,
        origin=Origin(xyz=(0.0, -0.085, 0.0)),
        material=dark_iron,
        name="right_outer_hub_cap",
    )
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.62, length=0.12),
        mass=48.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "barrel_elevation",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(0.05, 0.0, 0.94)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.8, lower=-0.08, upper=0.42),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=left_wheel,
        origin=Origin(xyz=(-0.02, 0.635, 0.62)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=right_wheel,
        origin=Origin(xyz=(-0.02, -0.635, 0.62)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    barrel_elevation = object_model.get_articulation("barrel_elevation")
    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")

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
    ctx.allow_overlap(
        carriage,
        left_wheel,
        elem_a="axle",
        elem_b="left_hub_band",
        reason="The iron axle passes through the left wheel hub band as on the real gun carriage.",
    )
    ctx.allow_overlap(
        carriage,
        right_wheel,
        elem_a="axle",
        elem_b="right_hub_band",
        reason="The iron axle passes through the right wheel hub band as on the real gun carriage.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(barrel, carriage, name="barrel_supported_on_trunnions")
    ctx.expect_contact(left_wheel, carriage, name="left_wheel_seated_on_axle")
    ctx.expect_contact(right_wheel, carriage, name="right_wheel_seated_on_axle")
    ctx.expect_origin_distance(
        left_wheel,
        right_wheel,
        axes="y",
        min_dist=1.24,
        max_dist=1.30,
        name="wheel_track_width",
    )
    ctx.check(
        "barrel_joint_axis_is_lateral",
        barrel_elevation.axis == (0.0, 1.0, 0.0),
        f"Unexpected barrel axis: {barrel_elevation.axis}",
    )
    ctx.check(
        "wheel_joint_axes_are_lateral",
        left_wheel_spin.axis == (0.0, 1.0, 0.0) and right_wheel_spin.axis == (0.0, 1.0, 0.0),
        f"Unexpected wheel axes: left={left_wheel_spin.axis}, right={right_wheel_spin.axis}",
    )

    barrel_rest = ctx.part_world_aabb(barrel)
    assert barrel_rest is not None
    with ctx.pose({barrel_elevation: 0.30}):
        barrel_raised = ctx.part_world_aabb(barrel)
        assert barrel_raised is not None
        ctx.check(
            "barrel_muzzle_rises_when_elevated",
            barrel_raised[1][2] > barrel_rest[1][2] + 0.08,
            f"Rest max z={barrel_rest[1][2]:.3f}, raised max z={barrel_raised[1][2]:.3f}",
        )
        ctx.expect_contact(barrel, carriage, name="trunnions_remain_seated_when_elevated")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

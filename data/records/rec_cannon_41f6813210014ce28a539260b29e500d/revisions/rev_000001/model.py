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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_barrel_mesh():
    outer_profile = [
        (0.000, -0.145),
        (0.040, -0.140),
        (0.080, -0.118),
        (0.098, -0.080),
        (0.096, -0.020),
        (0.090, 0.040),
        (0.082, 0.180),
        (0.074, 0.360),
        (0.066, 0.560),
        (0.058, 0.710),
        (0.055, 0.770),
        (0.061, 0.810),
    ]
    inner_profile = [
        (0.000, -0.102),
        (0.024, -0.060),
        (0.028, 0.140),
        (0.031, 0.560),
        (0.032, 0.790),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    ).rotate_y(math.pi / 2.0)


def _build_cheek_mesh():
    profile = [
        (-0.290, 0.225),
        (-0.240, 0.220),
        (-0.165, 0.255),
        (-0.080, 0.355),
        (0.005, 0.480),
        (0.085, 0.610),
        (0.165, 0.690),
        (0.020, 0.710),
        (-0.135, 0.610),
        (-0.240, 0.460),
    ]
    return ExtrudeGeometry(profile, 0.040, center=True)


def _build_trail_beam_mesh():
    return sweep_profile_along_spline(
        [
            (-0.105, 0.000, 0.000),
            (-0.265, 0.000, -0.010),
            (-0.545, 0.000, -0.045),
            (-0.915, 0.000, -0.125),
            (-1.115, 0.000, -0.180),
        ],
        profile=rounded_rect_profile(0.050, 0.092, radius=0.010, corner_segments=6),
        samples_per_segment=16,
        cap_profile=True,
    )


def _build_wheel_rim_mesh(radius: float, width: float):
    half_width = width * 0.5
    profile = [
        (radius * 0.770, -half_width * 0.88),
        (radius * 0.840, -half_width),
        (radius * 0.935, -half_width * 0.88),
        (radius * 0.985, -half_width * 0.48),
        (radius, -half_width * 0.14),
        (radius, half_width * 0.14),
        (radius * 0.985, half_width * 0.48),
        (radius * 0.935, half_width * 0.88),
        (radius * 0.840, half_width),
        (radius * 0.770, half_width * 0.88),
        (radius * 0.715, half_width * 0.24),
        (radius * 0.700, 0.000),
        (radius * 0.715, -half_width * 0.24),
        (radius * 0.770, -half_width * 0.88),
    ]
    return LatheGeometry(profile, segments=72).rotate_x(math.pi / 2.0)


def _add_spoked_wheel(
    part,
    *,
    mesh_name: str,
    radius: float,
    width: float,
    tire_material,
    wood_material,
    hub_material,
) -> None:
    part.visual(
        _mesh(mesh_name, _build_wheel_rim_mesh(radius, width)),
        material=tire_material,
        name="rim_tire",
    )
    part.visual(
        Cylinder(radius=0.082, length=0.072),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_material,
        name="hub_shell",
    )
    part.visual(
        Cylinder(radius=0.056, length=0.104),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.028, length=0.094),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tire_material,
        name="axle_cap",
    )
    spoke_count = 10
    spoke_radius = 0.176
    spoke_length = 0.244
    for index in range(spoke_count):
        angle = (math.tau * index) / spoke_count
        part.visual(
            Box((spoke_length, 0.020, 0.018)),
            origin=Origin(
                xyz=(spoke_radius * math.cos(angle), 0.0, spoke_radius * math.sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=wood_material,
            name=f"spoke_{index:02d}",
        )


def _aabb_center_y(aabb) -> float | None:
    if aabb is None:
        return None
    return (aabb[0][1] + aabb[1][1]) * 0.5


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mountain_pack_cannon")

    bronze = model.material("bronze", rgba=(0.70, 0.47, 0.20, 1.0))
    bronze_dark = model.material("bronze_dark", rgba=(0.50, 0.31, 0.13, 1.0))
    carriage_green = model.material("carriage_green", rgba=(0.34, 0.40, 0.26, 1.0))
    carriage_dark = model.material("carriage_dark", rgba=(0.18, 0.21, 0.16, 1.0))
    wheel_tire = model.material("wheel_tire", rgba=(0.14, 0.14, 0.14, 1.0))
    wheel_wood = model.material("wheel_wood", rgba=(0.56, 0.42, 0.25, 1.0))
    metal = model.material("metal", rgba=(0.46, 0.48, 0.49, 1.0))

    carriage = model.part("carriage")
    carriage.inertial = Inertial.from_geometry(
        Box((1.35, 1.15, 0.82)),
        mass=220.0,
        origin=Origin(xyz=(-0.18, 0.0, 0.41)),
    )

    cheek_mesh = _mesh("cannon_cheek", _build_cheek_mesh())
    carriage.visual(
        cheek_mesh,
        origin=Origin(xyz=(0.0, 0.140, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=carriage_green,
        name="left_cheek",
    )
    carriage.visual(
        cheek_mesh,
        origin=Origin(xyz=(0.0, -0.140, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=carriage_green,
        name="right_cheek",
    )
    carriage.visual(
        Cylinder(radius=0.052, length=0.820),
        origin=Origin(xyz=(-0.025, 0.0, 0.360), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=carriage_dark,
        name="axle_beam",
    )
    carriage.visual(
        Box((0.220, 0.240, 0.082)),
        origin=Origin(xyz=(0.015, 0.0, 0.418)),
        material=carriage_green,
        name="cradle_block",
    )
    carriage.visual(
        Box((0.120, 0.240, 0.160)),
        origin=Origin(xyz=(-0.230, 0.0, 0.290)),
        material=carriage_green,
        name="trail_transom",
    )
    carriage.visual(
        Box((0.180, 0.260, 0.070)),
        origin=Origin(xyz=(-0.115, 0.0, 0.405)),
        material=carriage_green,
        name="rear_bridge",
    )
    carriage.visual(
        Box((0.100, 0.040, 0.060)),
        origin=Origin(xyz=(-0.220, 0.145, 0.235)),
        material=carriage_dark,
        name="left_hinge_block",
    )
    carriage.visual(
        Box((0.100, 0.040, 0.060)),
        origin=Origin(xyz=(-0.220, -0.145, 0.235)),
        material=carriage_dark,
        name="right_hinge_block",
    )
    barrel = model.part("barrel")
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.090, length=0.980),
        mass=140.0,
        origin=Origin(xyz=(0.320, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    barrel.visual(
        _mesh("pack_cannon_barrel", _build_barrel_mesh()),
        material=bronze,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.052, length=0.240),
        origin=Origin(xyz=(0.060, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bronze_dark,
        name="trunnion_shaft",
    )
    barrel.visual(
        Cylinder(radius=0.072, length=0.055),
        origin=Origin(xyz=(-0.120, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bronze_dark,
        name="breech_face",
    )
    barrel.visual(
        Sphere(radius=0.030),
        origin=Origin(xyz=(-0.158, 0.0, 0.0)),
        material=bronze_dark,
        name="cascabel",
    )

    left_trail = model.part("left_trail")
    left_trail.inertial = Inertial.from_geometry(
        Box((1.18, 0.14, 0.26)),
        mass=34.0,
        origin=Origin(xyz=(-0.55, 0.0, -0.08)),
    )
    left_trail.visual(
        Box((0.090, 0.060, 0.060)),
        origin=Origin(xyz=(-0.054, 0.0, 0.000)),
        material=carriage_dark,
        name="hinge_knuckle",
    )
    left_trail.visual(
        Box((0.440, 0.082, 0.082)),
        origin=Origin(xyz=(-0.310, 0.0, -0.035)),
        material=carriage_green,
        name="front_beam",
    )
    left_trail.visual(
        Box((0.700, 0.062, 0.070)),
        origin=Origin(xyz=(-0.880, 0.0, -0.110)),
        material=carriage_green,
        name="rear_beam",
    )
    left_trail.visual(
        Box((0.180, 0.060, 0.120)),
        origin=Origin(xyz=(-1.070, 0.0, -0.195)),
        material=metal,
        name="spade",
    )

    right_trail = model.part("right_trail")
    right_trail.inertial = Inertial.from_geometry(
        Box((1.18, 0.14, 0.26)),
        mass=34.0,
        origin=Origin(xyz=(-0.55, 0.0, -0.08)),
    )
    right_trail.visual(
        Box((0.090, 0.060, 0.060)),
        origin=Origin(xyz=(-0.054, 0.0, 0.000)),
        material=carriage_dark,
        name="hinge_knuckle",
    )
    right_trail.visual(
        Box((0.440, 0.082, 0.082)),
        origin=Origin(xyz=(-0.310, 0.0, -0.035)),
        material=carriage_green,
        name="front_beam",
    )
    right_trail.visual(
        Box((0.700, 0.062, 0.070)),
        origin=Origin(xyz=(-0.880, 0.0, -0.110)),
        material=carriage_green,
        name="rear_beam",
    )
    right_trail.visual(
        Box((0.180, 0.060, 0.120)),
        origin=Origin(xyz=(-1.070, 0.0, -0.195)),
        material=metal,
        name="spade",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.360, length=0.100),
        mass=28.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _add_spoked_wheel(
        left_wheel,
        mesh_name="left_spoked_wheel",
        radius=0.360,
        width=0.082,
        tire_material=wheel_tire,
        wood_material=wheel_wood,
        hub_material=carriage_green,
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.360, length=0.100),
        mass=28.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _add_spoked_wheel(
        right_wheel,
        mesh_name="right_spoked_wheel",
        radius=0.360,
        width=0.082,
        tire_material=wheel_tire,
        wood_material=wheel_wood,
        hub_material=carriage_green,
    )

    model.articulation(
        "carriage_to_barrel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(0.060, 0.0, 0.612), rpy=(0.0, math.radians(7.0), 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3200.0,
            velocity=0.9,
            lower=math.radians(-3.0),
            upper=math.radians(38.0),
        ),
    )
    model.articulation(
        "carriage_to_left_trail",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=left_trail,
        origin=Origin(xyz=(-0.290, 0.080, 0.240), rpy=(0.0, 0.0, -math.radians(18.0))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(18.0),
        ),
    )
    model.articulation(
        "carriage_to_right_trail",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=right_trail,
        origin=Origin(xyz=(-0.290, -0.080, 0.240), rpy=(0.0, 0.0, math.radians(18.0))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=1.2,
            lower=-math.radians(18.0),
            upper=0.0,
        ),
    )
    model.articulation(
        "carriage_to_left_wheel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=left_wheel,
        origin=Origin(xyz=(-0.025, 0.460, 0.360)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=12.0,
            lower=-math.tau,
            upper=math.tau,
        ),
    )
    model.articulation(
        "carriage_to_right_wheel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=right_wheel,
        origin=Origin(xyz=(-0.025, -0.460, 0.360)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=12.0,
            lower=-math.tau,
            upper=math.tau,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    left_trail = object_model.get_part("left_trail")
    right_trail = object_model.get_part("right_trail")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")

    barrel_joint = object_model.get_articulation("carriage_to_barrel")
    left_trail_joint = object_model.get_articulation("carriage_to_left_trail")
    right_trail_joint = object_model.get_articulation("carriage_to_right_trail")
    left_wheel_joint = object_model.get_articulation("carriage_to_left_wheel")
    right_wheel_joint = object_model.get_articulation("carriage_to_right_wheel")

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
        barrel,
        carriage,
        elem_a="trunnion_shaft",
        elem_b="left_cheek",
        contact_tol=0.002,
        name="barrel trunnion bears on left cheek",
    )
    ctx.expect_contact(
        left_trail,
        carriage,
        elem_a="hinge_knuckle",
        elem_b="trail_transom",
        contact_tol=0.002,
        name="left trail is hinged against transom",
    )
    ctx.expect_contact(
        right_trail,
        carriage,
        elem_a="hinge_knuckle",
        elem_b="trail_transom",
        contact_tol=0.002,
        name="right trail is hinged against transom",
    )
    ctx.expect_contact(
        left_wheel,
        carriage,
        elem_a="hub",
        elem_b="axle_beam",
        contact_tol=0.002,
        name="left wheel hub seats on axle",
    )
    ctx.expect_contact(
        right_wheel,
        carriage,
        elem_a="hub",
        elem_b="axle_beam",
        contact_tol=0.002,
        name="right wheel hub seats on axle",
    )

    ctx.check(
        "barrel hinge axis pitches upward with positive motion",
        tuple(barrel_joint.axis) == (0.0, -1.0, 0.0),
        details=f"axis={barrel_joint.axis}",
    )
    ctx.check(
        "trail hinges yaw about vertical pins",
        tuple(left_trail_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(right_trail_joint.axis) == (0.0, 0.0, 1.0),
        details=f"left={left_trail_joint.axis}, right={right_trail_joint.axis}",
    )
    ctx.check(
        "wheel axles spin about the lateral axis",
        tuple(left_wheel_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(right_wheel_joint.axis) == (0.0, 1.0, 0.0),
        details=f"left={left_wheel_joint.axis}, right={right_wheel_joint.axis}",
    )

    rest_barrel = ctx.part_element_world_aabb(barrel, elem="barrel_shell")
    upper_barrel_limit = barrel_joint.motion_limits.upper if barrel_joint.motion_limits is not None else None
    with ctx.pose({barrel_joint: upper_barrel_limit if upper_barrel_limit is not None else 0.0}):
        raised_barrel = ctx.part_element_world_aabb(barrel, elem="barrel_shell")
    ctx.check(
        "barrel elevation raises the muzzle",
        rest_barrel is not None
        and raised_barrel is not None
        and raised_barrel[1][2] > rest_barrel[1][2] + 0.10,
        details=f"rest={rest_barrel}, raised={raised_barrel}",
    )

    left_rest = ctx.part_world_aabb(left_trail)
    left_upper = left_trail_joint.motion_limits.upper if left_trail_joint.motion_limits is not None else None
    with ctx.pose({left_trail_joint: left_upper if left_upper is not None else 0.0}):
        left_folded = ctx.part_world_aabb(left_trail)
    ctx.check(
        "left trail folds inward from the deployed split-trail stance",
        _aabb_center_y(left_rest) is not None
        and _aabb_center_y(left_folded) is not None
        and _aabb_center_y(left_folded) < _aabb_center_y(left_rest) - 0.08,
        details=f"rest={left_rest}, folded={left_folded}",
    )

    right_rest = ctx.part_world_aabb(right_trail)
    right_lower = right_trail_joint.motion_limits.lower if right_trail_joint.motion_limits is not None else None
    with ctx.pose({right_trail_joint: right_lower if right_lower is not None else 0.0}):
        right_folded = ctx.part_world_aabb(right_trail)
    ctx.check(
        "right trail folds inward from the deployed split-trail stance",
        _aabb_center_y(right_rest) is not None
        and _aabb_center_y(right_folded) is not None
        and _aabb_center_y(right_folded) > _aabb_center_y(right_rest) + 0.08,
        details=f"rest={right_rest}, folded={right_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

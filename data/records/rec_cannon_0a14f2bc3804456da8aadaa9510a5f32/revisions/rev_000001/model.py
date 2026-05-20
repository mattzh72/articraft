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
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_barrel_shell():
    outer_profile = [
        (0.050, -0.98),
        (0.090, -0.93),
        (0.062, -0.87),
        (0.168, -0.82),
        (0.282, -0.72),
        (0.312, -0.54),
        (0.302, -0.22),
        (0.286, 0.08),
        (0.264, 0.44),
        (0.234, 1.06),
        (0.206, 1.66),
        (0.186, 1.96),
        (0.222, 2.05),
        (0.184, 2.12),
    ]
    inner_profile = [
        (0.000, -0.82),
        (0.024, -0.74),
        (0.082, -0.66),
        (0.100, -0.56),
        (0.104, 1.98),
        (0.098, 2.08),
    ]
    return _save_mesh(
        "victorian_garrison_cannon_barrel_shell",
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=80,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ).rotate_y(math.pi / 2.0),
    )


def _build_solid_wheel_mesh(name: str):
    profile = [
        (0.000, -0.11),
        (0.082, -0.11),
        (0.118, -0.092),
        (0.248, -0.074),
        (0.470, -0.042),
        (0.588, -0.018),
        (0.618, -0.006),
        (0.622, 0.000),
        (0.618, 0.006),
        (0.588, 0.018),
        (0.470, 0.042),
        (0.248, 0.074),
        (0.118, 0.092),
        (0.082, 0.11),
        (0.000, 0.11),
    ]
    return _save_mesh(
        name,
        LatheGeometry(profile, segments=72).rotate_x(math.pi / 2.0),
    )


def _build_handwheel_rim():
    return _save_mesh(
        "victorian_garrison_cannon_handwheel_rim",
        TorusGeometry(
            radius=0.155,
            tube=0.012,
            radial_segments=16,
            tubular_segments=42,
        ).rotate_x(math.pi / 2.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="victorian_coastal_garrison_cannon")

    timber = model.material("weathered_timber", rgba=(0.43, 0.33, 0.23, 1.0))
    timber_dark = model.material("timber_dark", rgba=(0.31, 0.24, 0.16, 1.0))
    iron_dark = model.material("iron_dark", rgba=(0.15, 0.16, 0.17, 1.0))
    iron_mid = model.material("iron_mid", rgba=(0.24, 0.25, 0.27, 1.0))
    iron_highlight = model.material("iron_highlight", rgba=(0.33, 0.34, 0.36, 1.0))

    barrel_shell_mesh = _build_barrel_shell()
    wheel_mesh = _build_solid_wheel_mesh("victorian_garrison_cannon_wheel")
    handwheel_rim_mesh = _build_handwheel_rim()

    carriage = model.part("carriage")
    carriage.visual(
        Box((1.95, 0.74, 0.18)),
        origin=Origin(xyz=(0.00, 0.00, 0.09)),
        material=timber,
        name="base_platform",
    )
    carriage.visual(
        Box((0.70, 0.46, 0.10)),
        origin=Origin(xyz=(-0.06, 0.00, 0.23)),
        material=timber_dark,
        name="center_step",
    )
    carriage.visual(
        Box((0.54, 0.22, 0.10)),
        origin=Origin(xyz=(0.00, 0.00, 0.33)),
        material=timber,
        name="axle_bed",
    )
    carriage.visual(
        Box((1.48, 0.10, 0.82)),
        origin=Origin(xyz=(0.05, 0.37, 0.55)),
        material=timber,
        name="left_cheek_lower",
    )
    carriage.visual(
        Box((1.48, 0.10, 0.82)),
        origin=Origin(xyz=(0.05, -0.37, 0.55)),
        material=timber,
        name="right_cheek_lower",
    )
    carriage.visual(
        Box((0.24, 0.10, 0.10)),
        origin=Origin(xyz=(-0.48, 0.37, 0.97)),
        material=timber,
        name="left_cheek_upper_rear",
    )
    carriage.visual(
        Box((0.24, 0.10, 0.10)),
        origin=Origin(xyz=(-0.48, -0.37, 0.97)),
        material=timber,
        name="right_cheek_upper_rear",
    )
    carriage.visual(
        Box((0.22, 0.10, 0.10)),
        origin=Origin(xyz=(0.56, 0.37, 0.97)),
        material=timber,
        name="left_cheek_upper_front",
    )
    carriage.visual(
        Box((0.22, 0.10, 0.10)),
        origin=Origin(xyz=(0.56, -0.37, 0.97)),
        material=timber,
        name="right_cheek_upper_front",
    )
    carriage.visual(
        Box((0.40, 0.64, 0.10)),
        origin=Origin(xyz=(-0.56, 0.00, 0.29)),
        material=timber_dark,
        name="rear_transom",
    )
    carriage.visual(
        Box((0.24, 0.64, 0.10)),
        origin=Origin(xyz=(0.66, 0.00, 0.29)),
        material=timber_dark,
        name="front_transom",
    )
    carriage.visual(
        Box((0.12, 0.20, 0.30)),
        origin=Origin(xyz=(0.42, 0.00, 0.29)),
        material=timber_dark,
        name="front_pedestal",
    )
    carriage.visual(
        Box((0.14, 0.20, 0.24)),
        origin=Origin(xyz=(-0.34, 0.00, 0.25)),
        material=timber_dark,
        name="rear_pedestal",
    )
    carriage.visual(
        Box((0.24, 0.10, 0.09)),
        origin=Origin(xyz=(0.08, 0.37, 0.917)),
        material=iron_mid,
        name="left_trunnion_seat",
    )
    carriage.visual(
        Box((0.24, 0.10, 0.09)),
        origin=Origin(xyz=(0.08, -0.37, 0.917)),
        material=iron_mid,
        name="right_trunnion_seat",
    )
    carriage.visual(
        Box((0.28, 0.34, 0.62)),
        origin=Origin(xyz=(0.00, 0.59, 0.38)),
        material=timber_dark,
        name="left_axle_support",
    )
    carriage.visual(
        Box((0.28, 0.34, 0.62)),
        origin=Origin(xyz=(0.00, -0.59, 0.38)),
        material=timber_dark,
        name="right_axle_support",
    )
    carriage.visual(
        Cylinder(radius=0.07, length=0.08),
        origin=Origin(xyz=(0.00, 0.80, 0.62), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron_mid,
        name="left_axle_stub",
    )
    carriage.visual(
        Cylinder(radius=0.07, length=0.08),
        origin=Origin(xyz=(0.00, -0.80, 0.62), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron_mid,
        name="right_axle_stub",
    )
    carriage.visual(
        Box((0.18, 0.10, 0.22)),
        origin=Origin(xyz=(-0.42, -0.47, 0.68)),
        material=iron_mid,
        name="handwheel_bracket",
    )
    carriage.visual(
        Cylinder(radius=0.025, length=0.06),
        origin=Origin(xyz=(-0.42, -0.52, 0.68), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron_highlight,
        name="handwheel_bearing",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((1.95, 0.90, 1.12)),
        mass=1450.0,
        origin=Origin(xyz=(0.00, 0.00, 0.56)),
    )

    barrel = model.part("barrel")
    barrel.visual(
        barrel_shell_mesh,
        material=iron_dark,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.110, length=0.12),
        origin=Origin(xyz=(-0.56, 0.00, 0.00), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_highlight,
        name="reinforce_band",
    )
    barrel.visual(
        Cylinder(radius=0.220, length=0.12),
        origin=Origin(xyz=(2.00, 0.00, 0.00), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_highlight,
        name="muzzle_band",
    )
    barrel.visual(
        Cylinder(radius=0.088, length=0.16),
        origin=Origin(xyz=(0.00, 0.37, 0.00), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron_mid,
        name="left_trunnion",
    )
    barrel.visual(
        Cylinder(radius=0.088, length=0.16),
        origin=Origin(xyz=(0.00, -0.37, 0.00), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron_mid,
        name="right_trunnion",
    )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.27, length=3.10),
        mass=2950.0,
        origin=Origin(xyz=(0.55, 0.00, 0.00), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    left_wheel = model.part("left_wheel")
    left_wheel.visual(
        wheel_mesh,
        material=iron_mid,
        name="wheel_disc",
    )
    left_wheel.visual(
        Cylinder(radius=0.12, length=0.22),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron_dark,
        name="hub_boss",
    )
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.62, length=0.22),
        mass=210.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(
        wheel_mesh,
        material=iron_mid,
        name="wheel_disc",
    )
    right_wheel.visual(
        Cylinder(radius=0.12, length=0.22),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron_dark,
        name="hub_boss",
    )
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.62, length=0.22),
        mass=210.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    handwheel = model.part("traverse_handwheel")
    handwheel.visual(
        handwheel_rim_mesh,
        origin=Origin(xyz=(0.0, -0.07, 0.0)),
        material=iron_mid,
        name="rim",
    )
    handwheel.visual(
        Cylinder(radius=0.035, length=0.08),
        origin=Origin(xyz=(0.0, -0.07, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron_dark,
        name="hub",
    )
    handwheel.visual(
        Box((0.30, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.07, 0.0)),
        material=iron_highlight,
        name="spoke_horizontal",
    )
    handwheel.visual(
        Box((0.018, 0.018, 0.30)),
        origin=Origin(xyz=(0.0, -0.07, 0.0)),
        material=iron_highlight,
        name="spoke_vertical",
    )
    handwheel.visual(
        Box((0.42, 0.016, 0.016)),
        origin=Origin(xyz=(0.0, -0.07, 0.0), rpy=(0.0, math.pi / 4.0, 0.0)),
        material=iron_highlight,
        name="spoke_diag_a",
    )
    handwheel.visual(
        Box((0.42, 0.016, 0.016)),
        origin=Origin(xyz=(0.0, -0.07, 0.0), rpy=(0.0, -math.pi / 4.0, 0.0)),
        material=iron_highlight,
        name="spoke_diag_b",
    )
    handwheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.17, length=0.08),
        mass=16.0,
        origin=Origin(xyz=(0.0, -0.07, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "carriage_to_barrel_elevation",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(0.08, 0.00, 1.05)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18000.0,
            velocity=0.45,
            lower=-0.10,
            upper=0.55,
        ),
    )
    model.articulation(
        "carriage_to_left_wheel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=left_wheel,
        origin=Origin(xyz=(0.00, 0.95, 0.62)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=500.0,
            velocity=6.0,
            lower=-2.0 * math.pi,
            upper=2.0 * math.pi,
        ),
    )
    model.articulation(
        "carriage_to_right_wheel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=right_wheel,
        origin=Origin(xyz=(0.00, -0.95, 0.62)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=500.0,
            velocity=6.0,
            lower=-2.0 * math.pi,
            upper=2.0 * math.pi,
        ),
    )
    model.articulation(
        "carriage_to_traverse_handwheel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=handwheel,
        origin=Origin(xyz=(-0.42, -0.52, 0.68)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=5.0,
            lower=-8.0 * math.pi,
            upper=8.0 * math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    handwheel = object_model.get_part("traverse_handwheel")

    barrel_joint = object_model.get_articulation("carriage_to_barrel_elevation")
    left_wheel_joint = object_model.get_articulation("carriage_to_left_wheel")
    right_wheel_joint = object_model.get_articulation("carriage_to_right_wheel")
    handwheel_joint = object_model.get_articulation("carriage_to_traverse_handwheel")

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
        elem_a="left_trunnion",
        elem_b="left_trunnion_seat",
        contact_tol=1e-4,
        name="left trunnion is seated in the carriage",
    )
    ctx.expect_contact(
        barrel,
        carriage,
        elem_a="right_trunnion",
        elem_b="right_trunnion_seat",
        contact_tol=1e-4,
        name="right trunnion is seated in the carriage",
    )
    ctx.expect_contact(
        left_wheel,
        carriage,
        elem_a="hub_boss",
        elem_b="left_axle_stub",
        contact_tol=1e-4,
        name="left wheel is carried on the left axle stub",
    )
    ctx.expect_contact(
        right_wheel,
        carriage,
        elem_a="hub_boss",
        elem_b="right_axle_stub",
        contact_tol=1e-4,
        name="right wheel is carried on the right axle stub",
    )
    ctx.expect_contact(
        handwheel,
        carriage,
        elem_a="hub",
        elem_b="handwheel_bearing",
        contact_tol=1e-4,
        name="traversing handwheel is mounted on its bearing",
    )

    ctx.check(
        "barrel elevation axis pitches upward",
        tuple(round(v, 3) for v in barrel_joint.axis) == (0.0, -1.0, 0.0),
        details=f"axis={barrel_joint.axis}",
    )
    ctx.check(
        "wheel axles run laterally through the carriage",
        all(abs(joint.axis[1]) > 0.99 for joint in (left_wheel_joint, right_wheel_joint)),
        details=f"left_axis={left_wheel_joint.axis}, right_axis={right_wheel_joint.axis}",
    )
    ctx.check(
        "handwheel axis matches a side-mounted traversing wheel",
        abs(handwheel_joint.axis[1]) > 0.99,
        details=f"axis={handwheel_joint.axis}",
    )

    handwheel_pos = ctx.part_world_position(handwheel)
    ctx.check(
        "handwheel sits on the right carriage cheek",
        handwheel_pos is not None and handwheel_pos[1] < -0.45,
        details=f"handwheel_position={handwheel_pos}",
    )

    rest_muzzle = ctx.part_element_world_aabb(barrel, elem="muzzle_band")
    upper_limit = barrel_joint.motion_limits.upper if barrel_joint.motion_limits is not None else 0.45
    with ctx.pose({barrel_joint: upper_limit}):
        elevated_muzzle = ctx.part_element_world_aabb(barrel, elem="muzzle_band")
        ctx.fail_if_parts_overlap_in_current_pose(name="elevated barrel pose stays clear of carriage")
        ctx.expect_contact(
            barrel,
            carriage,
            elem_a="left_trunnion",
            elem_b="left_trunnion_seat",
            contact_tol=1e-4,
            name="left trunnion stays seated when elevated",
        )
        ctx.expect_contact(
            barrel,
            carriage,
            elem_a="right_trunnion",
            elem_b="right_trunnion_seat",
            contact_tol=1e-4,
            name="right trunnion stays seated when elevated",
        )

    def _center_z(aabb):
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) * 0.5

    rest_muzzle_z = _center_z(rest_muzzle)
    elevated_muzzle_z = _center_z(elevated_muzzle)
    ctx.check(
        "barrel elevation lifts the muzzle",
        rest_muzzle_z is not None
        and elevated_muzzle_z is not None
        and elevated_muzzle_z > rest_muzzle_z + 0.20,
        details=f"rest_muzzle_z={rest_muzzle_z}, elevated_muzzle_z={elevated_muzzle_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

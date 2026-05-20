from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 32,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * cos((2.0 * pi * index) / segments),
            cy + radius * sin((2.0 * pi * index) / segments),
        )
        for index in range(segments)
    ]


def _carriage_cheek_mesh():
    outer_profile = [
        (-0.95, 0.10),
        (-0.91, 0.27),
        (-0.58, 0.34),
        (-0.18, 0.42),
        (0.02, 0.56),
        (0.12, 0.76),
        (0.22, 0.94),
        (0.38, 0.92),
        (0.57, 0.68),
        (0.45, 0.44),
        (-0.10, 0.33),
        (-0.78, 0.14),
    ]
    trunnion_hole = _circle_profile(0.066, center=(0.20, 0.82), segments=36)
    return _save_mesh(
        "cannon_carriage_cheek",
        ExtrudeWithHolesGeometry(
            outer_profile,
            [trunnion_hole],
            height=0.035,
            center=True,
        ).rotate_x(pi / 2.0),
    )


def _barrel_shell_mesh():
    outer_profile = [
        (0.0, -0.50),
        (0.055, -0.49),
        (0.090, -0.46),
        (0.106, -0.40),
        (0.086, -0.34),
        (0.120, -0.29),
        (0.155, -0.20),
        (0.176, -0.08),
        (0.182, 0.00),
        (0.180, 0.18),
        (0.171, 0.42),
        (0.158, 0.66),
        (0.148, 0.88),
        (0.146, 0.97),
        (0.165, 1.04),
        (0.175, 1.08),
        (0.0, 1.11),
    ]
    inner_profile = [
        (0.0, -0.08),
        (0.040, -0.02),
        (0.052, 0.40),
        (0.060, 0.88),
        (0.064, 1.03),
    ]
    return _save_mesh(
        "park_cannon_barrel_shell",
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=88,
            start_cap="flat",
            end_cap="flat",
            lip_samples=10,
        ).rotate_y(pi / 2.0),
    )


def _wheel_rim_mesh():
    ring_profile = [
        (0.338, -0.050),
        (0.382, -0.050),
        (0.422, -0.040),
        (0.445, -0.022),
        (0.454, 0.0),
        (0.445, 0.022),
        (0.422, 0.040),
        (0.382, 0.050),
        (0.338, 0.050),
        (0.326, 0.030),
        (0.322, 0.0),
        (0.326, -0.030),
    ]
    return _save_mesh(
        "park_cannon_wheel_rim",
        LatheGeometry(ring_profile, segments=80).rotate_x(pi / 2.0),
    )


def _add_wheel_visuals(part, rim_mesh, *, wheel_iron, highlight_iron, side_sign: float) -> None:
    spin_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(rim_mesh, material=wheel_iron, name="rim")
    part.visual(
        Cylinder(radius=0.094, length=0.105),
        origin=spin_origin,
        material=wheel_iron,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.050, length=0.055),
        origin=Origin(xyz=(0.0, side_sign * 0.080, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=highlight_iron,
        name="axle_cap",
    )
    for index in range(10):
        angle = (2.0 * pi * index) / 10.0
        spoke_radius = 0.205
        part.visual(
            Box((0.270, 0.018, 0.030)),
            origin=Origin(
                xyz=(cos(angle) * spoke_radius, 0.0, sin(angle) * spoke_radius),
                rpy=(0.0, angle, 0.0),
            ),
            material=wheel_iron,
            name=f"spoke_{index:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="decorative_park_cannon")

    carriage_iron = model.material("carriage_iron", rgba=(0.16, 0.17, 0.18, 1.0))
    barrel_iron = model.material("barrel_iron", rgba=(0.11, 0.12, 0.13, 1.0))
    worn_iron = model.material("worn_iron", rgba=(0.34, 0.35, 0.37, 1.0))
    wheel_iron = model.material("wheel_iron", rgba=(0.20, 0.21, 0.22, 1.0))

    barrel_mesh = _barrel_shell_mesh()
    wheel_rim_mesh = _wheel_rim_mesh()

    carriage = model.part("carriage")
    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        y_pos = side_sign * 0.22
        carriage.visual(
            Box((1.42, 0.035, 0.24)),
            origin=Origin(xyz=(-0.16, y_pos, 0.22)),
            material=carriage_iron,
            name=f"{side_name}_lower_panel",
        )
        carriage.visual(
            Box((0.82, 0.035, 0.20)),
            origin=Origin(xyz=(0.00, y_pos, 0.42)),
            material=carriage_iron,
            name=f"{side_name}_mid_panel",
        )
        carriage.visual(
            Box((0.22, 0.035, 0.26)),
            origin=Origin(xyz=(0.06, y_pos, 0.63)),
            material=carriage_iron,
            name=f"{side_name}_rear_shoulder",
        )
        carriage.visual(
            Box((0.18, 0.035, 0.26)),
            origin=Origin(xyz=(0.30, y_pos, 0.63)),
            material=carriage_iron,
            name=f"{side_name}_front_shoulder",
        )
        carriage.visual(
            Box((0.12, 0.035, 0.024)),
            origin=Origin(xyz=(0.20, y_pos, 0.760)),
            material=worn_iron,
            name=f"{side_name}_trunnion_saddle",
        )
    carriage.visual(
        Cylinder(radius=0.052, length=0.605),
        origin=Origin(xyz=(-0.10, 0.0, 0.46), rpy=(pi / 2.0, 0.0, 0.0)),
        material=carriage_iron,
        name="axle_beam",
    )
    carriage.visual(
        Box((0.18, 0.405, 0.10)),
        origin=Origin(xyz=(-0.12, 0.0, 0.44)),
        material=carriage_iron,
        name="rear_transom",
    )
    carriage.visual(
        Box((0.16, 0.405, 0.10)),
        origin=Origin(xyz=(0.42, 0.0, 0.50)),
        material=carriage_iron,
        name="front_transom",
    )
    carriage.visual(
        Box((0.76, 0.18, 0.16)),
        origin=Origin(xyz=(-0.56, 0.0, 0.22)),
        material=carriage_iron,
        name="trail_beam",
    )
    carriage.visual(
        Box((0.22, 0.22, 0.12)),
        origin=Origin(xyz=(0.04, 0.0, 0.56)),
        material=worn_iron,
        name="elevating_block",
    )
    carriage.visual(
        Box((0.12, 0.16, 0.10)),
        origin=Origin(xyz=(-0.90, 0.0, 0.14)),
        material=carriage_iron,
        name="trail_cap",
    )
    carriage.visual(
        Box((0.12, 0.18, 0.26)),
        origin=Origin(xyz=(-0.18, 0.0, 0.29)),
        material=carriage_iron,
        name="trail_socket",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((1.65, 0.74, 1.02)),
        mass=540.0,
        origin=Origin(xyz=(-0.18, 0.0, 0.51)),
    )

    barrel = model.part("barrel")
    barrel.visual(
        barrel_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=barrel_iron,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.048, length=0.460),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=barrel_iron,
        name="trunnion_shaft",
    )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=1.56),
        mass=360.0,
        origin=Origin(xyz=(0.30, 0.0, 0.145), rpy=(0.0, pi / 2.0, 0.0)),
    )

    left_wheel = model.part("left_wheel")
    _add_wheel_visuals(
        left_wheel,
        wheel_rim_mesh,
        wheel_iron=wheel_iron,
        highlight_iron=worn_iron,
        side_sign=1.0,
    )
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.455, length=0.105),
        mass=48.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    _add_wheel_visuals(
        right_wheel,
        wheel_rim_mesh,
        wheel_iron=wheel_iron,
        highlight_iron=worn_iron,
        side_sign=-1.0,
    )
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.455, length=0.105),
        mass=48.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "barrel_elevation",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(0.20, 0.0, 0.82)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2500.0,
            velocity=0.6,
            lower=-0.18,
            upper=0.45,
        ),
    )
    model.articulation(
        "left_wheel_axle",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=left_wheel,
        origin=Origin(xyz=(-0.10, 0.355, 0.46)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=4.0,
            lower=-12.0,
            upper=12.0,
        ),
    )
    model.articulation(
        "right_wheel_axle",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=right_wheel,
        origin=Origin(xyz=(-0.10, -0.355, 0.46)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=4.0,
            lower=-12.0,
            upper=12.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    barrel_elevation = object_model.get_articulation("barrel_elevation")
    left_wheel_axle = object_model.get_articulation("left_wheel_axle")
    right_wheel_axle = object_model.get_articulation("right_wheel_axle")

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

    ctx.check(
        "barrel articulation uses trunnion axis",
        barrel_elevation.axis == (0.0, -1.0, 0.0),
        details=f"expected (0, -1, 0), got {barrel_elevation.axis}",
    )
    ctx.check(
        "wheel articulations use axle axis",
        left_wheel_axle.axis == (0.0, 1.0, 0.0) and right_wheel_axle.axis == (0.0, 1.0, 0.0),
        details=f"left axis={left_wheel_axle.axis}, right axis={right_wheel_axle.axis}",
    )

    barrel_limits = barrel_elevation.motion_limits
    ctx.check(
        "barrel elevation limits are plausible",
        (
            barrel_limits is not None
            and barrel_limits.lower is not None
            and barrel_limits.upper is not None
            and barrel_limits.lower < 0.0
            and barrel_limits.upper >= 0.40
        ),
        details=f"limits={barrel_limits}",
    )
    ctx.expect_contact(
        barrel,
        carriage,
        elem_a="trunnion_shaft",
        elem_b="left_trunnion_saddle",
        contact_tol=0.001,
        name="left trunnion saddle supports barrel",
    )
    ctx.expect_contact(
        barrel,
        carriage,
        elem_a="trunnion_shaft",
        elem_b="right_trunnion_saddle",
        contact_tol=0.001,
        name="right trunnion saddle supports barrel",
    )
    ctx.expect_contact(
        left_wheel,
        carriage,
        elem_a="hub",
        elem_b="axle_beam",
        contact_tol=0.001,
        name="left wheel hub seats on axle beam",
    )
    ctx.expect_contact(
        right_wheel,
        carriage,
        elem_a="hub",
        elem_b="axle_beam",
        contact_tol=0.001,
        name="right wheel hub seats on axle beam",
    )
    ctx.expect_origin_distance(
        left_wheel,
        right_wheel,
        axes="y",
        min_dist=0.70,
        max_dist=0.72,
        name="wheel track width stays realistic",
    )

    with ctx.pose({barrel_elevation: -0.18}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no part overlap at barrel depression limit")
    with ctx.pose({barrel_elevation: 0.45}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no part overlap at barrel elevation limit")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

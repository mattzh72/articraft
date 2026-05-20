from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin, sqrt

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
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _rot_z(
    point: tuple[float, float, float], angle: float
) -> tuple[float, float, float]:
    x, y, z = point
    c = cos(angle)
    s = sin(angle)
    return (c * x - s * y, s * x + c * y, z)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_turnstile")

    heritage_green = model.material("heritage_green", rgba=(0.20, 0.29, 0.24, 1.0))
    machine_gray = model.material("machine_gray", rgba=(0.35, 0.37, 0.39, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.78, 1.0))
    zinc = model.material("zinc", rgba=(0.61, 0.63, 0.66, 1.0))

    pedestal = model.part("pedestal")
    pedestal.inertial = Inertial.from_geometry(
        Box((0.58, 0.46, 0.98)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
    )

    pedestal.visual(
        Box((0.58, 0.46, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=machine_gray,
        name="base_plate",
    )
    pedestal.visual(
        Box((0.38, 0.32, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=machine_gray,
        name="plinth_block",
    )

    wall_height = 0.78
    wall_center_z = 0.43
    pedestal.visual(
        Box((0.010, 0.26, wall_height)),
        origin=Origin(xyz=(-0.155, 0.0, wall_center_z)),
        material=heritage_green,
        name="left_wall",
    )
    pedestal.visual(
        Box((0.010, 0.26, wall_height)),
        origin=Origin(xyz=(0.155, 0.0, wall_center_z)),
        material=heritage_green,
        name="right_wall",
    )
    pedestal.visual(
        Box((0.30, 0.010, wall_height)),
        origin=Origin(xyz=(0.0, 0.125, wall_center_z)),
        material=heritage_green,
        name="front_wall",
    )
    pedestal.visual(
        Box((0.30, 0.010, wall_height)),
        origin=Origin(xyz=(0.0, -0.125, wall_center_z)),
        material=heritage_green,
        name="rear_wall",
    )

    pedestal.visual(
        Box((0.30, 0.07, 0.018)),
        origin=Origin(xyz=(0.0, 0.086, 0.829)),
        material=heritage_green,
        name="top_plate_front",
    )
    pedestal.visual(
        Box((0.30, 0.07, 0.018)),
        origin=Origin(xyz=(0.0, -0.086, 0.829)),
        material=heritage_green,
        name="top_plate_rear",
    )
    pedestal.visual(
        Box((0.07, 0.10, 0.018)),
        origin=Origin(xyz=(-0.115, 0.0, 0.829)),
        material=heritage_green,
        name="top_plate_left",
    )
    pedestal.visual(
        Box((0.07, 0.10, 0.018)),
        origin=Origin(xyz=(0.115, 0.0, 0.829)),
        material=heritage_green,
        name="top_plate_right",
    )

    for x in (-0.145, 0.145):
        for y in (-0.115, 0.115):
            pedestal.visual(
                Cylinder(radius=0.012, length=0.78),
                origin=Origin(xyz=(x, y, 0.43)),
                material=machine_gray,
            )

    pedestal.visual(
        Box((0.026, 0.014, 0.52)),
        origin=Origin(xyz=(0.0, 0.129, 0.47)),
        material=machine_gray,
        name="front_center_stiffener",
    )
    pedestal.visual(
        Box((0.026, 0.014, 0.36)),
        origin=Origin(xyz=(0.0, -0.129, 0.60)),
        material=machine_gray,
        name="rear_center_stiffener",
    )
    pedestal.visual(
        Box((0.016, 0.09, 0.46)),
        origin=Origin(xyz=(-0.158, 0.0, 0.50)),
        material=machine_gray,
        name="left_side_stiffener",
    )
    pedestal.visual(
        Box((0.016, 0.09, 0.46)),
        origin=Origin(xyz=(0.158, 0.0, 0.50)),
        material=machine_gray,
        name="right_side_stiffener",
    )

    front_hatch_y = 0.132
    pedestal.visual(
        Box((0.18, 0.008, 0.32)),
        origin=Origin(xyz=(0.0, front_hatch_y, 0.43)),
        material=machine_gray,
        name="front_hatch",
    )
    for z in (0.31, 0.43, 0.55):
        pedestal.visual(
            Box((0.018, 0.014, 0.034)),
            origin=Origin(xyz=(-0.094, 0.134, z)),
            material=dark_steel,
        )
    pedestal.visual(
        Box((0.024, 0.016, 0.040)),
        origin=Origin(xyz=(0.082, 0.137, 0.43)),
        material=dark_steel,
        name="front_hatch_latch",
    )
    pedestal.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(xyz=(0.064, 0.145, 0.43), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="front_hatch_handle",
    )

    rear_hatch_y = -0.132
    pedestal.visual(
        Box((0.14, 0.008, 0.20)),
        origin=Origin(xyz=(0.0, rear_hatch_y, 0.62)),
        material=machine_gray,
        name="rear_hatch",
    )
    for z in (0.56, 0.68):
        pedestal.visual(
            Box((0.016, 0.014, 0.032)),
            origin=Origin(xyz=(0.074, -0.134, z)),
            material=dark_steel,
        )
    pedestal.visual(
        Box((0.022, 0.016, 0.034)),
        origin=Origin(xyz=(-0.060, -0.137, 0.62)),
        material=dark_steel,
        name="rear_hatch_latch",
    )

    bearing_core = LatheGeometry.from_shell_profiles(
        [
            (0.070, 0.040),
            (0.070, 0.740),
            (0.090, 0.780),
            (0.090, 0.860),
            (0.082, 0.888),
            (0.082, 0.904),
        ],
        [
            (0.042, 0.045),
            (0.042, 0.886),
            (0.050, 0.898),
            (0.050, 0.904),
        ],
        segments=56,
    )
    pedestal.visual(
        _save_mesh("turnstile_bearing_core", bearing_core),
        material=dark_steel,
        name="bearing_core",
    )

    pedestal.visual(
        _save_mesh(
            "turnstile_adapter_ring",
            LatheGeometry.from_shell_profiles(
                [
                    (0.140, 0.904),
                    (0.140, 0.928),
                ],
                [
                    (0.082, 0.904),
                    (0.082, 0.928),
                ],
                segments=56,
            ),
        ),
        material=stainless,
        name="adapter_ring",
    )
    pedestal.visual(
        Box((0.050, 0.018, 0.024)),
        origin=Origin(xyz=(0.106, 0.0, 0.916)),
        material=machine_gray,
        name="adapter_web_pos_x",
    )
    pedestal.visual(
        Box((0.050, 0.018, 0.024)),
        origin=Origin(xyz=(-0.106, 0.0, 0.916)),
        material=machine_gray,
        name="adapter_web_neg_x",
    )
    pedestal.visual(
        Box((0.018, 0.050, 0.024)),
        origin=Origin(xyz=(0.0, 0.106, 0.916)),
        material=machine_gray,
        name="adapter_web_pos_y",
    )
    pedestal.visual(
        Box((0.018, 0.050, 0.024)),
        origin=Origin(xyz=(0.0, -0.106, 0.916)),
        material=machine_gray,
        name="adapter_web_neg_y",
    )

    for idx in range(8):
        angle = idx * (pi / 4.0)
        x = cos(angle) * 0.110
        y = sin(angle) * 0.110
        pedestal.visual(
            Cylinder(radius=0.007, length=0.028),
            origin=Origin(xyz=(x, y, 0.918)),
            material=zinc,
        )

    for x in (-0.21, 0.21):
        for y in (-0.15, 0.15):
            pedestal.visual(
                Cylinder(radius=0.008, length=0.030),
                origin=Origin(xyz=(x, y, 0.055)),
                material=zinc,
            )

    for point in (
        (0.13, 0.07, 0.82),
        (-0.13, 0.07, 0.82),
        (0.13, -0.07, 0.82),
        (-0.13, -0.07, 0.82),
    ):
        _add_member(
            pedestal,
            point,
            (point[0] * 0.56, point[1] * 0.56, 0.775),
            radius=0.010,
            material=machine_gray,
        )

    for y in (-0.09, 0.09):
        _add_member(
            pedestal,
            (0.155, y, 0.18),
            (0.125, y, 0.08),
            radius=0.011,
            material=machine_gray,
        )
        _add_member(
            pedestal,
            (-0.155, y, 0.18),
            (-0.125, y, 0.08),
            radius=0.011,
            material=machine_gray,
        )

    rotor = model.part("rotor")
    rotor.inertial = Inertial.from_geometry(
        Box((0.94, 0.94, 0.34)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
    )
    rotor.visual(
        Cylinder(radius=0.031, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=dark_steel,
        name="spindle_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.100, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=stainless,
        name="lower_collar",
    )
    rotor.visual(
        Cylinder(radius=0.078, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=dark_steel,
        name="hub_shell",
    )
    rotor.visual(
        Cylinder(radius=0.060, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.122)),
        material=stainless,
        name="hub_cap",
    )
    rotor.visual(
        Cylinder(radius=0.026, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.146)),
        material=zinc,
        name="retaining_nut",
    )

    arm_mesh = _save_mesh(
        "turnstile_arm",
        tube_from_spline_points(
            [
                (0.088, 0.0, 0.055),
                (0.180, 0.0, 0.055),
                (0.300, 0.0, 0.054),
                (0.405, 0.0, 0.052),
                (0.465, 0.0, 0.050),
            ],
            radius=0.017,
            samples_per_segment=12,
            radial_segments=18,
            cap_ends=True,
        ),
    )

    for idx in range(3):
        angle = idx * (2.0 * pi / 3.0)
        rotor.visual(
            arm_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=stainless,
            name=f"arm_{idx}",
        )
        adapter_xyz = _rot_z((0.108, 0.0, 0.054), angle)
        flange_xyz = _rot_z((0.086, 0.0, 0.030), angle)
        rotor.visual(
            Box((0.065, 0.038, 0.052)),
            origin=Origin(xyz=adapter_xyz, rpy=(0.0, 0.0, angle)),
            material=stainless,
            name=f"adapter_{idx}",
        )
        rotor.visual(
            Box((0.046, 0.054, 0.012)),
            origin=Origin(xyz=flange_xyz, rpy=(0.0, 0.0, angle)),
            material=machine_gray,
            name=f"flange_{idx}",
        )
        for sign in (-1.0, 1.0):
            bolt_xyz = _rot_z((0.108, sign * 0.012, 0.0795), angle)
            rotor.visual(
                Cylinder(radius=0.006, length=0.011),
                origin=Origin(xyz=bolt_xyz),
                material=zinc,
            )
        _add_member(
            rotor,
            _rot_z((0.060, 0.0, 0.024), angle),
            _rot_z((0.160, 0.0, 0.041), angle),
            radius=0.010,
            material=machine_gray,
            name=f"brace_{idx}",
        )

    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.94)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("rotor_spin")
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

    try:
        pedestal.get_visual("front_hatch")
        pedestal.get_visual("rear_hatch")
        pedestal.get_visual("adapter_ring")
        rotor.get_visual("arm_0")
        rotor.get_visual("spindle_shaft")
        ctx.check("key visual features present", True)
    except Exception as exc:
        ctx.check("key visual features present", False, details=str(exc))

    ctx.expect_overlap(
        rotor,
        pedestal,
        axes="xy",
        min_overlap=0.06,
        elem_a="spindle_shaft",
        elem_b="bearing_core",
        name="spindle sits inside bearing core footprint",
    )
    ctx.expect_gap(
        rotor,
        pedestal,
        axis="z",
        min_gap=0.010,
        max_gap=0.030,
        positive_elem="hub_shell",
        negative_elem="adapter_ring",
        name="hub clears adapter ring vertically",
    )
    ctx.expect_gap(
        rotor,
        pedestal,
        axis="z",
        min_gap=0.110,
        max_gap=0.200,
        positive_elem="arm_0",
        negative_elem="top_plate_front",
        name="arm plane stays above cabinet top",
    )

    with ctx.pose({spin: 0.0}):
        arm_closed = _aabb_center(ctx.part_element_world_aabb(rotor, elem="arm_0"))
    with ctx.pose({spin: 2.0 * pi / 3.0}):
        arm_rotated = _aabb_center(ctx.part_element_world_aabb(rotor, elem="arm_0"))
    if arm_closed is None or arm_rotated is None:
        ctx.check(
            "arm rotates around supported spindle",
            False,
            details="Could not resolve arm_0 world AABB in one or more poses.",
        )
    else:
        motion_ok = (
            arm_closed[0] > 0.18
            and abs(arm_closed[1]) < 0.03
            and arm_rotated[0] < -0.05
            and arm_rotated[1] > 0.15
            and abs(arm_closed[2] - arm_rotated[2]) < 0.02
        )
        ctx.check(
            "arm rotates around supported spindle",
            motion_ok,
            details=(
                f"closed_center={arm_closed}, rotated_center={arm_rotated}; "
                "expected arm_0 to swing from +X toward +Y while staying at the same height."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

from __future__ import annotations

from math import asin, atan2, cos, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(v: tuple[float, float, float]) -> tuple[float, float, float]:
    length = sqrt(_dot(v, v))
    if length <= 1e-9:
        return (0.0, 0.0, 1.0)
    return (v[0] / length, v[1] / length, v[2] / length)


def _rpy_from_axes(
    x_axis: tuple[float, float, float],
    y_axis: tuple[float, float, float],
    z_axis: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Return URDF roll/pitch/yaw for a rotation matrix described by columns."""

    r00, r01, r02 = x_axis[0], y_axis[0], z_axis[0]
    r10, r11, r12 = x_axis[1], y_axis[1], z_axis[1]
    r20, r21, r22 = x_axis[2], y_axis[2], z_axis[2]
    if abs(r20) < 0.999999:
        pitch = asin(-r20)
        roll = atan2(r21, r22)
        yaw = atan2(r10, r00)
    else:
        pitch = pi / 2.0 if r20 <= -0.999999 else -pi / 2.0
        roll = 0.0
        yaw = atan2(-r01, r11)
    return (roll, pitch, yaw)


def _rpy_for_z_axis(direction: tuple[float, float, float]) -> tuple[float, float, float]:
    z_axis = _norm(direction)
    reference = (0.0, 0.0, 1.0) if abs(_dot(z_axis, (0.0, 0.0, 1.0))) < 0.88 else (0.0, 1.0, 0.0)
    x_axis = _norm(_cross(reference, z_axis))
    y_axis = _cross(z_axis, x_axis)
    return _rpy_from_axes(x_axis, y_axis, z_axis)


def _rpy_for_x_axis(direction: tuple[float, float, float]) -> tuple[float, float, float]:
    x_axis = _norm(direction)
    reference = (0.0, 0.0, 1.0) if abs(_dot(x_axis, (0.0, 0.0, 1.0))) < 0.88 else (0.0, 1.0, 0.0)
    y_axis = _norm(_cross(reference, x_axis))
    z_axis = _cross(x_axis, y_axis)
    return _rpy_from_axes(x_axis, y_axis, z_axis)


def _beam_between(
    part,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    thickness: float,
    *,
    material,
    name: str,
) -> None:
    direction = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    length = sqrt(_dot(direction, direction))
    center = ((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5)
    part.visual(
        Box((length, thickness, thickness)),
        origin=Origin(xyz=center, rpy=_rpy_for_x_axis(direction)),
        material=material,
        name=name,
    )


def _bucket_rpy(theta: float) -> tuple[float, float, float]:
    tangent = (0.0, -sin(theta), cos(theta))
    across_axle = (1.0, 0.0, 0.0)
    radial = (0.0, cos(theta), sin(theta))
    return _rpy_from_axes(tangent, across_axle, radial)


def _partition_rpy(theta: float) -> tuple[float, float, float]:
    radial = (0.0, cos(theta), sin(theta))
    across_axle = (1.0, 0.0, 0.0)
    inward_tangent = (0.0, sin(theta), -cos(theta))
    return _rpy_from_axes(radial, across_axle, inward_tangent)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_waterwheel")

    aged_oak = model.material("aged_oak", rgba=(0.50, 0.31, 0.15, 1.0))
    dark_oak = model.material("dark_oak", rgba=(0.30, 0.18, 0.09, 1.0))
    wet_wood = model.material("wet_wood", rgba=(0.23, 0.14, 0.07, 1.0))
    iron = model.material("blackened_iron", rgba=(0.07, 0.07, 0.065, 1.0))
    stone = model.material("rough_stone", rgba=(0.45, 0.43, 0.39, 1.0))
    water = model.material("clear_blue_water", rgba=(0.20, 0.48, 0.82, 0.55))

    frame = model.part("frame")

    # Stone pads and ground timbers establish a heavy, historical mill setting.
    frame.visual(Box((1.65, 0.24, 0.12)), origin=Origin(xyz=(0.0, -1.25, 0.06)), material=stone, name="stone_pad_0")
    frame.visual(Box((1.65, 0.24, 0.12)), origin=Origin(xyz=(0.0, 1.25, 0.06)), material=stone, name="stone_pad_1")
    frame.visual(Box((0.18, 3.05, 0.14)), origin=Origin(xyz=(0.62, 0.0, 0.17)), material=dark_oak, name="ground_sill_0")
    frame.visual(Box((0.18, 3.05, 0.14)), origin=Origin(xyz=(-0.62, 0.0, 0.17)), material=dark_oak, name="ground_sill_1")
    for i, y in enumerate((-1.18, 0.0, 1.18)):
        frame.visual(Box((1.42, 0.16, 0.12)), origin=Origin(xyz=(0.0, y, 0.24)), material=dark_oak, name=f"cross_tie_{i}")

    # A-frame posts carry the bearing load on both sides of the wheel.
    for side_index, x in enumerate((-0.62, 0.62)):
        _beam_between(frame, (x, -1.18, 0.28), (x, -0.18, 1.43), 0.13, material=aged_oak, name=f"front_post_{side_index}")
        _beam_between(frame, (x, 1.18, 0.28), (x, 0.18, 1.43), 0.13, material=aged_oak, name=f"rear_post_{side_index}")
        frame.visual(Box((0.18, 2.05, 0.13)), origin=Origin(xyz=(x, 0.0, 0.96)), material=aged_oak, name=f"side_tie_{side_index}")
        if side_index == 0:
            frame.visual(Box((0.22, 0.34, 0.10)), origin=Origin(xyz=(x, 0.0, 1.43)), material=dark_oak, name="bearing_base_0")
        else:
            frame.visual(Box((0.22, 0.34, 0.10)), origin=Origin(xyz=(x, 0.0, 1.43)), material=dark_oak, name="bearing_base_1")
        frame.visual(Box((0.22, 0.34, 0.08)), origin=Origin(xyz=(x, 0.0, 1.69)), material=dark_oak, name=f"bearing_cap_{side_index}")
        frame.visual(Box((0.22, 0.045, 0.24)), origin=Origin(xyz=(x, 0.16, 1.56)), material=dark_oak, name=f"bearing_cheek_{side_index}_0")
        frame.visual(Box((0.22, 0.045, 0.24)), origin=Origin(xyz=(x, -0.16, 1.56)), material=dark_oak, name=f"bearing_cheek_{side_index}_1")
        collar = TorusGeometry(0.103, 0.017, radial_segments=18, tubular_segments=48).rotate_y(pi / 2.0)
        frame.visual(mesh_from_geometry(collar, f"bearing_collar_{side_index}"), origin=Origin(xyz=(x, 0.0, 1.55)), material=iron, name=f"bearing_collar_{side_index}")

    _beam_between(frame, (-0.62, -1.12, 0.35), (-0.62, 1.12, 1.18), 0.08, material=aged_oak, name="diagonal_brace_0")
    _beam_between(frame, (0.62, -1.12, 0.35), (0.62, 1.12, 1.18), 0.08, material=aged_oak, name="diagonal_brace_1")

    # Elevated overshot flume: a supported wooden launder with visible water.
    frame.visual(Box((1.42, 0.14, 0.12)), origin=Origin(xyz=(0.0, -1.45, 0.24)), material=dark_oak, name="flume_foot_tie")
    for i, x in enumerate((-0.34, 0.34)):
        frame.visual(Box((0.11, 0.11, 2.62)), origin=Origin(xyz=(x, -1.45, 1.55)), material=aged_oak, name=f"flume_post_{i}")
    frame.visual(Box((0.88, 0.13, 0.12)), origin=Origin(xyz=(0.0, -1.45, 2.80)), material=aged_oak, name="flume_crosshead")
    frame.visual(Box((0.42, 1.54, 0.06)), origin=Origin(xyz=(0.0, -1.04, 2.86)), material=wet_wood, name="flume_bottom")
    frame.visual(Box((0.05, 1.54, 0.25)), origin=Origin(xyz=(0.235, -1.04, 2.99)), material=wet_wood, name="flume_side_0")
    frame.visual(Box((0.05, 1.54, 0.25)), origin=Origin(xyz=(-0.235, -1.04, 2.99)), material=wet_wood, name="flume_side_1")
    frame.visual(Box((0.40, 0.06, 0.18)), origin=Origin(xyz=(0.0, -1.78, 2.97)), material=wet_wood, name="flume_backstop")
    frame.visual(Box((0.30, 0.96, 0.040)), origin=Origin(xyz=(0.0, -1.22, 2.900)), material=water, name="water_in_flume")
    _beam_between(frame, (0.0, -0.39, 2.85), (0.0, -0.62, 2.83), 0.04, material=water, name="falling_water")
    frame.visual(Box((0.34, 0.08, 0.045)), origin=Origin(xyz=(0.0, -0.36, 2.835)), material=wet_wood, name="spout_lip")

    wheel = model.part("wheel")

    # Axle, hub, rings, spokes, and buckets are one rotating assembly about the axle.
    wheel.visual(Cylinder(radius=0.070, length=1.42), origin=Origin(rpy=(0.0, pi / 2.0, 0.0)), material=iron, name="axle")
    wheel.visual(Cylinder(radius=0.18, length=0.62), origin=Origin(rpy=(0.0, pi / 2.0, 0.0)), material=dark_oak, name="hub")
    wheel.visual(Cylinder(radius=0.22, length=0.10), origin=Origin(xyz=(-0.22, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=iron, name="hub_band_0")
    wheel.visual(Cylinder(radius=0.22, length=0.10), origin=Origin(xyz=(0.22, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=iron, name="hub_band_1")

    for i, x in enumerate((-0.30, 0.30)):
        outer_rim = TorusGeometry(1.20, 0.045, radial_segments=20, tubular_segments=96).rotate_y(pi / 2.0)
        inner_rim = TorusGeometry(1.02, 0.032, radial_segments=16, tubular_segments=96).rotate_y(pi / 2.0)
        wheel.visual(mesh_from_geometry(outer_rim, f"outer_rim_{i}"), origin=Origin(xyz=(x, 0.0, 0.0)), material=aged_oak, name=f"outer_rim_{i}")
        wheel.visual(mesh_from_geometry(inner_rim, f"inner_rim_{i}"), origin=Origin(xyz=(x, 0.0, 0.0)), material=aged_oak, name=f"inner_rim_{i}")

    spoke_count = 12
    for side_index, x in enumerate((-0.20, 0.20)):
        for i in range(spoke_count):
            theta = 2.0 * pi * i / spoke_count
            center_radius = 0.62
            direction = (0.0, cos(theta), sin(theta))
            wheel.visual(
                Cylinder(radius=0.030, length=0.95),
                origin=Origin(
                    xyz=(x, center_radius * cos(theta), center_radius * sin(theta)),
                    rpy=_rpy_for_z_axis(direction),
                ),
                material=aged_oak,
                name=f"spoke_{side_index}_{i}",
            )

    bucket_count = 24
    bucket_width = 0.56
    for i in range(bucket_count):
        theta = pi / 2.0 + 2.0 * pi * i / bucket_count
        pitch = 2.0 * pi / bucket_count
        r_floor = 1.105
        r_partition = 1.135
        floor_center = (0.0, r_floor * cos(theta), r_floor * sin(theta))
        if i == 1:
            wheel.visual(
                Box((0.36, bucket_width, 0.040)),
                origin=Origin(xyz=floor_center, rpy=_bucket_rpy(theta)),
                material=wet_wood,
                name="bucket_floor_1",
            )
        else:
            wheel.visual(
                Box((0.36, bucket_width, 0.040)),
                origin=Origin(xyz=floor_center, rpy=_bucket_rpy(theta)),
                material=wet_wood,
                name=f"bucket_floor_{i}",
            )
        boundary = theta + pitch * 0.46
        partition_center = (0.0, r_partition * cos(boundary), r_partition * sin(boundary))
        wheel.visual(
            Box((0.30, bucket_width, 0.034)),
            origin=Origin(xyz=partition_center, rpy=_partition_rpy(boundary)),
            material=wet_wood,
            name=f"bucket_partition_{i}",
        )
        # A few filled cells on the upper descending side make the overshot action legible.
        if i in (0, 1, 2, 3):
            water_center = (0.0, (r_floor + 0.030) * cos(theta), (r_floor + 0.030) * sin(theta))
            if i == 1:
                wheel.visual(
                    Box((0.25, bucket_width * 0.82, 0.020)),
                    origin=Origin(xyz=water_center, rpy=_bucket_rpy(theta)),
                    material=water,
                    name="bucket_water_1",
                )
            else:
                wheel.visual(
                    Box((0.25, bucket_width * 0.82, 0.020)),
                    origin=Origin(xyz=water_center, rpy=_bucket_rpy(theta)),
                    material=water,
                    name=f"bucket_water_{i}",
                )

    # A simple wooden crown gear on the mill side communicates power takeoff.
    wheel.visual(Cylinder(radius=0.36, length=0.070), origin=Origin(xyz=(-0.42, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=dark_oak, name="crown_gear_disc")
    for i in range(18):
        theta = 2.0 * pi * i / 18
        wheel.visual(
            Box((0.10, 0.055, 0.075)),
            origin=Origin(
                xyz=(-0.47, 0.39 * cos(theta), 0.39 * sin(theta)),
                rpy=_bucket_rpy(theta),
            ),
            material=dark_oak,
            name=f"gear_tooth_{i}",
        )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 1.55)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.6),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("wheel_spin")

    ctx.check("primary_spin_joint_present", spin is not None, "Expected a continuous wheel spin articulation.")
    ctx.check("two_major_links_present", frame is not None and wheel is not None, "Expected frame and wheel links.")
    if frame is None or wheel is None or spin is None:
        return ctx.report()

    wheel_aabb = ctx.part_world_aabb(wheel)
    ctx.check("wheel_aabb_present", wheel_aabb is not None, "Expected a world AABB for the wheel.")
    if wheel_aabb is not None:
        mins, maxs = wheel_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("large_waterwheel_diameter", 2.35 <= max(size[1], size[2]) <= 2.65, f"wheel size={size!r}")
        ctx.check("load_bearing_axle_width", size[0] >= 1.30, f"wheel size={size!r}")

    ctx.expect_gap(
        wheel,
        frame,
        axis="z",
        positive_elem="axle",
        negative_elem="bearing_base_0",
        min_gap=0.0,
        max_gap=0.004,
        name="axle rests on first bearing block",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="z",
        positive_elem="axle",
        negative_elem="bearing_base_1",
        min_gap=0.0,
        max_gap=0.004,
        name="axle rests on second bearing block",
    )
    def _center_z_y(aabb):
        mins, maxs = aabb
        return ((float(mins[1]) + float(maxs[1])) * 0.5, (float(mins[2]) + float(maxs[2])) * 0.5)

    with ctx.pose({spin: 0.0}):
        top_bucket_rest = ctx.part_element_world_aabb(wheel, elem="bucket_floor_0")
    with ctx.pose({spin: 0.55}):
        top_bucket_rotated = ctx.part_element_world_aabb(wheel, elem="bucket_floor_0")
    if top_bucket_rest is not None and top_bucket_rotated is not None:
        rest_y, rest_z = _center_z_y(top_bucket_rest)
        moved_y, moved_z = _center_z_y(top_bucket_rotated)
        ctx.check(
            "overshot_rotation_moves_top_bucket_downstream",
            moved_y < rest_y - 0.45 and moved_z < rest_z - 0.12,
            details=f"rest_yz={(rest_y, rest_z)!r}, moved_yz={(moved_y, moved_z)!r}",
        )
    else:
        ctx.fail("top_bucket_aabb_present", "Expected bucket_floor_0 AABBs in rest and rotated poses.")

    return ctx.report()


object_model = build_object_model()

from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, pi, sin, sqrt
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, rgba=rgba)
    except TypeError:
        try:
            return Material(name=name, color=rgba)
        except TypeError:
            return Material(name=name)


FRAME_METAL = _make_material("frame_metal", (0.70, 0.73, 0.77, 1.0))
UPHOLSTERY = _make_material("upholstery_black", (0.12, 0.12, 0.13, 1.0))
RUBBER = _make_material("rubber_black", (0.07, 0.07, 0.08, 1.0))
ALUMINUM = _make_material("aluminum", (0.82, 0.84, 0.87, 1.0))
STEEL = _make_material("brushed_steel", (0.69, 0.71, 0.74, 1.0))
PLASTIC = _make_material("black_plastic", (0.14, 0.15, 0.16, 1.0))


def _mesh_path(name: str) -> Path:
    if hasattr(ASSETS, "mesh_path"):
        return Path(ASSETS.mesh_path(name))
    mesh_dir = ASSETS.mesh_dir
    mesh_dir.mkdir(parents=True, exist_ok=True)
    return mesh_dir / name


def _merge_geometries(geometries):
    base = geometries[0].clone()
    for geom in geometries[1:]:
        base.merge(geom)
    return base


def _mirror_path(points):
    return [(-x, y, z) for (x, y, z) in points]


def _cylinder_between(p0, p1, radius: float, radial_segments: int = 16):
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = sqrt(dx * dx + dy * dy + dz * dz)
    geom = CylinderGeometry(
        radius=radius,
        height=length,
        radial_segments=radial_segments,
        closed=True,
    )
    yaw = atan2(dy, dx)
    pitch = atan2(sqrt(dx * dx + dy * dy), dz)
    geom.rotate_y(pitch)
    geom.rotate_z(yaw)
    geom.translate(
        (p0[0] + p1[0]) * 0.5,
        (p0[1] + p1[1]) * 0.5,
        (p0[2] + p1[2]) * 0.5,
    )
    return geom


def _tube(points, radius: float, samples: int = 16, radial_segments: int = 18):
    return tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=samples,
        radial_segments=radial_segments,
        cap_ends=True,
    )


def _rear_wheel_tire_geometry():
    return TorusGeometry(
        radius=0.286,
        tube=0.019,
        radial_segments=20,
        tubular_segments=64,
    )


def _rear_wheel_metal_geometry():
    geoms = [
        TorusGeometry(radius=0.255, tube=0.015, radial_segments=18, tubular_segments=56),
        CylinderGeometry(radius=0.034, height=0.058, radial_segments=24, closed=True),
    ]
    spoke_length = 0.205
    spoke_radius = 0.0023
    hub_offset = 0.037
    for i in range(10):
        angle = 2.0 * pi * i / 10.0
        for angle_offset, z_offset in ((0.0, 0.0065), (pi / 10.0, -0.0065)):
            spoke = CylinderGeometry(
                radius=spoke_radius,
                height=spoke_length,
                radial_segments=12,
                closed=True,
            )
            spoke.rotate_y(pi / 2.0)
            spoke.translate(hub_offset + spoke_length * 0.5, 0.0, z_offset)
            spoke.rotate_z(angle + angle_offset)
            geoms.append(spoke)
    return _merge_geometries(geoms)


def _rear_handrim_geometry():
    geoms = []
    handrim = TorusGeometry(
        radius=0.263,
        tube=0.006,
        radial_segments=16,
        tubular_segments=56,
    )
    handrim.translate(0.0, 0.0, 0.023)
    geoms.append(handrim)
    for angle in (pi / 6.0, 5.0 * pi / 6.0, 3.0 * pi / 2.0):
        bracket = CylinderGeometry(radius=0.0035, height=0.023, radial_segments=10, closed=True)
        bracket.translate(0.238 * cos(angle), 0.238 * sin(angle), 0.0115)
        geoms.append(bracket)
    return _merge_geometries(geoms)


def _caster_tire_geometry():
    return TorusGeometry(
        radius=0.060,
        tube=0.010,
        radial_segments=16,
        tubular_segments=40,
    )


def _caster_hub_geometry():
    geoms = [
        CylinderGeometry(radius=0.052, height=0.018, radial_segments=20, closed=True),
        CylinderGeometry(radius=0.015, height=0.024, radial_segments=18, closed=True),
    ]
    for offset in (-0.007, 0.007):
        flange = CylinderGeometry(radius=0.028, height=0.004, radial_segments=18, closed=True)
        flange.translate(0.0, 0.0, offset)
        geoms.append(flange)
    return _merge_geometries(geoms)


def _frame_geometry():
    left_upper = [
        (0.220, -0.205, 0.885),
        (0.220, -0.160, 0.780),
        (0.220, -0.105, 0.585),
        (0.220, 0.165, 0.490),
        (0.205, 0.285, 0.360),
        (0.180, 0.340, 0.170),
    ]
    left_lower = [
        (0.226, -0.080, 0.360),
        (0.220, 0.060, 0.350),
        (0.205, 0.220, 0.275),
        (0.180, 0.340, 0.170),
    ]
    geoms = [
        _tube(left_upper, 0.012),
        _tube(_mirror_path(left_upper), 0.012),
        _tube(left_lower, 0.011),
        _tube(_mirror_path(left_lower), 0.011),
        _cylinder_between((0.220, -0.205, 0.885), (0.165, -0.225, 0.890), 0.011),
        _cylinder_between((-0.220, -0.205, 0.885), (-0.165, -0.225, 0.890), 0.011),
        _cylinder_between((0.224, -0.105, 0.585), (0.226, -0.080, 0.360), 0.011),
        _cylinder_between((-0.224, -0.105, 0.585), (-0.226, -0.080, 0.360), 0.011),
        _cylinder_between((0.220, 0.180, 0.490), (-0.220, 0.180, 0.490), 0.011),
        _cylinder_between((0.220, -0.145, 0.700), (-0.220, -0.145, 0.700), 0.011),
        _cylinder_between((0.240, -0.060, 0.310), (-0.240, -0.060, 0.310), 0.010),
        _cylinder_between((0.240, -0.060, 0.310), (0.321, -0.060, 0.310), 0.009),
        _cylinder_between((-0.240, -0.060, 0.310), (-0.321, -0.060, 0.310), 0.009),
        _cylinder_between((0.210, -0.020, 0.360), (-0.210, 0.170, 0.340), 0.010),
        _cylinder_between((-0.210, -0.020, 0.360), (0.210, 0.170, 0.340), 0.010),
        _cylinder_between((0.180, 0.340, 0.170), (-0.180, 0.340, 0.170), 0.010),
        _cylinder_between((0.155, 0.310, 0.280), (-0.155, 0.310, 0.280), 0.010),
        _cylinder_between((0.160, 0.300, 0.290), (0.105, 0.370, 0.145), 0.010),
        _cylinder_between((-0.160, 0.300, 0.290), (-0.105, 0.370, 0.145), 0.010),
        _cylinder_between((0.220, -0.010, 0.490), (0.232, -0.010, 0.680), 0.009),
        _cylinder_between((0.232, -0.010, 0.680), (0.232, -0.150, 0.680), 0.009),
        _cylinder_between((0.220, -0.125, 0.585), (0.232, -0.125, 0.680), 0.008),
        _cylinder_between((-0.220, -0.010, 0.490), (-0.232, -0.010, 0.680), 0.009),
        _cylinder_between((-0.232, -0.010, 0.680), (-0.232, -0.150, 0.680), 0.009),
        _cylinder_between((-0.220, -0.125, 0.585), (-0.232, -0.125, 0.680), 0.008),
    ]
    return _merge_geometries(geoms)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_wheelchair", assets=ASSETS)

    frame_mesh = mesh_from_geometry(_frame_geometry(), _mesh_path("wheelchair_frame.obj"))
    rear_tire_mesh = mesh_from_geometry(
        _rear_wheel_tire_geometry(),
        _mesh_path("wheelchair_rear_tire.obj"),
    )
    rear_metal_mesh = mesh_from_geometry(
        _rear_wheel_metal_geometry(),
        _mesh_path("wheelchair_rear_wheel_core.obj"),
    )
    rear_handrim_mesh = mesh_from_geometry(
        _rear_handrim_geometry(),
        _mesh_path("wheelchair_handrim.obj"),
    )
    caster_tire_mesh = mesh_from_geometry(
        _caster_tire_geometry(),
        _mesh_path("wheelchair_caster_tire.obj"),
    )
    caster_hub_mesh = mesh_from_geometry(
        _caster_hub_geometry(),
        _mesh_path("wheelchair_caster_hub.obj"),
    )

    frame = model.part("frame")
    frame.visual(frame_mesh, material=FRAME_METAL)
    frame.visual(
        Box((0.460, 0.400, 0.018)),
        origin=Origin(xyz=(0.000, 0.030, 0.482), rpy=(-0.050, 0.000, 0.000)),
        material=UPHOLSTERY,
    )
    frame.visual(
        Box((0.440, 0.018, 0.340)),
        origin=Origin(xyz=(0.000, -0.145, 0.675), rpy=(-0.080, 0.000, 0.000)),
        material=UPHOLSTERY,
    )
    frame.visual(
        Box((0.040, 0.270, 0.028)),
        origin=Origin(xyz=(0.232, -0.045, 0.695)),
        material=PLASTIC,
    )
    frame.visual(
        Box((0.040, 0.270, 0.028)),
        origin=Origin(xyz=(-0.232, -0.045, 0.695)),
        material=PLASTIC,
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.700, 0.920, 0.960)),
        mass=11.5,
        origin=Origin(xyz=(0.000, 0.040, 0.480)),
    )

    left_rear_wheel = model.part("left_rear_wheel")
    left_rear_wheel.visual(
        rear_tire_mesh,
        origin=Origin(rpy=(0.000, pi / 2.0, 0.000)),
        material=RUBBER,
    )
    left_rear_wheel.visual(
        rear_metal_mesh,
        origin=Origin(rpy=(0.000, pi / 2.0, 0.000)),
        material=ALUMINUM,
    )
    left_rear_wheel.visual(
        rear_handrim_mesh,
        origin=Origin(rpy=(0.000, pi / 2.0, 0.000)),
        material=STEEL,
    )
    left_rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.305, length=0.032),
        mass=1.8,
        origin=Origin(rpy=(0.000, pi / 2.0, 0.000)),
    )

    right_rear_wheel = model.part("right_rear_wheel")
    right_rear_wheel.visual(
        rear_tire_mesh,
        origin=Origin(rpy=(0.000, -pi / 2.0, 0.000)),
        material=RUBBER,
    )
    right_rear_wheel.visual(
        rear_metal_mesh,
        origin=Origin(rpy=(0.000, -pi / 2.0, 0.000)),
        material=ALUMINUM,
    )
    right_rear_wheel.visual(
        rear_handrim_mesh,
        origin=Origin(rpy=(0.000, -pi / 2.0, 0.000)),
        material=STEEL,
    )
    right_rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.305, length=0.032),
        mass=1.8,
        origin=Origin(rpy=(0.000, -pi / 2.0, 0.000)),
    )

    for side in ("left", "right"):
        fork = model.part(f"{side}_caster_fork")
        fork.visual(
            Cylinder(radius=0.008, length=0.040),
            origin=Origin(xyz=(0.000, 0.000, -0.020)),
            material=FRAME_METAL,
        )
        fork.visual(
            Box((0.048, 0.018, 0.010)),
            origin=Origin(xyz=(0.000, -0.020, -0.046)),
            material=FRAME_METAL,
        )
        fork.visual(
            Box((0.006, 0.016, 0.084)),
            origin=Origin(xyz=(0.020, -0.034, -0.090)),
            material=FRAME_METAL,
        )
        fork.visual(
            Box((0.006, 0.016, 0.084)),
            origin=Origin(xyz=(-0.020, -0.034, -0.090)),
            material=FRAME_METAL,
        )
        fork.visual(
            Cylinder(radius=0.0045, length=0.040),
            origin=Origin(xyz=(0.000, -0.034, -0.112), rpy=(0.000, pi / 2.0, 0.000)),
            material=STEEL,
        )
        fork.inertial = Inertial.from_geometry(
            Box((0.060, 0.050, 0.120)),
            mass=0.35,
            origin=Origin(xyz=(0.000, -0.024, -0.072)),
        )

        caster_wheel = model.part(f"{side}_caster_wheel")
        caster_wheel.visual(
            caster_tire_mesh,
            origin=Origin(rpy=(0.000, pi / 2.0, 0.000)),
            material=RUBBER,
        )
        caster_wheel.visual(
            caster_hub_mesh,
            origin=Origin(rpy=(0.000, pi / 2.0, 0.000)),
            material=ALUMINUM,
        )
        caster_wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.070, length=0.020),
            mass=0.28,
            origin=Origin(rpy=(0.000, pi / 2.0, 0.000)),
        )

        footplate = model.part(f"{side}_footplate")
        footplate.visual(
            Box((0.050, 0.010, 0.012)),
            origin=Origin(xyz=(0.000, 0.005, -0.003)),
            material=FRAME_METAL,
        )
        footplate.visual(
            Box((0.012, 0.020, 0.078)),
            origin=Origin(xyz=(0.000, 0.024, -0.045)),
            material=FRAME_METAL,
        )
        footplate.visual(
            Box((0.090, 0.120, 0.008)),
            origin=Origin(xyz=(0.000, 0.082, -0.085)),
            material=PLASTIC,
        )
        footplate.visual(
            Box((0.090, 0.008, 0.020)),
            origin=Origin(xyz=(0.000, 0.138, -0.072)),
            material=PLASTIC,
        )
        footplate.visual(
            Box((0.008, 0.120, 0.012)),
            origin=Origin(xyz=(0.041, 0.082, -0.078)),
            material=PLASTIC,
        )
        footplate.visual(
            Box((0.008, 0.120, 0.012)),
            origin=Origin(xyz=(-0.041, 0.082, -0.078)),
            material=PLASTIC,
        )
        footplate.inertial = Inertial.from_geometry(
            Box((0.090, 0.120, 0.020)),
            mass=0.25,
            origin=Origin(xyz=(0.000, 0.080, -0.070)),
        )

    model.articulation(
        "left_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="frame",
        child="left_rear_wheel",
        origin=Origin(xyz=(0.330, -0.060, 0.310)),
        axis=(1.000, 0.000, 0.000),
        motion_limits=MotionLimits(effort=12.0, velocity=30.0),
    )
    model.articulation(
        "right_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="frame",
        child="right_rear_wheel",
        origin=Origin(xyz=(-0.330, -0.060, 0.310)),
        axis=(1.000, 0.000, 0.000),
        motion_limits=MotionLimits(effort=12.0, velocity=30.0),
    )
    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent="frame",
        child="left_caster_fork",
        origin=Origin(xyz=(0.180, 0.340, 0.170)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent="frame",
        child="right_caster_fork",
        origin=Origin(xyz=(-0.180, 0.340, 0.170)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    model.articulation(
        "left_caster_roll",
        ArticulationType.CONTINUOUS,
        parent="left_caster_fork",
        child="left_caster_wheel",
        origin=Origin(xyz=(0.000, -0.034, -0.112)),
        axis=(1.000, 0.000, 0.000),
        motion_limits=MotionLimits(effort=3.0, velocity=22.0),
    )
    model.articulation(
        "right_caster_roll",
        ArticulationType.CONTINUOUS,
        parent="right_caster_fork",
        child="right_caster_wheel",
        origin=Origin(xyz=(0.000, -0.034, -0.112)),
        axis=(1.000, 0.000, 0.000),
        motion_limits=MotionLimits(effort=3.0, velocity=22.0),
    )
    model.articulation(
        "left_footplate_hinge",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="left_footplate",
        origin=Origin(xyz=(0.100, 0.370, 0.145)),
        axis=(1.000, 0.000, 0.000),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "right_footplate_hinge",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="right_footplate",
        origin=Origin(xyz=(-0.100, 0.370, 0.145)),
        axis=(1.000, 0.000, 0.000),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "frame",
        "left_rear_wheel",
        reason="quick-release axle receiver sits inside the rear hub shell",
    )
    ctx.allow_overlap(
        "frame",
        "right_rear_wheel",
        reason="quick-release axle receiver sits inside the rear hub shell",
    )
    ctx.allow_overlap(
        "frame",
        "left_caster_fork",
        reason="caster stem is intentionally nested into the front head tube",
    )
    ctx.allow_overlap(
        "frame",
        "right_caster_fork",
        reason="caster stem is intentionally nested into the front head tube",
    )
    ctx.allow_overlap(
        "left_caster_fork",
        "left_caster_wheel",
        reason="fork tines and axle overlap the conservative caster wheel collision hull",
    )
    ctx.allow_overlap(
        "right_caster_fork",
        "right_caster_wheel",
        reason="fork tines and axle overlap the conservative caster wheel collision hull",
    )
    ctx.allow_overlap(
        "frame",
        "left_footplate",
        reason="footrest hanger nests into the front frame bracket at the hinge",
    )
    ctx.allow_overlap(
        "frame",
        "right_footplate",
        reason="footrest hanger nests into the front frame bracket at the hinge",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.004, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("left_caster_fork", "left_caster_wheel", axes="xy", max_dist=0.050)
    ctx.expect_origin_distance("right_caster_fork", "right_caster_wheel", axes="xy", max_dist=0.050)
    ctx.expect_joint_motion_axis(
        "left_footplate_hinge",
        "left_footplate",
        world_axis="z",
        direction="positive",
        min_delta=0.060,
    )
    ctx.expect_joint_motion_axis(
        "right_footplate_hinge",
        "right_footplate",
        world_axis="z",
        direction="positive",
        min_delta=0.060,
    )

    def _assert(condition: bool, message: str) -> None:
        if not condition:
            raise AssertionError(message)

    def _aabb_center(part_name: str):
        aabb = ctx.part_world_aabb(part_name)
        if hasattr(aabb, "center"):
            center = aabb.center() if callable(aabb.center) else aabb.center
            return tuple(center)
        if all(
            hasattr(aabb, attr) for attr in ("min_x", "max_x", "min_y", "max_y", "min_z", "max_z")
        ):
            return (
                0.5 * (aabb.min_x + aabb.max_x),
                0.5 * (aabb.min_y + aabb.max_y),
                0.5 * (aabb.min_z + aabb.max_z),
            )
        if hasattr(aabb, "min") and hasattr(aabb, "max"):
            return tuple(0.5 * (aabb.min[i] + aabb.max[i]) for i in range(3))
        if hasattr(aabb, "minimum") and hasattr(aabb, "maximum"):
            return tuple(0.5 * (aabb.minimum[i] + aabb.maximum[i]) for i in range(3))
        if hasattr(aabb, "lower") and hasattr(aabb, "upper"):
            return tuple(0.5 * (aabb.lower[i] + aabb.upper[i]) for i in range(3))
        if hasattr(aabb, "mins") and hasattr(aabb, "maxs"):
            return tuple(0.5 * (aabb.mins[i] + aabb.maxs[i]) for i in range(3))
        if hasattr(aabb, "bounds"):
            bounds = aabb.bounds() if callable(aabb.bounds) else aabb.bounds
            if len(bounds) == 2:
                return tuple(0.5 * (bounds[0][i] + bounds[1][i]) for i in range(3))
        if isinstance(aabb, dict):
            if all(key in aabb for key in ("min", "max")):
                return tuple(0.5 * (aabb["min"][i] + aabb["max"][i]) for i in range(3))
            if all(key in aabb for key in ("min_x", "max_x", "min_y", "max_y", "min_z", "max_z")):
                return (
                    0.5 * (aabb["min_x"] + aabb["max_x"]),
                    0.5 * (aabb["min_y"] + aabb["max_y"]),
                    0.5 * (aabb["min_z"] + aabb["max_z"]),
                )
        try:
            bounds = tuple(aabb)
            if len(bounds) == 2 and all(len(item) == 3 for item in bounds):
                return tuple(0.5 * (bounds[0][i] + bounds[1][i]) for i in range(3))
        except TypeError:
            pass
        pos = ctx.part_world_position(part_name)
        return tuple(pos)

    frame_center = _aabb_center("frame")
    left_rear_center = _aabb_center("left_rear_wheel")
    right_rear_center = _aabb_center("right_rear_wheel")
    left_caster_center = _aabb_center("left_caster_wheel")
    right_caster_center = _aabb_center("right_caster_wheel")
    left_footplate_center = _aabb_center("left_footplate")
    right_footplate_center = _aabb_center("right_footplate")

    _assert(
        left_rear_center[0] > frame_center[0] + 0.28,
        "Left rear wheel should sit well outboard of the chair frame",
    )
    _assert(
        right_rear_center[0] < frame_center[0] - 0.28,
        "Right rear wheel should sit well outboard of the chair frame",
    )
    _assert(
        abs(left_rear_center[1] - right_rear_center[1]) < 0.03,
        "Rear wheels should share a common axle line",
    )
    _assert(
        left_caster_center[1] > left_rear_center[1] + 0.30,
        "Left caster should establish a realistic front wheelbase",
    )
    _assert(
        right_caster_center[1] > right_rear_center[1] + 0.30,
        "Right caster should establish a realistic front wheelbase",
    )
    _assert(
        left_footplate_center[1] > left_caster_center[1] + 0.06,
        "Left footplate should project ahead of the caster",
    )
    _assert(
        right_footplate_center[1] > right_caster_center[1] + 0.06,
        "Right footplate should project ahead of the caster",
    )

    with ctx.pose(left_caster_swivel=1.10, right_caster_swivel=-1.10):
        left_swiveled = _aabb_center("left_caster_wheel")
        right_swiveled = _aabb_center("right_caster_wheel")
        ctx.expect_origin_distance("left_caster_fork", "left_caster_wheel", axes="xy", max_dist=0.050)
        ctx.expect_origin_distance("right_caster_fork", "right_caster_wheel", axes="xy", max_dist=0.050)
        _assert(
            left_swiveled[0] > left_caster_center[0] + 0.015,
            "Left caster wheel should trail and swing outward when steered",
        )
        _assert(
            right_swiveled[0] < right_caster_center[0] - 0.015,
            "Right caster wheel should trail and swing outward when steered",
        )

    with ctx.pose(left_footplate_hinge=1.25, right_footplate_hinge=1.25):
        folded_left = _aabb_center("left_footplate")
        folded_right = _aabb_center("right_footplate")
        _assert(
            folded_left[2] > left_footplate_center[2] + 0.07,
            "Left footplate should fold upward for transfers and storage",
        )
        _assert(
            folded_right[2] > right_footplate_center[2] + 0.07,
            "Right footplate should fold upward for transfers and storage",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

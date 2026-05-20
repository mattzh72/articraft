from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, color=rgba)
    except TypeError:
        return Material(name=name, rgba=rgba)


def _segment_origin(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = sqrt(dx * dx + dy * dy + dz * dz)
    yaw = atan2(dy, dx)
    pitch = atan2(sqrt(dx * dx + dy * dy), dz)
    origin = Origin(
        xyz=(
            0.5 * (start[0] + end[0]),
            0.5 * (start[1] + end[1]),
            0.5 * (start[2] + end[2]),
        ),
        rpy=(0.0, pitch, yaw),
    )
    return origin, length


def _profile_at_z(
    width: float,
    depth: float,
    radius: float,
    z: float,
    corner_segments: int = 10,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for x, y in rounded_rect_profile(
            width,
            depth,
            radius,
            corner_segments=corner_segments,
        )
    ]


def _build_base_disc_mesh():
    profile = [
        (0.0, 0.0),
        (0.110, 0.0),
        (0.168, 0.003),
        (0.184, 0.010),
        (0.188, 0.020),
        (0.178, 0.028),
        (0.120, 0.030),
        (0.0, 0.030),
    ]
    return mesh_from_geometry(
        LatheGeometry(profile, segments=56),
        ASSETS.mesh_path("bar_stool_base_disc.obj"),
    )


def _build_seat_pan_mesh():
    profiles = [
        _profile_at_z(0.320, 0.300, 0.055, 0.006),
        _profile_at_z(0.356, 0.334, 0.065, 0.018),
        _profile_at_z(0.346, 0.326, 0.060, 0.030),
    ]
    return mesh_from_geometry(
        LoftGeometry(profiles, cap=True, closed=True),
        ASSETS.mesh_path("bar_stool_seat_pan.obj"),
    )


def _build_seat_cushion_mesh():
    profiles = [
        _profile_at_z(0.350, 0.330, 0.070, 0.022),
        _profile_at_z(0.404, 0.384, 0.090, 0.048),
        _profile_at_z(0.430, 0.404, 0.105, 0.072),
        _profile_at_z(0.418, 0.394, 0.095, 0.092),
    ]
    return mesh_from_geometry(
        LoftGeometry(profiles, cap=True, closed=True),
        ASSETS.mesh_path("bar_stool_seat_cushion.obj"),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_bar_stool", assets=ASSETS)

    materials = {
        "brushed_steel": _make_material("brushed_steel", (0.72, 0.74, 0.76, 1.0)),
        "polished_steel": _make_material("polished_steel", (0.82, 0.83, 0.85, 1.0)),
        "black_plastic": _make_material("black_plastic", (0.10, 0.11, 0.12, 1.0)),
        "espresso_vinyl": _make_material("espresso_vinyl", (0.20, 0.13, 0.11, 1.0)),
        "rubber": _make_material("rubber", (0.07, 0.07, 0.08, 1.0)),
    }
    model.materials.extend(materials.values())

    base_disc_mesh = _build_base_disc_mesh()
    seat_pan_mesh = _build_seat_pan_mesh()
    seat_cushion_mesh = _build_seat_cushion_mesh()
    footrest_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.155, tube=0.011, radial_segments=18, tubular_segments=48),
        ASSETS.mesh_path("bar_stool_footrest_ring.obj"),
    )
    footrest_hub_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.078, tube=0.008, radial_segments=16, tubular_segments=40),
        ASSETS.mesh_path("bar_stool_footrest_hub.obj"),
    )

    base = model.part("base")
    base.visual(base_disc_mesh, material=materials["brushed_steel"], name="disc_base")
    base.visual(
        Cylinder(radius=0.168, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=materials["rubber"],
        name="floor_glide",
    )
    base.visual(
        Cylinder(radius=0.048, length=0.630),
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        material=materials["polished_steel"],
        name="main_column",
    )
    base.visual(
        Cylinder(radius=0.062, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=materials["polished_steel"],
        name="lower_trim",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.190, length=0.660),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
    )

    footrest = model.part("footrest")
    footrest.visual(
        footrest_hub_mesh,
        origin=Origin(xyz=(-0.086, 0.0, 0.0)),
        material=materials["polished_steel"],
        name="footrest_hub",
    )
    footrest.visual(
        footrest_ring_mesh,
        origin=Origin(xyz=(-0.086, 0.0, 0.0)),
        material=materials["polished_steel"],
        name="footrest_ring",
    )
    for idx, angle in enumerate((0.0, 0.5 * pi, pi, 1.5 * pi), start=1):
        inner = (-0.086 + 0.086 * cos(angle), 0.086 * sin(angle), -0.003)
        outer = (-0.086 + 0.144 * cos(angle), 0.144 * sin(angle), 0.0)
        brace_origin, brace_length = _segment_origin(inner, outer)
        footrest.visual(
            Cylinder(radius=0.009, length=brace_length),
            origin=brace_origin,
            material=materials["polished_steel"],
            name=f"brace_{idx}",
        )
    footrest.inertial = Inertial.from_geometry(
        Cylinder(radius=0.170, length=0.070),
        mass=1.8,
        origin=Origin(xyz=(-0.086, 0.0, 0.0)),
    )

    lift = model.part("lift")
    lift.visual(
        Cylinder(radius=0.058, length=0.260),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=materials["polished_steel"],
        name="upper_sleeve",
    )
    lift.visual(
        Cylinder(radius=0.061, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
        material=materials["black_plastic"],
        name="wiper_ring",
    )
    lift.visual(
        Cylinder(radius=0.066, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.146)),
        material=materials["polished_steel"],
        name="mount_flange",
    )
    lift.visual(
        Cylinder(radius=0.074, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.177)),
        material=materials["black_plastic"],
        name="swivel_housing",
    )
    support_arm_origin, support_arm_length = _segment_origin(
        (0.045, 0.0, -0.085),
        (0.086, 0.0, -0.085),
    )
    lift.visual(
        Cylinder(radius=0.010, length=support_arm_length),
        origin=support_arm_origin,
        material=materials["polished_steel"],
        name="footrest_support_arm",
    )
    lift.inertial = Inertial.from_geometry(
        Cylinder(radius=0.074, length=0.286),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.074, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=materials["black_plastic"],
        name="swivel_plate",
    )
    seat.visual(
        seat_pan_mesh,
        material=materials["black_plastic"],
        name="seat_pan",
    )
    seat.visual(
        seat_cushion_mesh,
        material=materials["espresso_vinyl"],
        name="seat_cushion",
    )
    lever_stem_origin, lever_stem_length = _segment_origin(
        (0.108, -0.062, 0.018),
        (0.144, -0.098, -0.004),
    )
    seat.visual(
        Cylinder(radius=0.004, length=lever_stem_length),
        origin=lever_stem_origin,
        material=materials["polished_steel"],
        name="height_lever_stem",
    )
    seat.visual(
        Cylinder(radius=0.005, length=0.050),
        origin=Origin(xyz=(0.144, -0.098, -0.004), rpy=(-0.5 * pi, 0.0, 0.0)),
        material=materials["black_plastic"],
        name="height_lever_grip",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.440, 0.420, 0.100)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    model.articulation(
        "base_to_lift",
        ArticulationType.PRISMATIC,
        parent="base",
        child="lift",
        origin=Origin(xyz=(0.0, 0.0, 0.520)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=0.18,
            lower=0.0,
            upper=0.100,
        ),
    )
    model.articulation(
        "lift_to_footrest",
        ArticulationType.FIXED,
        parent="lift",
        child="footrest",
        origin=Origin(xyz=(0.086, 0.0, -0.085)),
    )
    model.articulation(
        "lift_to_seat_swivel",
        ArticulationType.CONTINUOUS,
        parent="lift",
        child="seat",
        origin=Origin(xyz=(0.0, 0.0, 0.199)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=4.0,
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
        "base",
        "lift",
        reason="upper lift sleeve visually telescopes over the fixed center column for the gas-lift effect",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
    )

    ctx.expect_joint_motion_axis(
        "base_to_lift",
        "lift",
        world_axis="z",
        direction="positive",
        min_delta=0.04,
    )
    ctx.expect_aabb_overlap("footrest", "base", axes="xy", min_overlap=0.10)
    ctx.expect_origin_distance("lift", "base", axes="xy", max_dist=0.001)

    with ctx.pose(base_to_lift=0.0, lift_to_seat_swivel=0.0):
        ctx.expect_origin_distance("seat", "base", axes="xy", max_dist=0.03)
        ctx.expect_aabb_overlap("seat", "base", axes="xy", min_overlap=0.25)
        ctx.expect_aabb_overlap("seat", "footrest", axes="xy", min_overlap=0.20)
        ctx.expect_aabb_gap("seat", "lift", axis="z", max_gap=0.003, max_penetration=0.001)
        ctx.expect_aabb_gap("seat", "base", axis="z", max_gap=0.07, max_penetration=0.0)
        ctx.expect_aabb_gap("seat", "footrest", axis="z", max_gap=0.45, max_penetration=0.0)

    with ctx.pose(base_to_lift=0.0, lift_to_seat_swivel=0.5 * pi):
        ctx.expect_origin_distance("seat", "base", axes="xy", max_dist=0.03)
        ctx.expect_aabb_overlap("seat", "base", axes="xy", min_overlap=0.25)
        ctx.expect_aabb_gap("seat", "lift", axis="z", max_gap=0.003, max_penetration=0.001)

    with ctx.pose(base_to_lift=0.100, lift_to_seat_swivel=pi):
        ctx.expect_origin_distance("seat", "base", axes="xy", max_dist=0.03)
        ctx.expect_aabb_overlap("seat", "base", axes="xy", min_overlap=0.25)
        ctx.expect_aabb_overlap("seat", "footrest", axes="xy", min_overlap=0.20)
        ctx.expect_aabb_gap("seat", "lift", axis="z", max_gap=0.003, max_penetration=0.001)
        ctx.expect_aabb_gap("seat", "base", axis="z", max_gap=0.17, max_penetration=0.0)
        ctx.expect_aabb_gap("seat", "footrest", axis="z", max_gap=0.55, max_penetration=0.0)

    with ctx.pose(base_to_lift=0.100, lift_to_seat_swivel=1.5 * pi):
        ctx.expect_origin_distance("seat", "base", axes="xy", max_dist=0.03)
        ctx.expect_aabb_overlap("seat", "base", axes="xy", min_overlap=0.25)
        ctx.expect_aabb_gap("seat", "lift", axis="z", max_gap=0.003, max_penetration=0.001)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir


def _build_deck_mesh():
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    deck_profile = [
        (-0.055, -0.255),
        (0.055, -0.255),
        (0.064, -0.225),
        (0.064, 0.090),
        (0.050, 0.215),
        (0.028, 0.255),
        (-0.028, 0.255),
        (-0.050, 0.215),
        (-0.064, 0.090),
        (-0.064, -0.225),
    ]
    deck_geom = ExtrudeGeometry.from_z0(deck_profile, 0.018, cap=True, closed=True)
    return mesh_from_geometry(deck_geom, MESH_DIR / "folding_scooter_deck.obj")


def _build_handlebar_mesh():
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    handlebar_geom = tube_from_spline_points(
        [
            (-0.235, -0.012, 0.0),
            (-0.175, -0.008, 0.003),
            (-0.090, -0.003, 0.001),
            (0.0, 0.0, 0.0),
            (0.090, 0.003, 0.001),
            (0.175, -0.008, 0.003),
            (0.235, -0.012, 0.0),
        ],
        radius=0.012,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(handlebar_geom, MESH_DIR / "folding_scooter_handlebar.obj")


def _add_wheel_visuals(part, tire_radius: float = 0.100, tire_width: float = 0.024) -> None:
    wheel_rotation = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(Cylinder(radius=tire_radius, length=tire_width), origin=wheel_rotation)
    part.visual(Cylinder(radius=0.086, length=tire_width + 0.002), origin=wheel_rotation)
    part.visual(Cylinder(radius=0.076, length=tire_width + 0.006), origin=wheel_rotation)
    part.visual(Cylinder(radius=0.030, length=0.042), origin=wheel_rotation)

    spoke_box = Box((0.008, 0.058, 0.018))
    for i in range(6):
        angle = i * (pi / 3.0)
        y = 0.051 * cos(angle)
        z = 0.051 * sin(angle)
        part.visual(
            spoke_box,
            origin=Origin(xyz=(0.0, y, z), rpy=(angle, 0.0, 0.0)),
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_kick_scooter", assets=ASSETS)

    deck_mesh = _build_deck_mesh()
    handlebar_mesh = _build_handlebar_mesh()

    deck = model.part("deck")
    deck.visual(deck_mesh, origin=Origin(xyz=(0.0, 0.0, 0.017)))
    deck.visual(Box((0.108, 0.335, 0.002)), origin=Origin(xyz=(0.0, -0.010, 0.036)))
    deck.visual(Box((0.006, 0.405, 0.010)), origin=Origin(xyz=(0.053, -0.015, 0.030)))
    deck.visual(Box((0.006, 0.405, 0.010)), origin=Origin(xyz=(-0.053, -0.015, 0.030)))
    deck.visual(Box((0.058, 0.060, 0.086)), origin=Origin(xyz=(0.0, 0.220, 0.078)))
    deck.visual(Box((0.014, 0.046, 0.058)), origin=Origin(xyz=(0.026, 0.220, 0.092)))
    deck.visual(Box((0.014, 0.046, 0.058)), origin=Origin(xyz=(-0.026, 0.220, 0.092)))
    deck.visual(Box((0.010, 0.045, 0.064)), origin=Origin(xyz=(0.030, -0.236, 0.067)))
    deck.visual(Box((0.010, 0.045, 0.064)), origin=Origin(xyz=(-0.030, -0.236, 0.067)))
    deck.visual(
        Cylinder(radius=0.006, length=0.090),
        origin=Origin(xyz=(0.0, -0.236, 0.100), rpy=(0.0, pi / 2.0, 0.0)),
    )
    deck.visual(Box((0.028, 0.018, 0.100)), origin=Origin(xyz=(0.0, -0.236, 0.085)))
    deck.visual(Box((0.090, 0.040, 0.018)), origin=Origin(xyz=(0.0, -0.244, 0.144)))
    deck.visual(Box((0.064, 0.014, 0.008)), origin=Origin(xyz=(0.0, -0.270, 0.153)))
    deck.inertial = Inertial.from_geometry(
        Box((0.130, 0.540, 0.170)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )

    stem_base = model.part("stem_base")
    stem_base.visual(Box((0.030, 0.022, 0.020)), origin=Origin(xyz=(0.0, 0.0, 0.010)))
    stem_base.visual(Box((0.008, 0.038, 0.064)), origin=Origin(xyz=(0.019, 0.0, 0.032)))
    stem_base.visual(Box((0.008, 0.038, 0.064)), origin=Origin(xyz=(-0.019, 0.0, 0.032)))
    stem_base.visual(Box((0.038, 0.060, 0.036)), origin=Origin(xyz=(0.0, 0.018, 0.050)))
    stem_base.visual(Cylinder(radius=0.020, length=0.170), origin=Origin(xyz=(0.0, 0.010, 0.105)))
    stem_base.visual(Cylinder(radius=0.024, length=0.034), origin=Origin(xyz=(0.0, 0.010, 0.192)))
    stem_base.visual(Box((0.034, 0.020, 0.026)), origin=Origin(xyz=(0.0, -0.018, 0.018)))
    stem_base.visual(Box((0.012, 0.018, 0.032)), origin=Origin(xyz=(0.0, -0.034, 0.030)))
    stem_base.visual(Box((0.016, 0.014, 0.050)), origin=Origin(xyz=(0.0, -0.012, 0.118)))
    stem_base.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.100)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.008, 0.110)),
    )

    steering_assembly = model.part("steering_assembly")
    steering_assembly.visual(
        Cylinder(radius=0.023, length=0.036), origin=Origin(xyz=(0.0, 0.0, -0.092))
    )
    steering_assembly.visual(
        Cylinder(radius=0.022, length=0.052), origin=Origin(xyz=(0.0, 0.0, -0.060))
    )
    steering_assembly.visual(
        Cylinder(radius=0.018, length=0.520), origin=Origin(xyz=(0.0, 0.0, 0.226))
    )
    steering_assembly.visual(Box((0.036, 0.060, 0.040)), origin=Origin(xyz=(0.0, 0.034, -0.090)))
    steering_assembly.visual(Box((0.082, 0.050, 0.036)), origin=Origin(xyz=(0.0, 0.082, -0.086)))
    steering_assembly.visual(Box((0.060, 0.024, 0.018)), origin=Origin(xyz=(0.0, 0.110, -0.090)))
    steering_assembly.visual(Box((0.012, 0.028, 0.156)), origin=Origin(xyz=(0.029, 0.122, -0.126)))
    steering_assembly.visual(Box((0.012, 0.028, 0.156)), origin=Origin(xyz=(-0.029, 0.122, -0.126)))
    steering_assembly.visual(
        Cylinder(radius=0.006, length=0.082),
        origin=Origin(xyz=(0.0, 0.136, -0.204), rpy=(0.0, pi / 2.0, 0.0)),
    )
    steering_assembly.visual(Box((0.072, 0.042, 0.040)), origin=Origin(xyz=(0.0, -0.004, 0.506)))
    steering_assembly.visual(Box((0.024, 0.024, 0.018)), origin=Origin(xyz=(0.0, -0.002, 0.532)))
    steering_assembly.visual(handlebar_mesh, origin=Origin(xyz=(0.0, -0.008, 0.526)))
    steering_assembly.visual(Box((0.058, 0.032, 0.018)), origin=Origin(xyz=(0.0, 0.006, 0.546)))
    steering_assembly.visual(
        Cylinder(radius=0.017, length=0.095),
        origin=Origin(xyz=(0.212, -0.008, 0.526), rpy=(0.0, pi / 2.0, 0.0)),
    )
    steering_assembly.visual(
        Cylinder(radius=0.017, length=0.095),
        origin=Origin(xyz=(-0.212, -0.008, 0.526), rpy=(0.0, pi / 2.0, 0.0)),
    )
    steering_assembly.visual(
        Box((0.008, 0.046, 0.016)),
        origin=Origin(xyz=(-0.176, -0.024, 0.510), rpy=(0.0, 0.0, -0.72)),
    )
    steering_assembly.inertial = Inertial.from_geometry(
        Box((0.500, 0.140, 0.760)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.025, 0.318)),
    )

    front_wheel = model.part("front_wheel")
    _add_wheel_visuals(front_wheel)
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.100, length=0.024),
        mass=0.7,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    rear_wheel = model.part("rear_wheel")
    _add_wheel_visuals(rear_wheel)
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.100, length=0.024),
        mass=0.7,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "stem_fold",
        ArticulationType.REVOLUTE,
        parent="deck",
        child="stem_base",
        origin=Origin(xyz=(0.0, 0.220, 0.124)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=0.72),
    )
    model.articulation(
        "steering_head",
        ArticulationType.REVOLUTE,
        parent="stem_base",
        child="steering_assembly",
        origin=Origin(xyz=(0.0, 0.010, 0.190)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0, lower=-0.90, upper=0.90),
    )
    model.articulation(
        "front_wheel_roll",
        ArticulationType.CONTINUOUS,
        parent="steering_assembly",
        child="front_wheel",
        origin=Origin(xyz=(0.0, 0.136, -0.204)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=25.0),
    )
    model.articulation(
        "rear_wheel_roll",
        ArticulationType.CONTINUOUS,
        parent="deck",
        child="rear_wheel",
        origin=Origin(xyz=(0.0, -0.236, 0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "deck",
        "rear_wheel",
        reason="rear axle sleeve passes through the wheel hub and the generated wheel collision stays conservative around the hub",
    )
    ctx.allow_overlap(
        "steering_assembly",
        "front_wheel",
        reason="front axle bridge and the wheel hub share the fork dropout centerline",
    )
    ctx.allow_overlap(
        "steering_assembly",
        "stem_base",
        reason="the steering tube is intentionally nested inside the folding headset sleeve",
    )
    ctx.allow_overlap(
        "deck",
        "stem_base",
        reason="at full fold the latch shoe tucks into the deck-side catch and the generated hulls slightly overestimate that compact contact",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.005, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("rear_wheel", "deck", axes="xy", min_overlap=0.04)
    ctx.expect_aabb_overlap("stem_base", "deck", axes="xy", min_overlap=0.02)
    ctx.expect_origin_distance("front_wheel", "steering_assembly", axes="xy", max_dist=0.18)
    ctx.expect_joint_motion_axis(
        "stem_fold",
        "steering_assembly",
        world_axis="y",
        direction="negative",
        min_delta=0.18,
    )
    ctx.expect_joint_motion_axis(
        "stem_fold",
        "steering_assembly",
        world_axis="z",
        direction="negative",
        min_delta=0.04,
    )
    ctx.expect_joint_motion_axis(
        "steering_head",
        "front_wheel",
        world_axis="x",
        direction="negative",
        min_delta=0.08,
    )

    rear_pos = ctx.part_world_position("rear_wheel")
    front_pos = ctx.part_world_position("front_wheel")
    stem_pos = ctx.part_world_position("steering_assembly")
    if front_pos[1] - rear_pos[1] <= 0.55:
        raise AssertionError("Wheelbase should read as a full-size consumer scooter.")
    if front_pos[1] <= 0.30:
        raise AssertionError(
            "The front wheel should project clearly ahead of the deck for a stable scooter stance."
        )
    if stem_pos[2] <= rear_pos[2] + 0.05:
        raise AssertionError("The steering head should sit visibly above the wheel centers.")

    with ctx.pose(steering_head=0.55):
        front_left = ctx.part_world_position("front_wheel")
        if front_left[0] >= -0.05:
            raise AssertionError(
                "Positive steering should swing the front wheel to the rider's left."
            )
        ctx.expect_aabb_overlap("front_wheel", "steering_assembly", axes="xy", min_overlap=0.015)
        ctx.expect_origin_distance("front_wheel", "steering_assembly", axes="xy", max_dist=0.20)

    with ctx.pose(steering_head=-0.55):
        front_right = ctx.part_world_position("front_wheel")
        if front_right[0] <= 0.05:
            raise AssertionError(
                "Negative steering should swing the front wheel to the rider's right."
            )
        ctx.expect_aabb_overlap("front_wheel", "steering_assembly", axes="xy", min_overlap=0.015)
        ctx.expect_origin_distance("front_wheel", "steering_assembly", axes="xy", max_dist=0.20)

    with ctx.pose(stem_fold=0.65):
        folded_stem = ctx.part_world_position("steering_assembly")
        if folded_stem[1] >= stem_pos[1] - 0.02:
            raise AssertionError("The folded stem should shift back over the deck.")
        if folded_stem[2] >= stem_pos[2] - 0.03:
            raise AssertionError("The folded stem should also settle lower toward the deck.")
        ctx.expect_aabb_overlap("steering_assembly", "deck", axes="xy", min_overlap=0.06)

    with ctx.pose(stem_fold=0.65, steering_head=0.35):
        ctx.expect_aabb_overlap("front_wheel", "steering_assembly", axes="xy", min_overlap=0.012)
        ctx.expect_origin_distance("front_wheel", "steering_assembly", axes="xy", max_dist=0.24)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

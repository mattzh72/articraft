from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stunt_kick_scooter")

    deck_alloy = model.material("deck_alloy", rgba=(0.19, 0.20, 0.22, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_black = model.material("grip_black", rgba=(0.04, 0.04, 0.04, 1.0))
    fork_black = model.material("fork_black", rgba=(0.10, 0.10, 0.11, 1.0))
    wheel_core = model.material("wheel_core", rgba=(0.70, 0.72, 0.76, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    rubber = model.material("rubber", rgba=(0.03, 0.03, 0.03, 1.0))

    def _deck_outline() -> list[tuple[float, float]]:
        return [
            (-0.175, -0.058),
            (-0.156, -0.058),
            (0.120, -0.052),
            (0.150, -0.046),
            (0.170, -0.038),
            (0.182, -0.030),
            (0.188, -0.018),
            (0.190, -0.008),
            (0.190, 0.008),
            (0.188, 0.018),
            (0.182, 0.030),
            (0.170, 0.038),
            (0.150, 0.046),
            (0.120, 0.052),
            (-0.156, 0.058),
            (-0.175, 0.058),
        ]

    def _wheel_part(part_name: str) -> object:
        wheel = model.part(part_name)
        wheel.visual(
            Cylinder(radius=0.055, length=0.024),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.042, length=0.019),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=wheel_core,
            name="core",
        )
        wheel.visual(
            Cylinder(radius=0.015, length=0.030),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=axle_steel,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.010, length=0.034),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=axle_steel,
            name="axle",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.055, length=0.024),
            mass=0.42,
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        )
        return wheel

    deck = model.part("deck")

    deck_shell_geom = ExtrudeGeometry.from_z0(_deck_outline(), 0.022)
    deck_shell_geom.translate(0.0, 0.0, 0.070)
    deck.visual(
        mesh_from_geometry(deck_shell_geom, "scooter_deck_shell"),
        material=deck_alloy,
        name="deck_shell",
    )

    deck.visual(
        Box((0.320, 0.080, 0.0015)),
        origin=Origin(xyz=(-0.004, 0.0, 0.09275)),
        material=grip_black,
        name="grip_tape",
    )
    deck.visual(
        Box((0.045, 0.070, 0.018)),
        origin=Origin(xyz=(0.158, 0.0, 0.079)),
        material=deck_alloy,
        name="front_neck_block",
    )
    deck.visual(
        Box((0.056, 0.042, 0.020)),
        origin=Origin(xyz=(0.206, 0.0, 0.082)),
        material=deck_alloy,
        name="headtube_bridge",
    )
    deck.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(0.238, 0.0, 0.092)),
        material=deck_alloy,
        name="lower_headset_race",
    )
    deck.visual(
        Box((0.118, 0.008, 0.040)),
        origin=Origin(xyz=(-0.228, 0.022, 0.074)),
        material=fork_black,
        name="rear_left_dropout",
    )
    deck.visual(
        Box((0.118, 0.008, 0.040)),
        origin=Origin(xyz=(-0.228, -0.022, 0.074)),
        material=fork_black,
        name="rear_right_dropout",
    )
    deck.visual(
        Box((0.095, 0.028, 0.004)),
        origin=Origin(xyz=(-0.234, 0.0, 0.115)),
        material=fork_black,
        name="rear_fender",
    )
    deck.visual(
        Box((0.010, 0.012, 0.012)),
        origin=Origin(xyz=(-0.086, 0.054, 0.075)),
        material=fork_black,
        name="kickstand_bracket_web",
    )
    deck.visual(
        Box((0.022, 0.010, 0.020)),
        origin=Origin(xyz=(-0.240, 0.016, 0.103)),
        material=fork_black,
        name="rear_fender_left_support",
    )
    deck.visual(
        Box((0.022, 0.010, 0.020)),
        origin=Origin(xyz=(-0.240, -0.016, 0.103)),
        material=fork_black,
        name="rear_fender_right_support",
    )
    deck.visual(
        Cylinder(radius=0.0045, length=0.006),
        origin=Origin(xyz=(-0.093, 0.048, 0.067), rpy=(0.0, pi / 2.0, 0.0)),
        material=fork_black,
        name="kickstand_bracket_front_barrel",
    )
    deck.visual(
        Cylinder(radius=0.0045, length=0.006),
        origin=Origin(xyz=(-0.079, 0.048, 0.067), rpy=(0.0, pi / 2.0, 0.0)),
        material=fork_black,
        name="kickstand_bracket_rear_barrel",
    )

    left_neck_brace = tube_from_spline_points(
        [
            (0.148, 0.026, 0.082),
            (0.192, 0.026, 0.094),
            (0.224, 0.020, 0.108),
        ],
        radius=0.009,
        samples_per_segment=12,
        radial_segments=14,
    )
    deck.visual(
        mesh_from_geometry(left_neck_brace, "scooter_left_neck_brace"),
        material=fork_black,
        name="left_neck_brace",
    )
    right_neck_brace = tube_from_spline_points(
        [
            (0.148, -0.026, 0.082),
            (0.192, -0.026, 0.094),
            (0.224, -0.020, 0.108),
        ],
        radius=0.009,
        samples_per_segment=12,
        radial_segments=14,
    )
    deck.visual(
        mesh_from_geometry(right_neck_brace, "scooter_right_neck_brace"),
        material=fork_black,
        name="right_neck_brace",
    )

    deck.inertial = Inertial.from_geometry(
        Box((0.58, 0.12, 0.08)),
        mass=2.15,
        origin=Origin(xyz=(0.015, 0.0, 0.082)),
    )

    front_assembly = model.part("front_assembly")
    front_assembly.visual(
        Cylinder(radius=0.016, length=0.720),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=matte_black,
        name="stem_tube",
    )
    front_assembly.visual(
        Box((0.050, 0.040, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.640)),
        material=matte_black,
        name="bar_clamp",
    )
    front_assembly.visual(
        Cylinder(radius=0.016, length=0.520),
        origin=Origin(xyz=(0.0, 0.0, 0.690), rpy=(pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="handlebar_bar",
    )
    front_assembly.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(xyz=(0.0, 0.205, 0.690), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    front_assembly.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(xyz=(0.0, -0.205, 0.690), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    front_assembly.visual(
        Cylinder(radius=0.012, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=fork_black,
        name="steerer_lower",
    )
    front_assembly.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=fork_black,
        name="lower_headset_collar",
    )
    front_assembly.visual(
        Box((0.032, 0.010, 0.018)),
        origin=Origin(xyz=(0.008, 0.018, 0.034)),
        material=fork_black,
        name="left_fork_yoke",
    )
    front_assembly.visual(
        Box((0.032, 0.010, 0.018)),
        origin=Origin(xyz=(0.008, -0.018, 0.034)),
        material=fork_black,
        name="right_fork_yoke",
    )
    left_fork_leg = tube_from_spline_points(
        [
            (0.018, 0.0225, 0.028),
            (0.036, 0.0225, -0.006),
            (0.068, 0.0225, -0.055),
        ],
        radius=0.004,
        samples_per_segment=12,
        radial_segments=14,
    )
    front_assembly.visual(
        mesh_from_geometry(left_fork_leg, "front_left_fork_leg"),
        material=fork_black,
        name="left_fork_leg",
    )
    right_fork_leg = tube_from_spline_points(
        [
            (0.018, -0.0225, 0.028),
            (0.036, -0.0225, -0.006),
            (0.068, -0.0225, -0.055),
        ],
        radius=0.004,
        samples_per_segment=12,
        radial_segments=14,
    )
    front_assembly.visual(
        mesh_from_geometry(right_fork_leg, "front_right_fork_leg"),
        material=fork_black,
        name="right_fork_leg",
    )
    front_assembly.visual(
        Box((0.012, 0.008, 0.020)),
        origin=Origin(xyz=(0.068, 0.021, -0.055)),
        material=fork_black,
        name="left_dropout",
    )
    front_assembly.visual(
        Box((0.012, 0.008, 0.020)),
        origin=Origin(xyz=(0.068, -0.021, -0.055)),
        material=fork_black,
        name="right_dropout",
    )
    front_assembly.inertial = Inertial.from_geometry(
        Box((0.56, 0.07, 0.82)),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
    )

    front_wheel = _wheel_part("front_wheel")
    rear_wheel = _wheel_part("rear_wheel")

    kickstand = model.part("kickstand")
    kickstand.visual(
        Cylinder(radius=0.0042, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=fork_black,
        name="kickstand_hinge_barrel",
    )
    kickstand.visual(
        Cylinder(radius=0.006, length=0.082),
        origin=Origin(xyz=(0.0, 0.038, -0.010), rpy=(pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="kickstand_leg",
    )
    kickstand.visual(
        Box((0.018, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, 0.078, -0.010)),
        material=matte_black,
        name="kickstand_foot",
    )
    kickstand.inertial = Inertial.from_geometry(
        Box((0.018, 0.090, 0.018)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.040, -0.010)),
    )

    model.articulation(
        "deck_to_front_assembly",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_assembly,
        origin=Origin(xyz=(0.238, 0.0, 0.096)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=3.0,
            lower=-0.85,
            upper=0.85,
        ),
    )
    model.articulation(
        "front_assembly_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_assembly,
        child=front_wheel,
        origin=Origin(xyz=(0.068, 0.0, -0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=45.0,
        ),
    )
    model.articulation(
        "deck_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.234, 0.0, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=45.0,
        ),
    )
    model.articulation(
        "deck_to_kickstand",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=kickstand,
        origin=Origin(xyz=(-0.086, 0.048, 0.067)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=1.02,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    deck = object_model.get_part("deck")
    front_assembly = object_model.get_part("front_assembly")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    kickstand = object_model.get_part("kickstand")

    steering = object_model.get_articulation("deck_to_front_assembly")
    front_spin = object_model.get_articulation("front_assembly_to_front_wheel")
    rear_spin = object_model.get_articulation("deck_to_rear_wheel")
    stand_hinge = object_model.get_articulation("deck_to_kickstand")

    ctx.check(
        "steering joint is vertical yaw",
        steering.axis == (0.0, 0.0, 1.0)
        and steering.motion_limits is not None
        and steering.motion_limits.lower is not None
        and steering.motion_limits.upper is not None
        and steering.motion_limits.lower < 0.0 < steering.motion_limits.upper,
        details=f"axis={steering.axis}, limits={steering.motion_limits}",
    )
    ctx.check(
        "wheel joints spin on axle axes",
        front_spin.axis == (0.0, 1.0, 0.0)
        and rear_spin.axis == (0.0, 1.0, 0.0)
        and front_spin.motion_limits is not None
        and rear_spin.motion_limits is not None
        and front_spin.motion_limits.lower is None
        and front_spin.motion_limits.upper is None
        and rear_spin.motion_limits.lower is None
        and rear_spin.motion_limits.upper is None,
        details=(
            f"front_axis={front_spin.axis}, rear_axis={rear_spin.axis}, "
            f"front_limits={front_spin.motion_limits}, rear_limits={rear_spin.motion_limits}"
        ),
    )
    ctx.expect_origin_gap(
        front_wheel,
        rear_wheel,
        axis="x",
        min_gap=0.42,
        max_gap=0.55,
        name="scooter wheelbase is realistic",
    )

    front_rest = ctx.part_world_position(front_wheel)
    with ctx.pose({steering: 0.45}):
        front_turned = ctx.part_world_position(front_wheel)
    ctx.check(
        "steering swings the fork laterally",
        front_rest is not None
        and front_turned is not None
        and abs(front_turned[1] - front_rest[1]) > 0.010,
        details=f"rest={front_rest}, steered={front_turned}",
    )

    deck_aabb = ctx.part_world_aabb(deck)
    bars_aabb = ctx.part_world_aabb(front_assembly)
    ctx.check(
        "bar rises well above the deck",
        deck_aabb is not None
        and bars_aabb is not None
        and bars_aabb[1][2] > deck_aabb[1][2] + 0.65,
        details=f"deck_aabb={deck_aabb}, bars_aabb={bars_aabb}",
    )

    kickstand_rest = ctx.part_world_aabb(kickstand)
    with ctx.pose({stand_hinge: 1.0}):
        kickstand_deployed = ctx.part_world_aabb(kickstand)
    ctx.check(
        "kickstand folds down from its tucked position",
        kickstand_rest is not None
        and kickstand_deployed is not None
        and kickstand_deployed[0][2] < kickstand_rest[0][2] - 0.035,
        details=f"rest={kickstand_rest}, deployed={kickstand_deployed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _wheel_visuals(
    part,
    prefix: str,
    *,
    tire_radius: float,
    tire_width: float,
    hub_width: float,
    rubber,
    rim_metal,
    hub_metal,
) -> None:
    half_width = tire_width * 0.5
    tire_profile = [
        (tire_radius * 0.46, -half_width * 0.96),
        (tire_radius * 0.70, -half_width),
        (tire_radius * 0.90, -half_width * 0.82),
        (tire_radius * 0.99, -half_width * 0.40),
        (tire_radius, 0.0),
        (tire_radius * 0.99, half_width * 0.40),
        (tire_radius * 0.90, half_width * 0.82),
        (tire_radius * 0.70, half_width),
        (tire_radius * 0.46, half_width * 0.96),
        (tire_radius * 0.36, half_width * 0.24),
        (tire_radius * 0.34, 0.0),
        (tire_radius * 0.36, -half_width * 0.24),
        (tire_radius * 0.46, -half_width * 0.96),
    ]
    tire_geom = LatheGeometry(tire_profile, segments=64)
    tire_geom.rotate_x(pi / 2.0)
    part.visual(_save_mesh(f"{prefix}_tire", tire_geom), material=rubber, name="tire")

    spin_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=tire_radius * 0.72, length=hub_width * 0.88),
        origin=spin_origin,
        material=rim_metal,
        name="rim_barrel",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.60, length=hub_width * 0.18),
        origin=Origin(xyz=(0.0, hub_width * 0.34, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rim_metal,
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.60, length=hub_width * 0.18),
        origin=Origin(xyz=(0.0, -hub_width * 0.34, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rim_metal,
    )
    for spoke_index in range(6):
        spoke_angle = spoke_index * (pi / 3.0)
        part.visual(
            Box((tire_radius * 0.58, 0.008, 0.018)),
            origin=Origin(
                xyz=(tire_radius * 0.28, 0.0, 0.0),
                rpy=(0.0, spoke_angle, 0.0),
            ),
            material=rim_metal,
        )
    part.visual(
        Cylinder(radius=tire_radius * 0.20, length=hub_width),
        origin=spin_origin,
        material=hub_metal,
        name="hub_shell",
    )
    part.visual(
        Cylinder(radius=0.011, length=hub_width + 0.012),
        origin=spin_origin,
        material=hub_metal,
        name="axle",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="off_road_kick_scooter")

    frame_green = model.material("frame_green", rgba=(0.24, 0.31, 0.25, 1.0))
    charcoal = model.material("charcoal", rgba=(0.13, 0.14, 0.15, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    alloy = model.material("alloy", rgba=(0.71, 0.74, 0.77, 1.0))
    fork_gold = model.material("fork_gold", rgba=(0.71, 0.58, 0.18, 1.0))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.04, 1.0))

    wheel_radius = 0.18
    tire_width = 0.078
    hub_width = 0.092

    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((1.18, 0.22, 0.30)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
    )

    deck_profile = [
        (-0.34, -0.045),
        (-0.32, -0.070),
        (-0.20, -0.078),
        (0.17, -0.078),
        (0.26, -0.070),
        (0.31, -0.050),
        (0.33, -0.022),
        (0.34, 0.0),
        (0.33, 0.022),
        (0.31, 0.050),
        (0.26, 0.068),
        (0.17, 0.078),
        (-0.20, 0.078),
        (-0.32, 0.070),
        (-0.34, 0.045),
    ]
    deck_geom = ExtrudeGeometry.from_z0(deck_profile, 0.050)
    chassis.visual(
        _save_mesh("deck_shell", deck_geom),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=frame_green,
        name="deck_shell",
    )
    chassis.visual(
        Box((0.74, 0.142, 0.006)),
        origin=Origin(xyz=(-0.01, 0.0, 0.158)),
        material=charcoal,
        name="grip_pad",
    )
    chassis.visual(
        Box((0.60, 0.026, 0.048)),
        origin=Origin(xyz=(-0.02, 0.055, 0.104)),
        material=dark_metal,
    )
    chassis.visual(
        Box((0.60, 0.026, 0.048)),
        origin=Origin(xyz=(-0.02, -0.055, 0.104)),
        material=dark_metal,
    )

    left_rear_stay = tube_from_spline_points(
        [(-0.18, 0.060, 0.112), (-0.30, 0.063, 0.124), (-0.43, 0.068, 0.174), (-0.56, 0.072, 0.236)],
        radius=0.012,
        samples_per_segment=12,
        radial_segments=16,
    )
    right_rear_stay = tube_from_spline_points(
        [(-0.18, -0.060, 0.112), (-0.30, -0.063, 0.124), (-0.43, -0.068, 0.174), (-0.56, -0.072, 0.236)],
        radius=0.012,
        samples_per_segment=12,
        radial_segments=16,
    )
    chassis.visual(_save_mesh("left_rear_stay", left_rear_stay), material=dark_metal)
    chassis.visual(_save_mesh("right_rear_stay", right_rear_stay), material=dark_metal)
    chassis.visual(
        Box((0.090, 0.008, 0.110)),
        origin=Origin(xyz=(-0.56, 0.056, 0.191)),
        material=dark_metal,
        name="rear_left_dropout",
    )
    chassis.visual(
        Box((0.090, 0.008, 0.110)),
        origin=Origin(xyz=(-0.56, -0.056, 0.191)),
        material=dark_metal,
        name="rear_right_dropout",
    )

    nose_brace = sweep_profile_along_spline(
        [(0.18, 0.0, 0.128), (0.26, 0.0, 0.152), (0.34, 0.0, 0.202)],
        profile=rounded_rect_profile(0.075, 0.055, radius=0.014, corner_segments=6),
        samples_per_segment=12,
        cap_profile=True,
    )
    chassis.visual(_save_mesh("nose_brace", nose_brace), material=dark_metal, name="nose_brace")
    chassis.visual(
        Cylinder(radius=0.036, length=0.060),
        origin=Origin(xyz=(0.42, 0.0, 0.200)),
        material=dark_metal,
        name="head_tube",
    )
    chassis.visual(
        Box((0.090, 0.090, 0.050)),
        origin=Origin(xyz=(0.34, 0.0, 0.175)),
        material=frame_green,
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=hub_width),
        mass=3.8,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(
        rear_wheel,
        "rear_wheel",
        tire_radius=wheel_radius,
        tire_width=tire_width,
        hub_width=hub_width,
        rubber=rubber,
        rim_metal=alloy,
        hub_metal=dark_metal,
    )

    front_assembly = model.part("front_assembly")
    front_assembly.inertial = Inertial.from_geometry(
        Box((0.30, 0.16, 0.42)),
        mass=5.5,
        origin=Origin(xyz=(0.09, 0.0, 0.08)),
    )
    front_assembly.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_metal,
        name="lower_bearing_collar",
    )
    front_assembly.visual(
        Cylinder(radius=0.022, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=dark_metal,
        name="steering_column_lower",
    )
    front_assembly.visual(
        Box((0.070, 0.036, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=frame_green,
        name="hinge_block",
    )
    front_assembly.visual(
        Box((0.085, 0.030, 0.070)),
        origin=Origin(xyz=(0.040, 0.0, 0.140)),
        material=dark_metal,
        name="fork_neck",
    )
    front_assembly.visual(
        Box((0.125, 0.110, 0.024)),
        origin=Origin(xyz=(0.125, 0.0, 0.167)),
        material=frame_green,
        name="fork_crown",
    )
    front_assembly.visual(
        Cylinder(radius=0.012, length=0.130),
        origin=Origin(xyz=(0.155, 0.055, 0.105)),
        material=fork_gold,
        name="left_stanchion",
    )
    front_assembly.visual(
        Cylinder(radius=0.012, length=0.130),
        origin=Origin(xyz=(0.155, -0.055, 0.105)),
        material=fork_gold,
        name="right_stanchion",
    )
    front_assembly.visual(
        Box((0.120, 0.012, 0.200)),
        origin=Origin(xyz=(0.220, 0.058, -0.040)),
        material=charcoal,
        name="left_lower_leg",
    )
    front_assembly.visual(
        Box((0.120, 0.012, 0.200)),
        origin=Origin(xyz=(0.220, -0.058, -0.040)),
        material=charcoal,
        name="right_lower_leg",
    )
    front_assembly.visual(
        Box((0.040, 0.008, 0.060)),
        origin=Origin(xyz=(0.290, 0.056, -0.050)),
        material=charcoal,
        name="left_dropout_tab",
    )
    front_assembly.visual(
        Box((0.040, 0.008, 0.060)),
        origin=Origin(xyz=(0.290, -0.056, -0.050)),
        material=charcoal,
        name="right_dropout_tab",
    )
    front_assembly.visual(
        Box((0.120, 0.120, 0.014)),
        origin=Origin(xyz=(0.280, 0.0, 0.148)),
        material=frame_green,
        name="front_fender",
    )
    front_assembly.visual(
        Box((0.040, 0.008, 0.130)),
        origin=Origin(xyz=(0.240, 0.055, 0.085)),
        material=charcoal,
    )
    front_assembly.visual(
        Box((0.040, 0.008, 0.130)),
        origin=Origin(xyz=(0.240, -0.055, 0.085)),
        material=charcoal,
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=hub_width),
        mass=3.8,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(
        front_wheel,
        "front_wheel",
        tire_radius=wheel_radius,
        tire_width=tire_width,
        hub_width=hub_width,
        rubber=rubber,
        rim_metal=alloy,
        hub_metal=dark_metal,
    )

    stem = model.part("stem")
    stem.inertial = Inertial.from_geometry(
        Box((0.24, 0.66, 1.02)),
        mass=4.4,
        origin=Origin(xyz=(-0.04, 0.0, 0.44)),
    )
    stem.visual(
        Box((0.070, 0.010, 0.100)),
        origin=Origin(xyz=(0.0, 0.023, 0.005)),
        material=dark_metal,
        name="left_clevis_plate",
    )
    stem.visual(
        Box((0.070, 0.010, 0.100)),
        origin=Origin(xyz=(0.0, -0.023, 0.005)),
        material=dark_metal,
        name="right_clevis_plate",
    )
    stem.visual(
        Box((0.065, 0.046, 0.060)),
        origin=Origin(xyz=(-0.0675, 0.0, 0.055)),
        material=frame_green,
        name="stem_base_clamp",
    )
    stem_spine = tube_from_spline_points(
        [(-0.045, 0.0, 0.085), (-0.045, 0.0, 0.340), (-0.060, 0.0, 0.730), (-0.060, 0.0, 0.860)],
        radius=0.022,
        samples_per_segment=16,
        radial_segments=18,
    )
    stem.visual(_save_mesh("stem_spine", stem_spine), material=dark_metal, name="stem_spine")
    stem.visual(
        Box((0.050, 0.040, 0.110)),
        origin=Origin(xyz=(-0.045, 0.0, 0.140)),
        material=frame_green,
        name="stem_head",
    )
    stem.visual(
        Cylinder(radius=0.018, length=0.120),
        origin=Origin(xyz=(-0.060, 0.0, 0.850)),
        material=dark_metal,
        name="bar_riser",
    )
    stem.visual(
        Cylinder(radius=0.016, length=0.620),
        origin=Origin(xyz=(-0.060, 0.0, 0.910), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="handlebar",
    )
    stem.visual(
        Cylinder(radius=0.020, length=0.130),
        origin=Origin(xyz=(-0.060, 0.245, 0.910), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
    )
    stem.visual(
        Cylinder(radius=0.020, length=0.130),
        origin=Origin(xyz=(-0.060, -0.245, 0.910), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
    )

    steering_joint = model.articulation(
        "steering_yaw",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_assembly,
        origin=Origin(xyz=(0.42, 0.0, 0.230)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.60, upper=0.60),
    )
    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=rear_wheel,
        origin=Origin(xyz=(-0.56, 0.0, 0.180)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_assembly,
        child=front_wheel,
        origin=Origin(xyz=(0.280, 0.0, -0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )
    model.articulation(
        "stem_fold",
        ArticulationType.REVOLUTE,
        parent=front_assembly,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-1.15, upper=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    front_assembly = object_model.get_part("front_assembly")
    stem = object_model.get_part("stem")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    steering_yaw = object_model.get_articulation("steering_yaw")
    front_wheel_spin = object_model.get_articulation("front_wheel_spin")
    rear_wheel_spin = object_model.get_articulation("rear_wheel_spin")
    stem_fold = object_model.get_articulation("stem_fold")

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
        front_assembly,
        stem,
        elem_a="hinge_block",
        elem_b="stem_base_clamp",
        reason="Captured folding clamp wraps the hinge block so the stem stays attached while folding.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(front_assembly, chassis, name="front assembly seated in head tube")
    ctx.expect_contact(stem, front_assembly, name="stem clipped to hinge block")
    ctx.expect_contact(front_wheel, front_assembly, name="front wheel mounted in fork")
    ctx.expect_contact(rear_wheel, chassis, name="rear wheel mounted in rear dropouts")
    ctx.expect_gap(stem, chassis, axis="z", min_gap=0.05, name="upright stem clears deck")

    ctx.check(
        "steering joint uses vertical yaw axis",
        steering_yaw.axis == (0.0, 0.0, 1.0),
        details=f"axis={steering_yaw.axis}",
    )
    ctx.check(
        "front wheel spins on lateral axle",
        front_wheel_spin.axis == (0.0, 1.0, 0.0),
        details=f"axis={front_wheel_spin.axis}",
    )
    ctx.check(
        "rear wheel spins on lateral axle",
        rear_wheel_spin.axis == (0.0, 1.0, 0.0),
        details=f"axis={rear_wheel_spin.axis}",
    )
    ctx.check(
        "stem folds on horizontal hinge axis",
        stem_fold.axis == (0.0, 1.0, 0.0),
        details=f"axis={stem_fold.axis}",
    )

    front_wheel_rest = ctx.part_world_position(front_wheel)
    handlebar_rest = ctx.part_element_world_aabb(stem, elem="handlebar")

    with ctx.pose({steering_yaw: 0.45}):
        front_wheel_steered = ctx.part_world_position(front_wheel)
        ctx.expect_contact(front_assembly, chassis, name="steering head stays seated when steered")
        if front_wheel_rest is not None and front_wheel_steered is not None:
            ctx.check(
                "steering yaw swings front wheel sideways",
                abs(front_wheel_steered[1]) > abs(front_wheel_rest[1]) + 0.03,
                details=f"rest_y={front_wheel_rest[1]:.4f}, steered_y={front_wheel_steered[1]:.4f}",
            )

    with ctx.pose({stem_fold: -1.05}):
        handlebar_folded = ctx.part_element_world_aabb(stem, elem="handlebar")
        ctx.expect_contact(stem, front_assembly, name="folded stem remains clipped to hinge block")
        ctx.expect_gap(stem, chassis, axis="z", max_penetration=0.0, name="folded stem clears chassis")
        if handlebar_rest is not None and handlebar_folded is not None:
            ctx.check(
                "folded stem lowers handlebar over deck",
                handlebar_folded[1][2] < handlebar_rest[1][2] - 0.25,
                details=f"rest_top_z={handlebar_rest[1][2]:.4f}, folded_top_z={handlebar_folded[1][2]:.4f}",
            )
            ctx.check(
                "folded stem swings handlebar rearward",
                handlebar_folded[0][0] < handlebar_rest[0][0] - 0.25,
                details=f"rest_min_x={handlebar_rest[0][0]:.4f}, folded_min_x={handlebar_folded[0][0]:.4f}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

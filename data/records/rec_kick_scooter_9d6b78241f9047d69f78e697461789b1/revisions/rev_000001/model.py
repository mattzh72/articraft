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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
)


_MESH_CACHE: dict[str, object] = {}


def _managed_mesh(name: str, geometry):
    mesh = _MESH_CACHE.get(name)
    if mesh is None:
        mesh = mesh_from_geometry(geometry, name)
        _MESH_CACHE[name] = mesh
    return mesh


def _deck_outline() -> list[tuple[float, float]]:
    control_points = [
        (-0.205, 0.000),
        (-0.190, 0.046),
        (-0.140, 0.063),
        (0.115, 0.063),
        (0.160, 0.055),
        (0.188, 0.032),
        (0.198, 0.000),
        (0.188, -0.032),
        (0.160, -0.055),
        (0.115, -0.063),
        (-0.140, -0.063),
        (-0.190, -0.046),
    ]
    return sample_catmull_rom_spline_2d(
        control_points,
        samples_per_segment=8,
        closed=True,
    )


def _wheel_tire_mesh(radius: float, width: float):
    half_width = width * 0.5
    profile = [
        (radius * 0.42, -half_width * 0.96),
        (radius * 0.60, -half_width),
        (radius * 0.82, -half_width * 0.92),
        (radius * 0.95, -half_width * 0.55),
        (radius, -half_width * 0.18),
        (radius, half_width * 0.18),
        (radius * 0.95, half_width * 0.55),
        (radius * 0.82, half_width * 0.92),
        (radius * 0.60, half_width),
        (radius * 0.42, half_width * 0.96),
        (radius * 0.36, half_width * 0.28),
        (radius * 0.34, 0.0),
        (radius * 0.36, -half_width * 0.28),
        (radius * 0.42, -half_width * 0.96),
    ]
    return _managed_mesh(
        "scooter_wheel_tire",
        LatheGeometry(profile, segments=56).rotate_x(math.pi / 2.0),
    )


def _add_wheel_visuals(part, tire_mesh, *, wheel_width: float, tire_radius: float, tire, rim, metal) -> None:
    spin_origin = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    part.visual(tire_mesh, material=tire, name="tire")
    part.visual(
        Cylinder(radius=tire_radius * 0.78, length=wheel_width * 0.70),
        origin=spin_origin,
        material=rim,
        name="rim_barrel",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.42, length=wheel_width * 1.08),
        origin=spin_origin,
        material=metal,
        name="hub",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.77, length=0.004),
        origin=Origin(xyz=(0.0, wheel_width * 0.18, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rim,
        name="left_sidewall",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.77, length=0.004),
        origin=Origin(xyz=(0.0, -wheel_width * 0.18, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rim,
        name="right_sidewall",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commuting_kick_scooter")

    deck_color = model.material("deck_color", rgba=(0.14, 0.15, 0.16, 1.0))
    grip_color = model.material("grip_color", rgba=(0.06, 0.06, 0.07, 1.0))
    silver = model.material("silver", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.29, 0.31, 1.0))
    tire = model.material("tire", rgba=(0.05, 0.05, 0.05, 1.0))
    accent = model.material("accent", rgba=(0.84, 0.39, 0.16, 1.0))

    deck_bottom = 0.070
    deck_thickness = 0.030
    wheel_radius = 0.100
    wheel_width = 0.036
    steering_origin = (0.185, 0.0, 0.200)
    front_axle_rel = (0.155, 0.0, -0.100)
    rear_axle = (-0.270, 0.0, wheel_radius)

    wheel_origin = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    deck = model.part("deck")
    deck.visual(
        Box((0.390, 0.130, deck_thickness)),
        origin=Origin(xyz=(0.035, 0.0, deck_bottom + deck_thickness * 0.5)),
        material=deck_color,
        name="deck_shell",
    )
    deck.visual(
        Box((0.300, 0.098, 0.004)),
        origin=Origin(xyz=(-0.015, 0.0, deck_bottom + deck_thickness + 0.002)),
        material=grip_color,
        name="grip_tape",
    )
    deck.visual(
        Box((0.060, 0.060, 0.060)),
        origin=Origin(xyz=(0.185, 0.0, 0.130)),
        material=dark_metal,
        name="head_block",
    )
    deck.visual(
        Box((0.072, 0.094, 0.018)),
        origin=Origin(xyz=(0.178, 0.0, 0.109)),
        material=deck_color,
        name="front_platform_riser",
    )
    deck.visual(
        Box((0.088, 0.050, 0.030)),
        origin=Origin(xyz=(0.140, 0.0, 0.089)),
        material=dark_metal,
        name="neck_brace",
    )
    deck.visual(
        Box((0.030, 0.010, 0.120)),
        origin=Origin(xyz=(rear_axle[0], 0.030, 0.130)),
        material=dark_metal,
        name="rear_dropout_left",
    )
    deck.visual(
        Box((0.030, 0.010, 0.120)),
        origin=Origin(xyz=(rear_axle[0], -0.030, 0.130)),
        material=dark_metal,
        name="rear_dropout_right",
    )
    deck.visual(
        Box((0.140, 0.010, 0.020)),
        origin=Origin(xyz=(-0.215, 0.030, 0.110)),
        material=dark_metal,
        name="rear_frame_left",
    )
    deck.visual(
        Box((0.140, 0.010, 0.020)),
        origin=Origin(xyz=(-0.215, -0.030, 0.110)),
        material=dark_metal,
        name="rear_frame_right",
    )
    deck.visual(
        Box((0.180, 0.046, 0.012)),
        origin=Origin(xyz=(-0.012, 0.0, deck_bottom + 0.006)),
        material=dark_metal,
        name="underside_stiffener",
    )
    deck.visual(
        Box((0.040, 0.014, 0.018)),
        origin=Origin(xyz=(0.010, -0.064, 0.078)),
        material=dark_metal,
        name="kickstand_bracket",
    )

    front_fork = model.part("front_fork")
    front_fork.visual(
        Cylinder(radius=0.016, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=dark_metal,
        name="steerer",
    )
    front_fork.visual(
        Box((0.044, 0.052, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=dark_metal,
        name="stem_clamp",
    )
    front_fork.visual(
        Cylinder(radius=0.017, length=0.760),
        origin=Origin(xyz=(0.0, 0.0, 0.520)),
        material=silver,
        name="main_stem",
    )
    front_fork.visual(
        Cylinder(radius=0.013, length=0.070),
        origin=Origin(
            xyz=(-0.010, 0.0, 0.875),
        ),
        material=dark_metal,
        name="handlebar_riser",
    )
    front_fork.visual(
        Cylinder(radius=0.012, length=0.500),
        origin=Origin(
            xyz=(-0.010, 0.0, 0.905),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="handlebar",
    )
    front_fork.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(
            xyz=(-0.010, 0.250, 0.905),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=grip_color,
        name="left_grip",
    )
    front_fork.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(
            xyz=(-0.010, -0.250, 0.905),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=grip_color,
        name="right_grip",
    )
    front_fork.visual(
        Box((0.110, 0.060, 0.024)),
        origin=Origin(xyz=(0.055, 0.0, 0.026)),
        material=dark_metal,
        name="fork_crown",
    )
    front_fork.visual(
        Cylinder(radius=0.008, length=0.195),
        origin=Origin(
            xyz=(0.102, 0.032, -0.020),
            rpy=(0.0, 2.546, 0.0),
        ),
        material=silver,
        name="fork_leg_left",
    )
    front_fork.visual(
        Cylinder(radius=0.008, length=0.195),
        origin=Origin(
            xyz=(0.102, -0.032, -0.020),
            rpy=(0.0, 2.546, 0.0),
        ),
        material=silver,
        name="fork_leg_right",
    )
    front_fork.visual(
        Box((0.018, 0.014, 0.032)),
        origin=Origin(xyz=(front_axle_rel[0], 0.032, -0.100)),
        material=dark_metal,
        name="dropout_left",
    )
    front_fork.visual(
        Box((0.018, 0.014, 0.032)),
        origin=Origin(xyz=(front_axle_rel[0], -0.032, -0.100)),
        material=dark_metal,
        name="dropout_right",
    )
    front_fork.visual(
        Box((0.086, 0.052, 0.012)),
        origin=Origin(xyz=(front_axle_rel[0] - 0.010, 0.0, 0.013)),
        material=accent,
        name="front_fender",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=wheel_origin,
        material=tire,
        name="tire",
    )
    front_wheel.visual(
        Cylinder(radius=0.074, length=0.024),
        origin=wheel_origin,
        material=silver,
        name="rim_barrel",
    )
    front_wheel.visual(
        Cylinder(radius=0.028, length=0.050),
        origin=wheel_origin,
        material=dark_metal,
        name="hub",
    )
    front_wheel.visual(
        Cylinder(radius=0.074, length=0.004),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="left_sidewall",
    )
    front_wheel.visual(
        Cylinder(radius=0.074, length=0.004),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="right_sidewall",
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=wheel_origin,
        material=tire,
        name="tire",
    )
    rear_wheel.visual(
        Cylinder(radius=0.074, length=0.024),
        origin=wheel_origin,
        material=silver,
        name="rim_barrel",
    )
    rear_wheel.visual(
        Cylinder(radius=0.028, length=0.050),
        origin=wheel_origin,
        material=dark_metal,
        name="hub",
    )
    rear_wheel.visual(
        Cylinder(radius=0.074, length=0.004),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="left_sidewall",
    )
    rear_wheel.visual(
        Cylinder(radius=0.074, length=0.004),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="right_sidewall",
    )

    kickstand = model.part("kickstand")
    kickstand.visual(
        Box((0.016, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.012, -0.012)),
        material=dark_metal,
        name="stand_hinge_tab",
    )
    kickstand.visual(
        Cylinder(radius=0.006, length=0.096),
        origin=Origin(
            xyz=(0.0, -0.060, -0.017),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="stand_leg",
    )
    kickstand.visual(
        Box((0.018, 0.014, 0.006)),
        origin=Origin(
            xyz=(0.0, -0.110, -0.022),
        ),
        material=dark_metal,
        name="stand_foot",
    )

    model.articulation(
        "steering_head",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_fork,
        origin=Origin(xyz=steering_origin),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.2,
            lower=-0.75,
            upper=0.75,
        ),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_wheel,
        origin=Origin(xyz=front_axle_rel),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=35.0,
        ),
    )
    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=rear_axle),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=35.0,
        ),
    )
    model.articulation(
        "kickstand_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=kickstand,
        origin=Origin(xyz=(0.010, -0.064, 0.078)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_fork = object_model.get_part("front_fork")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    kickstand = object_model.get_part("kickstand")
    steering = object_model.get_articulation("steering_head")
    front_spin = object_model.get_articulation("front_wheel_spin")
    rear_spin = object_model.get_articulation("rear_wheel_spin")
    kickstand_hinge = object_model.get_articulation("kickstand_hinge")

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
        "steering_axis_vertical",
        steering.axis == (0.0, 0.0, 1.0),
        f"expected steering axis (0, 0, 1), got {steering.axis}",
    )
    ctx.check(
        "wheel_axes_lateral",
        front_spin.axis == (0.0, 1.0, 0.0) and rear_spin.axis == (0.0, 1.0, 0.0),
        f"expected wheel axes (0, 1, 0), got front={front_spin.axis}, rear={rear_spin.axis}",
    )
    ctx.check(
        "kickstand_axis_longitudinal",
        kickstand_hinge.axis == (1.0, 0.0, 0.0),
        f"expected kickstand axis (1, 0, 0), got {kickstand_hinge.axis}",
    )
    ctx.check(
        "steering_limits_realistic",
        steering.motion_limits is not None
        and steering.motion_limits.lower == -0.75
        and steering.motion_limits.upper == 0.75,
        f"unexpected steering limits: {steering.motion_limits}",
    )

    ctx.expect_origin_gap(
        front_wheel,
        rear_wheel,
        axis="x",
        min_gap=0.55,
        max_gap=0.70,
        name="realistic_wheelbase",
    )

    with ctx.pose({steering: 0.45}):
        front_pos = ctx.part_world_position(front_wheel)
        rear_pos = ctx.part_world_position(rear_wheel)
        ctx.check(
            "front_wheel_moves_sideways_when_steered",
            front_pos is not None and front_pos[1] > 0.02,
            f"expected front wheel y > 0.02 when steered, got {front_pos}",
        )
        ctx.check(
            "rear_wheel_stays_centered_when_steered",
            rear_pos is not None and abs(rear_pos[1]) < 1e-6,
            f"rear wheel should stay centered during steering, got {rear_pos}",
        )

    with ctx.pose({kickstand_hinge: 0.0}):
        ctx.expect_gap(
            deck,
            kickstand,
            axis="z",
            min_gap=0.0,
            max_gap=0.012,
            negative_elem="stand_leg",
            name="kickstand_tucks_under_deck",
        )
        ctx.expect_contact(
            deck,
            kickstand,
            elem_a="kickstand_bracket",
            elem_b="stand_hinge_tab",
            name="kickstand_hinge_mount_contacts_bracket",
        )

    with ctx.pose({kickstand_hinge: 0.95}):
        deck_aabb = ctx.part_world_aabb(deck)
        kickstand_aabb = ctx.part_world_aabb(kickstand)
        outboard = False
        reaches_ground = False
        if deck_aabb is not None and kickstand_aabb is not None:
            outboard = kickstand_aabb[0][1] < deck_aabb[0][1] - 0.01
            reaches_ground = kickstand_aabb[0][2] <= 0.002
        ctx.check(
            "kickstand_swings_outboard",
            outboard,
            f"expected deployed kickstand to swing beyond deck side, deck={deck_aabb}, kickstand={kickstand_aabb}",
        )
        ctx.check(
            "kickstand_reaches_ground_plane",
            reaches_ground,
            f"expected deployed kickstand near ground, got {kickstand_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

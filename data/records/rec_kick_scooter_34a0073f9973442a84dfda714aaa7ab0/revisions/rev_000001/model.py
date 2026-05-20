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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commuter_kick_scooter")

    deck_paint = model.material("deck_paint", rgba=(0.17, 0.19, 0.20, 1.0))
    deck_grip = model.material("deck_grip", rgba=(0.08, 0.08, 0.08, 1.0))
    satin_black = model.material("satin_black", rgba=(0.11, 0.12, 0.13, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.27, 0.29, 0.31, 1.0))
    bright_metal = model.material("bright_metal", rgba=(0.71, 0.73, 0.76, 1.0))
    wheel_metal = model.material("wheel_metal", rgba=(0.62, 0.65, 0.68, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    accent = model.material("accent", rgba=(0.48, 0.51, 0.54, 1.0))

    wheel_radius = 0.11
    wheel_width = 0.035
    front_hub_width = 0.080
    rear_hub_width = 0.070
    steering_tilt = 0.35
    steering_origin = (0.325, 0.0, 0.160)
    rear_axle = (-0.310, 0.0, 0.110)
    front_axle_local = (0.145, 0.0, 0.000)
    rear_brake_origin = (-0.222, 0.0, 0.205)
    kickstand_origin = (-0.100, 0.064, 0.048)

    def _save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def _tube(name: str, points, *, radius: float, samples: int = 12, radial: int = 16):
        return _save_mesh(
            name,
            tube_from_spline_points(
                points,
                radius=radius,
                samples_per_segment=samples,
                radial_segments=radial,
            ),
        )

    def _wheel_visuals(part, prefix: str, *, hub_width: float) -> None:
        tire_minor = 0.016
        tire_major = wheel_radius - tire_minor
        tire = TorusGeometry(
            radius=tire_major,
            tube=tire_minor,
            radial_segments=18,
            tubular_segments=40,
        ).rotate_x(pi / 2.0)
        part.visual(_save_mesh(f"{prefix}_tire", tire), material=rubber, name="tire")
        part.visual(
            Cylinder(radius=0.078, length=0.028),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=wheel_metal,
            name="rim",
        )
        part.visual(
            Cylinder(radius=0.072, length=0.008),
            origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=wheel_metal,
        )
        part.visual(
            Cylinder(radius=0.072, length=0.008),
            origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=wheel_metal,
        )
        part.visual(
            Cylinder(radius=0.022, length=hub_width),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="hub",
        )
        part.visual(
            Cylinder(radius=0.010, length=hub_width * 0.92),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=bright_metal,
        )

    head_axis = (sin(steering_tilt), 0.0, cos(steering_tilt))
    head_lower = (
        steering_origin[0] - head_axis[0] * 0.060,
        0.0,
        steering_origin[2] - head_axis[2] * 0.060,
    )

    deck = model.part("deck")
    deck.inertial = Inertial.from_geometry(
        Box((0.96, 0.18, 0.28)),
        mass=8.2,
        origin=Origin(xyz=(0.00, 0.00, 0.14)),
    )
    deck.visual(
        Box((0.430, 0.140, 0.028)),
        origin=Origin(xyz=(0.015, 0.0, 0.071)),
        material=deck_paint,
        name="deck_shell",
    )
    deck.visual(
        Box((0.360, 0.108, 0.004)),
        origin=Origin(xyz=(0.005, 0.0, 0.087)),
        material=deck_grip,
        name="deck_grip",
    )
    deck.visual(
        Cylinder(radius=0.032, length=0.120),
        origin=Origin(xyz=steering_origin, rpy=(0.0, steering_tilt, 0.0)),
        material=dark_metal,
        name="head_tube",
    )
    deck.visual(
        _tube(
            "deck_neck",
            [
                (0.215, 0.0, 0.074),
                (0.248, 0.0, 0.084),
                (head_lower[0] - 0.040, 0.0, head_lower[2] - 0.010),
            ],
            radius=0.021,
            samples=10,
        ),
        material=dark_metal,
        name="deck_neck",
    )
    deck.visual(
        Box((0.050, 0.014, 0.040)),
        origin=Origin(xyz=(0.252, 0.038, 0.106)),
        material=dark_metal,
        name="left_head_gusset",
    )
    deck.visual(
        Box((0.050, 0.014, 0.040)),
        origin=Origin(xyz=(0.252, -0.038, 0.106)),
        material=dark_metal,
        name="right_head_gusset",
    )
    deck.visual(
        _tube(
            "left_rear_stay",
            [
                (-0.185, 0.040, 0.067),
                (-0.262, 0.040, 0.086),
                (-0.300, 0.048, 0.132),
            ],
            radius=0.009,
            samples=8,
        ),
        material=dark_metal,
        name="left_rear_stay",
    )
    deck.visual(
        _tube(
            "right_rear_stay",
            [
                (-0.185, -0.040, 0.067),
                (-0.262, -0.040, 0.086),
                (-0.300, -0.048, 0.132),
            ],
            radius=0.009,
            samples=8,
        ),
        material=dark_metal,
        name="right_rear_stay",
    )
    deck.visual(
        Box((0.018, 0.010, 0.050)),
        origin=Origin(xyz=(-0.310, 0.040, 0.110)),
        material=dark_metal,
        name="rear_left_axle_boss",
    )
    deck.visual(
        Box((0.018, 0.010, 0.050)),
        origin=Origin(xyz=(-0.310, -0.040, 0.110)),
        material=dark_metal,
        name="rear_right_axle_boss",
    )
    deck.visual(
        _tube(
            "left_brake_support",
            [
                (-0.205, 0.031, 0.074),
                (-0.214, 0.031, 0.136),
                (-0.222, 0.031, 0.198),
            ],
            radius=0.008,
            samples=8,
        ),
        material=dark_metal,
        name="left_brake_support",
    )
    deck.visual(
        _tube(
            "right_brake_support",
            [
                (-0.205, -0.031, 0.074),
                (-0.214, -0.031, 0.136),
                (-0.222, -0.031, 0.198),
            ],
            radius=0.008,
            samples=8,
        ),
        material=dark_metal,
        name="right_brake_support",
    )
    deck.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(-0.222, 0.031, 0.205), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="brake_hinge_left",
    )
    deck.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(-0.222, -0.031, 0.205), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="brake_hinge_right",
    )
    deck.visual(
        Box((0.022, 0.032, 0.014)),
        origin=Origin(xyz=(-0.100, 0.064, 0.051)),
        material=dark_metal,
        name="kickstand_bracket",
    )
    deck.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(-0.100, 0.074, 0.048), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="kickstand_hinge_outer",
    )
    deck.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(-0.100, 0.054, 0.048), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="kickstand_hinge_inner",
    )

    front_assembly = model.part("front_assembly")
    front_assembly.inertial = Inertial.from_geometry(
        Box((0.42, 0.62, 0.92)),
        mass=4.8,
        origin=Origin(xyz=(0.06, 0.0, 0.36)),
    )
    front_assembly.visual(
        Cylinder(radius=0.031, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.067)),
        material=bright_metal,
        name="lower_headset_collar",
    )
    front_assembly.visual(
        Cylinder(radius=0.031, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=bright_metal,
        name="upper_headset_collar",
    )
    front_assembly.visual(
        Cylinder(radius=0.016, length=0.154),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=satin_black,
        name="steerer_spine",
    )
    front_assembly.visual(
        Cylinder(radius=0.022, length=0.670),
        origin=Origin(xyz=(0.0, 0.0, 0.410)),
        material=satin_black,
        name="steering_column",
    )
    front_assembly.visual(
        Box((0.032, 0.096, 0.026)),
        origin=Origin(xyz=(0.000, 0.0, 0.085)),
        material=satin_black,
        name="upper_crown",
    )
    front_assembly.visual(
        _tube(
            "left_stanchion",
            [
                (0.004, 0.034, 0.085),
                (0.060, 0.034, 0.054),
                (0.105, 0.034, 0.032),
            ],
            radius=0.008,
            samples=10,
        ),
        material=bright_metal,
        name="left_stanchion",
    )
    front_assembly.visual(
        _tube(
            "right_stanchion",
            [
                (0.004, -0.034, 0.085),
                (0.060, -0.034, 0.054),
                (0.105, -0.034, 0.032),
            ],
            radius=0.008,
            samples=10,
        ),
        material=bright_metal,
        name="right_stanchion",
    )
    front_assembly.visual(
        _tube(
            "left_slider",
            [
                (0.102, 0.050, 0.032),
                (0.126, 0.052, 0.016),
                (0.138, 0.052, 0.004),
            ],
            radius=0.012,
            samples=8,
        ),
        material=satin_black,
        name="left_slider",
    )
    front_assembly.visual(
        _tube(
            "right_slider",
            [
                (0.102, -0.050, 0.032),
                (0.126, -0.052, 0.016),
                (0.138, -0.052, 0.004),
            ],
            radius=0.012,
            samples=8,
        ),
        material=satin_black,
        name="right_slider",
    )
    front_assembly.visual(
        Box((0.018, 0.010, 0.034)),
        origin=Origin(xyz=(0.145, 0.045, 0.000)),
        material=dark_metal,
        name="left_axle_boss",
    )
    front_assembly.visual(
        Box((0.018, 0.010, 0.034)),
        origin=Origin(xyz=(0.145, -0.045, 0.000)),
        material=dark_metal,
        name="right_axle_boss",
    )
    front_assembly.visual(
        Box((0.058, 0.090, 0.032)),
        origin=Origin(xyz=(0.000, 0.0, 0.709)),
        material=dark_metal,
        name="handlebar_clamp",
    )
    front_assembly.visual(
        _tube(
            "handlebar",
            [
                (-0.010, -0.290, 0.701),
                (0.008, -0.220, 0.718),
                (0.014, 0.0, 0.728),
                (0.008, 0.220, 0.718),
                (-0.010, 0.290, 0.701),
            ],
            radius=0.012,
            samples=12,
            radial=18,
        ),
        material=satin_black,
        name="handlebar",
    )
    front_assembly.visual(
        Cylinder(radius=0.017, length=0.110),
        origin=Origin(xyz=(-0.010, 0.305, 0.701), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    front_assembly.visual(
        Cylinder(radius=0.017, length=0.110),
        origin=Origin(xyz=(-0.010, -0.305, 0.701), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    front_assembly.visual(
        Box((0.040, 0.024, 0.020)),
        origin=Origin(xyz=(0.012, 0.0, 0.739)),
        material=accent,
        name="display_mount",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=1.6,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(front_wheel, "front_wheel", hub_width=front_hub_width)

    rear_wheel = model.part("rear_wheel")
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=1.7,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visuals(rear_wheel, "rear_wheel", hub_width=rear_hub_width)

    rear_brake = model.part("rear_brake")
    rear_brake.inertial = Inertial.from_geometry(
        Box((0.22, 0.08, 0.06)),
        mass=0.55,
        origin=Origin(xyz=(-0.10, 0.0, 0.028)),
    )
    rear_brake.visual(
        Cylinder(radius=0.009, length=0.086),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=bright_metal,
        name="hinge_sleeve",
    )
    rear_brake.visual(
        Box((0.022, 0.016, 0.016)),
        origin=Origin(xyz=(-0.028, 0.028, 0.008)),
        material=satin_black,
        name="hinge_bridge",
    )
    rear_brake.visual(
        Box((0.022, 0.016, 0.016)),
        origin=Origin(xyz=(-0.028, -0.028, 0.008)),
        material=satin_black,
        name="right_hinge_arm",
    )
    rear_brake.visual(
        Box((0.100, 0.078, 0.016)),
        origin=Origin(xyz=(-0.018, 0.0, 0.026)),
        material=deck_paint,
        name="fender_front",
    )
    rear_brake.visual(
        Box((0.088, 0.078, 0.016)),
        origin=Origin(xyz=(-0.100, 0.0, 0.031)),
        material=deck_paint,
        name="fender_mid",
    )
    rear_brake.visual(
        Box((0.060, 0.076, 0.018)),
        origin=Origin(xyz=(-0.168, 0.0, 0.027)),
        material=deck_paint,
        name="fender_rear",
    )
    rear_brake.visual(
        Box((0.044, 0.060, 0.010)),
        origin=Origin(xyz=(-0.132, 0.0, 0.021)),
        material=deck_grip,
        name="brake_pad",
    )

    kickstand = model.part("kickstand")
    kickstand.inertial = Inertial.from_geometry(
        Box((0.10, 0.04, 0.22)),
        mass=0.28,
        origin=Origin(xyz=(-0.035, 0.014, -0.096)),
    )
    kickstand.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=bright_metal,
        name="hinge_sleeve",
    )
    kickstand.visual(
        Box((0.016, 0.022, 0.018)),
        origin=Origin(xyz=(-0.010, 0.010, -0.012)),
        material=satin_black,
        name="kickstand_head",
    )
    kickstand.visual(
        _tube(
            "kickstand_leg",
            [
                (-0.012, 0.010, -0.018),
                (-0.024, 0.014, -0.060),
                (-0.046, 0.022, -0.138),
                (-0.060, 0.027, -0.186),
            ],
            radius=0.007,
            samples=10,
        ),
        material=dark_metal,
        name="leg",
    )
    kickstand.visual(
        Box((0.024, 0.014, 0.010)),
        origin=Origin(xyz=(-0.062, 0.027, -0.191)),
        material=dark_metal,
        name="foot",
    )

    model.articulation(
        "steering_yaw",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_assembly,
        origin=Origin(xyz=steering_origin, rpy=(0.0, steering_tilt, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_assembly,
        child=front_wheel,
        origin=Origin(xyz=front_axle_local),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=35.0),
    )
    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=rear_axle),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=35.0),
    )
    model.articulation(
        "rear_brake_pivot",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_brake,
        origin=Origin(xyz=rear_brake_origin),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.0, lower=-0.12, upper=0.08),
    )
    model.articulation(
        "kickstand_pivot",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=kickstand,
        origin=Origin(xyz=kickstand_origin),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=-1.00, upper=0.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_assembly = object_model.get_part("front_assembly")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    rear_brake = object_model.get_part("rear_brake")
    kickstand = object_model.get_part("kickstand")

    steering_yaw = object_model.get_articulation("steering_yaw")
    front_wheel_spin = object_model.get_articulation("front_wheel_spin")
    rear_wheel_spin = object_model.get_articulation("rear_wheel_spin")
    rear_brake_pivot = object_model.get_articulation("rear_brake_pivot")
    kickstand_pivot = object_model.get_articulation("kickstand_pivot")

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
        deck,
        front_assembly,
        elem_a="head_tube",
        elem_b="steerer_spine",
        reason="The steering steerer nests concentrically inside the deck head tube as an intentional headset overlap.",
    )
    ctx.allow_overlap(
        deck,
        rear_brake,
        elem_a="brake_hinge_left",
        elem_b="hinge_sleeve",
        reason="The fender brake pivot sleeve spans the deck hinge tabs as an intentional hinge overlap.",
    )
    ctx.allow_overlap(
        deck,
        rear_brake,
        elem_a="brake_hinge_right",
        elem_b="hinge_sleeve",
        reason="The fender brake pivot sleeve spans the deck hinge tabs as an intentional hinge overlap.",
    )
    ctx.allow_overlap(
        deck,
        kickstand,
        elem_a="kickstand_bracket",
        elem_b="hinge_sleeve",
        reason="The kickstand pivot sleeve is nested inside the welded bracket ears.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "mechanism_axes_and_limits_configured",
        steering_yaw.articulation_type == ArticulationType.REVOLUTE
        and steering_yaw.axis == (0.0, 0.0, 1.0)
        and steering_yaw.origin.rpy[1] > 0.25
        and front_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and front_wheel_spin.axis == (0.0, 1.0, 0.0)
        and rear_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_wheel_spin.axis == (0.0, 1.0, 0.0)
        and rear_brake_pivot.axis == (0.0, 1.0, 0.0)
        and kickstand_pivot.axis == (0.0, 1.0, 0.0),
        "Expected scooter steering around the tilted column, wheel spin on lateral axles, and brake/kickstand fold on lateral hinges.",
    )
    ctx.expect_origin_gap(
        front_wheel,
        rear_wheel,
        axis="x",
        min_gap=0.68,
        max_gap=0.86,
        name="wheelbase_is_big_wheel_commuter_scale",
    )

    ctx.expect_contact(
        deck,
        front_assembly,
        elem_a="head_tube",
        elem_b="lower_headset_collar",
        name="lower_headset_is_seated",
    )
    ctx.expect_contact(
        deck,
        front_assembly,
        elem_a="head_tube",
        elem_b="upper_headset_collar",
        name="upper_headset_is_seated",
    )
    ctx.expect_contact(
        front_wheel,
        front_assembly,
        elem_a="hub",
        elem_b="left_axle_boss",
        name="front_wheel_contacts_left_fork_boss",
    )
    ctx.expect_contact(
        front_wheel,
        front_assembly,
        elem_a="hub",
        elem_b="right_axle_boss",
        name="front_wheel_contacts_right_fork_boss",
    )
    ctx.expect_contact(
        rear_wheel,
        deck,
        elem_a="hub",
        elem_b="rear_left_axle_boss",
        name="rear_wheel_contacts_left_dropout",
    )
    ctx.expect_contact(
        rear_wheel,
        deck,
        elem_a="hub",
        elem_b="rear_right_axle_boss",
        name="rear_wheel_contacts_right_dropout",
    )
    ctx.expect_contact(
        deck,
        rear_brake,
        elem_a="brake_hinge_left",
        elem_b="hinge_sleeve",
        name="rear_brake_left_hinge_contact",
    )
    ctx.expect_contact(
        deck,
        rear_brake,
        elem_a="brake_hinge_right",
        elem_b="hinge_sleeve",
        name="rear_brake_right_hinge_contact",
    )
    ctx.expect_contact(
        deck,
        kickstand,
        elem_a="kickstand_hinge_outer",
        elem_b="hinge_sleeve",
        name="kickstand_outer_hinge_contact",
    )
    ctx.expect_contact(
        deck,
        kickstand,
        elem_a="kickstand_hinge_inner",
        elem_b="hinge_sleeve",
        name="kickstand_inner_hinge_contact",
    )

    with ctx.pose({rear_brake_pivot: 0.0}):
        ctx.expect_gap(
            rear_brake,
            rear_wheel,
            axis="z",
            min_gap=0.0005,
            max_gap=0.008,
            positive_elem="brake_pad",
            negative_elem="tire",
            name="rear_brake_released_clearance",
        )

    with ctx.pose({rear_brake_pivot: -0.005}):
        ctx.expect_gap(
            rear_brake,
            rear_wheel,
            axis="z",
            min_gap=0.0,
            max_gap=0.002,
            positive_elem="brake_pad",
            negative_elem="tire",
            name="rear_brake_pressed_near_tire",
        )

    with ctx.pose({steering_yaw: 0.60}):
        ctx.expect_contact(
            deck,
            front_assembly,
            elem_a="head_tube",
            elem_b="lower_headset_collar",
            name="lower_headset_contact_in_steer_pose",
        )
        ctx.expect_contact(
            front_wheel,
            front_assembly,
            elem_a="hub",
            elem_b="left_axle_boss",
            name="front_wheel_left_boss_contact_in_steer_pose",
        )
        ctx.expect_contact(
            front_wheel,
            front_assembly,
            elem_a="hub",
            elem_b="right_axle_boss",
            name="front_wheel_right_boss_contact_in_steer_pose",
        )

    with ctx.pose({kickstand_pivot: -0.95}):
        ctx.expect_gap(
            deck,
            kickstand,
            axis="z",
            min_gap=0.0,
            max_gap=0.030,
            positive_elem="deck_shell",
            negative_elem="leg",
            name="kickstand_folds_up_below_deck",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

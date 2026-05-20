from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commuter_kick_scooter", assets=ASSETS)

    frame_silver = model.material("frame_silver", rgba=(0.76, 0.79, 0.82, 1.0))
    trim_black = model.material("trim_black", rgba=(0.12, 0.13, 0.14, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_gray = model.material("wheel_gray", rgba=(0.66, 0.68, 0.71, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.55, 0.57, 0.60, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def wheel_inertial(radius: float, width: float, mass: float) -> Inertial:
        return Inertial.from_geometry(
            Cylinder(radius=radius, length=width),
            mass=mass,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )

    def add_wheel_visuals(part, *, tire_radius: float, tire_width: float) -> None:
        spin_origin = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
        half_width = tire_width * 0.5
        tire_inner_radius = tire_radius * 0.70
        tire_profile = [
            (tire_inner_radius, -half_width * 0.92),
            (tire_radius * 0.91, -half_width),
            (tire_radius * 0.98, -half_width * 0.58),
            (tire_radius, 0.0),
            (tire_radius * 0.98, half_width * 0.58),
            (tire_radius * 0.91, half_width),
            (tire_inner_radius, half_width * 0.92),
            (tire_inner_radius * 0.95, half_width * 0.45),
            (tire_inner_radius * 0.95, -half_width * 0.45),
            (tire_inner_radius, -half_width * 0.92),
        ]
        tire_mesh = save_mesh(
            f"{part.name}_tire.obj",
            LatheGeometry(tire_profile, segments=56).rotate_x(math.pi / 2.0),
        )
        part.visual(
            tire_mesh,
            material=rubber_black,
            name="tire",
        )
        part.visual(
            Cylinder(radius=tire_inner_radius + 0.001, length=0.040),
            origin=spin_origin,
            material=axle_steel,
            name="hub",
        )

    deck_frame = model.part("deck_frame")
    deck_frame.inertial = Inertial.from_geometry(
        Box((0.64, 0.14, 0.28)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
    )

    deck_shell = save_mesh(
        "scooter_deck_shell.obj",
        ExtrudeGeometry.centered(
            rounded_rect_profile(0.44, 0.13, 0.024, corner_segments=8),
            0.022,
        ),
    )
    deck_frame.visual(
        deck_shell,
        origin=Origin(xyz=(0.030, 0.0, 0.091)),
        material=frame_silver,
        name="deck_shell",
    )
    deck_frame.visual(
        Box((0.34, 0.10, 0.003)),
        origin=Origin(xyz=(0.005, 0.0, 0.1045)),
        material=trim_black,
        name="grip_tape",
    )
    deck_frame.visual(
        Box((0.11, 0.06, 0.045)),
        origin=Origin(xyz=(0.182, 0.0, 0.1025)),
        material=frame_silver,
        name="front_gusset",
    )
    deck_frame.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.228, 0.0, 0.100)),
        material=trim_black,
        name="lower_headset_cup",
    )
    deck_frame.visual(
        Cylinder(radius=0.030, length=0.050),
        origin=Origin(xyz=(0.228, 0.0, 0.125)),
        material=frame_silver,
        name="head_tube",
    )
    deck_frame.visual(
        Box((0.11, 0.010, 0.022)),
        origin=Origin(xyz=(-0.228, 0.036, 0.105)),
        material=frame_silver,
        name="tail_bridge",
    )
    deck_frame.visual(
        Box((0.11, 0.010, 0.022)),
        origin=Origin(xyz=(-0.228, -0.036, 0.105)),
        material=frame_silver,
        name="right_tail_rail",
    )
    deck_frame.visual(
        Box((0.018, 0.014, 0.102)),
        origin=Origin(xyz=(-0.256, 0.025, 0.161)),
        material=trim_black,
        name="left_fender_support",
    )
    deck_frame.visual(
        Box((0.018, 0.014, 0.102)),
        origin=Origin(xyz=(-0.256, -0.025, 0.161)),
        material=trim_black,
        name="right_fender_support",
    )
    deck_frame.visual(
        Box((0.095, 0.060, 0.010)),
        origin=Origin(xyz=(-0.286, 0.0, 0.215)),
        material=trim_black,
        name="rear_fender",
    )
    deck_frame.visual(
        Box((0.032, 0.050, 0.014)),
        origin=Origin(xyz=(-0.332, 0.0, 0.218)),
        material=trim_black,
        name="brake_flap",
    )
    deck_frame.visual(
        Box((0.028, 0.010, 0.032)),
        origin=Origin(xyz=(-0.290, 0.030, 0.100)),
        material=trim_black,
        name="rear_left_dropout",
    )
    deck_frame.visual(
        Box((0.028, 0.010, 0.032)),
        origin=Origin(xyz=(-0.290, -0.030, 0.100)),
        material=trim_black,
        name="rear_right_dropout",
    )
    deck_frame.visual(
        Cylinder(radius=0.006, length=0.062),
        origin=Origin(
            xyz=(-0.290, 0.0, 0.100),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=axle_steel,
        name="rear_axle",
    )

    steering_assembly = model.part("steering_assembly")
    steering_assembly.inertial = Inertial.from_geometry(
        Box((0.20, 0.44, 0.92)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
    )
    steering_assembly.visual(
        Cylinder(radius=0.0285, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=trim_black,
        name="steerer_collar",
    )
    steering_assembly.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=frame_silver,
        name="steerer_spigot",
    )
    steering_stem_mesh = save_mesh(
        "scooter_steering_stem.obj",
        tube_from_spline_points(
            [
                (0.0, 0.0, 0.020),
                (-0.010, 0.0, 0.220),
                (-0.028, 0.0, 0.470),
                (-0.040, 0.0, 0.680),
            ],
            radius=0.016,
            samples_per_segment=18,
            radial_segments=18,
        ),
    )
    steering_assembly.visual(
        steering_stem_mesh,
        material=frame_silver,
        name="steering_stem",
    )
    steering_assembly.visual(
        Box((0.050, 0.040, 0.060)),
        origin=Origin(xyz=(-0.040, 0.0, 0.675)),
        material=trim_black,
        name="stem_clamp",
    )
    steering_assembly.visual(
        Cylinder(radius=0.013, length=0.420),
        origin=Origin(
            xyz=(-0.040, 0.0, 0.705),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=frame_silver,
        name="handlebar",
    )
    steering_assembly.visual(
        Cylinder(radius=0.016, length=0.100),
        origin=Origin(
            xyz=(-0.040, 0.160, 0.705),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=rubber_black,
        name="left_grip",
    )
    steering_assembly.visual(
        Cylinder(radius=0.016, length=0.100),
        origin=Origin(
            xyz=(-0.040, -0.160, 0.705),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=rubber_black,
        name="right_grip",
    )
    fork_neck = save_mesh(
        "scooter_fork_neck.obj",
        tube_from_spline_points(
            [
                (0.0, 0.0, 0.020),
                (0.030, 0.0, 0.050),
                (0.075, 0.0, 0.078),
            ],
            radius=0.010,
            samples_per_segment=16,
            radial_segments=16,
        ),
    )
    steering_assembly.visual(fork_neck, material=trim_black, name="fork_neck")
    steering_assembly.visual(
        Cylinder(radius=0.010, length=0.092),
        origin=Origin(
            xyz=(0.085, 0.0, 0.078),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_black,
        name="fork_bridge",
    )
    fork_left = save_mesh(
        "scooter_fork_left.obj",
        tube_from_spline_points(
            [
                (0.085, 0.032, 0.078),
                (0.130, 0.032, 0.020),
                (0.165, 0.032, -0.052),
            ],
            radius=0.0075,
            samples_per_segment=16,
            radial_segments=16,
        ),
    )
    fork_right = save_mesh(
        "scooter_fork_right.obj",
        tube_from_spline_points(
            [
                (0.085, -0.032, 0.078),
                (0.130, -0.032, 0.020),
                (0.165, -0.032, -0.052),
            ],
            radius=0.0075,
            samples_per_segment=16,
            radial_segments=16,
        ),
    )
    steering_assembly.visual(fork_left, material=trim_black, name="fork_left_blade")
    steering_assembly.visual(fork_right, material=trim_black, name="fork_right_blade")
    steering_assembly.visual(
        Cylinder(radius=0.006, length=0.056),
        origin=Origin(
            xyz=(0.165, 0.0, -0.055),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=axle_steel,
        name="front_axle",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = wheel_inertial(0.10, 0.030, mass=1.2)
    add_wheel_visuals(front_wheel, tire_radius=0.10, tire_width=0.030)

    rear_wheel = model.part("rear_wheel")
    rear_wheel.inertial = wheel_inertial(0.10, 0.030, mass=1.3)
    add_wheel_visuals(rear_wheel, tire_radius=0.10, tire_width=0.030)

    model.articulation(
        "headset_steer",
        ArticulationType.REVOLUTE,
        parent=deck_frame,
        child=steering_assembly,
        origin=Origin(xyz=(0.228, 0.0, 0.150)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=3.0,
            lower=-math.pi / 4.0,
            upper=math.pi / 4.0,
        ),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=steering_assembly,
        child=front_wheel,
        origin=Origin(xyz=(0.165, 0.0, -0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=deck_frame,
        child=rear_wheel,
        origin=Origin(xyz=(-0.290, 0.0, 0.100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    deck_frame = object_model.get_part("deck_frame")
    steering_assembly = object_model.get_part("steering_assembly")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    headset_steer = object_model.get_articulation("headset_steer")
    front_wheel_spin = object_model.get_articulation("front_wheel_spin")
    rear_wheel_spin = object_model.get_articulation("rear_wheel_spin")

    head_tube = deck_frame.get_visual("head_tube")
    deck_shell = deck_frame.get_visual("deck_shell")
    rear_fender = deck_frame.get_visual("rear_fender")
    rear_axle = deck_frame.get_visual("rear_axle")
    rear_left_dropout = deck_frame.get_visual("rear_left_dropout")
    rear_right_dropout = deck_frame.get_visual("rear_right_dropout")

    steerer_collar = steering_assembly.get_visual("steerer_collar")
    handlebar = steering_assembly.get_visual("handlebar")
    front_axle = steering_assembly.get_visual("front_axle")
    fork_left_blade = steering_assembly.get_visual("fork_left_blade")
    fork_right_blade = steering_assembly.get_visual("fork_right_blade")

    front_tire = front_wheel.get_visual("tire")
    front_hub = front_wheel.get_visual("hub")

    rear_tire = rear_wheel.get_visual("tire")
    rear_hub = rear_wheel.get_visual("hub")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        front_wheel,
        steering_assembly,
        reason="Front wheel hub surrounds the fork axle.",
        elem_a=front_hub,
        elem_b=front_axle,
    )
    ctx.allow_overlap(
        rear_wheel,
        deck_frame,
        reason="Rear wheel hub surrounds the rear axle.",
        elem_a=rear_hub,
        elem_b=rear_axle,
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "headset_axis_vertical",
        tuple(round(value, 6) for value in headset_steer.axis) == (0.0, 0.0, 1.0),
        f"Expected vertical steering axis, got {headset_steer.axis!r}",
    )
    ctx.check(
        "headset_limits_are_plus_minus_45_deg",
        abs(headset_steer.motion_limits.lower + math.pi / 4.0) < 1e-6
        and abs(headset_steer.motion_limits.upper - math.pi / 4.0) < 1e-6,
        "Front steering should sweep roughly +/-45 degrees.",
    )
    ctx.check(
        "front_wheel_spins_continuously_on_y_axis",
        front_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 6) for value in front_wheel_spin.axis) == (0.0, 1.0, 0.0),
        f"Expected continuous front wheel spin about y, got {front_wheel_spin.articulation_type!r} {front_wheel_spin.axis!r}",
    )
    ctx.check(
        "rear_wheel_spins_continuously_on_y_axis",
        rear_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 6) for value in rear_wheel_spin.axis) == (0.0, 1.0, 0.0),
        f"Expected continuous rear wheel spin about y, got {rear_wheel_spin.articulation_type!r} {rear_wheel_spin.axis!r}",
    )

    ctx.expect_gap(
        steering_assembly,
        deck_frame,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=steerer_collar,
        negative_elem=head_tube,
        name="steerer_collar_seats_on_head_tube",
    )
    ctx.expect_overlap(
        steering_assembly,
        deck_frame,
        axes="xy",
        min_overlap=0.05,
        elem_a=steerer_collar,
        elem_b=head_tube,
        name="steerer_centered_over_head_tube",
    )
    ctx.expect_overlap(
        front_wheel,
        steering_assembly,
        axes="xz",
        min_overlap=0.012,
        elem_a=front_hub,
        elem_b=front_axle,
        name="front_hub_centered_on_axle",
    )
    ctx.expect_overlap(
        rear_wheel,
        deck_frame,
        axes="xz",
        min_overlap=0.012,
        elem_a=rear_hub,
        elem_b=rear_axle,
        name="rear_hub_centered_on_axle",
    )
    ctx.expect_gap(
        steering_assembly,
        front_wheel,
        axis="y",
        min_gap=-0.001,
        max_gap=0.006,
        positive_elem=fork_left_blade,
        negative_elem=front_hub,
        name="left_fork_blade_sits_just_outboard_of_hub",
    )
    ctx.expect_gap(
        front_wheel,
        steering_assembly,
        axis="y",
        min_gap=-0.001,
        max_gap=0.006,
        positive_elem=front_hub,
        negative_elem=fork_right_blade,
        name="right_fork_blade_sits_just_outboard_of_hub",
    )
    ctx.expect_gap(
        deck_frame,
        rear_wheel,
        axis="y",
        min_gap=0.004,
        max_gap=0.016,
        positive_elem=rear_left_dropout,
        negative_elem=rear_hub,
        name="left_tail_dropout_clears_rear_hub",
    )
    ctx.expect_gap(
        rear_wheel,
        deck_frame,
        axis="y",
        min_gap=0.004,
        max_gap=0.016,
        positive_elem=rear_hub,
        negative_elem=rear_right_dropout,
        name="right_tail_dropout_clears_rear_hub",
    )
    ctx.expect_origin_distance(
        front_wheel,
        rear_wheel,
        axes="x",
        min_dist=0.67,
        max_dist=0.71,
        name="commuter_wheelbase",
    )
    ctx.expect_gap(
        steering_assembly,
        deck_frame,
        axis="z",
        min_gap=0.68,
        max_gap=0.78,
        positive_elem=handlebar,
        negative_elem=deck_shell,
        name="t_bar_height_above_deck",
    )
    ctx.expect_gap(
        deck_frame,
        rear_wheel,
        axis="z",
        min_gap=0.008,
        max_gap=0.025,
        positive_elem=rear_fender,
        negative_elem=rear_tire,
        name="rear_fender_clears_rear_wheel",
    )
    ctx.expect_overlap(
        deck_frame,
        rear_wheel,
        axes="x",
        min_overlap=0.03,
        elem_a=rear_fender,
        elem_b=rear_tire,
        name="rear_fender_spans_rear_wheel",
    )

    front_rest_pos = ctx.part_world_position(front_wheel)
    rear_rest_pos = ctx.part_world_position(rear_wheel)
    assert front_rest_pos is not None
    assert rear_rest_pos is not None

    with ctx.pose({headset_steer: math.radians(35.0)}):
        front_steered_left = ctx.part_world_position(front_wheel)
        assert front_steered_left is not None
        ctx.check(
            "front_wheel_moves_laterally_when_steered_left",
            front_steered_left[1] > front_rest_pos[1] + 0.035 and front_steered_left[0] < front_rest_pos[0],
            f"Front wheel should swing left under steering pose, got rest={front_rest_pos!r}, steered={front_steered_left!r}",
        )
        ctx.expect_overlap(front_wheel, steering_assembly, axes="xz", min_overlap=0.012, elem_a=front_hub, elem_b=front_axle)

    with ctx.pose({headset_steer: math.radians(-35.0)}):
        front_steered_right = ctx.part_world_position(front_wheel)
        assert front_steered_right is not None
        ctx.check(
            "front_wheel_moves_laterally_when_steered_right",
            front_steered_right[1] < front_rest_pos[1] - 0.035 and front_steered_right[0] < front_rest_pos[0],
            f"Front wheel should swing right under steering pose, got rest={front_rest_pos!r}, steered={front_steered_right!r}",
        )

    with ctx.pose({front_wheel_spin: math.pi / 2.0, rear_wheel_spin: math.pi / 3.0}):
        front_spin_pos = ctx.part_world_position(front_wheel)
        rear_spin_pos = ctx.part_world_position(rear_wheel)
        assert front_spin_pos is not None
        assert rear_spin_pos is not None
        ctx.check(
            "wheel_spin_preserves_front_axle_position",
            all(abs(a - b) < 1e-9 for a, b in zip(front_spin_pos, front_rest_pos)),
            f"Front wheel origin shifted under spin pose: rest={front_rest_pos!r}, spin={front_spin_pos!r}",
        )
        ctx.check(
            "wheel_spin_preserves_rear_axle_position",
            all(abs(a - b) < 1e-9 for a, b in zip(rear_spin_pos, rear_rest_pos)),
            f"Rear wheel origin shifted under spin pose: rest={rear_rest_pos!r}, spin={rear_spin_pos!r}",
        )
        ctx.expect_overlap(rear_wheel, deck_frame, axes="xz", min_overlap=0.012, elem_a=rear_hub, elem_b=rear_axle)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

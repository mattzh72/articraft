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
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def _add_wheel_visuals(part, *, tire_radius: float, tire_width: float, rubber, rim, hub) -> None:
    spin_axis = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    flange_offset = tire_width * 0.17

    part.visual(
        Cylinder(radius=tire_radius, length=tire_width),
        origin=spin_axis,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.68, length=tire_width * 0.72),
        origin=spin_axis,
        material=rim,
        name="rim_core",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.50, length=0.006),
        origin=Origin(xyz=(0.0, flange_offset, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rim,
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.50, length=0.006),
        origin=Origin(xyz=(0.0, -flange_offset, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rim,
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.28, length=tire_width * 1.08),
        origin=spin_axis,
        material=hub,
        name="hub",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rental_knee_scooter")

    frame_gray = model.material("frame_gray", rgba=(0.63, 0.66, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.06, 1.0))
    wheel_silver = model.material("wheel_silver", rgba=(0.77, 0.79, 0.82, 1.0))
    deck_blue = model.material("deck_blue", rgba=(0.20, 0.33, 0.63, 1.0))
    deck_shell = model.material("deck_shell", rgba=(0.16, 0.18, 0.20, 1.0))

    tire_radius = 0.10
    tire_width = 0.035

    center_frame = model.part("center_frame")
    center_frame.inertial = Inertial.from_geometry(
        Box((0.76, 0.42, 0.58)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
    )

    center_frame.visual(
        Box((0.36, 0.08, 0.05)),
        origin=Origin(xyz=(-0.01, 0.0, 0.21)),
        material=frame_gray,
        name="main_spine",
    )
    center_frame.visual(
        Box((0.07, 0.10, 0.14)),
        origin=Origin(xyz=(0.17, 0.0, 0.27)),
        material=frame_gray,
        name="front_gusset",
    )
    center_frame.visual(
        Box((0.34, 0.14, 0.04)),
        origin=Origin(xyz=(-0.04, 0.0, 0.37)),
        material=frame_gray,
        name="deck_support",
    )
    center_frame.visual(
        Box((0.04, 0.10, 0.17)),
        origin=Origin(xyz=(0.05, 0.0, 0.305)),
        material=frame_gray,
        name="front_support_post",
    )
    center_frame.visual(
        Box((0.04, 0.14, 0.23)),
        origin=Origin(xyz=(-0.15, 0.0, 0.275)),
        material=frame_gray,
        name="rear_support_post",
    )

    head_tube = LatheGeometry.from_shell_profiles(
        [(0.035, -0.09), (0.035, 0.09)],
        [(0.026, -0.09), (0.026, 0.09)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    center_frame.visual(
        _mesh("knee_scooter_head_tube", head_tube),
        origin=Origin(xyz=(0.24, 0.0, 0.33)),
        material=dark_steel,
        name="head_tube",
    )

    upper_brace = tube_from_spline_points(
        [(0.21, 0.0, 0.36), (0.14, 0.0, 0.35), (0.05, 0.0, 0.34)],
        radius=0.014,
        samples_per_segment=12,
        radial_segments=18,
    )
    center_frame.visual(
        _mesh("knee_scooter_upper_brace", upper_brace),
        material=frame_gray,
        name="upper_brace",
    )

    left_rail_points = [
        (0.15, 0.05, 0.23),
        (0.00, 0.10, 0.24),
        (-0.15, 0.14, 0.24),
        (-0.24, 0.18, 0.24),
    ]
    left_side_rail = tube_from_spline_points(
        left_rail_points,
        radius=0.016,
        samples_per_segment=12,
        radial_segments=18,
    )
    right_side_rail = tube_from_spline_points(
        _mirror_y(left_rail_points),
        radius=0.016,
        samples_per_segment=12,
        radial_segments=18,
    )
    center_frame.visual(_mesh("knee_scooter_left_side_rail", left_side_rail), material=frame_gray)
    center_frame.visual(_mesh("knee_scooter_right_side_rail", right_side_rail), material=frame_gray)

    center_frame.visual(
        Box((0.05, 0.36, 0.03)),
        origin=Origin(xyz=(-0.24, 0.0, 0.226)),
        material=frame_gray,
        name="rear_pivot_beam",
    )
    center_frame.visual(
        Box((0.05, 0.03, 0.15)),
        origin=Origin(xyz=(-0.25, 0.19, 0.18)),
        material=dark_steel,
        name="left_rear_bracket",
    )
    center_frame.visual(
        Box((0.05, 0.03, 0.15)),
        origin=Origin(xyz=(-0.25, -0.19, 0.18)),
        material=dark_steel,
        name="right_rear_bracket",
    )
    center_frame.visual(
        Cylinder(radius=0.016, length=0.016),
        origin=Origin(xyz=(-0.25, 0.1719, 0.10), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_rear_axle_stub",
    )
    center_frame.visual(
        Cylinder(radius=0.016, length=0.016),
        origin=Origin(xyz=(-0.25, -0.1719, 0.10), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_rear_axle_stub",
    )

    knee_deck = model.part("knee_deck")
    knee_deck.inertial = Inertial.from_geometry(
        Box((0.32, 0.16, 0.07)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )
    deck_base = ExtrudeGeometry.centered(
        rounded_rect_profile(0.30, 0.16, 0.028),
        0.018,
        cap=True,
        closed=True,
    )
    deck_pad = ExtrudeGeometry.centered(
        rounded_rect_profile(0.28, 0.14, 0.040),
        0.046,
        cap=True,
        closed=True,
    )
    knee_deck.visual(
        _mesh("knee_scooter_deck_base", deck_base),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=deck_shell,
        name="deck_base",
    )
    knee_deck.visual(
        _mesh("knee_scooter_deck_pad", deck_pad),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=deck_blue,
        name="deck_pad",
    )

    front_fork = model.part("front_fork")
    front_fork.inertial = Inertial.from_geometry(
        Box((0.46, 0.42, 0.76)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
    )
    front_fork.visual(
        Cylinder(radius=0.021, length=0.28),
        material=dark_steel,
        name="steering_stem",
    )
    front_fork.visual(
        Cylinder(radius=0.033, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.096)),
        material=dark_steel,
        name="lower_headset_collar",
    )
    front_fork.visual(
        Cylinder(radius=0.033, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=dark_steel,
        name="upper_headset_collar",
    )
    front_fork.visual(
        Cylinder(radius=0.017, length=0.46),
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        material=dark_steel,
        name="handle_post",
    )
    front_fork.visual(
        Cylinder(radius=0.015, length=0.38),
        origin=Origin(xyz=(0.0, 0.0, 0.56), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="handlebar",
    )
    front_fork.visual(
        Cylinder(radius=0.019, length=0.09),
        origin=Origin(xyz=(0.0, 0.145, 0.56), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
    )
    front_fork.visual(
        Cylinder(radius=0.019, length=0.09),
        origin=Origin(xyz=(0.0, -0.145, 0.56), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
    )
    front_fork.visual(
        Box((0.05, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.53)),
        material=dark_steel,
    )
    front_fork.visual(
        Box((0.10, 0.24, 0.05)),
        origin=Origin(xyz=(0.06, 0.0, -0.145)),
        material=frame_gray,
        name="fork_crown",
    )
    front_fork.visual(
        Box((0.06, 0.36, 0.03)),
        origin=Origin(xyz=(0.06, 0.0, -0.11)),
        material=frame_gray,
        name="fork_bridge",
    )
    front_fork.visual(
        Box((0.05, 0.03, 0.16)),
        origin=Origin(xyz=(0.06, 0.19, -0.18)),
        material=frame_gray,
        name="left_fork_leg",
    )
    front_fork.visual(
        Box((0.05, 0.03, 0.16)),
        origin=Origin(xyz=(0.06, -0.19, -0.18)),
        material=frame_gray,
        name="right_fork_leg",
    )
    front_fork.visual(
        Cylinder(radius=0.016, length=0.016),
        origin=Origin(xyz=(0.06, 0.1719, -0.23), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_front_axle_stub",
    )
    front_fork.visual(
        Cylinder(radius=0.016, length=0.016),
        origin=Origin(xyz=(0.06, -0.1719, -0.23), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_front_axle_stub",
    )

    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=tire_radius, length=tire_width),
        mass=1.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel_visuals(
        front_left_wheel,
        tire_radius=tire_radius,
        tire_width=tire_width,
        rubber=rubber_black,
        rim=wheel_silver,
        hub=dark_steel,
    )

    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=tire_radius, length=tire_width),
        mass=1.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel_visuals(
        front_right_wheel,
        tire_radius=tire_radius,
        tire_width=tire_width,
        rubber=rubber_black,
        rim=wheel_silver,
        hub=dark_steel,
    )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=tire_radius, length=tire_width),
        mass=1.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel_visuals(
        rear_left_wheel,
        tire_radius=tire_radius,
        tire_width=tire_width,
        rubber=rubber_black,
        rim=wheel_silver,
        hub=dark_steel,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=tire_radius, length=tire_width),
        mass=1.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_wheel_visuals(
        rear_right_wheel,
        tire_radius=tire_radius,
        tire_width=tire_width,
        rubber=rubber_black,
        rim=wheel_silver,
        hub=dark_steel,
    )

    brake_bar = model.part("brake_bar")
    brake_bar.inertial = Inertial.from_geometry(
        Box((0.14, 0.32, 0.08)),
        mass=0.8,
        origin=Origin(xyz=(-0.04, 0.0, 0.01)),
    )
    brake_bar.visual(
        Cylinder(radius=0.014, length=0.30),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_tube",
    )
    brake_bar.visual(
        Box((0.06, 0.018, 0.018)),
        origin=Origin(xyz=(-0.03, 0.145, 0.0)),
        material=dark_steel,
    )
    brake_bar.visual(
        Box((0.06, 0.018, 0.018)),
        origin=Origin(xyz=(-0.03, -0.145, 0.0)),
        material=dark_steel,
    )
    brake_bar.visual(
        Box((0.016, 0.018, 0.030)),
        origin=Origin(xyz=(-0.055, 0.145, -0.012)),
        material=dark_steel,
    )
    brake_bar.visual(
        Box((0.016, 0.018, 0.030)),
        origin=Origin(xyz=(-0.055, -0.145, -0.012)),
        material=dark_steel,
    )
    brake_bar.visual(
        Box((0.025, 0.028, 0.014)),
        origin=Origin(xyz=(-0.055, 0.145, -0.023)),
        material=rubber_black,
        name="left_shoe",
    )
    brake_bar.visual(
        Box((0.025, 0.028, 0.014)),
        origin=Origin(xyz=(-0.055, -0.145, -0.023)),
        material=rubber_black,
        name="right_shoe",
    )
    brake_bar.visual(
        Box((0.08, 0.025, 0.018)),
        origin=Origin(xyz=(-0.040, 0.0, 0.0)),
        material=dark_steel,
    )
    brake_bar.visual(
        Box((0.04, 0.04, 0.055)),
        origin=Origin(xyz=(-0.050, 0.0, 0.015)),
        material=dark_steel,
    )
    brake_bar.visual(
        Box((0.07, 0.10, 0.018)),
        origin=Origin(xyz=(-0.055, 0.0, 0.039)),
        material=dark_steel,
        name="pedal_plate",
    )
    brake_bar.visual(
        Box((0.05, 0.08, 0.006)),
        origin=Origin(xyz=(-0.055, 0.0, 0.049)),
        material=rubber_black,
    )

    model.articulation(
        "deck_mount",
        ArticulationType.FIXED,
        parent=center_frame,
        child=knee_deck,
        origin=Origin(xyz=(-0.04, 0.0, 0.39)),
    )
    model.articulation(
        "front_steer",
        ArticulationType.REVOLUTE,
        parent=center_frame,
        child=front_fork,
        origin=Origin(xyz=(0.24, 0.0, 0.33)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.5, lower=-0.60, upper=0.60),
    )
    model.articulation(
        "front_left_spin",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_left_wheel,
        origin=Origin(xyz=(0.06, 0.145, -0.23)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "front_right_spin",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_right_wheel,
        origin=Origin(xyz=(0.06, -0.145, -0.23)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "rear_left_spin",
        ArticulationType.CONTINUOUS,
        parent=center_frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(-0.25, 0.145, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "rear_right_spin",
        ArticulationType.CONTINUOUS,
        parent=center_frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.25, -0.145, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "brake_bar_pivot",
        ArticulationType.REVOLUTE,
        parent=center_frame,
        child=brake_bar,
        origin=Origin(xyz=(-0.24, 0.0, 0.255)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=0.42),
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

    center_frame = object_model.get_part("center_frame")
    knee_deck = object_model.get_part("knee_deck")
    front_fork = object_model.get_part("front_fork")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    brake_bar = object_model.get_part("brake_bar")

    front_steer = object_model.get_articulation("front_steer")
    front_left_spin = object_model.get_articulation("front_left_spin")
    front_right_spin = object_model.get_articulation("front_right_spin")
    rear_left_spin = object_model.get_articulation("rear_left_spin")
    rear_right_spin = object_model.get_articulation("rear_right_spin")
    brake_bar_pivot = object_model.get_articulation("brake_bar_pivot")

    ctx.expect_contact(
        knee_deck,
        center_frame,
        elem_a="deck_base",
        elem_b="deck_support",
        contact_tol=0.001,
        name="knee deck sits on the frame support",
    )

    ctx.check(
        "steering joint is vertical revolute",
        front_steer.articulation_type == ArticulationType.REVOLUTE and front_steer.axis == (0.0, 0.0, 1.0),
        details=f"type={front_steer.articulation_type}, axis={front_steer.axis}",
    )
    ctx.check(
        "all wheel joints are continuous on lateral axles",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS and joint.axis == (0.0, 1.0, 0.0)
            for joint in (front_left_spin, front_right_spin, rear_left_spin, rear_right_spin)
        ),
        details=str(
            {
                joint.name: {"type": joint.articulation_type, "axis": joint.axis}
                for joint in (front_left_spin, front_right_spin, rear_left_spin, rear_right_spin)
            }
        ),
    )
    ctx.check(
        "brake bar pivots above the rear wheels on a lateral axis",
        brake_bar_pivot.articulation_type == ArticulationType.REVOLUTE
        and brake_bar_pivot.axis == (0.0, -1.0, 0.0)
        and brake_bar_pivot.origin.xyz[2] > 0.20,
        details=f"type={brake_bar_pivot.articulation_type}, axis={brake_bar_pivot.axis}, origin={brake_bar_pivot.origin}",
    )

    with ctx.pose({brake_bar_pivot: 0.0}):
        ctx.expect_gap(
            brake_bar,
            rear_left_wheel,
            axis="z",
            min_gap=0.01,
            max_gap=0.04,
            positive_elem="left_shoe",
            negative_elem="tire",
            name="left brake shoe rests above the rear wheel",
        )
        ctx.expect_gap(
            brake_bar,
            rear_right_wheel,
            axis="z",
            min_gap=0.01,
            max_gap=0.04,
            positive_elem="right_shoe",
            negative_elem="tire",
            name="right brake shoe rests above the rear wheel",
        )

    with ctx.pose({brake_bar_pivot: 0.42}):
        ctx.expect_gap(
            brake_bar,
            rear_left_wheel,
            axis="z",
            max_gap=0.004,
            max_penetration=0.0,
            positive_elem="left_shoe",
            negative_elem="tire",
            name="left brake shoe swings down to the tire",
        )
        ctx.expect_gap(
            brake_bar,
            rear_right_wheel,
            axis="z",
            max_gap=0.004,
            max_penetration=0.0,
            positive_elem="right_shoe",
            negative_elem="tire",
            name="right brake shoe swings down to the tire",
        )

    left_front_rest = ctx.part_world_position(front_left_wheel)
    right_front_rest = ctx.part_world_position(front_right_wheel)
    with ctx.pose({front_steer: 0.35}):
        left_front_steered = ctx.part_world_position(front_left_wheel)
        right_front_steered = ctx.part_world_position(front_right_wheel)
        fork_steered = ctx.part_world_position(front_fork)
    fork_rest = ctx.part_world_position(front_fork)

    ctx.check(
        "positive steering yaws the front fork and wheels to the rider left",
        left_front_rest is not None
        and left_front_steered is not None
        and right_front_rest is not None
        and right_front_steered is not None
        and fork_rest is not None
        and fork_steered is not None
        and left_front_steered[1] > left_front_rest[1] + 0.01
        and right_front_steered[1] > right_front_rest[1] + 0.01
        and abs(fork_steered[0] - fork_rest[0]) < 1e-6
        and abs(fork_steered[1] - fork_rest[1]) < 1e-6,
        details=(
            f"left_rest={left_front_rest}, left_steered={left_front_steered}, "
            f"right_rest={right_front_rest}, right_steered={right_front_steered}, "
            f"fork_rest={fork_rest}, fork_steered={fork_steered}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

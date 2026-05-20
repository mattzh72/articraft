from __future__ import annotations

from pathlib import Path

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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ABS_ASSET_ROOT = Path("/")
ASSETS = AssetContext(ABS_ASSET_ROOT)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kids_folding_scooter", assets=ASSETS)

    deck_teal = model.material("deck_teal", rgba=(0.20, 0.70, 0.74, 1.0))
    stem_silver = model.material("stem_silver", rgba=(0.80, 0.83, 0.86, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    grip_orange = model.material("grip_orange", rgba=(0.94, 0.53, 0.19, 1.0))
    wheel_core = model.material("wheel_core", rgba=(0.86, 0.88, 0.90, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((0.33, 0.11, 0.016)),
        origin=Origin(xyz=(-0.005, 0.0, 0.078)),
        material=deck_teal,
        name="deck_shell",
    )
    deck.visual(
        Box((0.22, 0.082, 0.004)),
        origin=Origin(xyz=(-0.025, 0.0, 0.088)),
        material=charcoal,
        name="deck_grip",
    )
    deck.visual(
        Box((0.030, 0.058, 0.014)),
        origin=Origin(xyz=(0.094, 0.0, 0.073)),
        material=stem_silver,
        name="hinge_block",
    )
    deck.visual(
        Box((0.022, 0.014, 0.030)),
        origin=Origin(xyz=(0.145, -0.028, 0.096)),
        material=stem_silver,
        name="hinge_left_cheek",
    )
    deck.visual(
        Box((0.022, 0.014, 0.030)),
        origin=Origin(xyz=(0.145, 0.028, 0.096)),
        material=stem_silver,
        name="hinge_right_cheek",
    )
    deck.visual(
        Box((0.090, 0.026, 0.010)),
        origin=Origin(xyz=(-0.205, -0.083, 0.091)),
        material=stem_silver,
        name="rear_left_arm",
    )
    deck.visual(
        Box((0.090, 0.026, 0.010)),
        origin=Origin(xyz=(-0.205, 0.083, 0.091)),
        material=stem_silver,
        name="rear_right_arm",
    )
    deck.visual(
        Box((0.020, 0.010, 0.050)),
        origin=Origin(xyz=(-0.225, -0.084, 0.061)),
        material=stem_silver,
        name="rear_left_mount",
    )
    deck.visual(
        Box((0.020, 0.010, 0.050)),
        origin=Origin(xyz=(-0.225, 0.084, 0.061)),
        material=stem_silver,
        name="rear_right_mount",
    )
    deck.visual(
        Box((0.062, 0.140, 0.010)),
        origin=Origin(xyz=(-0.191, 0.0, 0.091)),
        material=deck_teal,
        name="rear_bridge",
    )
    deck.visual(
        Box((0.012, 0.040, 0.046)),
        origin=Origin(xyz=(-0.227, 0.0, 0.063)),
        material=stem_silver,
        name="rear_bearing_block",
    )
    deck.inertial = Inertial.from_geometry(
        Box((0.37, 0.18, 0.07)),
        mass=1.2,
        origin=Origin(xyz=(-0.02, 0.0, 0.080)),
    )

    fold_stem = model.part("fold_stem")
    fold_stem.visual(
        Cylinder(radius=0.010, length=0.042),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="hinge_barrel",
    )
    fold_stem.visual(
        Box((0.034, 0.050, 0.024)),
        origin=Origin(xyz=(0.004, 0.0, 0.021)),
        material=stem_silver,
        name="stem_base",
    )
    fold_stem.visual(
        Cylinder(radius=0.017, length=0.036),
        origin=Origin(xyz=(0.010, 0.0, 0.018)),
        material=stem_silver,
        name="steering_pedestal",
    )
    fold_stem.inertial = Inertial.from_geometry(
        Box((0.07, 0.08, 0.08)),
        mass=0.35,
        origin=Origin(xyz=(0.008, 0.0, 0.025)),
    )

    steering = model.part("steering_assembly")
    steering.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=stem_silver,
        name="headset_base",
    )
    steering.visual(
        Cylinder(radius=0.013, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, 0.213)),
        material=stem_silver,
        name="main_stem",
    )
    steering.visual(
        Box((0.028, 0.052, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.402)),
        material=stem_silver,
        name="y_junction",
    )
    _add_tube(
        steering,
        "y_left_branch",
        (0.0, 0.0, 0.395),
        (0.0, -0.115, 0.540),
        radius=0.010,
        material=stem_silver,
    )
    _add_tube(
        steering,
        "y_right_branch",
        (0.0, 0.0, 0.395),
        (0.0, 0.115, 0.540),
        radius=0.010,
        material=stem_silver,
    )
    steering.visual(
        Cylinder(radius=0.011, length=0.290),
        origin=Origin(xyz=(0.0, 0.0, 0.540), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stem_silver,
        name="handlebar_cross",
    )
    steering.visual(
        Cylinder(radius=0.016, length=0.070),
        origin=Origin(xyz=(0.0, -0.180, 0.540), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_orange,
        name="left_grip",
    )
    steering.visual(
        Cylinder(radius=0.016, length=0.070),
        origin=Origin(xyz=(0.0, 0.180, 0.540), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_orange,
        name="right_grip",
    )
    _add_tube(
        steering,
        "fork_neck",
        (0.0, 0.0, 0.016),
        (0.034, 0.0, 0.002),
        radius=0.006,
        material=stem_silver,
    )
    steering.visual(
        Box((0.036, 0.068, 0.012)),
        origin=Origin(xyz=(0.052, 0.0, 0.000)),
        material=stem_silver,
        name="fork_crown",
    )
    _add_tube(
        steering,
        "left_fork_leg",
        (0.058, -0.036, -0.004),
        (0.096, -0.036, -0.078),
        radius=0.004,
        material=stem_silver,
    )
    _add_tube(
        steering,
        "right_fork_leg",
        (0.058, 0.036, -0.004),
        (0.096, 0.036, -0.078),
        radius=0.004,
        material=stem_silver,
    )
    steering.visual(
        Box((0.008, 0.008, 0.016)),
        origin=Origin(xyz=(0.096, -0.036, -0.078)),
        material=stem_silver,
        name="left_dropout",
    )
    steering.visual(
        Box((0.008, 0.008, 0.016)),
        origin=Origin(xyz=(0.096, 0.036, -0.078)),
        material=stem_silver,
        name="right_dropout",
    )
    steering.visual(
        Cylinder(radius=0.004, length=0.072),
        origin=Origin(xyz=(0.096, 0.0, -0.082), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="front_axle",
    )
    steering.inertial = Inertial.from_geometry(
        Box((0.18, 0.42, 0.62)),
        mass=0.85,
        origin=Origin(xyz=(0.02, 0.0, 0.260)),
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        Cylinder(radius=0.050, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="front_tire",
    )
    front_wheel.visual(
        Cylinder(radius=0.028, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_core,
        name="front_core",
    )
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.018),
        mass=0.18,
        origin=Origin(),
    )

    rear_wheelset = model.part("rear_wheelset")
    rear_wheelset.visual(
        Cylinder(radius=0.006, length=0.138),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="axle_shaft",
    )
    rear_wheelset.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.0, -0.074, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="left_axle_end",
    )
    rear_wheelset.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.0, 0.074, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="right_axle_end",
    )
    rear_wheelset.visual(
        Cylinder(radius=0.040, length=0.018),
        origin=Origin(xyz=(0.0, -0.055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_rear_wheel",
    )
    rear_wheelset.visual(
        Cylinder(radius=0.040, length=0.018),
        origin=Origin(xyz=(0.0, 0.055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_rear_wheel",
    )
    rear_wheelset.visual(
        Cylinder(radius=0.025, length=0.020),
        origin=Origin(xyz=(0.0, -0.055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_core,
        name="left_rear_core",
    )
    rear_wheelset.visual(
        Cylinder(radius=0.025, length=0.020),
        origin=Origin(xyz=(0.0, 0.055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_core,
        name="right_rear_core",
    )
    rear_wheelset.inertial = Inertial.from_geometry(
        Box((0.09, 0.15, 0.08)),
        mass=0.35,
        origin=Origin(),
    )

    model.articulation(
        "stem_fold",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=fold_stem,
        origin=Origin(xyz=(0.145, 0.0, 0.096)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-1.35,
            upper=0.15,
        ),
    )
    model.articulation(
        "steering_turn",
        ArticulationType.REVOLUTE,
        parent=fold_stem,
        child=steering,
        origin=Origin(xyz=(0.010, 0.0, 0.036)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=9.0,
            velocity=3.0,
            lower=-0.70,
            upper=0.70,
        ),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=steering,
        child=front_wheel,
        origin=Origin(xyz=(0.096, 0.0, -0.082)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=22.0),
    )
    model.articulation(
        "rear_axle_spin",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheelset,
        origin=Origin(xyz=(-0.215, 0.0, 0.040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    fold_stem = object_model.get_part("fold_stem")
    steering = object_model.get_part("steering_assembly")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheelset = object_model.get_part("rear_wheelset")
    stem_fold = object_model.get_articulation("stem_fold")
    steering_turn = object_model.get_articulation("steering_turn")
    front_wheel_spin = object_model.get_articulation("front_wheel_spin")
    rear_axle_spin = object_model.get_articulation("rear_axle_spin")

    deck_shell = deck.get_visual("deck_shell")
    hinge_block = deck.get_visual("hinge_block")
    hinge_left_cheek = deck.get_visual("hinge_left_cheek")
    hinge_right_cheek = deck.get_visual("hinge_right_cheek")
    rear_left_mount = deck.get_visual("rear_left_mount")
    rear_right_mount = deck.get_visual("rear_right_mount")
    rear_bridge = deck.get_visual("rear_bridge")
    rear_bearing_block = deck.get_visual("rear_bearing_block")

    hinge_barrel = fold_stem.get_visual("hinge_barrel")
    stem_base = fold_stem.get_visual("stem_base")
    steering_pedestal = fold_stem.get_visual("steering_pedestal")

    headset_base = steering.get_visual("headset_base")
    main_stem = steering.get_visual("main_stem")
    handlebar_cross = steering.get_visual("handlebar_cross")
    left_grip = steering.get_visual("left_grip")
    right_grip = steering.get_visual("right_grip")
    left_dropout = steering.get_visual("left_dropout")
    right_dropout = steering.get_visual("right_dropout")
    front_axle = steering.get_visual("front_axle")

    front_core = front_wheel.get_visual("front_core")
    front_tire = front_wheel.get_visual("front_tire")

    axle_shaft = rear_wheelset.get_visual("axle_shaft")
    left_axle_end = rear_wheelset.get_visual("left_axle_end")
    right_axle_end = rear_wheelset.get_visual("right_axle_end")
    left_rear_wheel = rear_wheelset.get_visual("left_rear_wheel")
    right_rear_wheel = rear_wheelset.get_visual("right_rear_wheel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        steering,
        front_wheel,
        elem_a=front_axle,
        elem_b=front_core,
        reason="front wheel hub is represented as a solid core surrounding the steering axle",
    )
    ctx.allow_overlap(
        steering,
        front_wheel,
        elem_a=front_axle,
        elem_b=front_tire,
        reason="front tire is authored as a solid primitive, so the steering axle intentionally passes through its implied center bore",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    ctx.expect_contact(fold_stem, deck, elem_a=hinge_barrel, elem_b=hinge_left_cheek)
    ctx.expect_contact(fold_stem, deck, elem_a=hinge_barrel, elem_b=hinge_right_cheek)
    ctx.expect_contact(steering, fold_stem, elem_a=headset_base, elem_b=steering_pedestal)
    ctx.expect_contact(steering, steering, elem_a=front_axle, elem_b=left_dropout)
    ctx.expect_contact(steering, steering, elem_a=front_axle, elem_b=right_dropout)
    ctx.expect_overlap(front_wheel, steering, axes="y", min_overlap=0.010, elem_a=front_core, elem_b=front_axle)
    ctx.expect_contact(rear_wheelset, deck, elem_a=left_axle_end, elem_b=rear_left_mount)
    ctx.expect_contact(rear_wheelset, deck, elem_a=right_axle_end, elem_b=rear_right_mount)
    ctx.expect_contact(rear_wheelset, deck, elem_a=axle_shaft, elem_b=rear_bearing_block)
    ctx.expect_gap(
        steering,
        deck,
        axis="z",
        min_gap=0.44,
        positive_elem=handlebar_cross,
        negative_elem=deck_shell,
        name="upright_handlebar_sits_well_above_deck",
    )
    ctx.expect_gap(
        steering,
        steering,
        axis="y",
        min_gap=0.28,
        positive_elem=right_grip,
        negative_elem=left_grip,
        name="y_bar_handlebar_spans_wide_between_grips",
    )
    ctx.expect_origin_gap(
        front_wheel,
        rear_wheelset,
        axis="x",
        min_gap=0.40,
        name="wheelbase_is_long_for_a_stable_child_scooter",
    )
    ctx.expect_gap(
        rear_wheelset,
        rear_wheelset,
        axis="y",
        min_gap=0.09,
        positive_elem=right_rear_wheel,
        negative_elem=left_rear_wheel,
        name="rear_wheels_are_split_across_shared_axle",
    )

    with ctx.pose({stem_fold: stem_fold.motion_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="stem_fold_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="stem_fold_lower_no_floating")
        ctx.expect_overlap(
            steering,
            deck,
            axes="x",
            min_overlap=0.015,
            elem_a=main_stem,
            elem_b=deck_shell,
            name="folded_stem_projects_back_over_deck",
        )
        ctx.expect_gap(
            steering,
            deck,
            axis="z",
            min_gap=0.10,
            max_gap=0.22,
            positive_elem=handlebar_cross,
            negative_elem=deck_shell,
            name="folded_handlebar_stays_low_but_clear_of_deck",
        )
    with ctx.pose({steering_turn: steering_turn.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="steering_turn_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="steering_turn_upper_no_floating")
        ctx.expect_origin_gap(
            front_wheel,
            fold_stem,
            axis="y",
            min_gap=0.03,
            name="front_wheel_tracks_to_the_right_when_steered_right",
        )
    with ctx.pose({steering_turn: steering_turn.motion_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="steering_turn_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="steering_turn_lower_no_floating")
        ctx.expect_origin_gap(
            fold_stem,
            front_wheel,
            axis="y",
            min_gap=0.03,
            name="front_wheel_tracks_to_the_left_when_steered_left",
        )
    with ctx.pose({front_wheel_spin: 1.2}):
        ctx.expect_overlap(front_wheel, steering, axes="y", min_overlap=0.010, elem_a=front_core, elem_b=front_axle)
    with ctx.pose({rear_axle_spin: 1.1}):
        ctx.expect_contact(rear_wheelset, deck, elem_a=left_axle_end, elem_b=rear_left_mount)
        ctx.expect_contact(rear_wheelset, deck, elem_a=right_axle_end, elem_b=rear_right_mount)
        ctx.expect_contact(rear_wheelset, deck, elem_a=axle_shaft, elem_b=rear_bearing_block)
    return ctx.report()


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_tube(
    part,
    name: str,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


# >>> USER_CODE_END

object_model = build_object_model()

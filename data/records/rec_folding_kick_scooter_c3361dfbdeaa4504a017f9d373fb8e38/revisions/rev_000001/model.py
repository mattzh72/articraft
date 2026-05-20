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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


DECK_LENGTH = 0.46
DECK_WIDTH = 0.13
DECK_THICKNESS = 0.016
DECK_CENTER_X = 0.05
DECK_CENTER_Z = 0.052

HINGE_X = DECK_CENTER_X + DECK_LENGTH * 0.5
HINGE_Z = 0.073

CROWN_X_REL = 0.060
CROWN_Z_REL = 0.175

FRONT_WHEEL_RADIUS = 0.060
FRONT_WHEEL_WIDTH = 0.024
FRONT_TRACK_HALF = 0.115
FRONT_WHEEL_X = 0.060
FRONT_WHEEL_Z = -0.188

REAR_WHEEL_RADIUS = 0.045
REAR_WHEEL_WIDTH = 0.024
REAR_WHEEL_X = -0.247
REAR_WHEEL_Z = REAR_WHEEL_RADIUS

FOLD_UPPER = math.radians(72.0)
TILT_LIMIT = math.radians(22.0)


def _rod_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[float, Origin]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (
        length,
        Origin(
            xyz=((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
    )


def _add_rod(part, *, start, end, radius: float, material, name: str) -> None:
    length, origin = _rod_origin(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _add_wheel_visuals(part, *, radius: float, width: float, rubber, rim_metal) -> None:
    spin_origin = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=radius, length=width),
        origin=spin_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=radius * 0.70, length=width * 0.72),
        origin=spin_origin,
        material=rim_metal,
        name="rim",
    )
    part.visual(
        Cylinder(radius=radius * 0.25, length=width * 0.42),
        origin=spin_origin,
        material=rim_metal,
        name="hub_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_wheel_kick_scooter")

    frame_silver = model.material("frame_silver", rgba=(0.78, 0.80, 0.83, 1.0))
    deck_teal = model.material("deck_teal", rgba=(0.18, 0.69, 0.68, 1.0))
    deck_black = model.material("deck_black", rgba=(0.14, 0.15, 0.17, 1.0))
    grip_gray = model.material("grip_gray", rgba=(0.24, 0.26, 0.28, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    rim_metal = model.material("rim_metal", rgba=(0.70, 0.73, 0.76, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((DECK_LENGTH, DECK_WIDTH, DECK_THICKNESS)),
        origin=Origin(xyz=(DECK_CENTER_X, 0.0, DECK_CENTER_Z)),
        material=deck_teal,
        name="deck_shell",
    )
    deck.visual(
        Box((0.34, 0.096, 0.003)),
        origin=Origin(xyz=(0.015, 0.0, DECK_CENTER_Z + DECK_THICKNESS * 0.5 + 0.0015)),
        material=grip_gray,
        name="grip_pad",
    )
    deck.visual(
        Box((0.040, 0.050, 0.026)),
        origin=Origin(xyz=(0.250, 0.0, HINGE_Z)),
        material=frame_silver,
        name="front_hinge_block",
    )
    deck.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(HINGE_X, -0.012, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_silver,
        name="deck_hinge_barrel",
    )
    deck.visual(
        Box((0.090, 0.004, 0.038)),
        origin=Origin(xyz=(REAR_WHEEL_X + 0.022, 0.014, 0.060)),
        material=frame_silver,
        name="left_rear_stay",
    )
    deck.visual(
        Box((0.090, 0.004, 0.038)),
        origin=Origin(xyz=(REAR_WHEEL_X + 0.022, -0.014, 0.060)),
        material=frame_silver,
        name="right_rear_stay",
    )
    deck.inertial = Inertial.from_geometry(
        Box((0.52, 0.13, 0.08)),
        mass=2.8,
        origin=Origin(xyz=(0.03, 0.0, 0.056)),
    )

    stem = model.part("stem")
    stem.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_silver,
        name="stem_hinge_barrel",
    )
    stem.visual(
        Box((0.024, 0.024, 0.020)),
        origin=Origin(xyz=(0.012, 0.012, 0.020)),
        material=frame_silver,
        name="stem_hinge_web",
    )
    _add_rod(
        stem,
        start=(0.016, 0.012, 0.028),
        end=(CROWN_X_REL - 0.020, 0.0, CROWN_Z_REL),
        radius=0.014,
        material=frame_silver,
        name="stem_tube",
    )
    stem.visual(
        Box((0.040, 0.046, 0.040)),
        origin=Origin(xyz=(CROWN_X_REL - 0.020, 0.0, CROWN_Z_REL)),
        material=frame_silver,
        name="stem_crown_body",
    )
    stem.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(CROWN_X_REL - 0.012, 0.0, CROWN_Z_REL), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_silver,
        name="stem_tilt_barrel",
    )
    stem.inertial = Inertial.from_geometry(
        Box((0.10, 0.06, 0.22)),
        mass=0.8,
        origin=Origin(xyz=(0.028, 0.0, 0.095)),
    )

    fork = model.part("fork")
    fork.visual(
        Box((0.040, 0.050, 0.040)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=frame_silver,
        name="fork_crown_body",
    )
    fork.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_silver,
        name="fork_tilt_barrel",
    )
    fork.visual(
        Cylinder(radius=0.018, length=0.560),
        origin=Origin(xyz=(0.020, 0.0, 0.300)),
        material=frame_silver,
        name="steerer_tube",
    )
    fork.visual(
        Cylinder(radius=0.012, length=0.360),
        origin=Origin(xyz=(0.020, 0.0, 0.580), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_silver,
        name="handlebar",
    )
    fork.visual(
        Cylinder(radius=0.016, length=0.080),
        origin=Origin(xyz=(0.020, 0.220, 0.580), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=deck_black,
        name="left_grip",
    )
    fork.visual(
        Cylinder(radius=0.016, length=0.080),
        origin=Origin(xyz=(0.020, -0.220, 0.580), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=deck_black,
        name="right_grip",
    )
    _add_rod(
        fork,
        start=(0.024, 0.014, -0.004),
        end=(FRONT_WHEEL_X - 0.004, 0.089, FRONT_WHEEL_Z + 0.004),
        radius=0.006,
        material=frame_silver,
        name="left_fork_arm",
    )
    _add_rod(
        fork,
        start=(0.024, -0.014, -0.004),
        end=(FRONT_WHEEL_X - 0.004, -0.089, FRONT_WHEEL_Z + 0.004),
        radius=0.006,
        material=frame_silver,
        name="right_fork_arm",
    )
    fork.visual(
        Box((0.020, 0.014, 0.034)),
        origin=Origin(xyz=(FRONT_WHEEL_X, 0.096, FRONT_WHEEL_Z)),
        material=frame_silver,
        name="left_wheel_bracket",
    )
    fork.visual(
        Box((0.020, 0.014, 0.034)),
        origin=Origin(xyz=(FRONT_WHEEL_X, -0.096, FRONT_WHEEL_Z)),
        material=frame_silver,
        name="right_wheel_bracket",
    )
    fork.inertial = Inertial.from_geometry(
        Box((0.18, 0.52, 0.72)),
        mass=1.4,
        origin=Origin(xyz=(0.025, 0.0, 0.285)),
    )

    front_left_wheel = model.part("front_left_wheel")
    _add_wheel_visuals(
        front_left_wheel,
        radius=FRONT_WHEEL_RADIUS,
        width=FRONT_WHEEL_WIDTH,
        rubber=wheel_rubber,
        rim_metal=rim_metal,
    )
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=FRONT_WHEEL_RADIUS, length=FRONT_WHEEL_WIDTH),
        mass=0.26,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    front_right_wheel = model.part("front_right_wheel")
    _add_wheel_visuals(
        front_right_wheel,
        radius=FRONT_WHEEL_RADIUS,
        width=FRONT_WHEEL_WIDTH,
        rubber=wheel_rubber,
        rim_metal=rim_metal,
    )
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=FRONT_WHEEL_RADIUS, length=FRONT_WHEEL_WIDTH),
        mass=0.26,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    rear_wheel = model.part("rear_wheel")
    _add_wheel_visuals(
        rear_wheel,
        radius=REAR_WHEEL_RADIUS,
        width=REAR_WHEEL_WIDTH,
        rubber=wheel_rubber,
        rim_metal=rim_metal,
    )
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=REAR_WHEEL_RADIUS, length=REAR_WHEEL_WIDTH),
        mass=0.22,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    fold_joint = model.articulation(
        "deck_to_stem_fold",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=stem,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=FOLD_UPPER,
        ),
    )
    tilt_joint = model.articulation(
        "stem_to_fork_tilt",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=fork,
        origin=Origin(xyz=(CROWN_X_REL, 0.0, CROWN_Z_REL)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-TILT_LIMIT,
            upper=TILT_LIMIT,
        ),
    )
    model.articulation(
        "fork_to_front_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=front_left_wheel,
        origin=Origin(xyz=(FRONT_WHEEL_X, FRONT_TRACK_HALF, FRONT_WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
    )
    model.articulation(
        "fork_to_front_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=front_right_wheel,
        origin=Origin(xyz=(FRONT_WHEEL_X, -FRONT_TRACK_HALF, FRONT_WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
    )
    model.articulation(
        "deck_to_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(REAR_WHEEL_X, 0.0, REAR_WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
    )

    assert fold_joint.motion_limits is not None
    assert tilt_joint.motion_limits is not None
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    stem = object_model.get_part("stem")
    fork = object_model.get_part("fork")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    fold_joint = object_model.get_articulation("deck_to_stem_fold")
    tilt_joint = object_model.get_articulation("stem_to_fork_tilt")
    front_left_spin = object_model.get_articulation("fork_to_front_left_wheel_spin")
    front_right_spin = object_model.get_articulation("fork_to_front_right_wheel_spin")
    rear_spin = object_model.get_articulation("deck_to_rear_wheel_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        stem,
        deck,
        elem_a="stem_hinge_barrel",
        elem_b="deck_hinge_barrel",
        contact_tol=0.0005,
        name="fold_hinge_surfaces_touch",
    )
    ctx.expect_contact(
        fork,
        stem,
        elem_a="fork_tilt_barrel",
        elem_b="stem_tilt_barrel",
        contact_tol=0.0005,
        name="tilt_joint_surfaces_touch",
    )
    ctx.expect_contact(
        front_left_wheel,
        fork,
        elem_a="tire",
        elem_b="left_wheel_bracket",
        contact_tol=0.0005,
        name="front_left_wheel_mounted",
    )
    ctx.expect_contact(
        front_right_wheel,
        fork,
        elem_a="tire",
        elem_b="right_wheel_bracket",
        contact_tol=0.0005,
        name="front_right_wheel_mounted",
    )
    ctx.expect_contact(
        rear_wheel,
        deck,
        elem_a="tire",
        elem_b="left_rear_stay",
        contact_tol=0.0005,
        name="rear_wheel_left_stay_contact",
    )
    ctx.expect_contact(
        rear_wheel,
        deck,
        elem_a="tire",
        elem_b="right_rear_stay",
        contact_tol=0.0005,
        name="rear_wheel_right_stay_contact",
    )

    ctx.expect_gap(
        deck,
        rear_wheel,
        axis="x",
        positive_elem="deck_shell",
        negative_elem="tire",
        min_gap=0.01,
        max_gap=0.04,
        name="rear_wheel_behind_deck_tail",
    )
    ctx.expect_overlap(front_left_wheel, fork, axes="xz", min_overlap=0.01, name="left_wheel_under_fork")
    ctx.expect_overlap(front_right_wheel, fork, axes="xz", min_overlap=0.01, name="right_wheel_under_fork")

    ctx.check(
        "fold_axis_is_lateral",
        tuple(fold_joint.axis) == (0.0, 1.0, 0.0),
        f"expected fold axis (0, 1, 0), got {fold_joint.axis}",
    )
    ctx.check(
        "tilt_axis_is_longitudinal",
        tuple(tilt_joint.axis) == (1.0, 0.0, 0.0),
        f"expected tilt axis (1, 0, 0), got {tilt_joint.axis}",
    )
    ctx.check(
        "wheel_axes_are_transverse",
        tuple(front_left_spin.axis) == (0.0, 1.0, 0.0)
        and tuple(front_right_spin.axis) == (0.0, 1.0, 0.0)
        and tuple(rear_spin.axis) == (0.0, 1.0, 0.0),
        "wheel spin joints should rotate about the axle line",
    )

    deck_aabb = ctx.part_world_aabb(deck)
    fork_aabb = ctx.part_world_aabb(fork)
    fl_pos = ctx.part_world_position(front_left_wheel)
    fr_pos = ctx.part_world_position(front_right_wheel)
    rear_pos = ctx.part_world_position(rear_wheel)
    assert deck_aabb is not None
    assert fork_aabb is not None
    assert fl_pos is not None
    assert fr_pos is not None
    assert rear_pos is not None

    ctx.check(
        "front_track_is_wide",
        abs(fl_pos[1] - fr_pos[1]) > DECK_WIDTH + 0.07,
        "front wheel track should be clearly wider than the deck",
    )
    ctx.check(
        "rear_wheel_centered",
        abs(rear_pos[1]) < 0.002,
        f"rear wheel should be centered, got y={rear_pos[1]:.4f}",
    )
    ctx.check(
        "fork_stands_ahead_of_deck_front",
        fork_aabb[0][0] > deck_aabb[1][0] - 0.01,
        "front fork should rise from the deck nose, not from the middle",
    )

    fold_limits = fold_joint.motion_limits
    tilt_limits = tilt_joint.motion_limits
    assert fold_limits is not None and fold_limits.lower is not None and fold_limits.upper is not None
    assert tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None

    with ctx.pose({fold_joint: fold_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="fold_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="fold_lower_no_floating")

    with ctx.pose({fold_joint: fold_limits.upper}):
        folded_fork_aabb = ctx.part_world_aabb(fork)
        assert folded_fork_aabb is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="fold_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="fold_upper_no_floating")
        ctx.check(
            "stem_folds_forward",
            folded_fork_aabb[1][0] > fork_aabb[1][0] + 0.25
            and folded_fork_aabb[1][2] < fork_aabb[1][2] - 0.18,
            "folded fork and handlebar assembly should move forward and lower",
        )

    with ctx.pose({tilt_joint: tilt_limits.lower}):
        left_low = ctx.part_world_position(front_left_wheel)
        right_low = ctx.part_world_position(front_right_wheel)
        assert left_low is not None
        assert right_low is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="tilt_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="tilt_lower_no_floating")
        ctx.check(
            "negative_tilt_lowers_left_wheel",
            left_low[2] < fl_pos[2] - 0.01 and right_low[2] > fr_pos[2] + 0.01,
            "negative tilt should drop the left wheel and lift the right wheel",
        )

    with ctx.pose({tilt_joint: tilt_limits.upper}):
        left_high = ctx.part_world_position(front_left_wheel)
        right_high = ctx.part_world_position(front_right_wheel)
        assert left_high is not None
        assert right_high is not None
        ctx.fail_if_parts_overlap_in_current_pose(name="tilt_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="tilt_upper_no_floating")
        ctx.check(
            "positive_tilt_lowers_right_wheel",
            left_high[2] > fl_pos[2] + 0.01 and right_high[2] < fr_pos[2] - 0.01,
            "positive tilt should lift the left wheel and drop the right wheel",
        )

    with ctx.pose({front_left_spin: 1.2, front_right_spin: -0.9, rear_spin: 1.8}):
        ctx.fail_if_parts_overlap_in_current_pose(name="wheel_spin_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="wheel_spin_pose_no_floating")
        ctx.expect_contact(
            front_left_wheel,
            fork,
            elem_a="tire",
            elem_b="left_wheel_bracket",
            contact_tol=0.0005,
            name="left_wheel_stays_mounted_while_spinning",
        )
        ctx.expect_contact(
            front_right_wheel,
            fork,
            elem_a="tire",
            elem_b="right_wheel_bracket",
            contact_tol=0.0005,
            name="right_wheel_stays_mounted_while_spinning",
        )
        ctx.expect_contact(
            rear_wheel,
            deck,
            elem_a="tire",
            elem_b="left_rear_stay",
            contact_tol=0.0005,
            name="rear_wheel_stays_mounted_while_spinning",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

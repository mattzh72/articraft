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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_box(
    part,
    size,
    *,
    xyz=(0.0, 0.0, 0.0),
    rpy=(0.0, 0.0, 0.0),
    material=None,
    name=None,
):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(
    part,
    radius,
    length,
    *,
    xyz=(0.0, 0.0, 0.0),
    rpy=(0.0, 0.0, 0.0),
    material=None,
    name=None,
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="knee_scooter")

    frame_metal = model.material("frame_metal", rgba=(0.58, 0.11, 0.12, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.18, 0.20, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    silver = model.material("silver", rgba=(0.72, 0.74, 0.78, 1.0))
    pad_black = model.material("pad_black", rgba=(0.11, 0.11, 0.11, 1.0))

    wheel_radius = 0.125
    wheel_width = 0.038
    rear_wheel_x = -0.24
    rear_support_y = 0.11
    rear_wheel_y = 0.185
    steering_axis_x = 0.25
    steering_axis_z = 0.35
    deck_hinge_x = -0.04
    deck_hinge_z = 0.444

    frame = model.part("frame")

    rail_dx = 0.23 - rear_wheel_x
    rail_dz = 0.27 - 0.18
    rail_len = math.hypot(rail_dx, rail_dz)
    rail_pitch = math.atan2(rail_dx, rail_dz)
    for side, sign in (("left", 1.0), ("right", -1.0)):
        y = sign * rear_support_y
        _add_cylinder(
            frame,
            radius=0.018,
            length=rail_len,
            xyz=((rear_wheel_x + 0.23) / 2.0, y, (0.18 + 0.27) / 2.0),
            rpy=(0.0, rail_pitch, 0.0),
            material=frame_metal,
            name=f"{side}_lower_rail",
        )
        _add_box(
            frame,
            (0.06, 0.11, 0.11),
            xyz=(rear_wheel_x, y, 0.145),
            material=dark_metal,
            name=f"{side}_rear_support",
        )
        for post_x, post_name in ((-0.02, "rear"), (0.10, "front")):
            _add_cylinder(
                frame,
                radius=0.012,
                length=0.22,
                xyz=(post_x, y, 0.33),
                material=frame_metal,
                name=f"{side}_{post_name}_post",
            )
        _add_cylinder(
            frame,
            radius=0.013,
            length=0.16,
            xyz=(0.04, y, 0.43),
            rpy=(0.0, math.pi / 2.0, 0.0),
            material=frame_metal,
            name=f"{side}_upper_support",
        )

    _add_cylinder(
        frame,
        radius=0.014,
        length=0.22,
        xyz=(-0.225, 0.0, 0.21),
        rpy=(math.pi / 2.0, 0.0, 0.0),
        material=dark_metal,
        name="rear_crossmember",
    )
    _add_cylinder(
        frame,
        radius=0.014,
        length=0.22,
        xyz=(deck_hinge_x, 0.0, 0.43),
        rpy=(math.pi / 2.0, 0.0, 0.0),
        material=dark_metal,
        name="hinge_crossmember",
    )
    _add_cylinder(
        frame,
        radius=0.014,
        length=0.22,
        xyz=(0.214, 0.0, 0.27),
        rpy=(math.pi / 2.0, 0.0, 0.0),
        material=dark_metal,
        name="front_crossmember",
    )
    _add_box(
        frame,
        (0.05, 0.02, 0.15),
        xyz=(0.23, 0.075, 0.35),
        material=dark_metal,
        name="left_head_cheek",
    )
    _add_box(
        frame,
        (0.05, 0.02, 0.15),
        xyz=(0.23, -0.075, 0.35),
        material=dark_metal,
        name="right_head_cheek",
    )
    _add_box(
        frame,
        (0.03, 0.13, 0.055),
        xyz=(0.195, 0.0, 0.3025),
        material=dark_metal,
        name="head_rear_bridge",
    )
    _add_box(
        frame,
        (0.04, 0.09, 0.08),
        xyz=(0.214, 0.0, 0.31),
        material=dark_metal,
        name="head_support_block",
    )
    _add_box(
        frame,
        (0.05, 0.24, 0.03),
        xyz=(0.11, 0.0, 0.24),
        material=dark_metal,
        name="mid_tie_plate",
    )

    knee_deck = model.part("knee_deck")
    _add_box(
        knee_deck,
        (0.26, 0.24, 0.02),
        xyz=(0.13, 0.0, 0.01),
        material=dark_metal,
        name="deck_base",
    )
    _add_box(
        knee_deck,
        (0.24, 0.22, 0.05),
        xyz=(0.13, 0.0, 0.045),
        material=pad_black,
        name="deck_pad",
    )
    _add_box(
        knee_deck,
        (0.03, 0.24, 0.03),
        xyz=(0.015, 0.0, 0.015),
        material=silver,
        name="hinge_leaf",
    )

    steering_fork = model.part("steering_fork")
    _add_cylinder(
        steering_fork,
        radius=0.016,
        length=0.22,
        xyz=(0.0, 0.0, 0.0),
        material=dark_metal,
        name="lower_steerer",
    )
    _add_cylinder(
        steering_fork,
        radius=0.018,
        length=0.62,
        xyz=(0.0, 0.0, 0.31),
        material=dark_metal,
        name="upper_mast",
    )
    _add_box(
        steering_fork,
        (0.06, 0.045, 0.05),
        xyz=(0.03, 0.0, -0.055),
        material=dark_metal,
        name="crown_bridge",
    )
    _add_box(
        steering_fork,
        (0.05, 0.09, 0.03),
        xyz=(0.055, 0.0, -0.075),
        material=dark_metal,
        name="fork_crown",
    )
    for side, sign in (("left", 1.0), ("right", -1.0)):
        _add_cylinder(
            steering_fork,
            radius=0.012,
            length=0.16,
            xyz=(0.07, sign * 0.039, -0.150),
            material=dark_metal,
            name=f"{side}_fork_blade",
        )
        _add_box(
            steering_fork,
            (0.02, 0.03, 0.025),
            xyz=(0.07, sign * 0.039, -0.220),
            material=silver,
            name=f"{side}_dropout",
        )
    _add_cylinder(
        steering_fork,
        radius=0.014,
        length=0.46,
        xyz=(0.0, 0.0, 0.60),
        rpy=(math.pi / 2.0, 0.0, 0.0),
        material=silver,
        name="handlebar",
    )
    _add_box(
        steering_fork,
        (0.05, 0.08, 0.04),
        xyz=(0.0, 0.0, 0.60),
        material=silver,
        name="bar_clamp",
    )
    _add_cylinder(
        steering_fork,
        radius=0.017,
        length=0.09,
        xyz=(0.0, 0.185, 0.60),
        rpy=(math.pi / 2.0, 0.0, 0.0),
        material=rubber,
        name="left_grip",
    )
    _add_cylinder(
        steering_fork,
        radius=0.017,
        length=0.09,
        xyz=(0.0, -0.185, 0.60),
        rpy=(math.pi / 2.0, 0.0, 0.0),
        material=rubber,
        name="right_grip",
    )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_right_wheel = model.part("rear_right_wheel")
    front_wheel = model.part("front_wheel")

    for wheel_part, prefix in (
        (rear_left_wheel, "rear_left"),
        (rear_right_wheel, "rear_right"),
        (front_wheel, "front"),
    ):
        _add_cylinder(
            wheel_part,
            radius=wheel_radius,
            length=wheel_width,
            rpy=(math.pi / 2.0, 0.0, 0.0),
            material=rubber,
            name=f"{prefix}_tire",
        )
        _add_cylinder(
            wheel_part,
            radius=0.068,
            length=wheel_width + 0.004,
            rpy=(math.pi / 2.0, 0.0, 0.0),
            material=silver,
            name=f"{prefix}_hub",
        )
        _add_cylinder(
            wheel_part,
            radius=0.022,
            length=wheel_width + 0.010,
            rpy=(math.pi / 2.0, 0.0, 0.0),
            material=dark_metal,
            name=f"{prefix}_axle_cap",
        )

    model.articulation(
        "frame_to_knee_deck",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=knee_deck,
        origin=Origin(xyz=(deck_hinge_x, 0.0, deck_hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "frame_to_steering_fork",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=steering_fork,
        origin=Origin(xyz=(steering_axis_x, 0.0, steering_axis_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=-1.0,
            upper=1.0,
        ),
    )
    model.articulation(
        "frame_to_rear_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(rear_wheel_x, rear_wheel_y, wheel_radius)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=25.0),
    )
    model.articulation(
        "frame_to_rear_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(rear_wheel_x, -rear_wheel_y, wheel_radius)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=25.0),
    )
    model.articulation(
        "fork_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=steering_fork,
        child=front_wheel,
        origin=Origin(xyz=(0.07, 0.0, -0.225)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    knee_deck = object_model.get_part("knee_deck")
    steering_fork = object_model.get_part("steering_fork")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_wheel = object_model.get_part("front_wheel")

    deck_hinge = object_model.get_articulation("frame_to_knee_deck")
    steering = object_model.get_articulation("frame_to_steering_fork")
    rear_left_spin = object_model.get_articulation("frame_to_rear_left_wheel")
    rear_right_spin = object_model.get_articulation("frame_to_rear_right_wheel")
    front_spin = object_model.get_articulation("fork_to_front_wheel")

    for part_name in (
        "frame",
        "knee_deck",
        "steering_fork",
        "rear_left_wheel",
        "rear_right_wheel",
        "front_wheel",
    ):
        ctx.check(
            f"{part_name} exists",
            object_model.get_part(part_name) is not None,
            details=f"missing part {part_name}",
        )

    ctx.check(
        "deck hinge is transverse",
        tuple(deck_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={deck_hinge.axis}",
    )
    ctx.check(
        "steering axis is vertical",
        tuple(steering.axis) == (0.0, 0.0, 1.0),
        details=f"axis={steering.axis}",
    )
    for joint_name, joint_obj in (
        ("rear left wheel spin", rear_left_spin),
        ("rear right wheel spin", rear_right_spin),
        ("front wheel spin", front_spin),
    ):
        ctx.check(
            f"{joint_name} uses axle axis",
            tuple(joint_obj.axis) == (0.0, 1.0, 0.0),
            details=f"axis={joint_obj.axis}",
        )
        ctx.check(
            f"{joint_name} is continuous",
            joint_obj.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint_obj.articulation_type}",
        )

    with ctx.pose({deck_hinge: 0.0}):
        ctx.expect_gap(
            knee_deck,
            frame,
            axis="z",
            min_gap=0.0,
            max_gap=0.04,
            name="closed deck sits just above frame rails",
        )
        ctx.expect_overlap(
            knee_deck,
            frame,
            axes="xy",
            min_overlap=0.14,
            name="deck remains centered over the frame",
        )

    closed_pad_aabb = ctx.part_element_world_aabb(knee_deck, elem="deck_pad")
    with ctx.pose({deck_hinge: 1.1}):
        open_pad_aabb = ctx.part_element_world_aabb(knee_deck, elem="deck_pad")
    ctx.check(
        "deck folds upward for storage",
        closed_pad_aabb is not None
        and open_pad_aabb is not None
        and open_pad_aabb[1][2] > closed_pad_aabb[1][2] + 0.12,
        details=f"closed={closed_pad_aabb}, open={open_pad_aabb}",
    )

    front_rest = ctx.part_world_position(front_wheel)
    with ctx.pose({steering: 0.65}):
        front_turned = ctx.part_world_position(front_wheel)
    ctx.check(
        "steering swings front wheel sideways",
        front_rest is not None
        and front_turned is not None
        and front_turned[1] > front_rest[1] + 0.03,
        details=f"rest={front_rest}, turned={front_turned}",
    )

    ctx.expect_origin_distance(
        rear_left_wheel,
        rear_right_wheel,
        axes="y",
        min_dist=0.34,
        max_dist=0.40,
        name="rear wheel track matches a stable scooter stance",
    )
    ctx.expect_origin_distance(
        front_wheel,
        rear_left_wheel,
        axes="x",
        min_dist=0.52,
        max_dist=0.65,
        name="wheelbase matches a knee scooter footprint",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

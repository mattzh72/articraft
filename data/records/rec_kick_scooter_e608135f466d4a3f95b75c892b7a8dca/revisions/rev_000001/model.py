from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi, sqrt

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


DECK_TOP_LENGTH = 0.46
DECK_WIDTH = 0.12
DECK_THICKNESS = 0.018
DECK_CENTER_X = 0.04

WHEEL_RADIUS = 0.09
WHEEL_WIDTH = 0.024
HUB_RADIUS = 0.038
AXLE_CAP_RADIUS = 0.010
AXLE_CAP_LENGTH = 0.010
AXLE_CAP_OFFSET_Y = 0.017

STEER_X = 0.285
STEER_Z = 0.085
FRONT_AXLE_OFFSET_X = 0.125
FRONT_AXLE_OFFSET_Z = -0.040

REAR_AXLE_X = -0.30
REAR_AXLE_Z = 0.045

CARRY_HINGE_X = -0.198
CARRY_HINGE_Z = 0.0


def _add_wheel(part, *, tire_material, hub_material) -> None:
    part.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=tire_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=HUB_RADIUS, length=WHEEL_WIDTH + 0.004),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=AXLE_CAP_RADIUS, length=AXLE_CAP_LENGTH),
        origin=Origin(
            xyz=(0.0, AXLE_CAP_OFFSET_Y, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=hub_material,
        name="left_cap",
    )
    part.visual(
        Cylinder(radius=AXLE_CAP_RADIUS, length=AXLE_CAP_LENGTH),
        origin=Origin(
            xyz=(0.0, -AXLE_CAP_OFFSET_Y, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=hub_material,
        name="right_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kick_scooter")

    frame_black = model.material("frame_black", rgba=(0.12, 0.13, 0.15, 1.0))
    satin_gray = model.material("satin_gray", rgba=(0.63, 0.66, 0.70, 1.0))
    grip_black = model.material("grip_black", rgba=(0.06, 0.06, 0.06, 1.0))
    tire_black = model.material("tire_black", rgba=(0.04, 0.04, 0.04, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.74, 0.76, 0.78, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((DECK_TOP_LENGTH, DECK_WIDTH, DECK_THICKNESS)),
        origin=Origin(xyz=(DECK_CENTER_X, 0.0, -DECK_THICKNESS / 2.0)),
        material=frame_black,
        name="deck_shell",
    )
    deck.visual(
        Box((0.30, 0.09, 0.018)),
        origin=Origin(xyz=(0.01, 0.0, -0.026)),
        material=satin_gray,
        name="belly_pan",
    )
    deck.visual(
        Box((0.26, 0.088, 0.002)),
        origin=Origin(xyz=(0.06, 0.0, 0.001)),
        material=grip_black,
        name="grip_pad",
    )

    neck_angle = atan2(0.077, 0.075)
    for side_name, side_y in (("left", 0.034), ("right", -0.034)):
        deck.visual(
            Cylinder(radius=0.012, length=0.11),
            origin=Origin(
                xyz=(0.2435, side_y, 0.0375),
                rpy=(0.0, neck_angle, 0.0),
            ),
            material=frame_black,
            name=f"{side_name}_neck_strut",
        )
        deck.visual(
            Box((0.018, 0.012, 0.056)),
            origin=Origin(xyz=(0.262, side_y, 0.053)),
            material=frame_black,
            name=f"{side_name}_steer_ear",
        )

    deck.visual(
        Cylinder(radius=0.020, length=0.085),
        origin=Origin(xyz=(STEER_X, 0.0, 0.0425)),
        material=frame_black,
        name="head_tube",
    )

    for side_name, side_y in (("left", 0.038), ("right", -0.038)):
        deck.visual(
            Box((0.15, 0.014, 0.012)),
            origin=Origin(xyz=(-0.255, side_y, -0.006)),
            material=frame_black,
            name=f"{side_name}_tail_rail",
        )

    deck.visual(
        Box((0.07, 0.020, 0.070)),
        origin=Origin(xyz=(-0.300, 0.032, 0.020)),
        material=frame_black,
        name="left_rear_plate",
    )
    deck.visual(
        Box((0.07, 0.020, 0.070)),
        origin=Origin(xyz=(-0.300, -0.032, 0.020)),
        material=frame_black,
        name="right_rear_plate",
    )

    deck.visual(
        Box((0.020, 0.050, 0.010)),
        origin=Origin(xyz=(-0.197, 0.0, -0.005)),
        material=frame_black,
        name="tail_pad",
    )

    front_fork = model.part("front_fork")
    front_fork.visual(
        Cylinder(radius=0.019, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=satin_gray,
        name="stem_tube",
    )
    front_fork.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=frame_black,
        name="headset_collar",
    )
    front_fork.visual(
        Box((0.028, 0.074, 0.022)),
        origin=Origin(xyz=(0.008, 0.0, 0.068)),
        material=frame_black,
        name="fork_yoke",
    )

    blade_top_x = 0.010
    blade_top_z = 0.064
    blade_dx = FRONT_AXLE_OFFSET_X - blade_top_x
    blade_dz = FRONT_AXLE_OFFSET_Z - blade_top_z
    blade_length = sqrt(blade_dx * blade_dx + blade_dz * blade_dz)
    blade_angle = atan2(blade_dx, blade_dz)
    front_fork.visual(
        Cylinder(radius=0.0085, length=blade_length),
        origin=Origin(
            xyz=((FRONT_AXLE_OFFSET_X + blade_top_x) / 2.0, 0.032, (FRONT_AXLE_OFFSET_Z + blade_top_z) / 2.0),
            rpy=(0.0, blade_angle, 0.0),
        ),
        material=frame_black,
        name="left_blade",
    )
    front_fork.visual(
        Box((0.020, 0.020, 0.050)),
        origin=Origin(xyz=(FRONT_AXLE_OFFSET_X, 0.032, FRONT_AXLE_OFFSET_Z)),
        material=frame_black,
        name="left_dropout",
    )
    front_fork.visual(
        Cylinder(radius=0.0085, length=blade_length),
        origin=Origin(
            xyz=((FRONT_AXLE_OFFSET_X + blade_top_x) / 2.0, -0.032, (FRONT_AXLE_OFFSET_Z + blade_top_z) / 2.0),
            rpy=(0.0, blade_angle, 0.0),
        ),
        material=frame_black,
        name="right_blade",
    )
    front_fork.visual(
        Box((0.020, 0.020, 0.050)),
        origin=Origin(xyz=(FRONT_AXLE_OFFSET_X, -0.032, FRONT_AXLE_OFFSET_Z)),
        material=frame_black,
        name="right_dropout",
    )

    front_fork.visual(
        Cylinder(radius=0.013, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 0.57), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="handlebar",
    )

    front_wheel = model.part("front_wheel")
    _add_wheel(front_wheel, tire_material=tire_black, hub_material=hub_gray)

    rear_wheel = model.part("rear_wheel")
    _add_wheel(rear_wheel, tire_material=tire_black, hub_material=hub_gray)

    carry_hoop = model.part("carry_hoop")
    carry_hoop.visual(
        Box((0.018, 0.050, 0.012)),
        origin=Origin(xyz=(0.009, 0.0, 0.006)),
        material=satin_gray,
        name="hinge_leaf",
    )
    for side_name, side_y in (("left", 0.020), ("right", -0.020)):
        carry_hoop.visual(
            Cylinder(radius=0.006, length=0.100),
            origin=Origin(
                xyz=(0.058, side_y, 0.006),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=satin_gray,
            name=f"{side_name}_arm",
        )
    carry_hoop.visual(
        Cylinder(radius=0.006, length=0.052),
        origin=Origin(xyz=(0.108, 0.0, 0.006), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_gray,
        name="crossbar",
    )

    model.articulation(
        "deck_to_front_fork",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_fork,
        origin=Origin(xyz=(STEER_X, 0.0, STEER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=3.5,
            lower=-0.65,
            upper=0.65,
        ),
    )
    model.articulation(
        "front_fork_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_wheel,
        origin=Origin(xyz=(FRONT_AXLE_OFFSET_X, 0.0, FRONT_AXLE_OFFSET_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=24.0),
    )
    model.articulation(
        "deck_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(REAR_AXLE_X, 0.0, REAR_AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=24.0),
    )
    model.articulation(
        "deck_to_carry_hoop",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=carry_hoop,
        origin=Origin(xyz=(CARRY_HINGE_X, 0.0, CARRY_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    front_fork = object_model.get_part("front_fork")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    carry_hoop = object_model.get_part("carry_hoop")

    steering = object_model.get_articulation("deck_to_front_fork")
    front_spin = object_model.get_articulation("front_fork_to_front_wheel")
    rear_spin = object_model.get_articulation("deck_to_rear_wheel")
    carry_hinge = object_model.get_articulation("deck_to_carry_hoop")

    ctx.check(
        "steering uses a vertical yaw axis",
        steering.articulation_type == ArticulationType.REVOLUTE
        and steering.axis == (0.0, 0.0, 1.0),
        details=f"type={steering.articulation_type}, axis={steering.axis}",
    )
    ctx.check(
        "wheels spin on lateral axle axes",
        front_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_spin.articulation_type == ArticulationType.CONTINUOUS
        and front_spin.axis == (0.0, 1.0, 0.0)
        and rear_spin.axis == (0.0, 1.0, 0.0),
        details=(
            f"front={front_spin.articulation_type}/{front_spin.axis}, "
            f"rear={rear_spin.articulation_type}/{rear_spin.axis}"
        ),
    )
    ctx.check(
        "carry hoop folds on the rear hinge line",
        carry_hinge.articulation_type == ArticulationType.REVOLUTE
        and carry_hinge.axis == (0.0, -1.0, 0.0),
        details=f"type={carry_hinge.articulation_type}, axis={carry_hinge.axis}",
    )

    with ctx.pose({steering: 0.0, carry_hinge: 0.0}):
        ctx.expect_gap(
            front_wheel,
            deck,
            axis="x",
            min_gap=0.015,
            negative_elem="deck_shell",
            name="front wheel stays ahead of the deck",
        )
        ctx.expect_gap(
            deck,
            rear_wheel,
            axis="x",
            min_gap=0.010,
            positive_elem="deck_shell",
            name="rear wheel sits behind the deck tail",
        )
        ctx.expect_contact(
            front_wheel,
            front_fork,
            elem_a="left_cap",
            elem_b="left_dropout",
            contact_tol=0.001,
            name="front axle cap meets the fork dropout",
        )
        ctx.expect_contact(
            rear_wheel,
            deck,
            elem_a="left_cap",
            elem_b="left_rear_plate",
            contact_tol=0.001,
            name="rear axle cap meets the rear plate",
        )
        ctx.expect_gap(
            carry_hoop,
            deck,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem="hinge_leaf",
            negative_elem="tail_pad",
            name="carry hoop rests flat on the tail pad when folded",
        )

        straight_front_pos = ctx.part_world_position(front_wheel)
        closed_crossbar = ctx.part_element_world_aabb(carry_hoop, elem="crossbar")

    with ctx.pose({steering: 0.55}):
        steered_front_pos = ctx.part_world_position(front_wheel)

    ctx.check(
        "positive steering swings the front wheel to the rider's left",
        straight_front_pos is not None
        and steered_front_pos is not None
        and steered_front_pos[1] > straight_front_pos[1] + 0.04,
        details=f"straight={straight_front_pos}, steered={steered_front_pos}",
    )

    with ctx.pose({carry_hinge: 1.15}):
        raised_crossbar = ctx.part_element_world_aabb(carry_hoop, elem="crossbar")

    ctx.check(
        "carry hoop rises clear of the deck when opened",
        closed_crossbar is not None
        and raised_crossbar is not None
        and raised_crossbar[1][2] > closed_crossbar[1][2] + 0.08,
        details=f"closed={closed_crossbar}, raised={raised_crossbar}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

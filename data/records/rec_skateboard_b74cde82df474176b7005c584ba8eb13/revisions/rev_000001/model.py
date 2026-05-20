import math
import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_deck_mesh():
    # A simple rounded rectangle board with a slight fillet
    return (
        cq.Workplane("XY")
        .box(0.8, 0.2, 0.015)
        .edges("|Z").fillet(0.09)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="skateboard")

    # --- Deck (Root Part) ---
    deck = model.part("deck")
    deck.visual(
        mesh_from_cadquery(build_deck_mesh(), "deck_board"),
        origin=Origin(xyz=(0.0, 0.0, 0.1)),
        name="board"
    )
    # Front baseplate
    deck.visual(
        Box((0.08, 0.06, 0.01)),
        origin=Origin(xyz=(0.25, 0.0, 0.0875)),
        name="front_baseplate"
    )
    # Rear baseplate
    deck.visual(
        Box((0.08, 0.06, 0.01)),
        origin=Origin(xyz=(-0.25, 0.0, 0.0875)),
        name="rear_baseplate"
    )

    # --- Trucks ---
    # Front Truck
    front_truck = model.part("front_truck")
    front_truck.visual(
        Cylinder(radius=0.004, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi/2, 0.0, 0.0)),
        name="front_axle"
    )
    front_truck.visual(
        Box((0.03, 0.04, 0.056)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        name="front_strut"
    )
    model.articulation(
        "front_truck_steer",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_truck,
        origin=Origin(xyz=(0.25, 0.0, 0.0275)),
        axis=(-0.7071, 0.0, 0.7071),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-0.3, upper=0.3),
    )

    # Rear Truck
    rear_truck = model.part("rear_truck")
    rear_truck.visual(
        Cylinder(radius=0.004, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi/2, 0.0, 0.0)),
        name="rear_axle"
    )
    rear_truck.visual(
        Box((0.03, 0.04, 0.056)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        name="rear_strut"
    )
    model.articulation(
        "rear_truck_steer",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_truck,
        origin=Origin(xyz=(-0.25, 0.0, 0.0275)),
        axis=(0.7071, 0.0, 0.7071),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-0.3, upper=0.3),
    )

    # --- Wheels ---
    wheel_radius = 0.0275
    wheel_width = 0.035
    wheel_y_offset = 0.16

    # Front Left Wheel
    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi/2, 0.0, 0.0)),
        name="mesh"
    )
    model.articulation(
        "front_left_spin",
        ArticulationType.CONTINUOUS,
        parent=front_truck,
        child=front_left_wheel,
        origin=Origin(xyz=(0.0, wheel_y_offset, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )

    # Front Right Wheel
    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi/2, 0.0, 0.0)),
        name="mesh"
    )
    model.articulation(
        "front_right_spin",
        ArticulationType.CONTINUOUS,
        parent=front_truck,
        child=front_right_wheel,
        origin=Origin(xyz=(0.0, -wheel_y_offset, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )

    # Rear Left Wheel
    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi/2, 0.0, 0.0)),
        name="mesh"
    )
    model.articulation(
        "rear_left_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_truck,
        child=rear_left_wheel,
        origin=Origin(xyz=(0.0, wheel_y_offset, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )

    # Rear Right Wheel
    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi/2, 0.0, 0.0)),
        name="mesh"
    )
    model.articulation(
        "rear_right_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_truck,
        child=rear_right_wheel,
        origin=Origin(xyz=(0.0, -wheel_y_offset, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    front_truck = object_model.get_part("front_truck")
    rear_truck = object_model.get_part("rear_truck")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    # Allowances
    ctx.allow_overlap(deck, front_truck, elem_a="front_baseplate", elem_b="front_strut", reason="Truck hanger pivots against baseplate.")
    ctx.allow_overlap(deck, rear_truck, elem_a="rear_baseplate", elem_b="rear_strut", reason="Truck hanger pivots against baseplate.")
    
    ctx.allow_overlap(front_truck, front_left_wheel, elem_a="front_axle", elem_b="mesh", reason="Wheel is mounted on the axle.")
    ctx.allow_overlap(front_truck, front_right_wheel, elem_a="front_axle", elem_b="mesh", reason="Wheel is mounted on the axle.")
    ctx.allow_overlap(rear_truck, rear_left_wheel, elem_a="rear_axle", elem_b="mesh", reason="Wheel is mounted on the axle.")
    ctx.allow_overlap(rear_truck, rear_right_wheel, elem_a="rear_axle", elem_b="mesh", reason="Wheel is mounted on the axle.")

    # Exact checks
    ctx.expect_gap(deck, front_truck, axis="z", positive_elem="front_baseplate", negative_elem="front_strut", max_penetration=0.01)
    ctx.expect_gap(deck, rear_truck, axis="z", positive_elem="rear_baseplate", negative_elem="rear_strut", max_penetration=0.01)

    # Wheels are mounted on the axles
    ctx.expect_within(front_truck, front_left_wheel, axes="xz", inner_elem="front_axle", outer_elem="mesh", margin=0.001)
    ctx.expect_within(front_truck, front_right_wheel, axes="xz", inner_elem="front_axle", outer_elem="mesh", margin=0.001)
    ctx.expect_within(rear_truck, rear_left_wheel, axes="xz", inner_elem="rear_axle", outer_elem="mesh", margin=0.001)
    ctx.expect_within(rear_truck, rear_right_wheel, axes="xz", inner_elem="rear_axle", outer_elem="mesh", margin=0.001)
    
    # Outboard stance check
    deck_aabb = ctx.part_world_aabb(deck)
    fl_wheel_aabb = ctx.part_world_aabb(front_left_wheel)
    fr_wheel_aabb = ctx.part_world_aabb(front_right_wheel)
    
    if deck_aabb and fl_wheel_aabb and fr_wheel_aabb:
        ctx.check(
            "left_wheels_outboard",
            fl_wheel_aabb[0][1] > deck_aabb[1][1] + 0.01,
            f"Left wheel min Y ({fl_wheel_aabb[0][1]:.3f}) should be strictly greater than deck max Y ({deck_aabb[1][1]:.3f})"
        )
        ctx.check(
            "right_wheels_outboard",
            fr_wheel_aabb[1][1] < deck_aabb[0][1] - 0.01,
            f"Right wheel max Y ({fr_wheel_aabb[1][1]:.3f}) should be strictly less than deck min Y ({deck_aabb[0][1]:.3f})"
        )

    return ctx.report()


object_model = build_object_model()

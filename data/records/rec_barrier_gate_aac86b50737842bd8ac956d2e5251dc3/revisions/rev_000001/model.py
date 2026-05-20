from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="security_road_wedge_barrier")

    asphalt = Material("dark asphalt", rgba=(0.045, 0.047, 0.045, 1.0))
    concrete = Material("cast concrete", rgba=(0.37, 0.37, 0.34, 1.0))
    galvanized = Material("galvanized steel", rgba=(0.48, 0.50, 0.50, 1.0))
    black_steel = Material("blackened steel", rgba=(0.035, 0.038, 0.040, 1.0))
    safety_yellow = Material("safety yellow", rgba=(1.0, 0.76, 0.03, 1.0))
    red_reflector = Material("red reflector", rgba=(0.82, 0.05, 0.03, 1.0))

    # World/object frame:
    # X = vehicle travel direction, from rear hinge toward the rising leading edge.
    # Y = lane width.  Z = up, with road grade at z=0.
    cassette = model.part("cassette")

    # A road cassette set into grade.  The four road strips leave a rectangular
    # pit opening around the wedge, with enough clearance for the plate and ribs.
    cassette.visual(
        Box((1.90, 0.32, 0.12)),
        origin=Origin(xyz=(0.58, -1.21, -0.06)),
        material=asphalt,
        name="road_side_0",
    )
    cassette.visual(
        Box((1.90, 0.32, 0.12)),
        origin=Origin(xyz=(0.58, 1.21, -0.06)),
        material=asphalt,
        name="road_side_1",
    )
    cassette.visual(
        Box((0.40, 2.74, 0.12)),
        origin=Origin(xyz=(-0.30, 0.0, -0.06)),
        material=asphalt,
        name="rear_road",
    )
    cassette.visual(
        Box((0.35, 2.74, 0.12)),
        origin=Origin(xyz=(1.425, 0.0, -0.06)),
        material=asphalt,
        name="front_road",
    )

    # Concrete pit liner and floor, slightly overlapping the road strips so the
    # cassette reads as one continuous cast-in-place assembly.
    cassette.visual(
        Box((1.35, 0.07, 0.30)),
        origin=Origin(xyz=(0.58, -1.085, -0.15)),
        material=concrete,
        name="pit_side_0",
    )
    cassette.visual(
        Box((1.35, 0.07, 0.30)),
        origin=Origin(xyz=(0.58, 1.085, -0.15)),
        material=concrete,
        name="pit_side_1",
    )
    cassette.visual(
        Box((0.07, 2.17, 0.30)),
        origin=Origin(xyz=(-0.105, 0.0, -0.15)),
        material=concrete,
        name="rear_wall",
    )
    cassette.visual(
        Box((0.07, 2.17, 0.30)),
        origin=Origin(xyz=(1.265, 0.0, -0.15)),
        material=concrete,
        name="front_wall",
    )
    cassette.visual(
        Box((1.38, 2.17, 0.08)),
        origin=Origin(xyz=(0.58, 0.0, -0.30)),
        material=concrete,
        name="pit_floor",
    )

    # Galvanized top frame / curb angle around the opening.
    cassette.visual(
        Box((1.38, 0.075, 0.035)),
        origin=Origin(xyz=(0.58, -1.075, -0.0175)),
        material=galvanized,
        name="steel_side_0",
    )
    cassette.visual(
        Box((1.38, 0.075, 0.035)),
        origin=Origin(xyz=(0.58, 1.075, -0.0175)),
        material=galvanized,
        name="steel_side_1",
    )
    cassette.visual(
        Box((0.075, 2.15, 0.035)),
        origin=Origin(xyz=(-0.105, 0.0, -0.0175)),
        material=galvanized,
        name="steel_rear",
    )
    cassette.visual(
        Box((0.075, 2.15, 0.035)),
        origin=Origin(xyz=(1.265, 0.0, -0.0175)),
        material=galvanized,
        name="steel_front",
    )

    # End bearing blocks outside the plate width support the hinge tube without
    # colliding with the moving rectangular plate.
    cassette.visual(
        Box((0.13, 0.075, 0.16)),
        origin=Origin(xyz=(0.0, -1.105, -0.08)),
        material=galvanized,
        name="hinge_cheek_0",
    )
    cassette.visual(
        Box((0.13, 0.075, 0.16)),
        origin=Origin(xyz=(0.0, 1.105, -0.08)),
        material=galvanized,
        name="hinge_cheek_1",
    )
    cassette.visual(
        Cylinder(radius=0.038, length=0.09),
        origin=Origin(xyz=(0.0, -1.020, -0.04), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="hinge_bearing_0",
    )
    cassette.visual(
        Cylinder(radius=0.038, length=0.09),
        origin=Origin(xyz=(0.0, 1.020, -0.04), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="hinge_bearing_1",
    )

    # Painted safety outline at grade around the recess.
    cassette.visual(
        Box((1.30, 0.045, 0.004)),
        origin=Origin(xyz=(0.58, -1.155, 0.001)),
        material=safety_yellow,
        name="caution_side_0",
    )
    cassette.visual(
        Box((1.30, 0.045, 0.004)),
        origin=Origin(xyz=(0.58, 1.155, 0.001)),
        material=safety_yellow,
        name="caution_side_1",
    )
    cassette.visual(
        Box((0.045, 2.25, 0.004)),
        origin=Origin(xyz=(-0.185, 0.0, 0.001)),
        material=safety_yellow,
        name="caution_rear",
    )
    cassette.visual(
        Box((0.045, 2.25, 0.004)),
        origin=Origin(xyz=(1.345, 0.0, 0.001)),
        material=safety_yellow,
        name="caution_front",
    )

    wedge_plate = model.part("wedge_plate")

    # The child frame is exactly on the rear hinge axis.  At q=0 the top of the
    # rectangular plate lies at z=0, flush with the roadway.
    wedge_plate.visual(
        Box((1.20, 1.90, 0.08)),
        origin=Origin(xyz=(0.60, 0.0, 0.0)),
        material=black_steel,
        name="deck_plate",
    )
    wedge_plate.visual(
        Cylinder(radius=0.030, length=1.95),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_steel,
        name="hinge_tube",
    )
    wedge_plate.visual(
        Box((1.04, 0.045, 0.12)),
        origin=Origin(xyz=(0.68, -0.72, -0.10)),
        material=black_steel,
        name="rib_0",
    )
    wedge_plate.visual(
        Box((1.04, 0.045, 0.12)),
        origin=Origin(xyz=(0.68, -0.24, -0.10)),
        material=black_steel,
        name="rib_1",
    )
    wedge_plate.visual(
        Box((1.04, 0.045, 0.12)),
        origin=Origin(xyz=(0.68, 0.24, -0.10)),
        material=black_steel,
        name="rib_2",
    )
    wedge_plate.visual(
        Box((1.04, 0.045, 0.12)),
        origin=Origin(xyz=(0.68, 0.72, -0.10)),
        material=black_steel,
        name="rib_3",
    )
    wedge_plate.visual(
        Box((0.05, 1.82, 0.16)),
        origin=Origin(xyz=(1.175, 0.0, -0.08)),
        material=black_steel,
        name="front_face",
    )
    wedge_plate.visual(
        Box((0.006, 1.62, 0.070)),
        origin=Origin(xyz=(1.203, 0.0, -0.045)),
        material=red_reflector,
        name="front_reflector",
    )

    # Low-profile paint/anti-skid bands on the flush plate.
    for index, x in enumerate((0.18, 0.42, 0.66, 0.90, 1.10)):
        wedge_plate.visual(
            Box((0.075, 1.72, 0.004)),
            origin=Origin(xyz=(x, 0.0, 0.041)),
            material=safety_yellow,
            name=f"yellow_band_{index}",
        )

    model.articulation(
        "wedge_hinge",
        ArticulationType.REVOLUTE,
        parent=cassette,
        child=wedge_plate,
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        # The closed plate extends along local +X from the rear hinge.  Rotating
        # about -Y raises the front edge out of the road cassette.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25000.0, velocity=0.55, lower=0.0, upper=1.05),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cassette = object_model.get_part("cassette")
    wedge_plate = object_model.get_part("wedge_plate")
    hinge = object_model.get_articulation("wedge_hinge")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_within(
            wedge_plate,
            cassette,
            axes="xy",
            inner_elem="deck_plate",
            outer_elem="pit_floor",
            margin=0.0,
            name="flush plate fits inside pit footprint",
        )
        ctx.expect_gap(
            wedge_plate,
            cassette,
            axis="z",
            positive_elem="deck_plate",
            negative_elem="pit_floor",
            min_gap=0.18,
            name="closed wedge clears pit floor",
        )

    with ctx.pose({hinge: 1.05}):
        front_aabb = ctx.part_element_world_aabb(wedge_plate, elem="front_face")
        ctx.check(
            "raised plate presents vehicle blocking face",
            front_aabb is not None and front_aabb[1][2] > 0.75,
            details=f"front_face_aabb={front_aabb}",
        )

    return ctx.report()


object_model = build_object_model()

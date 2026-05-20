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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_leaf_drawbridge")

    concrete = model.material("concrete", rgba=(0.63, 0.64, 0.62, 1.0))
    deck_steel = model.material("deck_steel", rgba=(0.37, 0.43, 0.47, 1.0))
    girder_steel = model.material("girder_steel", rgba=(0.29, 0.33, 0.36, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.16, 0.18, 0.20, 1.0))
    curb_paint = model.material("curb_paint", rgba=(0.78, 0.76, 0.55, 1.0))

    leaf_width = 7.4
    leaf_length = 10.6
    half_leaf_width = leaf_width * 0.5
    side_girder_width = 0.22
    axis_height = 2.32

    shore_frame = model.part("shore_frame")
    shore_frame.visual(
        Box((10.0, 4.1, 2.4)),
        origin=Origin(xyz=(0.0, -2.05, 1.2)),
        material=concrete,
        name="abutment",
    )
    shore_frame.visual(
        Box((8.2, 2.5, 0.18)),
        origin=Origin(xyz=(0.0, -1.25, axis_height + 0.17)),
        material=deck_steel,
        name="approach_deck",
    )
    shore_frame.visual(
        Box((8.6, 0.30, 0.32)),
        origin=Origin(xyz=(0.0, -0.15, axis_height + 0.04)),
        material=girder_steel,
        name="threshold_beam",
    )
    for sign, name in ((-1.0, "left_tower"), (1.0, "right_tower")):
        shore_frame.visual(
            Box((0.85, 1.40, 1.65)),
            origin=Origin(xyz=(sign * 4.375, -0.55, 3.225)),
            material=concrete,
            name=name,
        )
        shore_frame.visual(
            Cylinder(radius=0.42, length=0.44),
            origin=Origin(
                xyz=(sign * 3.98, 0.0, axis_height),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=bearing_steel,
            name=f"{name}_bearing",
        )
    shore_frame.visual(
        Box((10.0, 0.40, 0.50)),
        origin=Origin(xyz=(0.0, -0.82, 4.00)),
        material=girder_steel,
        name="rear_crosshead",
    )
    shore_frame.inertial = Inertial.from_geometry(
        Box((10.0, 4.8, 4.2)),
        mass=62000.0,
        origin=Origin(xyz=(0.0, -2.0, 2.1)),
    )

    leaf = model.part("bridge_leaf")
    leaf.visual(
        Box((leaf_width, leaf_length, 0.08)),
        origin=Origin(xyz=(0.0, leaf_length * 0.5, 0.22)),
        material=deck_steel,
        name="deck_skin",
    )
    for sign, name in ((-1.0, "left_girder"), (1.0, "right_girder")):
        leaf.visual(
            Box((side_girder_width, leaf_length, 0.56)),
            origin=Origin(
                xyz=(sign * (half_leaf_width - side_girder_width * 0.5), leaf_length * 0.5, -0.02)
            ),
            material=girder_steel,
            name=name,
        )
        leaf.visual(
            Box((0.16, leaf_length - 0.40, 0.10)),
            origin=Origin(
                xyz=(sign * (half_leaf_width - 0.40), leaf_length * 0.5 + 0.20, 0.31)
            ),
            material=curb_paint,
            name=f"{name}_curb",
        )
    leaf.visual(
        Box((leaf_width - 0.20, 0.34, 0.48)),
        origin=Origin(xyz=(0.0, 0.17, 0.0)),
        material=bearing_steel,
        name="hinge_beam",
    )
    leaf.visual(
        Box((leaf_width - 0.20, 0.28, 0.24)),
        origin=Origin(xyz=(0.0, leaf_length - 0.14, 0.06)),
        material=girder_steel,
        name="toe_beam",
    )
    for y_pos, rib_name in ((2.9, "inner_rib_a"), (6.3, "inner_rib_b")):
        leaf.visual(
            Box((leaf_width - 0.44, 0.24, 0.26)),
            origin=Origin(xyz=(0.0, y_pos, 0.01)),
            material=girder_steel,
            name=rib_name,
        )
    leaf.inertial = Inertial.from_geometry(
        Box((leaf_width, leaf_length, 0.70)),
        mass=18500.0,
        origin=Origin(xyz=(0.0, leaf_length * 0.48, 0.02)),
    )

    model.articulation(
        "shore_to_leaf",
        ArticulationType.REVOLUTE,
        parent=shore_frame,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, axis_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=320000.0,
            velocity=0.35,
            lower=0.0,
            upper=1.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shore_frame = object_model.get_part("shore_frame")
    leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("shore_to_leaf")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            leaf,
            shore_frame,
            axis="y",
            positive_elem="deck_skin",
            negative_elem="approach_deck",
            max_gap=0.01,
            max_penetration=0.0,
            name="leaf closes flush to the shore deck",
        )
        ctx.expect_overlap(
            leaf,
            shore_frame,
            axes="x",
            elem_a="deck_skin",
            elem_b="approach_deck",
            min_overlap=6.8,
            name="leaf deck aligns with the approach width",
        )

    closed_toe = ctx.part_element_world_aabb(leaf, elem="toe_beam")
    with ctx.pose({hinge: 1.10}):
        open_toe = ctx.part_element_world_aabb(leaf, elem="toe_beam")

    if closed_toe is None or open_toe is None:
        ctx.fail("toe beam pose sampling available", "Could not resolve toe beam AABBs for closed/open poses.")
    else:
        closed_toe_center_z = (closed_toe[0][2] + closed_toe[1][2]) * 0.5
        open_toe_center_z = (open_toe[0][2] + open_toe[1][2]) * 0.5
        ctx.check(
            "leaf opens upward",
            open_toe_center_z > closed_toe_center_z + 6.0,
            details=f"closed_toe_center_z={closed_toe_center_z:.3f}, open_toe_center_z={open_toe_center_z:.3f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()

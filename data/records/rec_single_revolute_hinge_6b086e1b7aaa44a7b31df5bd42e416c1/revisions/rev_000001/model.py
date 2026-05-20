from __future__ import annotations

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_access_panel_hinge")

    zinc = model.material("satin_zinc", rgba=(0.64, 0.66, 0.64, 1.0))
    darker_zinc = model.material("dark_pin_steel", rgba=(0.22, 0.23, 0.24, 1.0))
    shadow = model.material("dark_recess", rgba=(0.025, 0.025, 0.023, 1.0))

    hinge_length = 0.200
    back_leaf_width = 0.026
    front_leaf_width = 0.018
    front_leaf_length = 0.140
    leaf_thickness = 0.0022
    knuckle_radius = 0.0045
    pin_radius = 0.0020
    screw_disk_radius = 0.0028

    back_leaf = model.part("back_leaf")
    back_leaf.visual(
        Box((back_leaf_width, leaf_thickness, hinge_length)),
        origin=Origin(xyz=(-(knuckle_radius + back_leaf_width / 2.0 - 0.0010), 0.0, 0.0)),
        material=zinc,
        name="back_leaf_plate",
    )

    # Three back-leaf barrels support the pin at the ends and at the center,
    # leaving clear windows where the shorter front leaf's barrels rotate.
    for name, z_center, length in (
        ("back_knuckle_lower", -0.081, 0.038),
        ("back_knuckle_center", 0.000, 0.042),
        ("back_knuckle_upper", 0.081, 0.038),
    ):
        back_leaf.visual(
            Cylinder(radius=knuckle_radius, length=length),
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            material=zinc,
            name=name,
        )

    back_leaf.visual(
        Cylinder(radius=pin_radius, length=hinge_length + 0.012),
        origin=Origin(),
        material=darker_zinc,
        name="pin",
    )
    for name, z_center in (("pin_cap_lower", -0.1035), ("pin_cap_upper", 0.1035)):
        back_leaf.visual(
            Cylinder(radius=0.0032, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            material=darker_zinc,
            name=name,
        )

    for i, z in enumerate((-0.070, -0.025, 0.025, 0.070)):
        back_leaf.visual(
            Cylinder(radius=screw_disk_radius, length=0.00065),
            origin=Origin(
                xyz=(-(knuckle_radius + back_leaf_width * 0.58), 0.00120, z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=darker_zinc,
            name=f"back_screw_{i}",
        )
        back_leaf.visual(
            Box((0.0042, 0.00045, 0.00045)),
            origin=Origin(xyz=(-(knuckle_radius + back_leaf_width * 0.58), 0.00158, z)),
            material=shadow,
            name=f"back_slot_{i}",
        )

    front_leaf = model.part("front_leaf")
    front_leaf.visual(
        Box((front_leaf_width, leaf_thickness, front_leaf_length)),
        origin=Origin(xyz=(knuckle_radius + front_leaf_width / 2.0 - 0.0010, 0.0, 0.0)),
        material=zinc,
        name="front_leaf_plate",
    )

    for name, z_center, length in (
        ("front_knuckle_lower", -0.0415, 0.029),
        ("front_knuckle_upper", 0.0415, 0.029),
    ):
        front_leaf.visual(
            Cylinder(radius=knuckle_radius, length=length),
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            material=zinc,
            name=name,
        )

    for i, z in enumerate((-0.045, 0.045)):
        front_leaf.visual(
            Cylinder(radius=screw_disk_radius, length=0.00065),
            origin=Origin(
                xyz=(knuckle_radius + front_leaf_width * 0.58, 0.00120, z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=darker_zinc,
            name=f"front_screw_{i}",
        )
        front_leaf.visual(
            Box((0.0038, 0.00045, 0.00045)),
            origin=Origin(xyz=(knuckle_radius + front_leaf_width * 0.58, 0.00158, z)),
            material=shadow,
            name=f"front_slot_{i}",
        )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=back_leaf,
        child=front_leaf,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    back_leaf = object_model.get_part("back_leaf")
    front_leaf = object_model.get_part("front_leaf")
    hinge = object_model.get_articulation("leaf_hinge")

    ctx.check(
        "single revolute hinge joint",
        len(object_model.articulations) == 1
        and object_model.articulations[0].articulation_type == ArticulationType.REVOLUTE,
    )

    for knuckle in ("front_knuckle_lower", "front_knuckle_upper"):
        ctx.allow_overlap(
            back_leaf,
            front_leaf,
            elem_a="pin",
            elem_b=knuckle,
            reason="The single hinge pin is intentionally captured inside the rotating front-leaf knuckle.",
        )
        ctx.expect_within(
            back_leaf,
            front_leaf,
            axes="xy",
            inner_elem="pin",
            outer_elem=knuckle,
            margin=0.0001,
            name=f"pin centered in {knuckle}",
        )
        ctx.expect_overlap(
            back_leaf,
            front_leaf,
            axes="z",
            elem_a="pin",
            elem_b=knuckle,
            min_overlap=0.020,
            name=f"pin spans {knuckle}",
        )

    ctx.expect_gap(
        front_leaf,
        back_leaf,
        axis="x",
        positive_elem="front_leaf_plate",
        negative_elem="back_leaf_plate",
        min_gap=0.006,
        name="leaves clear across the pin line",
    )

    rest_aabb = ctx.part_element_world_aabb(front_leaf, elem="front_leaf_plate")
    with ctx.pose({hinge: 1.25}):
        opened_aabb = ctx.part_element_world_aabb(front_leaf, elem="front_leaf_plate")
    ctx.check(
        "front leaf swings about the pin",
        rest_aabb is not None
        and opened_aabb is not None
        and ((opened_aabb[0][1] + opened_aabb[1][1]) * 0.5)
        > ((rest_aabb[0][1] + rest_aabb[1][1]) * 0.5) + 0.006,
        details=f"rest_aabb={rest_aabb}, opened_aabb={opened_aabb}",
    )

    return ctx.report()


object_model = build_object_model()

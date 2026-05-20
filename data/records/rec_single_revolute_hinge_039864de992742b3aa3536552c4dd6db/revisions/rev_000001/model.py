from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="heavy_strap_hinge")

    dark_blued_steel = model.material(
        "dark_blued_steel", color=(0.11, 0.12, 0.13, 1.0)
    )
    worn_edge_steel = model.material(
        "worn_edge_steel", color=(0.36, 0.37, 0.36, 1.0)
    )
    blackened_recess = model.material(
        "blackened_recess", color=(0.015, 0.014, 0.012, 1.0)
    )

    length = 0.420
    leaf_width = 0.120
    leaf_thickness = 0.012
    barrel_radius = 0.019
    pin_radius = 0.0075
    barrel_clearance = 0.006
    knuckle_count = 5
    knuckle_len = (length - barrel_clearance * (knuckle_count - 1)) / knuckle_count

    # The hinge axis is the part frame z-axis.  At q=0 the two heavy leaves lie
    # open and flat on opposite sides of the pin, like a robust utility hinge.
    fixed_leaf = model.part("leaf_0")
    fixed_leaf.visual(
        Box((leaf_width, leaf_thickness, length)),
        origin=Origin(xyz=(-(barrel_radius + leaf_width / 2.0), 0.0, 0.0)),
        material=dark_blued_steel,
        name="leaf_plate",
    )
    fixed_leaf.visual(
        Box((0.012, leaf_thickness * 1.45, length)),
        origin=Origin(xyz=(-(barrel_radius + 0.006), 0.0, 0.0)),
        material=worn_edge_steel,
        name="hinge_edge_rib",
    )
    fixed_leaf.visual(
        Box((0.010, leaf_thickness * 1.25, length * 0.82)),
        origin=Origin(xyz=(-(barrel_radius + leaf_width * 0.63), 0.0, 0.0)),
        material=worn_edge_steel,
        name="outer_stiffener",
    )

    moving_leaf = model.part("leaf_1")
    moving_leaf.visual(
        Box((leaf_width, leaf_thickness, length)),
        origin=Origin(xyz=(barrel_radius + leaf_width / 2.0, 0.0, 0.0)),
        material=dark_blued_steel,
        name="leaf_plate",
    )
    moving_leaf.visual(
        Box((0.012, leaf_thickness * 1.45, length)),
        origin=Origin(xyz=(barrel_radius + 0.006, 0.0, 0.0)),
        material=worn_edge_steel,
        name="hinge_edge_rib",
    )
    moving_leaf.visual(
        Box((0.010, leaf_thickness * 1.25, length * 0.82)),
        origin=Origin(xyz=(barrel_radius + leaf_width * 0.63, 0.0, 0.0)),
        material=worn_edge_steel,
        name="outer_stiffener",
    )

    # A continuous pin is visible in the gaps and protrudes with peened caps.
    fixed_leaf.visual(
        Cylinder(radius=pin_radius, length=length + 0.040),
        origin=Origin(),
        material=worn_edge_steel,
        name="pin_shaft",
    )
    fixed_leaf.visual(
        Cylinder(radius=barrel_radius * 0.82, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, length / 2.0 + 0.025)),
        material=worn_edge_steel,
        name="upper_pin_head",
    )
    fixed_leaf.visual(
        Cylinder(radius=barrel_radius * 0.82, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -length / 2.0 - 0.025)),
        material=worn_edge_steel,
        name="lower_pin_head",
    )

    def knuckle_center(index: int) -> float:
        return -length / 2.0 + knuckle_len / 2.0 + index * (
            knuckle_len + barrel_clearance
        )

    for index in (0, 2, 4):
        z = knuckle_center(index)
        fixed_leaf.visual(
            Cylinder(radius=barrel_radius, length=knuckle_len),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_blued_steel,
            name=f"knuckle_{index}",
        )
        fixed_leaf.visual(
            Box((0.016, leaf_thickness * 1.6, knuckle_len * 0.72)),
            origin=Origin(xyz=(-(barrel_radius - 0.0035), 0.0, z)),
            material=dark_blued_steel,
            name=f"barrel_web_{index}",
        )

    for index in (1, 3):
        z = knuckle_center(index)
        moving_leaf.visual(
            Cylinder(radius=barrel_radius, length=knuckle_len),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_blued_steel,
            name=f"knuckle_{index}",
        )
        moving_leaf.visual(
            Box((0.016, leaf_thickness * 1.6, knuckle_len * 0.72)),
            origin=Origin(xyz=(barrel_radius - 0.0035, 0.0, z)),
            material=dark_blued_steel,
            name=f"barrel_web_{index}",
        )

    # Bolt recesses are slightly sunk into the face so they read as load-bearing
    # strap plates rather than plain strips.
    bolt_zs = (-0.155, -0.055, 0.055, 0.155)
    for i, z in enumerate(bolt_zs):
        fixed_leaf.visual(
            Cylinder(radius=0.014, length=0.0025),
            origin=Origin(
                xyz=(-(barrel_radius + leaf_width * 0.52), -leaf_thickness / 2.0, z),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=blackened_recess,
            name=f"bolt_recess_{i}",
        )
        moving_leaf.visual(
            Cylinder(radius=0.014, length=0.0025),
            origin=Origin(
                xyz=(barrel_radius + leaf_width * 0.52, -leaf_thickness / 2.0, z),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=blackened_recess,
            name=f"bolt_recess_{i}",
        )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=fixed_leaf,
        child=moving_leaf,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-2.1, upper=2.1),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_leaf = object_model.get_part("leaf_0")
    moving_leaf = object_model.get_part("leaf_1")
    hinge = object_model.get_articulation("leaf_hinge")

    ctx.check(
        "single revolute barrel joint",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in hinge.axis) == (0.0, 0.0, 1.0),
        details=f"type={hinge.articulation_type}, axis={hinge.axis}",
    )

    # The moving knuckles are intentionally captured around the fixed pin.  The
    # simplified barrel solids stand in for hollow tubes, so the allowance is
    # scoped to the pin and each moving knuckle and paired with exact fit checks.
    for knuckle_name in ("knuckle_1", "knuckle_3"):
        ctx.allow_overlap(
            fixed_leaf,
            moving_leaf,
            elem_a="pin_shaft",
            elem_b=knuckle_name,
            reason="The hinge pin is intentionally captured inside the moving barrel knuckle.",
        )
        ctx.expect_within(
            fixed_leaf,
            moving_leaf,
            axes="xy",
            inner_elem="pin_shaft",
            outer_elem=knuckle_name,
            margin=0.0,
            name=f"pin centered in {knuckle_name}",
        )
        ctx.expect_overlap(
            fixed_leaf,
            moving_leaf,
            axes="z",
            elem_a="pin_shaft",
            elem_b=knuckle_name,
            min_overlap=0.070,
            name=f"pin spans {knuckle_name}",
        )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            moving_leaf,
            fixed_leaf,
            axis="x",
            positive_elem="leaf_plate",
            negative_elem="leaf_plate",
            min_gap=0.038,
            max_gap=0.045,
            name="open leaves clear across the barrel",
        )
        ctx.expect_overlap(
            fixed_leaf,
            moving_leaf,
            axes="z",
            elem_a="leaf_plate",
            elem_b="leaf_plate",
            min_overlap=0.40,
            name="long leaves align along the pin",
        )

    closed_aabb = ctx.part_element_world_aabb(moving_leaf, elem="leaf_plate")
    with ctx.pose({hinge: 1.2}):
        swung_aabb = ctx.part_element_world_aabb(moving_leaf, elem="leaf_plate")

    def aabb_center_y(aabb):
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) / 2.0

    closed_y = aabb_center_y(closed_aabb)
    swung_y = aabb_center_y(swung_aabb)
    ctx.check(
        "moving leaf swings about the pin",
        closed_y is not None and swung_y is not None and swung_y > closed_y + 0.055,
        details=f"closed_y={closed_y}, swung_y={swung_y}",
    )

    return ctx.report()


object_model = build_object_model()

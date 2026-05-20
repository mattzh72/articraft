from __future__ import annotations

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_leaf_enclosure_hinge")

    zinc = model.material("brushed_zinc", rgba=(0.70, 0.72, 0.70, 1.0))
    barrel_steel = model.material("barrel_steel", rgba=(0.58, 0.60, 0.60, 1.0))
    dark_recess = model.material("dark_recess", rgba=(0.06, 0.06, 0.055, 1.0))
    polished_pin = model.material("polished_pin", rgba=(0.82, 0.84, 0.82, 1.0))

    leaf_length = 0.132
    leaf_width = 0.052
    leaf_thickness = 0.0030
    barrel_radius = 0.0070
    knuckle_len = 0.0220

    fixed_leaf = model.part("fixed_leaf")
    fixed_leaf.visual(
        Box((leaf_width, leaf_thickness, leaf_length)),
        origin=Origin(xyz=(-0.0340, -0.0020, 0.0)),
        material=zinc,
        name="fixed_plate",
    )
    fixed_leaf.visual(
        Box((0.010, 0.0040, leaf_length)),
        origin=Origin(xyz=(-0.0115, -0.0017, 0.0)),
        material=zinc,
        name="fixed_edge_fold",
    )

    moving_leaf = model.part("moving_leaf")
    moving_leaf.visual(
        Box((leaf_width, leaf_thickness, leaf_length)),
        origin=Origin(xyz=(0.0360, 0.0060, 0.0)),
        material=zinc,
        name="moving_plate",
    )
    moving_leaf.visual(
        Box((0.010, 0.0040, leaf_length)),
        origin=Origin(xyz=(0.0120, 0.0060, 0.0)),
        material=zinc,
        name="moving_offset_flange",
    )

    # Five compact, interleaved knuckles around the local Z pin axis.  The
    # moving leaf is joggled outward in +Y, so the closed hinge has a clean,
    # stepped enclosure profile rather than stacked coplanar plates.
    knuckle_centers = (-0.048, -0.024, 0.0, 0.024, 0.048)
    fixed_knuckles = (0, 2, 4)
    moving_knuckles = (1, 3)
    for index in fixed_knuckles:
        z = knuckle_centers[index]
        fixed_leaf.visual(
            Cylinder(radius=barrel_radius, length=knuckle_len),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=barrel_steel,
            name=f"fixed_knuckle_{index}",
        )
        fixed_leaf.visual(
            Box((0.012, 0.006, knuckle_len)),
            origin=Origin(xyz=(-0.0070, -0.0010, z)),
            material=zinc,
            name=f"fixed_knuckle_web_{index}",
        )

    for index in moving_knuckles:
        z = knuckle_centers[index]
        moving_leaf.visual(
            Cylinder(radius=barrel_radius, length=knuckle_len),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=barrel_steel,
            name=f"moving_knuckle_{index}",
        )
        moving_leaf.visual(
            Box((0.012, 0.008, knuckle_len)),
            origin=Origin(xyz=(0.0090, 0.0035, z)),
            material=zinc,
            name=f"moving_offset_web_{index}",
        )

    fixed_leaf.visual(
        Cylinder(radius=0.0024, length=leaf_length + 0.006),
        origin=Origin(),
        material=polished_pin,
        name="hinge_pin",
    )

    # Flush dark countersunk screw recesses are modeled as shallow embedded
    # disks on each leaf face; small centre dots give them a machined look.
    for z in (-0.040, 0.040):
        fixed_leaf.visual(
            Cylinder(radius=0.0060, length=0.0009),
            origin=Origin(xyz=(-0.034, -0.00375, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_recess,
            name=f"fixed_countersink_{z:+.3f}",
        )
        fixed_leaf.visual(
            Cylinder(radius=0.0022, length=0.0011),
            origin=Origin(xyz=(-0.034, -0.00405, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=polished_pin,
            name=f"fixed_screw_head_{z:+.3f}",
        )
        moving_leaf.visual(
            Cylinder(radius=0.0060, length=0.0009),
            origin=Origin(xyz=(0.037, 0.00775, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_recess,
            name=f"moving_countersink_{z:+.3f}",
        )
        moving_leaf.visual(
            Cylinder(radius=0.0022, length=0.0011),
            origin=Origin(xyz=(0.037, 0.00805, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=polished_pin,
            name=f"moving_screw_head_{z:+.3f}",
        )

    # A small visible pin head and peened end cap sit just beyond the outer
    # knuckles without passing through the moving barrels as modeled solids.
    for z, cap_name in ((-0.065, "lower_pin_cap"), (0.065, "upper_pin_cap")):
        fixed_leaf.visual(
            Cylinder(radius=0.0042, length=0.0024),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=polished_pin,
            name=cap_name,
        )

    fixed_leaf.inertial = Inertial.from_geometry(
        Box((0.070, 0.024, 0.138)),
        mass=0.075,
        origin=Origin(xyz=(-0.025, -0.001, 0.0)),
    )
    moving_leaf.inertial = Inertial.from_geometry(
        Box((0.070, 0.026, 0.138)),
        mass=0.065,
        origin=Origin(xyz=(0.027, 0.005, 0.0)),
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=fixed_leaf,
        child=moving_leaf,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fixed_leaf = object_model.get_part("fixed_leaf")
    moving_leaf = object_model.get_part("moving_leaf")
    hinge = object_model.get_articulation("leaf_hinge")

    for moving_knuckle in ("moving_knuckle_1", "moving_knuckle_3"):
        ctx.allow_overlap(
            fixed_leaf,
            moving_leaf,
            elem_a="hinge_pin",
            elem_b=moving_knuckle,
            reason="The hinge pin is intentionally captured inside the moving barrel knuckle.",
        )
        ctx.expect_within(
            fixed_leaf,
            moving_leaf,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem=moving_knuckle,
            margin=0.0005,
            name=f"pin is centered inside {moving_knuckle}",
        )
        ctx.expect_overlap(
            fixed_leaf,
            moving_leaf,
            axes="z",
            elem_a="hinge_pin",
            elem_b=moving_knuckle,
            min_overlap=0.020,
            name=f"pin spans {moving_knuckle}",
        )

    ctx.expect_overlap(
        fixed_leaf,
        moving_leaf,
        axes="xy",
        elem_a="fixed_knuckle_2",
        elem_b="moving_knuckle_1",
        min_overlap=0.010,
        name="interleaved knuckles share the same pin axis",
    )
    ctx.expect_gap(
        moving_leaf,
        fixed_leaf,
        axis="x",
        positive_elem="moving_plate",
        negative_elem="fixed_plate",
        min_gap=0.010,
        max_gap=0.030,
        name="closed leaves remain cleanly separated across the barrel",
    )
    ctx.expect_gap(
        moving_leaf,
        fixed_leaf,
        axis="y",
        positive_elem="moving_plate",
        negative_elem="fixed_plate",
        min_gap=0.004,
        max_gap=0.012,
        name="moving leaf has the intended enclosure offset",
    )

    with ctx.pose({hinge: 1.2}):
        ctx.expect_gap(
            moving_leaf,
            fixed_leaf,
            axis="x",
            positive_elem="moving_plate",
            negative_elem="fixed_plate",
            min_gap=0.0,
            name="opened moving leaf clears the fixed plate",
        )
        ctx.expect_gap(
            moving_leaf,
            fixed_leaf,
            axis="y",
            positive_elem="moving_plate",
            negative_elem="fixed_plate",
            min_gap=0.010,
            name="positive rotation swings the offset leaf outward",
        )

    return ctx.report()


object_model = build_object_model()

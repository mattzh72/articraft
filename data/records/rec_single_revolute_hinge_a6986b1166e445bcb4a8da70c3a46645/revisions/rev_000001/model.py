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
    model = ArticulatedObject(name="narrow_inspection_lid_hinge")

    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.63, 0.60, 1.0))
    shadow = model.material("dark_recess", rgba=(0.035, 0.033, 0.030, 1.0))
    pin_end = model.material("polished_pin_end", rgba=(0.82, 0.82, 0.78, 1.0))

    length = 0.160
    clearance = 0.0012
    knuckle_count = 5
    knuckle_len = (length - clearance * (knuckle_count - 1)) / knuckle_count
    barrel_radius = 0.0055
    fixed_width = 0.028
    moving_width = 0.020
    leaf_thickness = 0.0022
    leaf_gap = 0.0010
    leaf_length = 0.154
    tab_depth = 0.0024
    tab_center = barrel_radius + leaf_gap * 0.30

    fixed_leaf = model.part("fixed_leaf")
    moving_leaf = model.part("moving_leaf")

    fixed_leaf.visual(
        Box((fixed_width, leaf_thickness, leaf_length)),
        origin=Origin(xyz=(-(barrel_radius + leaf_gap + fixed_width / 2.0), 0.0, 0.0)),
        material=brushed_steel,
        name="leaf_plate",
    )
    moving_leaf.visual(
        Box((moving_width, leaf_thickness, leaf_length)),
        origin=Origin(xyz=(barrel_radius + leaf_gap + moving_width / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="leaf_plate",
    )

    fixed_hole_x = -(barrel_radius + leaf_gap + fixed_width * 0.58)
    for i, z in enumerate((-0.052, 0.0, 0.052)):
        fixed_leaf.visual(
            Cylinder(radius=0.0032, length=0.0005),
            origin=Origin(xyz=(fixed_hole_x, 0.00125, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=shadow,
            name=f"screw_recess_{i}",
        )

    moving_hole_x = barrel_radius + leaf_gap + moving_width * 0.58
    for i, z in enumerate((-0.038, 0.038)):
        moving_leaf.visual(
            Cylinder(radius=0.0028, length=0.0005),
            origin=Origin(xyz=(moving_hole_x, 0.00125, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=shadow,
            name=f"screw_recess_{i}",
        )

    for i in range(knuckle_count):
        z = -length / 2.0 + knuckle_len / 2.0 + i * (knuckle_len + clearance)
        if i % 2 == 0:
            fixed_leaf.visual(
                Box((tab_depth, leaf_thickness, knuckle_len)),
                origin=Origin(xyz=(-tab_center, 0.0, z)),
                material=brushed_steel,
                name=f"barrel_tab_{i}",
            )
            fixed_leaf.visual(
                Cylinder(radius=barrel_radius, length=knuckle_len),
                origin=Origin(xyz=(0.0, 0.0, z)),
                material=brushed_steel,
                name=f"barrel_{i}",
            )
        else:
            moving_leaf.visual(
                Box((tab_depth, leaf_thickness, knuckle_len)),
                origin=Origin(xyz=(tab_center, 0.0, z)),
                material=brushed_steel,
                name=f"barrel_tab_{i}",
            )
            moving_leaf.visual(
                Cylinder(radius=barrel_radius, length=knuckle_len),
                origin=Origin(xyz=(0.0, 0.0, z)),
                material=brushed_steel,
                name=f"barrel_{i}",
            )

    cap_len = 0.0024
    fixed_leaf.visual(
        Cylinder(radius=0.0024, length=length + 0.0020),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pin_end,
        name="hinge_pin",
    )
    fixed_leaf.visual(
        Cylinder(radius=0.0034, length=cap_len),
        origin=Origin(xyz=(0.0, 0.0, -length / 2.0 - cap_len / 2.0 + 0.0002)),
        material=pin_end,
        name="lower_pin_head",
    )
    fixed_leaf.visual(
        Cylinder(radius=0.0034, length=cap_len),
        origin=Origin(xyz=(0.0, 0.0, length / 2.0 + cap_len / 2.0 - 0.0002)),
        material=pin_end,
        name="upper_pin_head",
    )

    model.articulation(
        "pin_hinge",
        ArticulationType.REVOLUTE,
        parent=fixed_leaf,
        child=moving_leaf,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_leaf = object_model.get_part("fixed_leaf")
    moving_leaf = object_model.get_part("moving_leaf")
    hinge = object_model.get_articulation("pin_hinge")

    fixed_barrels = [v for v in fixed_leaf.visuals if v.name and v.name.startswith("barrel_") and "tab" not in v.name]
    moving_barrels = [v for v in moving_leaf.visuals if v.name and v.name.startswith("barrel_") and "tab" not in v.name]
    ctx.check(
        "compact five-knuckle barrel",
        len(fixed_barrels) == 3 and len(moving_barrels) == 2,
        details=f"fixed={len(fixed_barrels)}, moving={len(moving_barrels)}",
    )

    ctx.check(
        "single revolute pin joint",
        len(object_model.articulations) == 1
        and hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in hinge.axis) == (0.0, 0.0, 1.0),
        details=f"joint_count={len(object_model.articulations)}, type={hinge.articulation_type}, axis={hinge.axis}",
    )

    ctx.expect_gap(
        moving_leaf,
        fixed_leaf,
        axis="x",
        positive_elem="leaf_plate",
        negative_elem="leaf_plate",
        min_gap=0.010,
        name="leaves sit on opposite sides of barrel",
    )
    ctx.expect_overlap(
        fixed_leaf,
        moving_leaf,
        axes="xy",
        min_overlap=0.008,
        name="alternating knuckles share pin centerline",
    )
    ctx.expect_overlap(
        fixed_leaf,
        moving_leaf,
        axes="z",
        min_overlap=0.145,
        name="both leaves span the inspection hinge length",
    )
    for barrel_name in ("barrel_1", "barrel_3"):
        ctx.allow_overlap(
            fixed_leaf,
            moving_leaf,
            elem_a="hinge_pin",
            elem_b=barrel_name,
            reason="The visible hinge pin is intentionally captured inside the slimmer moving-leaf knuckle.",
        )
        ctx.expect_within(
            fixed_leaf,
            moving_leaf,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem=barrel_name,
            margin=0.0,
            name=f"{barrel_name} surrounds the pin in plan",
        )
        ctx.expect_overlap(
            fixed_leaf,
            moving_leaf,
            axes="z",
            elem_a="hinge_pin",
            elem_b=barrel_name,
            min_overlap=0.030,
            name=f"{barrel_name} retains pin along length",
        )

    rest_aabb = ctx.part_element_world_aabb(moving_leaf, elem="leaf_plate")
    if rest_aabb is not None:
        rest_min, rest_max = rest_aabb
        rest_center = (
            (rest_min[0] + rest_max[0]) / 2.0,
            (rest_min[1] + rest_max[1]) / 2.0,
            (rest_min[2] + rest_max[2]) / 2.0,
        )
    else:
        rest_center = None

    with ctx.pose({hinge: 1.20}):
        posed_aabb = ctx.part_element_world_aabb(moving_leaf, elem="leaf_plate")
        if posed_aabb is not None:
            posed_min, posed_max = posed_aabb
            posed_center = (
                (posed_min[0] + posed_max[0]) / 2.0,
                (posed_min[1] + posed_max[1]) / 2.0,
                (posed_min[2] + posed_max[2]) / 2.0,
            )
        else:
            posed_center = None

    ctx.check(
        "moving leaf swings around pin",
        rest_center is not None
        and posed_center is not None
        and posed_center[1] > rest_center[1] + 0.012
        and posed_center[0] < rest_center[0] - 0.006,
        details=f"rest={rest_center}, posed={posed_center}",
    )

    return ctx.report()


object_model = build_object_model()

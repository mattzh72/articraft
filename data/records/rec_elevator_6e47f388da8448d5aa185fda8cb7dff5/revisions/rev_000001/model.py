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
    model = ArticulatedObject(name="cable_drum_residential_elevator")

    concrete = Material("painted_concrete", rgba=(0.62, 0.64, 0.64, 1.0))
    dark_floor = Material("dark_pit_floor", rgba=(0.12, 0.12, 0.13, 1.0))
    galvanized = Material("galvanized_steel", rgba=(0.58, 0.61, 0.62, 1.0))
    brushed = Material("brushed_aluminum", rgba=(0.75, 0.74, 0.69, 1.0))
    door_mat = Material("satin_door_panels", rgba=(0.82, 0.83, 0.80, 1.0))
    glass = Material("pale_smoked_glass", rgba=(0.45, 0.62, 0.75, 0.45))
    cable_mat = Material("black_steel_cable", rgba=(0.02, 0.02, 0.025, 1.0))
    counter_mat = Material("cast_iron_counterweight", rgba=(0.20, 0.22, 0.23, 1.0))

    shaft = model.part("shaft")

    # Connected shaft shell: rear wall, pit slab, top beam, guide rails, brackets,
    # and the overhead drum all belong to the fixed hoistway.
    shaft.visual(
        Box((2.00, 1.20, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_floor,
        name="pit_slab",
    )
    shaft.visual(
        Box((1.95, 0.05, 3.35)),
        origin=Origin(xyz=(0.0, 0.575, 1.675)),
        material=concrete,
        name="rear_wall",
    )
    shaft.visual(
        Box((2.00, 1.20, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 3.36)),
        material=concrete,
        name="top_slab",
    )
    for x in (-0.95, 0.95):
        shaft.visual(
            Box((0.08, 0.08, 3.35)),
            origin=Origin(xyz=(x, -0.56, 1.675)),
            material=galvanized,
            name=f"front_post_{'neg' if x < 0.0 else 'pos'}",
        )

    # Paired car guide rails, tied back to the rear wall at several heights.
    for x in (-0.56, 0.56):
        suffix = "neg" if x < 0.0 else "pos"
        shaft.visual(
            Box((0.035, 0.070, 3.12)),
            origin=Origin(xyz=(x, -0.02, 1.64)),
            material=galvanized,
            name=f"car_rail_{suffix}",
        )
        for i, z in enumerate((0.55, 1.55, 2.55)):
            shaft.visual(
                Box((0.14, 0.58, 0.035)),
                origin=Origin(xyz=(x, 0.27, z)),
                material=galvanized,
                name=f"rail_tie_{suffix}_{i}",
            )

    # Separate, slimmer counterweight guide rails on one side of the shaft.
    for x in (0.64, 0.93):
        suffix = "inner" if x < 0.8 else "outer"
        shaft.visual(
            Box((0.030, 0.055, 3.05)),
            origin=Origin(xyz=(x, 0.25, 1.63)),
            material=galvanized,
            name=f"weight_rail_{suffix}",
        )
        for i, z in enumerate((0.70, 1.70, 2.70)):
            shaft.visual(
                Box((0.070, 0.36, 0.030)),
                origin=Origin(xyz=(x, 0.41, z)),
                material=galvanized,
                name=f"weight_tie_{suffix}_{i}",
            )

    # Cable drum and support cheeks mounted high on the rear wall.
    shaft.visual(
        Cylinder(radius=0.16, length=0.58),
        origin=Origin(xyz=(0.02, 0.43, 3.12), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="cable_drum",
    )
    for x in (-0.34, 0.38):
        shaft.visual(
            Box((0.06, 0.18, 0.42)),
            origin=Origin(xyz=(x, 0.50, 3.09)),
            material=galvanized,
            name=f"drum_cheek_{'neg' if x < 0.0 else 'pos'}",
        )
    shaft.visual(
        Cylinder(radius=0.010, length=1.13),
        origin=Origin(xyz=(-0.24, 0.47, 2.515)),
        material=cable_mat,
        name="car_cable_drop",
    )
    shaft.visual(
        Cylinder(radius=0.010, length=0.90),
        origin=Origin(xyz=(0.80, 0.47, 2.62)),
        material=cable_mat,
        name="weight_cable_drop",
    )
    shaft.visual(
        Box((1.05, 0.020, 0.020)),
        origin=Origin(xyz=(0.28, 0.47, 3.07)),
        material=cable_mat,
        name="overhead_cable_run",
    )

    car = model.part("car")
    # Slim residential car shell with a clear front door opening.
    car.visual(
        Box((0.92, 0.76, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.835)),
        material=brushed,
        name="floor",
    )
    car.visual(
        Box((0.92, 0.76, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.835)),
        material=brushed,
        name="roof",
    )
    car.visual(
        Box((0.92, 0.040, 1.72)),
        origin=Origin(xyz=(0.0, 0.380, 0.0)),
        material=brushed,
        name="rear_panel",
    )
    for x in (-0.46, 0.46):
        car.visual(
            Box((0.040, 0.76, 1.72)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=brushed,
            name=f"side_panel_{'neg' if x < 0.0 else 'pos'}",
        )
    for x in (-0.43, 0.43):
        car.visual(
            Box((0.055, 0.050, 1.48)),
            origin=Origin(xyz=(x, -0.405, -0.050)),
            material=brushed,
            name=f"door_jamb_{'neg' if x < 0.0 else 'pos'}",
        )
    car.visual(
        Box((0.92, 0.050, 0.12)),
        origin=Origin(xyz=(0.0, -0.405, 0.705)),
        material=brushed,
        name="door_header",
    )
    car.visual(
        Box((0.90, 0.060, 0.045)),
        origin=Origin(xyz=(0.0, -0.410, -0.770)),
        material=brushed,
        name="door_threshold",
    )
    car.visual(
        Box((0.30, 0.012, 0.95)),
        origin=Origin(xyz=(0.0, 0.354, -0.05)),
        material=glass,
        name="rear_glass_panel",
    )
    car.visual(
        Box((0.18, 0.10, 0.055)),
        origin=Origin(xyz=(-0.24, 0.36, 0.8875)),
        material=galvanized,
        name="cable_hitch",
    )
    car.visual(
        Box((0.070, 0.030, 0.30)),
        origin=Origin(xyz=(-0.5075, -0.02, 0.0)),
        material=galvanized,
        name="guide_shoe_neg",
    )
    car.visual(
        Box((0.070, 0.030, 0.30)),
        origin=Origin(xyz=(0.5075, -0.02, 0.0)),
        material=galvanized,
        name="guide_shoe_pos",
    )

    model.articulation(
        "shaft_to_car",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=car,
        origin=Origin(xyz=(0.0, -0.03, 1.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.55, lower=0.0, upper=1.30),
    )

    counterweight = model.part("counterweight")
    counterweight.visual(
        Box((0.18, 0.16, 0.62)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=counter_mat,
        name="weight_stack",
    )
    counterweight.visual(
        Box((0.260, 0.035, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.3375)),
        material=galvanized,
        name="top_shoe_bar",
    )
    counterweight.visual(
        Box((0.260, 0.035, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, -0.3375)),
        material=galvanized,
        name="bottom_shoe_bar",
    )
    counterweight.visual(
        Box((0.06, 0.08, 0.050)),
        origin=Origin(xyz=(0.0, 0.08, 0.335)),
        material=galvanized,
        name="cable_socket",
    )

    model.articulation(
        "shaft_to_counterweight",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=counterweight,
        origin=Origin(xyz=(0.785, 0.25, 2.55)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.55, lower=0.0, upper=1.25),
    )

    leaf_0 = model.part("door_leaf_0")
    leaf_0.visual(
        Box((0.38, 0.030, 1.36)),
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        material=door_mat,
        name="panel",
    )
    leaf_0.visual(
        Cylinder(radius=0.018, length=1.43),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=galvanized,
        name="hinge_barrel",
    )
    leaf_0.visual(
        Cylinder(radius=0.012, length=0.12),
        origin=Origin(xyz=(0.315, -0.025, -0.03), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="pull_knob",
    )

    leaf_1 = model.part("door_leaf_1")
    leaf_1.visual(
        Box((0.38, 0.030, 1.36)),
        origin=Origin(xyz=(-0.190, 0.0, 0.0)),
        material=door_mat,
        name="panel",
    )
    leaf_1.visual(
        Cylinder(radius=0.018, length=1.43),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=galvanized,
        name="hinge_barrel",
    )
    leaf_1.visual(
        Cylinder(radius=0.012, length=0.12),
        origin=Origin(xyz=(-0.315, -0.025, -0.03), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="pull_knob",
    )

    model.articulation(
        "car_to_door_leaf_0",
        ArticulationType.REVOLUTE,
        parent=car,
        child=leaf_0,
        origin=Origin(xyz=(-0.40, -0.455, -0.04)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.35),
    )
    model.articulation(
        "car_to_door_leaf_1",
        ArticulationType.REVOLUTE,
        parent=car,
        child=leaf_1,
        origin=Origin(xyz=(0.40, -0.455, -0.04)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shaft = object_model.get_part("shaft")
    car = object_model.get_part("car")
    counterweight = object_model.get_part("counterweight")
    leaf_0 = object_model.get_part("door_leaf_0")
    leaf_1 = object_model.get_part("door_leaf_1")
    car_lift = object_model.get_articulation("shaft_to_car")
    weight_lift = object_model.get_articulation("shaft_to_counterweight")
    hinge_0 = object_model.get_articulation("car_to_door_leaf_0")
    hinge_1 = object_model.get_articulation("car_to_door_leaf_1")

    ctx.expect_gap(
        car,
        shaft,
        axis="z",
        positive_elem="floor",
        negative_elem="pit_slab",
        min_gap=0.05,
        max_gap=0.15,
        name="car clears the pit at the lower landing",
    )
    ctx.expect_gap(
        leaf_1,
        leaf_0,
        axis="x",
        positive_elem="panel",
        negative_elem="panel",
        min_gap=0.015,
        max_gap=0.060,
        name="closed hinged leaves meet at the center",
    )
    ctx.expect_overlap(
        leaf_0,
        leaf_1,
        axes="yz",
        elem_a="panel",
        elem_b="panel",
        min_overlap=0.02,
        name="closed leaves share height and front plane",
    )
    ctx.expect_origin_distance(
        car,
        counterweight,
        axes="x",
        min_dist=0.70,
        name="counterweight runs in a separate side bay",
    )

    rest_car_pos = ctx.part_world_position(car)
    rest_weight_pos = ctx.part_world_position(counterweight)
    rest_leaf_0_aabb = ctx.part_world_aabb(leaf_0)
    rest_leaf_1_aabb = ctx.part_world_aabb(leaf_1)

    with ctx.pose({car_lift: 1.30}):
        ctx.expect_gap(
            shaft,
            car,
            axis="z",
            positive_elem="top_slab",
            negative_elem="roof",
            min_gap=0.05,
            name="raised car stays below the shaft headroom",
        )
        raised_car_pos = ctx.part_world_position(car)

    with ctx.pose({weight_lift: 1.25}):
        lowered_weight_pos = ctx.part_world_position(counterweight)
        ctx.expect_gap(
            counterweight,
            shaft,
            axis="z",
            positive_elem="bottom_shoe_bar",
            negative_elem="pit_slab",
            min_gap=0.15,
            name="counterweight remains above the pit at lower travel",
        )

    with ctx.pose({hinge_0: 1.20, hinge_1: 1.20}):
        open_leaf_0_aabb = ctx.part_world_aabb(leaf_0)
        open_leaf_1_aabb = ctx.part_world_aabb(leaf_1)

    ctx.check(
        "car prismatic joint raises vertically",
        rest_car_pos is not None
        and raised_car_pos is not None
        and raised_car_pos[2] > rest_car_pos[2] + 1.20,
        details=f"rest={rest_car_pos}, raised={raised_car_pos}",
    )
    ctx.check(
        "counterweight prismatic joint travels downward",
        rest_weight_pos is not None
        and lowered_weight_pos is not None
        and lowered_weight_pos[2] < rest_weight_pos[2] - 1.15,
        details=f"rest={rest_weight_pos}, lowered={lowered_weight_pos}",
    )
    ctx.check(
        "hinged leaves swing outward from the frame",
        rest_leaf_0_aabb is not None
        and rest_leaf_1_aabb is not None
        and open_leaf_0_aabb is not None
        and open_leaf_1_aabb is not None
        and open_leaf_0_aabb[0][1] < rest_leaf_0_aabb[0][1] - 0.20
        and open_leaf_1_aabb[0][1] < rest_leaf_1_aabb[0][1] - 0.20,
        details=f"rest0={rest_leaf_0_aabb}, open0={open_leaf_0_aabb}, rest1={rest_leaf_1_aabb}, open1={open_leaf_1_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
